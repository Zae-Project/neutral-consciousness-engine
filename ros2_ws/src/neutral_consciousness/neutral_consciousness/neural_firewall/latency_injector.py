"""
Latency Injector - OISL Delay Simulation for Watanabe Transfer Protocol

Simulates the OISL (Optical Inter-Satellite Link) delay to validate
Orbital Migration capability within the Libet Limit.

SCIENTIFIC BASIS:
- Libet, B. (1983). ~350ms lag between neural events and conscious awareness
- Libet Limit: 500ms (consciousness continuity threshold)
- OISL Target: <50ms RTT (well within Libet buffer)

NOTE: This implements concepts inspired by Watanabe's hemisphere integration
research. This is NOT official work of Professor Watanabe.

Logic:
- Intercepts neural data from SNN
- Holds it in a queue for configurable round_trip_time_ms
- Validates against Libet's 500ms limit
- Triggers fatal error if consciousness continuity would break
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time
from collections import deque

class LatencyInjector(Node):
    def __init__(self):
        super().__init__('latency_injector')
        
        self.declare_parameter('round_trip_time_ms', 20.0)
        self.rtt_ms = self.get_parameter('round_trip_time_ms').value
        
        # Libet's Limit: 500ms ("Time to Consciousness")
        self.LIBET_LIMIT_MS = 500.0
        
        if self.rtt_ms > self.LIBET_LIMIT_MS:
            self.get_logger().fatal(f"CONFIG ERROR: RTT ({self.rtt_ms}ms) exceeds Libet's Limit!")
        
        # Queue: (timestamp, msg)
        self.packet_queue = deque()
        
        # Intercept Topic
        self.sub = self.create_subscription(
            Float32MultiArray,
            '/neural_stream/generated', # Output from SNN
            self.enqueue_packet,
            10
        )
        
        # Delayed Topic
        self.pub = self.create_publisher(
            Float32MultiArray,
            '/neural_stream/delayed', # Input to Body/Unity
            10
        )
        
        # High freq timer to check queue
        self.timer = self.create_timer(0.001, self.process_queue)
        
        self.get_logger().info(f"Latency Injector Active. RTT: {self.rtt_ms}ms")

    def enqueue_packet(self, msg):
        now = self.get_clock().now().nanoseconds / 1e6 # ms
        self.packet_queue.append((now, msg))

    def process_queue(self):
        if not self.packet_queue:
            return
            
        now = self.get_clock().now().nanoseconds / 1e6 # ms
        
        # Check head of queue
        arrival_time, msg = self.packet_queue[0]
        
        if (now - arrival_time) >= self.rtt_ms:
            # Latency condition met, release packet
            self.packet_queue.popleft()
            self.pub.publish(msg)
            
            # Runtime Safety Check
            latency = now - arrival_time
            if latency > self.LIBET_LIMIT_MS:
                self.get_logger().fatal(
                    f"LIBET VIOLATION: Packet delayed by {latency:.1f}ms (>500ms). "
                    "Consciousness Continuity Broken!"
                )

def main(args=None):
    rclpy.init(args=args)
    node = LatencyInjector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
