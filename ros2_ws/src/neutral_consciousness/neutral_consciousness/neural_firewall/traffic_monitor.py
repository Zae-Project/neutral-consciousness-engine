"""
Traffic Monitor Module

Neural Firewall security component that monitors spike rates
and neural traffic patterns to ensure safe operation.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool, String
import numpy as np


class NeuralFirewall(Node):
    """
    ROS 2 Node implementing the Neural Firewall.
    
    Monitors neural traffic for anomalies and implements
    safety mechanisms including a kill switch for emergency
    disconnection.
    """
    
    def __init__(self):
        super().__init__('neural_firewall')
        
        # Configuration parameters
        self.declare_parameter('spike_rate_limit', 200.0)
        self.declare_parameter('anomaly_threshold', 3.0)
        self.declare_parameter('monitoring_window', 1.0)
        
        # Threshold: Max spikes per second before "Kill Switch" triggers
        self.SPIKE_RATE_LIMIT = self.get_parameter('spike_rate_limit').value
        self.ANOMALY_THRESHOLD = self.get_parameter('anomaly_threshold').value
        self.MONITORING_WINDOW = self.get_parameter('monitoring_window').value
        
        # Internal state for running statistics
        self.spike_history = []
        self.baseline_mean = 50.0
        self.baseline_std = 20.0
        self.is_connected = True
        
        # Subscribers
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'incoming_neural_data',
            self.monitor_traffic,
            10
        )
        
        self.cortex_activity_sub = self.create_subscription(
            Float32MultiArray,
            'cortex/visual/activity',
            self.check_cortex_activity,
            10
        )
        
        # Publishers
        self.security_status_pub = self.create_publisher(
            String,
            'firewall/status',
            10
        )
        
        self.kill_switch_pub = self.create_publisher(
            Bool,
            'firewall/kill_switch',
            10
        )
        
        self.alert_pub = self.create_publisher(
            String,
            'firewall/alerts',
            10
        )
        
        # Status timer
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info('Neural Firewall initialized - Monitoring active')
        self.get_logger().info(f'Spike rate limit: {self.SPIKE_RATE_LIMIT} Hz')

    def monitor_traffic(self, msg: Float32MultiArray):
        """
        Monitor incoming neural data for security threats.
        
        Args:
            msg: Float32MultiArray containing neural activity data
        """
        if len(msg.data) == 0:
            return
            
        spike_rate = sum(msg.data) / len(msg.data)  # Simplified metric
        
        # Record for statistics
        self.spike_history.append(spike_rate)
        if len(self.spike_history) > 100:
            self.spike_history.pop(0)
        
        # Update baseline statistics
        if len(self.spike_history) >= 10:
            self.baseline_mean = np.mean(self.spike_history)
            self.baseline_std = np.std(self.spike_history)
        
        # Check for rate limit violation
        if spike_rate > self.SPIKE_RATE_LIMIT:
            self.get_logger().fatal(
                f'SECURITY ALERT: Spike rate {spike_rate:.2f}Hz exceeds safety limit!'
            )
            self._publish_alert(f'RATE_LIMIT_EXCEEDED: {spike_rate:.2f}Hz')
            self.trigger_kill_switch()
            return
        
        # Check for anomalous activity
        if self.baseline_std > 0:
            z_score = abs(spike_rate - self.baseline_mean) / self.baseline_std
            if z_score > self.ANOMALY_THRESHOLD:
                self.get_logger().warning(
                    f'Anomalous activity detected: z-score={z_score:.2f}'
                )
                self._publish_alert(f'ANOMALY_DETECTED: z-score={z_score:.2f}')
    
    def check_cortex_activity(self, msg: Float32MultiArray):
        """
        Monitor cortex activity for unusual patterns.
        
        Args:
            msg: Neural activity from cortex modules
        """
        if len(msg.data) == 0:
            return
        
        # Check for synchronization attacks (too many neurons firing together)
        activity = np.array(msg.data)
        synchrony = np.std(activity)
        
        if synchrony < 0.01 and np.mean(activity) > 0.5:
            self.get_logger().warning(
                'High synchronization detected - possible injection attack'
            )
            self._publish_alert('SYNC_ATTACK_POSSIBLE')

    def trigger_kill_switch(self):
        """
        Activate the kill switch to sever all external connections.
        
        This is the emergency response to detected security threats.
        """
        self.get_logger().info('KILL SWITCH ACTIVATED: Connection Severed.')
        self.is_connected = False
        
        # Publish kill switch activation
        kill_msg = Bool()
        kill_msg.data = True
        self.kill_switch_pub.publish(kill_msg)
        
        self._publish_alert('KILL_SWITCH_ACTIVATED')
        
        # In a real scenario, this would:
        # 1. Send a hardware interrupt signal
        # 2. Sever network connections
        # 3. Freeze SNN state
        # 4. Log forensic data
    
    def publish_status(self):
        """Publish current firewall status."""
        status_msg = String()
        
        if self.is_connected:
            avg_rate = np.mean(self.spike_history) if self.spike_history else 0.0
            status_msg.data = f'OK: avg_rate={avg_rate:.2f}Hz, connected=True'
        else:
            status_msg.data = 'DISCONNECTED: Kill switch activated'
        
        self.security_status_pub.publish(status_msg)
    
    def _publish_alert(self, alert_type: str):
        """Publish a security alert."""
        alert_msg = String()
        alert_msg.data = alert_type
        self.alert_pub.publish(alert_msg)


def main(args=None):
    """Main entry point for the Neural Firewall node."""
    rclpy.init(args=args)
    node = NeuralFirewall()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
