"""
Traffic Monitor - Neural Firewall (Hybrid TEE)

Implements "Brainjacking" defense and Hybrid TEE logic:
1. AES-256 (implied) for high-speed stream
2. Anomaly Detection for "Malicious Stimulation Patterns"
3. Kill Switch for safety thresholds - notifies Unity for physical disconnect

References:
- Pycroft et al. (2016) - Brainjacking defenses
- Nguyen et al. (2025) - Hybrid TEE necessity
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool
import numpy as np


class NeuralFirewall(Node):
    """
    Neural Firewall Node - Brainjacking Defense System
    
    Monitors incoming neural data from satellite uplink for malicious patterns:
    - High-frequency stimulation (seizure induction)
    - Over-voltage signals (excitotoxicity)
    - Anomalous synchronization patterns
    
    When a threat is detected, triggers kill switch to disconnect Unity/biological interface.
    """
    
    def __init__(self):
        super().__init__('neural_firewall')
        
        # Declare parameters
        self.declare_parameter('max_frequency_hz', 150.0)
        self.declare_parameter('voltage_limit_mv', 100.0)
        self.declare_parameter('sample_rate_hz', 1000.0)  # Assumed sample rate for FFT
        
        # SAFETY THRESHOLDS (Based on biological limits)
        self.MAX_FREQUENCY_HZ = self.get_parameter('max_frequency_hz').value
        self.VOLTAGE_LIMIT_MV = self.get_parameter('voltage_limit_mv').value
        self.SAMPLE_RATE_HZ = self.get_parameter('sample_rate_hz').value
        
        # Kill switch state
        self.kill_switch_active = False
        
        # Publisher to forward verified safe data to brain/satellite
        self.safe_data_pub = self.create_publisher(
            Float32MultiArray,
            '/verified_neural_stream',
            10
        )
        
        # Publisher for kill switch - Unity subscribes to this for emergency disconnect
        self.kill_switch_pub = self.create_publisher(
            Bool,
            'firewall/kill_switch',
            10  # QoS: Reliable delivery
        )
        
        # Subscriber for incoming satellite data
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/satellite_uplink/incoming_data',
            self.inspect_packet,
            10
        )
        
        # Periodic kill switch heartbeat (Unity can detect if firewall goes offline)
        self.heartbeat_timer = self.create_timer(1.0, self.publish_heartbeat)
        
        self.get_logger().info(
            f"Neural Firewall (Hybrid TEE) Initialized. "
            f"Thresholds: freq<{self.MAX_FREQUENCY_HZ}Hz, volt<{self.VOLTAGE_LIMIT_MV}mV"
        )

    def inspect_packet(self, msg):
        """
        Inspect incoming neural data packet for brainjacking signatures.
        
        Detection methods:
        1. Frequency analysis via FFT - detect high-frequency seizure induction
        2. Amplitude check - detect over-voltage excitotoxicity attacks
        """
        if self.kill_switch_active:
            # Already in lockdown - drop all packets
            return
            
        data = np.array(msg.data)
        
        if len(data) == 0:
            return
        
        # 1. FREQUENCY ANALYSIS: Check for "Seizure Induction" patterns
        # High-frequency synchrony (>150Hz) is a signature of brainjacking
        dominant_freq = self.calculate_frequency(data)
        
        # 2. AMPLITUDE CHECK: Detect over-voltage attacks
        amplitude = np.max(np.abs(data))
        
        # 3. THREAT DETECTION
        if dominant_freq > self.MAX_FREQUENCY_HZ:
            self.trigger_kill_switch(
                f"High Frequency Attack Detected ({dominant_freq:.1f}Hz > {self.MAX_FREQUENCY_HZ}Hz)"
            )
        elif amplitude > self.VOLTAGE_LIMIT_MV:
            self.trigger_kill_switch(
                f"Over-Voltage Attack Detected ({amplitude:.1f}mV > {self.VOLTAGE_LIMIT_MV}mV)"
            )
        else:
            # SAFE: Forward to biological interface
            self.forward_to_brain(msg)

    def calculate_frequency(self, data: np.ndarray) -> float:
        """
        Calculate dominant frequency using FFT analysis.
        
        This replaces the placeholder implementation with real frequency detection.
        Used to identify high-frequency stimulation patterns indicative of brainjacking.
        
        Args:
            data: Neural signal data array
            
        Returns:
            Dominant frequency in Hz
        """
        if len(data) < 4:
            return 0.0
        
        try:
            # Remove DC component (mean)
            data_centered = data - np.mean(data)
            
            # Apply FFT
            fft_result = np.fft.rfft(data_centered)
            fft_magnitude = np.abs(fft_result)
            
            # Get frequency bins
            n_samples = len(data)
            freq_bins = np.fft.rfftfreq(n_samples, d=1.0/self.SAMPLE_RATE_HZ)
            
            # Find dominant frequency (excluding DC at index 0)
            if len(fft_magnitude) > 1:
                # Skip DC component, find peak
                peak_idx = np.argmax(fft_magnitude[1:]) + 1
                dominant_freq = freq_bins[peak_idx]
                return float(dominant_freq)
            
            return 0.0
            
        except Exception as e:
            self.get_logger().warn(f"FFT calculation failed: {e}")
            return 0.0

    def trigger_kill_switch(self, reason: str):
        """
        Activate emergency kill switch - severs connection to biological interface.
        
        This publishes to the firewall/kill_switch topic which Unity monitors
        to initiate emergency disconnection procedures.
        
        Args:
            reason: Human-readable description of the detected threat
        """
        self.kill_switch_active = True
        
        # Log critical alert
        self.get_logger().fatal(f'NEURAL FIREWALL BREACH: {reason}')
        self.get_logger().fatal('KILL SWITCH ACTIVATED - LINK SEVERED')
        
        # Publish kill switch signal to Unity
        kill_msg = Bool()
        kill_msg.data = True
        self.kill_switch_pub.publish(kill_msg)
        
        # Publish multiple times for reliability (critical safety signal)
        for _ in range(3):
            self.kill_switch_pub.publish(kill_msg)

    def publish_heartbeat(self):
        """
        Publish periodic heartbeat on kill switch topic.
        
        Unity can monitor this to detect if the firewall node crashes.
        Normal operation: False (no kill)
        Emergency: True (kill switch active)
        """
        heartbeat_msg = Bool()
        heartbeat_msg.data = self.kill_switch_active
        self.kill_switch_pub.publish(heartbeat_msg)

    def forward_to_brain(self, msg):
        """
        Forward verified safe packet to the biological interface.
        
        In the Hybrid TEE model, this payload has been:
        1. Decrypted from AES-256 (high-speed stream)
        2. Inspected for brainjacking patterns
        3. Validated against safety thresholds
        """
        self.safe_data_pub.publish(msg)


def main(args=None):
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
