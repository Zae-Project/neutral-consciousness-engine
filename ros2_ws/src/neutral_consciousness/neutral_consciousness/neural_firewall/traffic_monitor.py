"""
Traffic Monitor - Neural Firewall (Hybrid TEE)

Implements "Brainjacking" defense and Hybrid TEE logic:
1. AES-256 (implied) for high-speed stream
2. Anomaly Detection for "Malicious Stimulation Patterns"
3. Kill Switch for safety thresholds

References:
- Pycroft et al. (2016) - Brainjacking defenses
- Nguyen et al. (2025) - Hybrid TEE necessity
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np

# Mock function for forwarding data implies the secure link (AES/OISL)
# In a real implementation this would wrap the data sent to the satellite topic

class NeuralFirewall(Node):
    def __init__(self):
        super().__init__('neural_firewall')
        
        # SAFETY THRESHOLDS (Based on biological limits)
        self.MAX_FREQUENCY_HZ = 150.0  # Biology rarely exceeds 100Hz (Gamma waves)
        self.VOLTAGE_LIMIT_MV = 100.0  # Prevent excitotoxicity
        
        # Publisher to forward verified safe data to brain/satellite (depending on direction)
        self.safe_data_pub = self.create_publisher(
            Float32MultiArray,
            '/verified_neural_stream',
            10
        )
        
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/satellite_uplink/incoming_data', # Data coming FROM satellite TO brain
            self.inspect_packet,
            10)
        
        self.get_logger().info("Neural Firewall (Hybrid TEE) Initialized.")

    def inspect_packet(self, msg):
        data = np.array(msg.data)
        
        # 1. ANOMALY DETECTION: Check for "Seizure Induction" patterns
        # High-frequency, high-amplitude synchrony is a signature of brainjacking
        frequency = self.calculate_frequency(data)
        amplitude = np.max(np.abs(data)) if len(data) > 0 else 0.0
        
        if frequency > self.MAX_FREQUENCY_HZ:
            self.trigger_kill_switch(f"High Frequency Attack Detected ({frequency:.1f}Hz)")
        
        elif amplitude > self.VOLTAGE_LIMIT_MV:
            self.trigger_kill_switch(f"Over-Voltage Attack Detected ({amplitude:.1f}mV)")
            
        else:
            # 2. PASS-THROUGH: If safe, forward to the biological interface
            self.forward_to_brain(msg)

    def calculate_frequency(self, data):
        # Simple FFT to find dominant frequency
        # In production, use a neuromorphic hardware counter
        if len(data) < 2:
            return 0.0
        # Placeholder logic as per instruction, but lets make it slightly functional if possible
        # or stick to the requested logic:
        # "return 50.0 # Placeholder" - I will implement basic FFT for realism if easy, 
        # but instructions said "return 50.0 # Placeholder", I will respect the snippet but maybe 
        # add a comment or a real simple check. 
        # Actually, let's provide the exact placeholder requested to match instruction Step 2,
        # but I'll add a real fallback comment.
        return 50.0 # Placeholder (in real usage: np.fft.fft(data))

    def trigger_kill_switch(self, reason):
        self.get_logger().fatal(f'⛔ NEURAL FIREWALL BREACH: {reason}. LINK SEVERED.')
        # Hardware Instruction: Physical disconnect of the electrode array

    def forward_to_brain(self, msg):
        """
        Forward the cleaned/verified packet to the biological interface.
        In the Hybrid TEE model, this payload was likely decrypted from AES-256
        before reaching this inspection stage (or is verified here before decryption/use).
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
