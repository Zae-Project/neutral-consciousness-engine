"""
Visual Cortex Module

Implements visual processing using Nengo Spiking Neural Network.
This module receives visual data from Unity and processes it through
the SNN to generate predictions.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
import numpy as np

try:
    import nengo
    NENGO_AVAILABLE = True
except ImportError:
    NENGO_AVAILABLE = False
    print("Warning: Nengo not installed. Running in stub mode.")


class VisualCortex(Node):
    """
    ROS 2 Node implementing the Visual Cortex SNN.
    
    Receives visual input from the Unity virtual environment,
    processes it through a Spiking Neural Network, and publishes
    the neural activity for downstream processing.
    """
    
    def __init__(self):
        super().__init__('visual_cortex')
        
        # Configuration parameters
        self.declare_parameter('n_neurons', 1000)
        self.declare_parameter('simulation_dt', 0.001)
        
        self.n_neurons = self.get_parameter('n_neurons').value
        self.simulation_dt = self.get_parameter('simulation_dt').value
        
        # Publishers and Subscribers
        self.visual_input_sub = self.create_subscription(
            Image,
            'unity/camera/raw',
            self.visual_input_callback,
            10
        )
        
        self.neural_activity_pub = self.create_publisher(
            Float32MultiArray,
            'cortex/visual/activity',
            10
        )
        
        # Initialize the SNN if Nengo is available
        if NENGO_AVAILABLE:
            self._build_snn()
        
        self.get_logger().info(f'Visual Cortex initialized with {self.n_neurons} neurons')
    
    def _build_snn(self):
        """Build the Nengo Spiking Neural Network model."""
        self.model = nengo.Network(label='Visual Cortex')
        
        with self.model:
            # Input layer - receives preprocessed visual data
            self.input_node = nengo.Node(output=np.zeros(64))
            
            # Visual cortex ensemble - primary visual processing
            self.v1_ensemble = nengo.Ensemble(
                n_neurons=self.n_neurons,
                dimensions=64,
                neuron_type=nengo.LIF(tau_rc=0.02, tau_ref=0.002),
                label='V1'
            )
            
            # Connect input to V1
            nengo.Connection(self.input_node, self.v1_ensemble)
            
            # Probe neural activity
            self.activity_probe = nengo.Probe(
                self.v1_ensemble.neurons, 
                sample_every=self.simulation_dt
            )
        
        # Create simulator
        self.sim = nengo.Simulator(self.model, dt=self.simulation_dt)
    
    def visual_input_callback(self, msg: Image):
        """
        Process incoming visual data from Unity.
        
        Args:
            msg: ROS Image message from Unity camera
        """
        # Convert ROS Image to numpy array and preprocess
        visual_data = self._preprocess_image(msg)
        
        if NENGO_AVAILABLE:
            # Feed data through SNN
            activity = self._run_snn(visual_data)
        else:
            # Stub: random activity for testing
            activity = np.random.rand(self.n_neurons).astype(np.float32)
        
        # Publish neural activity
        activity_msg = Float32MultiArray()
        activity_msg.data = activity.tolist()
        self.neural_activity_pub.publish(activity_msg)
    
    def _preprocess_image(self, msg: Image) -> np.ndarray:
        """
        Preprocess ROS Image message for SNN input.
        
        Args:
            msg: ROS Image message
            
        Returns:
            Preprocessed numpy array suitable for SNN input
        """
        # Convert image data to numpy array
        if len(msg.data) > 0:
            img_array = np.frombuffer(msg.data, dtype=np.uint8)
            # Reshape based on image dimensions
            img_array = img_array.reshape((msg.height, msg.width, -1))
            # Downsample to 8x8 and flatten for 64-dimensional input
            downsampled = img_array[::msg.height//8, ::msg.width//8, 0]
            return downsampled.flatten().astype(np.float32) / 255.0
        else:
            return np.zeros(64, dtype=np.float32)
    
    def _run_snn(self, input_data: np.ndarray) -> np.ndarray:
        """
        Run the SNN simulation step.
        
        Args:
            input_data: Input vector for the SNN
            
        Returns:
            Neural activity array
        """
        # Update input node
        self.input_node.output = input_data
        
        # Run one time step
        self.sim.run_steps(1)
        
        # Get activity from probe
        activity = self.sim.data[self.activity_probe][-1]
        return activity.astype(np.float32)


def main(args=None):
    """Main entry point for the Visual Cortex node."""
    rclpy.init(args=args)
    node = VisualCortex()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
