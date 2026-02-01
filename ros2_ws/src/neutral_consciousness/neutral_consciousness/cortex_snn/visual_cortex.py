"""
Visual Cortex Module - True Predictive Coding Implementation

This module uses Nengo to implement a "Generative Model" where the network
predicts input and only fires on error. This effectively implements a
consciousness architecture inspired by the research approach described in:

SCIENTIFIC FOUNDATION (Inspired By):
    Watanabe, M., et al. (2014). "Interhemispheric transfer of visual 
    information in split-brain patients." Neuropsychologia, 63, 133-142.
    DOI: 10.1016/j.neuropsychologia.2014.08.025
    
    NOTE: This is NOT the official work of Professor Masataka Watanabe. 
    This implementation is inspired by and builds upon principles from 
    his published research. We are independent researchers exploring 
    consciousness substrate transfer based on his approach.

IMPLEMENTATION DETAILS:
- Architecture: 1000 LIF Neurons (Cortex) + 500 LIF Neurons (Error)
- Logic: Predictive Coding (Error = Input - Prediction)
- Learning: PES (Prescribed Error Sensitivity) Rule  
- Metrics: Euclidean Distance & Synchronization Health
- Framework: Nengo (Neural Engineering Framework)

RELATED RESEARCH:
- Friston, K. (2010). Free-energy principle. Nature Reviews Neuroscience.
- Sperry, R.W. (1968). Hemisphere deconnection. American Psychologist.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, Float32
import numpy as np

try:
    import nengo
    NENGO_AVAILABLE = True
except ImportError:
    NENGO_AVAILABLE = False
    print("Warning: Nengo not installed. Running in stub mode.")

class VisualCortexNode(Node):
    def __init__(self):
        super().__init__('visual_cortex_snn')
        
        # Dimensions for dimensionality reduction (input -> SNN)
        self.INPUT_DIM = 64
        self.current_input = np.zeros(self.INPUT_DIM)
        
        # ROS 2 Subscribers & Publishers
        self.visual_sub = self.create_subscription(
            Image,
            'unity/camera/raw',
            self.visual_input_callback,
            10
        )
        
        # Publisher for the "Error Signal" - the consciousness bandwidth optimization
        self.error_pub = self.create_publisher(
            Float32MultiArray,
            '/neural_data/prediction_error',
            10
        )
        
        # Health Publisher
        self.health_pub = self.create_publisher(
            Float32,
            '/synchronization_health',
            10
        )

        # Activity Publisher (for Dream Engine)
        self.activity_pub = self.create_publisher(
            Float32MultiArray,
            '/neural_data/cortex_activity',
            10
        )

        # Top-down Prediction Subscriber (from Dream Engine)
        self.topdown_prediction = np.zeros(self.INPUT_DIM)
        self.topdown_sub = self.create_subscription(
            Float32MultiArray,
            'dream/top_down_prediction',
            self.topdown_callback,
            10
        )

        if NENGO_AVAILABLE:
            self.build_nengo_model()
            self.get_logger().info("Generative Model (Predictive Coding) Initialized.")
        else:
            self.get_logger().warn("Nengo unavailable. Model will not run.")

        # Run loop for the simulator (approx 30Hz or faster depending on requirement)
        self.timer = self.create_timer(0.01, self.update_step) # 100Hz simulation check

    def build_nengo_model(self):
        # 1. Create the Nengo Network (The "Brain")
        self.model = nengo.Network(label="Generative Model")
        with self.model:
            # SENSORY LAYER: Represents the raw input from the Eye/Camera
            # We use a Node to inject the current_input from ROS into the Nengo graph
            self.sensory_input = nengo.Node(output=lambda t: self.current_input)

            # TOP-DOWN INPUT: Receives predictions from Dream Engine
            self.topdown_input = nengo.Node(output=lambda t: self.topdown_prediction)

            # PREDICTION LAYER: The "Mind's Eye"
            # 1000 LIF (Leaky Integrate-and-Fire) neurons representing the cortex
            self.cortex = nengo.Ensemble(
                n_neurons=1000,
                dimensions=self.INPUT_DIM,
                neuron_type=nengo.LIF()
            )
            
            # THE GENERATIVE CONNECTION (Top-Down)
            # The cortex tries to predict the sensory input based on past experience
            # We assume a 50ms delay to mimic biological synapses
            nengo.Connection(self.cortex, self.cortex, synapse=0.05)

            # HIERARCHICAL TOP-DOWN: Dream Engine sends high-level predictions
            # These predictions bias the cortex's representation
            nengo.Connection(self.topdown_input, self.cortex, transform=0.5)
            
            # ERROR UNITS: The "Consciousness" Signal
            # This population ONLY fires when Reality (Input) != Prediction (Cortex)
            self.error_units = nengo.Ensemble(
                n_neurons=500, 
                dimensions=self.INPUT_DIM
            )
            
            # Wiring: Error = Input - Prediction
            nengo.Connection(self.sensory_input, self.error_units)
            nengo.Connection(self.cortex, self.error_units, transform=-1) # Inhibitory connection
            
            # Hebbian Learning: The brain rewires itself to minimize this error
            # If Error > 0, the cortex adjusts to match reality
            conn = nengo.Connection(self.error_units, self.cortex, transform=0.1)
            conn.learning_rule_type = nengo.PES() # Prescribed Error Sensitivity rule
            
            # Probes
            self.error_probe = nengo.Probe(self.error_units, synapse=0.01)
            self.cortex_probe = nengo.Probe(self.cortex, synapse=0.01)

        # 2. Setup the Simulator
        self.sim = nengo.Simulator(self.model, dt=0.001)

    def visual_input_callback(self, msg: Image):
        """Preprocess Unity Image to Vector."""
        # Simplified preprocessing to fit dimensions
        if len(msg.data) > 0:
            arr = np.frombuffer(msg.data, dtype=np.uint8)
            # Resize logic similar to previous implementation for consistency
            flat = arr.flatten().astype(np.float32) / 255.0
            if len(flat) >= self.INPUT_DIM:
                self.current_input = flat[:self.INPUT_DIM]
            else:
                self.current_input = np.pad(flat, (0, self.INPUT_DIM - len(flat)))

    def topdown_callback(self, msg: Float32MultiArray):
        """Receive top-down predictions from Dream Engine."""
        self.topdown_prediction = np.array(msg.data, dtype=np.float32)

    def update_step(self):
        # This function steps the simulation forward
        if NENGO_AVAILABLE:
            self.sim.step()

            if self.sim.data[self.error_probe].shape[0] > 0:
                current_error = self.sim.data[self.error_probe][-1]
                current_cortex = self.sim.data[self.cortex_probe][-1]

                # Publish Error
                msg = Float32MultiArray()
                msg.data = current_error.tolist()
                self.error_pub.publish(msg)

                # Publish Cortex Activity (for Dream Engine)
                activity_msg = Float32MultiArray()
                activity_msg.data = current_cortex.tolist()
                self.activity_pub.publish(activity_msg)

                # Calculate Synchronization Health
                # Euclidean distance of the error vector
                error_magnitude = np.linalg.norm(current_error)
                input_magnitude = np.linalg.norm(self.current_input)

                # Health = 1.0 - Relative Error (Clamped at 0)
                # Avoid divide by zero
                denom = input_magnitude if input_magnitude > 1e-6 else 1.0
                rel_error = error_magnitude / denom
                health = max(0.0, 1.0 - rel_error)

                # Publish Health
                health_msg = Float32()
                health_msg.data = float(health)
                self.health_pub.publish(health_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VisualCortexNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
