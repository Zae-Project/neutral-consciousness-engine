"""
Dream Engine Module - Generative Predictive Model

Implements the Generative Model logic inspired by:
- Watanabe's "neutral consciousness" framework (2021)
- Friston's Free Energy Principle (2010)

NOTE: This implementation is inspired by Professor Masataka Watanabe's research 
approach. This is NOT his official work. We are independent researchers exploring
the "consciousness generates reality" hypothesis.

This module generates internal predictions of sensory experiences,
comparing them against actual sensory input to minimize prediction error.
The goal is to create a substrate for consciousness that operates via 
continuous prediction rather than reactive processing.

RELATED RESEARCH:
- Friston, K. (2010). Free-energy principle. Nature Reviews Neuroscience.
- Watanabe, M. (2021). Neutral consciousness framework.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool
import numpy as np

try:
    import nengo
    NENGO_AVAILABLE = True
except ImportError:
    NENGO_AVAILABLE = False


class DreamEngine(Node):
    """
    ROS 2 Node implementing the Generative Model.
    
    The Dream Engine continuously generates predictions about
    incoming sensory data, updating its internal model based
    on prediction errors.
    """
    
    def __init__(self):
        super().__init__('dream_engine')
        
        # Configuration
        self.declare_parameter('prediction_rate_hz', 30.0)
        self.declare_parameter('dream_mode', False)
        
        self.prediction_rate = self.get_parameter('prediction_rate_hz').value
        self.dream_mode = self.get_parameter('dream_mode').value
        
        # Internal state
        self.current_prediction = np.zeros(64, dtype=np.float32)
        self.prediction_error = np.zeros(64, dtype=np.float32)
        self.internal_model = np.random.randn(64, 64).astype(np.float32) * 0.1
        
        # Subscribers
        self.sensory_input_sub = self.create_subscription(
            Float32MultiArray,
            'cortex/visual/activity',
            self.sensory_input_callback,
            10
        )
        
        self.dream_mode_sub = self.create_subscription(
            Bool,
            'dream/enable',
            self.dream_mode_callback,
            10
        )
        
        # Publishers
        self.prediction_pub = self.create_publisher(
            Float32MultiArray,
            'dream/prediction',
            10
        )
        
        self.error_pub = self.create_publisher(
            Float32MultiArray,
            'dream/prediction_error',
            10
        )
        
        # Prediction timer
        self.prediction_timer = self.create_timer(
            1.0 / self.prediction_rate,
            self.generate_prediction
        )
        
        self.get_logger().info('Dream Engine initialized - Generative Model active')
    
    def sensory_input_callback(self, msg: Float32MultiArray):
        """
        Receive actual sensory data and compute prediction error.
        
        Args:
            msg: Neural activity from the visual cortex
        """
        actual = np.array(msg.data, dtype=np.float32)
        
        # Compute prediction error (surprise signal)
        self.prediction_error = actual - self.current_prediction
        
        # Update internal model using prediction error
        self._update_model()
        
        # Publish prediction error
        error_msg = Float32MultiArray()
        error_msg.data = self.prediction_error.tolist()
        self.error_pub.publish(error_msg)
    
    def generate_prediction(self):
        """
        Generate the next prediction based on the internal model.
        
        In dream mode, predictions are generated without sensory input,
        allowing the system to "imagine" or "dream" experiences.
        """
        if self.dream_mode:
            # Dream mode: Free-running generation
            noise = np.random.randn(64).astype(np.float32) * 0.1
            self.current_prediction = np.tanh(
                self.internal_model @ self.current_prediction + noise
            )
        else:
            # Awake mode: Prediction based on recent experience
            self.current_prediction = np.tanh(
                self.internal_model @ self.current_prediction
            )
        
        # Publish prediction
        pred_msg = Float32MultiArray()
        pred_msg.data = self.current_prediction.tolist()
        self.prediction_pub.publish(pred_msg)
    
    def _update_model(self):
        """
        Update the internal generative model using prediction error.
        
        Implements a simplified Hebbian learning rule:
        "Neurons that fire together, wire together"
        """
        learning_rate = 0.01
        
        # Outer product learning rule
        delta = learning_rate * np.outer(
            self.prediction_error,
            self.current_prediction
        )
        
        self.internal_model += delta
        
        # Normalize to prevent explosion
        norm = np.linalg.norm(self.internal_model)
        if norm > 10.0:
            self.internal_model /= (norm / 10.0)
    
    def dream_mode_callback(self, msg: Bool):
        """
        Toggle dream mode on/off.
        
        Args:
            msg: Boolean indicating whether to enable dream mode
        """
        self.dream_mode = msg.data
        mode_str = "DREAM" if self.dream_mode else "AWAKE"
        self.get_logger().info(f'Mode switched to: {mode_str}')


def main(args=None):
    """Main entry point for the Dream Engine node."""
    rclpy.init(args=args)
    node = DreamEngine()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
