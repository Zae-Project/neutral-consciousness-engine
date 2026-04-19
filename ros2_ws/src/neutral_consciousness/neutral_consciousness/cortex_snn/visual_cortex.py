"""
Visual Cortex Module - True Predictive Coding Implementation

This module uses Nengo to implement a "Generative Model" where the network
predicts input and only fires on error. This effectively implements the
Watanabe "Consciousness" logic layout.

SCIENTIFIC VALIDATION:
- Architecture: 1000 LIF Neurons (Cortex) + 500 LIF Neurons (Error)
- Logic: Predictive Coding (Error = Input - Prediction)
- Learning: PES (Prescribed Error Sensitivity) Rule
- Metrics: Euclidean Distance, Synchronization Health, Phase-Locking Value

TRANSMISSIVE LAYER:
- Subscribes to firewall-cleared `/environment/em_field` (not the raw topic).
- Feeds the EM sample through `ephaptic_coupling.build_ephaptic_field`, which
  sums a low-passed self-field with the exogenous EM drive and broadcasts the
  scalar as a uniform additive bias to the cortex ensemble. The EM signal is
  NEVER fed directly into the cortex — doing so would trivialise PLV to 1.0
  and turn the second convergence criterion into a rubber stamp.
- Publishes the rolling θ-band PLV (cortex ↔ EM reference) on
  `/transmissive_sync/plv` at ~10 Hz.
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

from .ephaptic_coupling import EphapticState, build_ephaptic_field
from .plv_estimator import PLVEstimator


class VisualCortexNode(Node):
    def __init__(self):
        super().__init__('visual_cortex_snn')

        # Dimensions for dimensionality reduction (input -> SNN)
        self.INPUT_DIM = 64
        self.current_input = np.zeros(self.INPUT_DIM)

        # Transmissive-layer state. `eph_state.env_value` is updated from the
        # firewall-cleared EM topic; `build_ephaptic_field` reads it each step.
        self.eph_state = EphapticState()
        self._latest_em_sample = 0.0

        # PLV sampled at ~100 Hz over a 4-second rolling buffer (θ band).
        self.plv = PLVEstimator(sample_rate_hz=100.0, window_sec=4.0,
                                band_hz=(4.0, 7.0))
        self._plv_publish_counter = 0

        # ROS 2 Subscribers & Publishers
        self.visual_sub = self.create_subscription(
            Image,
            'unity/camera/raw',
            self.visual_input_callback,
            10
        )

        # Firewall-cleared ambient EM channel. The raw topic
        # `/environment/em_field_raw` is intentionally NOT subscribed to here —
        # it goes through the Neural Firewall first.
        self.em_sub = self.create_subscription(
            Float32MultiArray,
            '/environment/em_field',
            self.em_field_callback,
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

        # Phase-Locking Value publisher (θ-band cortex ↔ EM coherence).
        self.plv_pub = self.create_publisher(
            Float32,
            '/transmissive_sync/plv',
            10
        )

        if NENGO_AVAILABLE:
            self.build_nengo_model()
            self.get_logger().info(
                "Generative Model (Predictive Coding + Ephaptic Field) Initialized."
            )
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

            # TRANSMISSIVE LAYER: ephaptic-style scalar field. The env_node
            # reads `self.eph_state.env_value` (kept fresh by the ROS callback);
            # the field sums a self-probe of the cortex rate with the env drive
            # and broadcasts it uniformly back to the cortex.
            self.ephaptic_field_node = build_ephaptic_field(
                self.cortex, self.INPUT_DIM, self.eph_state
            )

            # Probes
            self.error_probe = nengo.Probe(self.error_units, synapse=0.01)
            # Probe the cortex's decoded representation so we can collapse it
            # into a scalar rate proxy for the PLV estimator.
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

    def em_field_callback(self, msg: Float32MultiArray):
        """Latest firewall-cleared EM sample. Single-channel Float32MultiArray."""
        if msg.data:
            sample = float(msg.data[0])
            self._latest_em_sample = sample
            self.eph_state.env_value = sample

    def update_step(self):
        # This function steps the simulation forward
        if NENGO_AVAILABLE:
            self.sim.step()

            if self.sim.data[self.error_probe].shape[0] > 0:
                current_error = self.sim.data[self.error_probe][-1]

                # Publish Error
                msg = Float32MultiArray()
                msg.data = current_error.tolist()
                self.error_pub.publish(msg)

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

                # --- PLV bookkeeping ---
                # Collapse the cortex's decoded representation to a scalar by
                # taking the mean of the latest decoded vector. This is the
                # population-rate proxy that we phase-compare against the EM
                # reference.
                cortex_vec = self.sim.data[self.cortex_probe][-1]
                cortex_scalar = float(np.mean(cortex_vec))
                self.plv.push(cortex_scalar, self._latest_em_sample)

                # Publish PLV at ~10 Hz (the update loop runs at 100 Hz).
                self._plv_publish_counter += 1
                if self._plv_publish_counter >= 10:
                    self._plv_publish_counter = 0
                    plv_msg = Float32()
                    plv_msg.data = float(self.plv.compute())
                    self.plv_pub.publish(plv_msg)


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
