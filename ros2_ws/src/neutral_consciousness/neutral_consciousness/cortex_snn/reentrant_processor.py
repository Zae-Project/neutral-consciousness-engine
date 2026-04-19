"""
Reentrant Processor Module - Bidirectional Binding Loop

Implements the thalamocortical-like reentrant loop that binds bottom-up sensory
input (from tectum/cortex) with top-down predictions (from dream engine) into a
unified conscious representation. Only after convergence does the processor
broadcast the bound representation downstream.

SCIENTIFIC FOUNDATION:
    Feinberg, T.E. & Mallatt, J. (2013). "The evolutionary and genetic origins
    of consciousness in the Cambrian Period over 500 million years ago."
    Frontiers in Psychology, 4, 667. DOI: 10.3389/fpsyg.2013.00667

    "Complex bidirectional communication across levels is required to bind
    sensory information into a unified conscious experience."

    Feinberg (2013): The cerebellum, despite its enormous complexity, operates
    NONCONSCIOUSLY because it lacks extensive cross-communication with other
    brain regions. Consciousness requires the crosstalk.

    Key requirements for reentrant binding:
    1. Recurrent processing — higher levels send signals back to lower levels
    2. Oscillatory synchronization — coordinated gamma/beta rhythms
    3. Convergence before broadcast — internal agreement before motor output

ARCHITECTURE:
    - Bottom-up pathway: 1,000 LIF neurons receiving tectum unified map (64D)
    - Top-down pathway: 1,000 LIF neurons receiving dream engine prediction (64D)
    - Binding layer: 1,500 LIF neurons, bidirectional recurrence
    - Convergence detector: 500 LIF neurons monitoring error magnitude
    - Total: 4,000 LIF neurons

    The processor runs a tight loop:
    1. Receive bottom-up sensory map from tectum
    2. Receive top-down prediction from dream engine
    3. Compute binding error = |bottom_up - top_down|
    4. Loop recurrently until binding error < convergence threshold
    5. Only THEN broadcast the unified bound representation

    Convergence triggers gamma-band oscillatory synchronization,
    which the oscillatory monitor can detect as a consciousness signature.

RELATED RESEARCH:
    Edelman, G.M. (1989). Neural Darwinism and reentrant signaling.
    Lamme, V.A.F. (2006). Towards a true neural stance on consciousness.
    Tononi, G. & Edelman, G.M. (1998). Consciousness and complexity.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32, Bool
import numpy as np

try:
    import nengo
    NENGO_AVAILABLE = True
except ImportError:
    NENGO_AVAILABLE = False
    print("Warning: Nengo not installed. Running in stub mode.")


class ReentrantProcessor(Node):
    """
    Bidirectional binding loop implementing reentrant processing.

    This is the mechanism that creates unified conscious experience from
    distributed sensory and predictive representations. Analogous to the
    thalamocortical loop in biological brains.

    Key behavior:
    - Continuously receives bottom-up (tectum) and top-down (dream engine) signals
    - Computes binding via recurrent integration of both streams
    - Monitors convergence (binding error magnitude)
    - Broadcasts unified representation only after convergence
    - Publishes convergence state for consciousness monitoring
    """

    def __init__(self):
        super().__init__('reentrant_processor')

        # Dimensions
        self.SPATIAL_DIM = 64  # 8x8 topographic map
        self.GRID_SIZE = 8

        # Convergence parameters
        self.declare_parameter('convergence_threshold', 0.15)
        self.declare_parameter('max_iterations', 10)
        self.declare_parameter('binding_rate_hz', 100.0)

        self.convergence_threshold = self.get_parameter('convergence_threshold').value
        self.max_iterations = int(self.get_parameter('max_iterations').value)
        binding_rate = self.get_parameter('binding_rate_hz').value

        # State buffers
        self.bottom_up_map = np.zeros(self.SPATIAL_DIM, dtype=np.float32)
        self.top_down_prediction = np.zeros(self.SPATIAL_DIM, dtype=np.float32)
        self.dopamine_signal = 1.0

        # Convergence tracking
        self.binding_error = 1.0  # Start unconverged
        self.is_converged = False
        self.iterations_since_convergence = 0

        # ============================================================
        # ROS2 SUBSCRIBERS
        # ============================================================

        # Bottom-up: unified multisensory map from tectum
        self.tectum_sub = self.create_subscription(
            Float32MultiArray,
            '/tectum/unified_map',
            self.tectum_callback,
            10
        )

        # Top-down: prediction from dream engine (pallium)
        self.topdown_sub = self.create_subscription(
            Float32MultiArray,
            'dream/top_down_prediction',
            self.topdown_callback,
            10
        )

        # Dopamine modulation from limbic node
        self.dopamine_sub = self.create_subscription(
            Float32,
            '/consciousness/affective/dopamine',
            self.dopamine_callback,
            10
        )

        # ============================================================
        # ROS2 PUBLISHERS
        # ============================================================

        # Unified bound representation (only published after convergence)
        self.bound_repr_pub = self.create_publisher(
            Float32MultiArray,
            '/reentrant/bound_representation',
            10
        )

        # Binding error magnitude (for consciousness monitors)
        self.binding_error_pub = self.create_publisher(
            Float32,
            '/reentrant/binding_error',
            10
        )

        # Convergence state (boolean: has binding converged?)
        self.convergence_pub = self.create_publisher(
            Bool,
            '/reentrant/converged',
            10
        )

        # Binding error map (spatial, 64D — shows WHERE binding disagrees)
        self.error_map_pub = self.create_publisher(
            Float32MultiArray,
            '/reentrant/error_map',
            10
        )

        # Reentrant activity for consciousness monitors
        self.activity_pub = self.create_publisher(
            Float32MultiArray,
            '/neural_data/reentrant_activity',
            10
        )

        # ============================================================
        # BUILD NENGO MODEL
        # ============================================================

        if NENGO_AVAILABLE:
            self.build_nengo_model()
            self.sim = nengo.Simulator(self.model, dt=0.001)
            self.get_logger().info(
                "Reentrant Processor initialized: 4,000 LIF neurons, "
                f"convergence threshold={self.convergence_threshold}, "
                f"max iterations={self.max_iterations}"
            )
        else:
            self.get_logger().warn("Nengo unavailable. Reentrant processor will not run.")

        # Binding loop timer (fast: 100Hz default)
        self.timer = self.create_timer(1.0 / binding_rate, self.binding_step)

    def build_nengo_model(self):
        """
        Build the reentrant binding network.

        Architecture:
        - Bottom-up pathway: 1,000 LIF neurons, receives tectum map (64D)
        - Top-down pathway: 1,000 LIF neurons, receives dream prediction (64D)
        - Binding layer: 1,500 LIF neurons, integrates both streams recurrently
        - Convergence detector: 500 LIF neurons, computes binding error

        Total: 4,000 LIF neurons

        The binding layer has bidirectional connections to both pathways:
        - Bottom-up → Binding (sensory evidence)
        - Top-down → Binding (predictive context)
        - Binding → Bottom-up (top-down modulation of sensory processing)
        - Binding → Top-down (bottom-up correction of predictions)

        This creates the "reentrant loop" that Edelman (1989) and
        Feinberg & Mallatt (2013) identify as critical for consciousness.
        """
        self.model = nengo.Network(label="Reentrant Processor")

        with self.model:
            # ============================================================
            # INPUT NODES (ROS2 → Nengo)
            # ============================================================
            bottom_up_input = nengo.Node(output=lambda t: self.bottom_up_map)
            top_down_input = nengo.Node(output=lambda t: self.top_down_prediction)
            dopamine_input = nengo.Node(output=lambda t: [self.dopamine_signal])

            # ============================================================
            # BOTTOM-UP PATHWAY: Sensory evidence from tectum
            # ============================================================
            self.bottom_up = nengo.Ensemble(
                n_neurons=1000,
                dimensions=self.SPATIAL_DIM,
                neuron_type=nengo.LIF(tau_rc=0.01),  # 10ms — medium speed
                label="Bottom-Up Pathway"
            )
            nengo.Connection(bottom_up_input, self.bottom_up, synapse=0.005)

            # ============================================================
            # TOP-DOWN PATHWAY: Predictive context from dream engine
            # ============================================================
            self.top_down = nengo.Ensemble(
                n_neurons=1000,
                dimensions=self.SPATIAL_DIM,
                neuron_type=nengo.LIF(tau_rc=0.01),  # 10ms — medium speed
                label="Top-Down Pathway"
            )
            nengo.Connection(top_down_input, self.top_down, synapse=0.005)

            # ============================================================
            # BINDING LAYER: Integrates both streams
            # ============================================================
            # This is the core of reentrant processing. The binding layer
            # receives both bottom-up and top-down signals and integrates
            # them into a unified representation through recurrence.
            self.binding = nengo.Ensemble(
                n_neurons=1500,
                dimensions=self.SPATIAL_DIM,
                neuron_type=nengo.LIF(tau_rc=0.02),  # 20ms — integration timescale
                label="Binding Layer"
            )

            # Bottom-up → Binding (sensory evidence drives binding)
            nengo.Connection(
                self.bottom_up, self.binding,
                transform=0.5,
                synapse=0.01
            )

            # Top-down → Binding (predictive context modulates binding)
            nengo.Connection(
                self.top_down, self.binding,
                transform=0.5,
                synapse=0.01
            )

            # REENTRANT CONNECTIONS (bidirectional):
            # Binding → Bottom-up (top-down modulation of sensory processing)
            nengo.Connection(
                self.binding, self.bottom_up,
                transform=0.3,
                synapse=0.01
            )

            # Binding → Top-down (bottom-up correction of predictions)
            nengo.Connection(
                self.binding, self.top_down,
                transform=0.3,
                synapse=0.01
            )

            # Recurrent self-connection for temporal integration
            # This allows the binding layer to accumulate evidence over
            # multiple iterations, critical for convergence.
            nengo.Connection(
                self.binding, self.binding,
                transform=0.4,
                synapse=0.02  # 20ms integration window
            )

            # Dopamine modulation: scales binding strength
            # High dopamine (salient event) → stronger binding
            nengo.Connection(
                dopamine_input,
                self.binding,
                transform=np.ones((self.SPATIAL_DIM, 1)) * 0.1,
                synapse=0.01
            )

            # ============================================================
            # CONVERGENCE DETECTOR: Monitors binding error
            # ============================================================
            # Computes the difference between bottom-up and top-down
            # streams within the binding layer. When this difference
            # drops below threshold, binding has converged.
            self.convergence_detector = nengo.Ensemble(
                n_neurons=500,
                dimensions=self.SPATIAL_DIM,
                neuron_type=nengo.LIF(tau_rc=0.01),
                label="Convergence Detector"
            )

            # Error = Bottom-up - Top-down (within binding context)
            nengo.Connection(
                self.bottom_up, self.convergence_detector,
                transform=1.0,
                synapse=0.005
            )
            nengo.Connection(
                self.top_down, self.convergence_detector,
                transform=-1.0,
                synapse=0.005
            )

            # ============================================================
            # OUTPUT NODES (Nengo → ROS2)
            # ============================================================
            binding_output = nengo.Node(
                input=self._publish_binding,
                size_in=self.SPATIAL_DIM
            )
            nengo.Connection(self.binding, binding_output, synapse=0.01)

            error_output = nengo.Node(
                input=self._publish_error_map,
                size_in=self.SPATIAL_DIM
            )
            nengo.Connection(self.convergence_detector, error_output, synapse=0.01)

            # ============================================================
            # PROBES (sample_every prevents unbounded memory)
            # ============================================================
            self.binding_probe = nengo.Probe(
                self.binding, synapse=0.01, sample_every=0.001
            )
            self.convergence_probe = nengo.Probe(
                self.convergence_detector, synapse=0.01, sample_every=0.001
            )

    # ================================================================
    # ROS2 CALLBACKS
    # ================================================================

    def tectum_callback(self, msg: Float32MultiArray):
        """Receive unified multisensory map from tectum (bottom-up)."""
        data = np.array(msg.data, dtype=np.float32)
        if len(data) == self.SPATIAL_DIM:
            self.bottom_up_map = data

    def topdown_callback(self, msg: Float32MultiArray):
        """Receive top-down prediction from dream engine (pallium)."""
        data = np.array(msg.data, dtype=np.float32)
        if len(data) == self.SPATIAL_DIM:
            self.top_down_prediction = data

    def dopamine_callback(self, msg: Float32):
        """Receive dopamine modulation signal from limbic node."""
        self.dopamine_signal = float(np.clip(msg.data, 0.0, 1.0))

    # ================================================================
    # NENGO OUTPUT CALLBACKS
    # ================================================================

    def _publish_binding(self, t, x):
        """Publish bound representation and activity data."""
        # Always publish activity for consciousness monitors
        activity_msg = Float32MultiArray()
        activity_msg.data = x.tolist()
        self.activity_pub.publish(activity_msg)

    def _publish_error_map(self, t, x):
        """Publish spatial binding error map."""
        # Compute binding error magnitude
        self.binding_error = float(np.linalg.norm(x))

        # Publish error map (spatial: where does binding disagree?)
        map_msg = Float32MultiArray()
        map_msg.data = x.tolist()
        self.error_map_pub.publish(map_msg)

    # ================================================================
    # BINDING LOOP
    # ================================================================

    def binding_step(self):
        """
        Step the reentrant binding loop.

        Each step:
        1. Run Nengo simulation forward
        2. Check convergence (binding error < threshold)
        3. If converged: broadcast bound representation
        4. Publish convergence state for consciousness monitors
        """
        if not NENGO_AVAILABLE or not hasattr(self, 'sim'):
            return

        self.sim.step()

        # Check convergence
        if len(self.sim.data[self.binding_probe]) == 0:
            return

        current_binding = self.sim.data[self.binding_probe][-1]
        current_error = self.sim.data[self.convergence_probe][-1]

        # Compute normalized binding error
        binding_magnitude = np.linalg.norm(current_binding)
        error_magnitude = np.linalg.norm(current_error)
        denom = binding_magnitude if binding_magnitude > 1e-6 else 1.0
        normalized_error = error_magnitude / denom

        # Check convergence
        was_converged = self.is_converged
        self.is_converged = normalized_error < self.convergence_threshold

        if self.is_converged:
            self.iterations_since_convergence += 1

            # Broadcast bound representation (convergence achieved)
            bound_msg = Float32MultiArray()
            bound_msg.data = current_binding.tolist()
            self.bound_repr_pub.publish(bound_msg)

            # Log convergence transitions
            if not was_converged:
                self.get_logger().info(
                    f'Reentrant binding CONVERGED: error={normalized_error:.4f} '
                    f'(threshold={self.convergence_threshold})'
                )
        else:
            self.iterations_since_convergence = 0
            if was_converged:
                self.get_logger().info(
                    f'Reentrant binding LOST convergence: error={normalized_error:.4f}'
                )

        # Publish binding error magnitude
        error_msg = Float32()
        error_msg.data = float(normalized_error)
        self.binding_error_pub.publish(error_msg)

        # Publish convergence state
        conv_msg = Bool()
        conv_msg.data = self.is_converged
        self.convergence_pub.publish(conv_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ReentrantProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
