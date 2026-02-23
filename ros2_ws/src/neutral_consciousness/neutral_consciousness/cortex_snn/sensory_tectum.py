"""
Sensory Tectum Module - Fast Multisensory Convergence Layer

Implements the optic tectum (superior colliculus), the ORIGINAL center of
multi-sensory conscious perception in early vertebrates. This is Step 1
of Feinberg & Mallatt's two-step hypothesis for consciousness evolution.

SCIENTIFIC FOUNDATION:
    Feinberg, T.E. & Mallatt, J. (2013). "The evolutionary and genetic origins
    of consciousness in the Cambrian Period over 500 million years ago."
    Frontiers in Psychology, 4, 667. DOI: 10.3389/fpsyg.2013.00667

    "We propose a two-step evolutionary history, in which the optic tectum
    was the original center of multi-sensory conscious perception (as in
    fish and amphibians: step 1)."

    The biological tectum has:
    - Visual, auditory, vestibular, somatosensory maps STACKED IN REGISTER
    - Ultra-fast processing (5-15ms latency)
    - Direct motor output for orientation/escape reflexes
    - Laminar organization for efficient cross-modal integration

ARCHITECTURE:
    - 2,000 LIF neurons with fast dynamics (tau_rc=5ms)
    - Receives: Visual map (8x8, 64D) from visual cortex
    - Receives: Proprioception (16D) from Unity joint states
    - Multisensory convergence: visual + proprioceptive maps combined
    - Outputs: Unified sensory map (64D) for pallium/dream engine
    - Outputs: Reflexive motor commands (8D) for fast responses
    - Outputs: Salience map (64D) for attention gating

RELATED RESEARCH:
    Rowland et al. (2007). Neural mechanisms underlying multisensory
    integration in the superior colliculus. Perception, 36, 1431-1443.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
import numpy as np

try:
    import nengo
    NENGO_AVAILABLE = True
except ImportError:
    NENGO_AVAILABLE = False
    print("Warning: Nengo not installed. Running in stub mode.")


class SensoryTectum(Node):
    """
    Fast multisensory convergence node implementing the optic tectum.

    The tectum is the evolutionary precursor to cortical consciousness.
    It creates a unified spatial map from multiple sensory modalities,
    enabling fast reflexive responses and providing the bottom-up
    foundation for higher pallial (dream engine) processing.
    """

    def __init__(self):
        super().__init__('sensory_tectum')

        # Dimensions
        self.VISUAL_DIM = 64    # 8x8 topographic visual map
        self.PROPRIO_DIM = 16   # Proprioceptive state (joint angles, velocities)
        self.MOTOR_DIM = 8      # Reflexive motor commands
        self.GRID_SIZE = 8

        # State buffers (injected into Nengo via Nodes)
        self.visual_map = np.zeros(self.VISUAL_DIM, dtype=np.float32)
        self.proprioception = np.zeros(self.PROPRIO_DIM, dtype=np.float32)
        self.spatial_error = np.zeros(self.VISUAL_DIM, dtype=np.float32)

        # Salience threshold for reflexive response
        self.SALIENCE_THRESHOLD = 0.3

        # ============================================================
        # ROS2 SUBSCRIBERS
        # ============================================================

        # Visual cortex activity (8x8 topographic map)
        self.visual_sub = self.create_subscription(
            Float32MultiArray,
            '/neural_data/cortex_activity',
            self.visual_callback,
            10
        )

        # Spatial error from visual cortex (for salience detection)
        self.spatial_error_sub = self.create_subscription(
            Float32MultiArray,
            '/neural_data/spatial_error_map',
            self.spatial_error_callback,
            10
        )

        # Proprioception from Unity digital twin
        self.proprio_sub = self.create_subscription(
            Float32MultiArray,
            '/unity/proprioception',
            self.proprio_callback,
            10
        )

        # ============================================================
        # ROS2 PUBLISHERS
        # ============================================================

        # Unified multisensory map (to dream engine / reentrant processor)
        self.unified_map_pub = self.create_publisher(
            Float32MultiArray,
            '/tectum/unified_map',
            10
        )

        # Salience map (spatial attention signal)
        self.salience_pub = self.create_publisher(
            Float32MultiArray,
            '/tectum/salience_map',
            10
        )

        # Reflexive motor commands (fast, bypasses pallium)
        self.reflex_pub = self.create_publisher(
            Float32MultiArray,
            '/tectum/reflex_motor',
            10
        )

        # Tectum activity for consciousness monitors
        self.activity_pub = self.create_publisher(
            Float32MultiArray,
            '/neural_data/tectum_activity',
            10
        )

        # ============================================================
        # BUILD NENGO MODEL
        # ============================================================

        if NENGO_AVAILABLE:
            self.build_nengo_model()
            self.sim = nengo.Simulator(self.model, dt=0.001)
            self.get_logger().info(
                "Sensory Tectum initialized: 2,000 LIF neurons, "
                "fast dynamics (5ms tau_rc), multisensory convergence"
            )
        else:
            self.get_logger().warn("Nengo unavailable. Tectum will not run.")

        # Fast update loop (100Hz to match tectum's low-latency requirement)
        self.timer = self.create_timer(0.01, self.update_step)

    def build_nengo_model(self):
        """
        Build the tectal multisensory convergence network.

        Architecture:
        - Visual map layer: 800 LIF, receives 64D topographic input
        - Proprioceptive layer: 200 LIF, receives 16D body state
        - Convergence layer: 700 LIF, combines both modalities in register
        - Salience detector: 200 LIF, computes spatial attention from error
        - Motor output: 100 LIF, generates reflexive commands

        Total: 2,000 LIF neurons
        All use fast dynamics (tau_rc=5ms) for minimal latency.
        """
        self.model = nengo.Network(label="Sensory Tectum")

        with self.model:
            # ============================================================
            # INPUT NODES (ROS2 → Nengo)
            # ============================================================
            visual_input = nengo.Node(output=lambda t: self.visual_map)
            proprio_input = nengo.Node(output=lambda t: self.proprioception)
            error_input = nengo.Node(output=lambda t: self.spatial_error)

            # ============================================================
            # VISUAL MAP LAYER: Fast isomorphic processing
            # ============================================================
            # 800 neurons processing 64D visual map with fast dynamics
            # The tectum preserves the retinotopic map from V1
            self.visual_layer = nengo.Ensemble(
                n_neurons=800,
                dimensions=self.VISUAL_DIM,
                neuron_type=nengo.LIF(tau_rc=0.005),  # 5ms — FAST
                label="Tectal Visual Map"
            )
            nengo.Connection(visual_input, self.visual_layer, synapse=0.005)

            # ============================================================
            # PROPRIOCEPTIVE LAYER: Body state representation
            # ============================================================
            # Somatotopic map of the body's joint states
            self.proprio_layer = nengo.Ensemble(
                n_neurons=200,
                dimensions=self.PROPRIO_DIM,
                neuron_type=nengo.LIF(tau_rc=0.005),  # 5ms — FAST
                label="Tectal Proprioceptive Map"
            )
            nengo.Connection(proprio_input, self.proprio_layer, synapse=0.005)

            # ============================================================
            # MULTISENSORY CONVERGENCE: Maps stacked in register
            # ============================================================
            # This is the critical tectum function: combining multiple
            # sensory modalities into a unified spatial representation.
            # The convergence layer receives visual (64D) + a projection
            # of proprioception (16D → 64D) so they align spatially.
            self.convergence = nengo.Ensemble(
                n_neurons=700,
                dimensions=self.VISUAL_DIM,
                neuron_type=nengo.LIF(tau_rc=0.005),  # 5ms — FAST
                label="Multisensory Convergence"
            )

            # Visual → Convergence (direct, identity-like transform)
            nengo.Connection(
                self.visual_layer, self.convergence,
                transform=0.7,  # Visual dominance (primary modality)
                synapse=0.005
            )

            # Proprioception → Convergence (projected to 64D visual space)
            # The 16D proprioceptive state is mapped onto the 64D spatial
            # grid via a learned/fixed projection. This is how the tectum
            # aligns body-state information with visual spatial coordinates.
            proprio_to_visual = np.random.randn(self.VISUAL_DIM, self.PROPRIO_DIM) * 0.1
            nengo.Connection(
                self.proprio_layer, self.convergence,
                transform=proprio_to_visual * 0.3,  # Weaker modality
                synapse=0.005
            )

            # Short recurrence for temporal smoothing
            nengo.Connection(
                self.convergence, self.convergence,
                transform=0.3,
                synapse=0.01  # 10ms integration
            )

            # ============================================================
            # SALIENCE DETECTOR: Attention from prediction error
            # ============================================================
            # The tectum detects "what's surprising" in the visual field
            # and directs attention there. Salience = |spatial error|.
            # This implements the reticular formation's attention function.
            self.salience = nengo.Ensemble(
                n_neurons=200,
                dimensions=self.VISUAL_DIM,
                neuron_type=nengo.LIF(tau_rc=0.005),
                intercepts=nengo.dists.Uniform(0.1, 0.7),  # Selective
                label="Salience Detector"
            )

            # Salience driven by absolute spatial prediction error
            # Using a function that computes element-wise absolute value
            nengo.Connection(
                error_input, self.salience,
                synapse=0.005
            )

            # Salience modulates the convergence layer (attention gating)
            nengo.Connection(
                self.salience, self.convergence,
                transform=0.2,
                synapse=0.005
            )

            # ============================================================
            # REFLEXIVE MOTOR OUTPUT: Fast escape/orient
            # ============================================================
            # Direct tectum → motor pathway (bypasses pallium for speed)
            # 8D motor output: [left, right, up, down, forward, back, rotate_l, rotate_r]
            self.motor = nengo.Ensemble(
                n_neurons=100,
                dimensions=self.MOTOR_DIM,
                neuron_type=nengo.LIF(tau_rc=0.005),
                label="Reflexive Motor"
            )

            # Salience-to-motor mapping: move TOWARD or AWAY from salient locations
            # This is a simple population-coded spatial-to-motor transform
            salience_to_motor = np.random.randn(self.MOTOR_DIM, self.VISUAL_DIM) * 0.05
            nengo.Connection(
                self.salience, self.motor,
                transform=salience_to_motor,
                synapse=0.005
            )

            # ============================================================
            # OUTPUT NODES (Nengo → ROS2)
            # ============================================================
            unified_output = nengo.Node(
                input=self._publish_unified_map,
                size_in=self.VISUAL_DIM
            )
            nengo.Connection(self.convergence, unified_output, synapse=0.01)

            salience_output = nengo.Node(
                input=self._publish_salience,
                size_in=self.VISUAL_DIM
            )
            nengo.Connection(self.salience, salience_output, synapse=0.01)

            motor_output = nengo.Node(
                input=self._publish_reflex,
                size_in=self.MOTOR_DIM
            )
            nengo.Connection(self.motor, motor_output, synapse=0.01)

            # ============================================================
            # PROBES
            # ============================================================
            self.convergence_probe = nengo.Probe(
                self.convergence, synapse=0.01, sample_every=0.001
            )
            self.salience_probe = nengo.Probe(
                self.salience, synapse=0.01, sample_every=0.001
            )

    # ================================================================
    # ROS2 CALLBACKS
    # ================================================================

    def visual_callback(self, msg: Float32MultiArray):
        """Receive 8x8 topographic visual map from cortex."""
        data = np.array(msg.data, dtype=np.float32)
        if len(data) == self.VISUAL_DIM:
            self.visual_map = data

    def spatial_error_callback(self, msg: Float32MultiArray):
        """Receive spatial prediction error for salience computation."""
        data = np.array(msg.data, dtype=np.float32)
        if len(data) == self.VISUAL_DIM:
            self.spatial_error = np.abs(data)  # Salience = |error|

    def proprio_callback(self, msg: Float32MultiArray):
        """Receive proprioceptive state from Unity digital twin."""
        data = np.array(msg.data, dtype=np.float32)
        if len(data) == self.PROPRIO_DIM:
            self.proprioception = data
        elif len(data) > self.PROPRIO_DIM:
            self.proprioception = data[:self.PROPRIO_DIM]
        else:
            self.proprioception[:len(data)] = data

    # ================================================================
    # NENGO OUTPUT CALLBACKS
    # ================================================================

    def _publish_unified_map(self, t, x):
        """Publish unified multisensory map."""
        msg = Float32MultiArray()
        msg.data = x.tolist()
        self.unified_map_pub.publish(msg)

        # Also publish for consciousness monitors
        activity_msg = Float32MultiArray()
        activity_msg.data = x.tolist()
        self.activity_pub.publish(activity_msg)

    def _publish_salience(self, t, x):
        """Publish spatial salience/attention map."""
        msg = Float32MultiArray()
        msg.data = x.tolist()
        self.salience_pub.publish(msg)

    def _publish_reflex(self, t, x):
        """Publish reflexive motor commands."""
        msg = Float32MultiArray()
        msg.data = x.tolist()
        self.reflex_pub.publish(msg)

    # ================================================================
    # SIMULATION LOOP
    # ================================================================

    def update_step(self):
        """Step the tectum simulation forward."""
        if not NENGO_AVAILABLE or not hasattr(self, 'sim'):
            return

        self.sim.step()


def main(args=None):
    rclpy.init(args=args)
    node = SensoryTectum()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
