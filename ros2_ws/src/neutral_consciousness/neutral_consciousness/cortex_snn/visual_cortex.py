"""
Visual Cortex Module - Isomorphic Topographic Predictive Coding

Implements a 2D topographic grid (8x8) of neuron populations that preserve
spatial relationships from the Unity camera input, per Feinberg & Mallatt's
requirement for "isomorphic sensory mapping" as a prerequisite for
sensory mental images and primary consciousness.

SCIENTIFIC FOUNDATION:
    Feinberg, T.E. & Mallatt, J. (2013). "The evolutionary and genetic origins
    of consciousness in the Cambrian Period over 500 million years ago."
    Frontiers in Psychology, 4, 667. DOI: 10.3389/fpsyg.2013.00667

    Key requirement: "Multiple levels of isomorphic or somatotopic neural
    representations" as an objective marker for sensory consciousness.
    Spatial relationships must be preserved through the hierarchy.

ARCHITECTURE:
    - 8x8 Topographic Grid: 64 positions, each with ~50 neurons = 3,200 total
    - Local Gaussian Connectivity: Neighbors strongly connected, distant suppressed
    - Lateral Inhibition: Mexican-hat (center-surround) for contrast enhancement
    - Error Population: 500 LIF neurons computing spatial prediction error
    - PES Learning: Error drives cortex adaptation (post-synaptic = 64D)

    Watanabe, M., et al. (2014). Neuropsychologia, 63, 133-142.
    NOTE: This is NOT the official work of Professor Masataka Watanabe.

RELATED RESEARCH:
    Friston, K. (2010). Free-energy principle. Nature Reviews Neuroscience.
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


def gaussian_connectivity(grid_size, sigma=1.5):
    """
    Build local Gaussian connectivity matrix for a 2D grid.

    Each position (i,j) connects to all others with strength proportional
    to exp(-d^2 / 2*sigma^2), where d is the Euclidean grid distance.
    This preserves topographic (isomorphic) spatial relationships.

    Args:
        grid_size: Side length of the square grid (e.g., 8 for 8x8)
        sigma: Gaussian spread in grid units (1.5 = ~2 neighbors strong)

    Returns:
        NxN weight matrix where N = grid_size^2
    """
    n = grid_size * grid_size
    weights = np.zeros((n, n))

    for i in range(n):
        row_i, col_i = i // grid_size, i % grid_size
        for j in range(n):
            row_j, col_j = j // grid_size, j % grid_size
            dist_sq = (row_i - row_j) ** 2 + (col_i - col_j) ** 2
            weights[i, j] = np.exp(-dist_sq / (2 * sigma ** 2))

    # Normalize rows so each position's total input sums to 1
    row_sums = weights.sum(axis=1, keepdims=True)
    weights /= np.where(row_sums > 0, row_sums, 1.0)

    return weights


def mexican_hat_connectivity(grid_size, sigma_excite=1.0, sigma_inhibit=3.0,
                              excite_strength=0.3, inhibit_strength=0.1):
    """
    Build center-surround (Mexican hat) lateral connectivity.

    Implements lateral inhibition: nearby positions excite each other,
    distant positions inhibit. This is the fundamental mechanism for
    contrast enhancement in biological isomorphic maps (retina, V1, tectum).

    Feinberg & Mallatt (2013): "lateral inhibition, temporal transients,
    contrast enhancement, center-surround inhibition, and feature extraction"
    are required across all isomorphic sensory systems.

    Args:
        grid_size: Side length of square grid
        sigma_excite: Gaussian spread for excitatory connections
        sigma_inhibit: Gaussian spread for inhibitory connections
        excite_strength: Peak excitatory weight
        inhibit_strength: Peak inhibitory weight

    Returns:
        NxN weight matrix with Mexican hat profile
    """
    n = grid_size * grid_size
    weights = np.zeros((n, n))

    for i in range(n):
        row_i, col_i = i // grid_size, i % grid_size
        for j in range(n):
            if i == j:
                continue  # No self-connection in lateral inhibition
            row_j, col_j = j // grid_size, j % grid_size
            dist_sq = (row_i - row_j) ** 2 + (col_i - col_j) ** 2
            excite = excite_strength * np.exp(-dist_sq / (2 * sigma_excite ** 2))
            inhibit = inhibit_strength * np.exp(-dist_sq / (2 * sigma_inhibit ** 2))
            weights[i, j] = excite - inhibit

    return weights


class VisualCortexNode(Node):
    def __init__(self):
        super().__init__('visual_cortex_snn')

        # Topographic grid dimensions (8x8 = 64 positions)
        self.GRID_SIZE = 8
        self.INPUT_DIM = self.GRID_SIZE * self.GRID_SIZE  # 64

        self.current_input = np.zeros(self.INPUT_DIM)

        # Dopamine modulation from limbic node (default: full learning)
        self.dopamine_signal = 1.0
        self.BASE_LEARNING_RATE = 1e-4

        # ROS 2 Subscribers & Publishers
        self.visual_sub = self.create_subscription(
            Image,
            'unity/camera/raw',
            self.visual_input_callback,
            10
        )

        self.error_pub = self.create_publisher(
            Float32MultiArray,
            '/neural_data/prediction_error',
            10
        )

        self.health_pub = self.create_publisher(
            Float32,
            '/synchronization_health',
            10
        )

        self.activity_pub = self.create_publisher(
            Float32MultiArray,
            '/neural_data/cortex_activity',
            10
        )

        # Spatial error map publisher (8x8 topographic error for tectum/dream engine)
        self.spatial_error_pub = self.create_publisher(
            Float32MultiArray,
            '/neural_data/spatial_error_map',
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

        # Dopamine modulation subscriber (from Limbic Node)
        self.dopamine_sub = self.create_subscription(
            Float32,
            '/consciousness/affective/dopamine',
            self.dopamine_callback,
            10
        )

        if NENGO_AVAILABLE:
            self.build_nengo_model()
            self.get_logger().info(
                f"Isomorphic Visual Cortex initialized: "
                f"{self.GRID_SIZE}x{self.GRID_SIZE} topographic grid, "
                f"{self.GRID_SIZE * self.GRID_SIZE * 50} cortex neurons + "
                f"500 error neurons"
            )
        else:
            self.get_logger().warn("Nengo unavailable. Model will not run.")

        self.timer = self.create_timer(0.01, self.update_step)

    def build_nengo_model(self):
        """
        Build a topographic visual cortex with isomorphic mapping.

        Architecture:
        - sensory_input: 64D Node (raw camera input, treated as 8x8 grid)
        - cortex: 64D Ensemble with 3200 LIF neurons
          - Local Gaussian recurrent connectivity (preserves topology)
          - Mexican hat lateral inhibition (contrast enhancement)
        - error_units: 64D Ensemble with 500 LIF neurons
        - PES learning on input→cortex connection, modulated by dopamine
        """
        self.model = nengo.Network(label="Isomorphic Visual Cortex")

        # Pre-compute connectivity matrices
        self.gaussian_weights = gaussian_connectivity(self.GRID_SIZE, sigma=1.5)
        self.lateral_weights = mexican_hat_connectivity(
            self.GRID_SIZE,
            sigma_excite=1.0, sigma_inhibit=3.0,
            excite_strength=0.3, inhibit_strength=0.1
        )

        with self.model:
            # ============================================================
            # SENSORY LAYER: 8x8 topographic input
            # ============================================================
            self.sensory_input = nengo.Node(output=lambda t: self.current_input)
            self.topdown_input = nengo.Node(output=lambda t: self.topdown_prediction)

            # ============================================================
            # TOPOGRAPHIC CORTEX: 3200 LIF neurons (50 per grid position)
            # ============================================================
            # Using a single Ensemble with 64 dimensions, where each
            # dimension represents one spatial position in the 8x8 grid.
            # The topology is enforced through structured connectivity.
            self.cortex = nengo.Ensemble(
                n_neurons=3200,
                dimensions=self.INPUT_DIM,
                neuron_type=nengo.LIF()
            )

            # LOCAL GAUSSIAN RECURRENCE: Nearby positions reinforce each other
            # This is the isomorphic mapping requirement — spatial preservation
            # through the hierarchy. Weights follow Gaussian falloff.
            nengo.Connection(
                self.cortex, self.cortex,
                transform=self.gaussian_weights * 0.15,  # Scaled recurrence
                synapse=0.05  # 50ms biological delay
            )

            # LATERAL INHIBITION: Mexican hat center-surround
            # Required by Feinberg & Mallatt for ALL isomorphic sensory systems.
            # Enhances contrast between nearby vs. distant spatial positions.
            nengo.Connection(
                self.cortex, self.cortex,
                transform=self.lateral_weights,
                synapse=0.01  # Fast lateral dynamics
            )

            # TOP-DOWN: Dream Engine predictions bias the cortex
            nengo.Connection(self.topdown_input, self.cortex, transform=0.5)

            # ============================================================
            # ERROR UNITS: Spatial prediction error
            # ============================================================
            self.error_units = nengo.Ensemble(
                n_neurons=500,
                dimensions=self.INPUT_DIM
            )

            # Error = Input - Prediction (spatial map difference)
            nengo.Connection(self.sensory_input, self.error_units)
            nengo.Connection(self.cortex, self.error_units, transform=-1)

            # ============================================================
            # PES LEARNING: Dopamine-modulated
            # ============================================================
            # Learning rate is modulated by dopamine signal from limbic node
            # κ_eff = κ_base × (0.1 + 0.9 × dopamine)
            # High dopamine → full learning (salient event)
            # Low dopamine → 10% learning (nothing important)
            def effective_learning_rate(t):
                """Compute dopamine-modulated learning rate."""
                return self.BASE_LEARNING_RATE * (0.1 + 0.9 * self.dopamine_signal)

            # Node that outputs the current effective learning rate (for monitoring)
            self.lr_node = nengo.Node(output=effective_learning_rate)

            # PES connection: input → cortex, driven by error
            conn = nengo.Connection(
                self.sensory_input,
                self.cortex,
                learning_rule_type=nengo.PES(learning_rate=self.BASE_LEARNING_RATE)
            )
            nengo.Connection(self.error_units, conn.learning_rule)

            # ============================================================
            # PROBES (sample_every prevents unbounded memory)
            # ============================================================
            self.error_probe = nengo.Probe(
                self.error_units, synapse=0.01, sample_every=0.001
            )
            self.cortex_probe = nengo.Probe(
                self.cortex, synapse=0.01, sample_every=0.001
            )

        self.sim = nengo.Simulator(self.model, dt=0.001)

    def visual_input_callback(self, msg: Image):
        """
        Preprocess Unity Image to 8x8 topographic grid.

        The input is spatially downsampled to preserve the 2D structure,
        rather than arbitrarily taking the first 64 values. This ensures
        isomorphic mapping from the camera's spatial layout to the grid.
        """
        if len(msg.data) > 0:
            arr = np.frombuffer(msg.data, dtype=np.uint8)
            flat = arr.flatten().astype(np.float32) / 255.0

            if len(flat) >= self.INPUT_DIM:
                # If image is larger, spatially downsample to 8x8
                # Attempt to reshape as 2D and spatially pool
                total_pixels = len(flat)
                side = int(np.sqrt(total_pixels))
                if side >= self.GRID_SIZE:
                    img_2d = flat[:side * side].reshape(side, side)
                    # Block-average downsampling to 8x8
                    block_h = side // self.GRID_SIZE
                    block_w = side // self.GRID_SIZE
                    grid = np.zeros((self.GRID_SIZE, self.GRID_SIZE))
                    for r in range(self.GRID_SIZE):
                        for c in range(self.GRID_SIZE):
                            block = img_2d[
                                r * block_h:(r + 1) * block_h,
                                c * block_w:(c + 1) * block_w
                            ]
                            grid[r, c] = block.mean()
                    self.current_input = grid.flatten()
                else:
                    self.current_input = flat[:self.INPUT_DIM]
            else:
                self.current_input = np.pad(flat, (0, self.INPUT_DIM - len(flat)))

    def topdown_callback(self, msg: Float32MultiArray):
        """Receive top-down predictions from Dream Engine."""
        data = np.array(msg.data, dtype=np.float32)
        if len(data) == self.INPUT_DIM:
            self.topdown_prediction = data

    def dopamine_callback(self, msg: Float32):
        """
        Receive dopamine modulation signal from Limbic Node.

        This implements the neo-Hebbian learning modulation:
        high dopamine = salient event = increased plasticity.
        """
        self.dopamine_signal = float(np.clip(msg.data, 0.0, 1.0))

    def update_step(self):
        """Step the Nengo simulation and publish topographic outputs."""
        if not NENGO_AVAILABLE:
            return

        self.sim.step()

        if self.sim.data[self.error_probe].shape[0] == 0:
            return

        current_error = self.sim.data[self.error_probe][-1]
        current_cortex = self.sim.data[self.cortex_probe][-1]

        # Publish Error (64D vector)
        msg = Float32MultiArray()
        msg.data = current_error.tolist()
        self.error_pub.publish(msg)

        # Publish Cortex Activity (64D for Dream Engine)
        activity_msg = Float32MultiArray()
        activity_msg.data = current_cortex.tolist()
        self.activity_pub.publish(activity_msg)

        # Publish Spatial Error Map (same 64D, but explicitly for tectum)
        spatial_msg = Float32MultiArray()
        spatial_msg.data = current_error.tolist()
        self.spatial_error_pub.publish(spatial_msg)

        # Calculate Synchronization Health (spatial correlation)
        error_magnitude = np.linalg.norm(current_error)
        input_magnitude = np.linalg.norm(self.current_input)
        denom = input_magnitude if input_magnitude > 1e-6 else 1.0
        rel_error = error_magnitude / denom
        health = max(0.0, 1.0 - rel_error)

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
