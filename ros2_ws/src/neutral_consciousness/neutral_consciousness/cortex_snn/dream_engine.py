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
from std_msgs.msg import Float32MultiArray, Bool, Float32, String
import numpy as np

try:
    import nengo
    NENGO_AVAILABLE = True
except ImportError:
    NENGO_AVAILABLE = False

# Import our consciousness monitoring modules
try:
    from .criticality_monitor import CriticalityMonitor
    from .consciousness_metrics import ConsciousnessMetrics
    CONSCIOUSNESS_MODULES_AVAILABLE = True
except ImportError:
    CONSCIOUSNESS_MODULES_AVAILABLE = False
    print("Warning: Consciousness monitoring modules not available")


class DreamEngine(Node):
    """
    ROS 2 Node implementing the Generative Model.
    
    The Dream Engine continuously generates predictions about
    incoming sensory data, updating its internal model based
    on prediction errors.
    """
    
    def __init__(self):
        super().__init__('dream_engine')

        # Dimensions
        self.SEMANTIC_DIM = 512  # High-dimensional semantic pointers
        self.VISUAL_DIM = 64     # Low-dimensional visual features

        # Configuration
        self.declare_parameter('prediction_rate_hz', 30.0)
        self.declare_parameter('dream_mode', False)
        self.declare_parameter('tau_slow', 100.0)  # Slow dynamics timescale (ms)

        self.prediction_rate = self.get_parameter('prediction_rate_hz').value

        # State (injected via Nengo Nodes)
        self.cortex_activity = np.zeros(self.VISUAL_DIM, dtype=np.float32)
        self.prediction_error_feedback = np.zeros(self.VISUAL_DIM, dtype=np.float32)

        # Build Nengo SNN model
        if NENGO_AVAILABLE:
            self.build_nengo_model()
            self.sim = nengo.Simulator(self.model, dt=0.001)
            self.get_logger().info("Dream Engine SPA initialized (10,000 neurons)")
        else:
            self.get_logger().warn("Nengo unavailable. Dream Engine will not run.")

        # Subscribers
        self.cortex_input_sub = self.create_subscription(
            Float32MultiArray,
            '/neural_data/cortex_activity',  # Updated from 'cortex/visual/activity'
            self.cortex_activity_callback,  # Renamed from sensory_input_callback
            10
        )

        self.error_feedback_sub = self.create_subscription(
            Float32MultiArray,
            '/neural_data/prediction_error',  # NEW: Receive error from visual cortex
            self.error_feedback_callback,
            10
        )

        self.dream_mode_sub = self.create_subscription(
            Bool,
            'dream/enable',
            self.dream_mode_callback,
            10
        )

        # Publishers
        self.topdown_pub = self.create_publisher(
            Float32MultiArray,
            'dream/top_down_prediction',  # NEW: Top-down predictions to cortex
            10
        )

        self.semantic_pub = self.create_publisher(
            Float32MultiArray,
            'dream/semantic_state',  # NEW: Semantic state monitoring
            10
        )

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

        # NEW: Consciousness Metrics Publishers
        self.criticality_pub = self.create_publisher(
            Float32,
            'consciousness/criticality/branching_ratio',
            10
        )

        self.criticality_state_pub = self.create_publisher(
            String,
            'consciousness/criticality/state',
            10
        )

        self.aci_pub = self.create_publisher(
            Float32,
            'consciousness/aci',
            10
        )

        self.consciousness_prob_pub = self.create_publisher(
            Float32,
            'consciousness/probability',
            10
        )

        # Initialize consciousness monitoring modules
        if CONSCIOUSNESS_MODULES_AVAILABLE:
            self.criticality_monitor = CriticalityMonitor(
                window_size=100,
                target_branching_ratio=1.0,
                tolerance=0.1
            )
            self.consciousness_metrics = ConsciousnessMetrics(
                history_length=50,
                n_bins=10
            )
            self.get_logger().info("Consciousness monitoring modules initialized")
        else:
            self.criticality_monitor = None
            self.consciousness_metrics = None
            self.get_logger().warn("Consciousness monitoring disabled")

        # Adaptive criticality tuning
        self.adaptive_tuning_enabled = True
        self.tau_slow_current = self.get_parameter('tau_slow').value

        # Prediction timer
        self.prediction_timer = self.create_timer(
            1.0 / self.prediction_rate,
            self.generate_prediction
        )
        
        self.get_logger().info('Dream Engine initialized - Generative Model active')

    def build_nengo_model(self):
        """
        Build the Semantic Pointer Architecture (SPA) network.

        Architecture:
        - Compression: 64D visual → 512D semantic (2000 neurons, fast)
        - Semantic State: 512D with slow dynamics (5000 neurons, tau_rc=100ms)
        - Clean-up Memory: Stabilizes semantic pointers (1000 neurons)
        - Decompression: 512D semantic → 64D visual prediction (2000 neurons, fast)
        Total: 10,000 neurons
        """
        self.model = nengo.Network(label="Dream Engine SPA")

        with self.model:
            # ============================================================
            # INPUT NODES (ROS2 → Nengo)
            # ============================================================

            cortex_input = nengo.Node(output=lambda t: self.cortex_activity)
            error_input = nengo.Node(output=lambda t: self.prediction_error_feedback)

            # ============================================================
            # COMPRESSION LAYER: 64D → 512D (Fast dynamics)
            # ============================================================

            self.encoder = nengo.Ensemble(
                n_neurons=2000,
                dimensions=self.VISUAL_DIM,
                neuron_type=nengo.LIF(tau_rc=0.02),  # Fast (20ms)
                label="Encoder"
            )
            nengo.Connection(cortex_input, self.encoder)

            # ============================================================
            # SEMANTIC STATE: 512D with SLOW dynamics
            # ============================================================

            tau_slow = self.get_parameter('tau_slow').value / 1000.0  # ms → seconds
            self.semantic_state = nengo.Ensemble(
                n_neurons=5000,
                dimensions=self.SEMANTIC_DIM,
                neuron_type=nengo.LIF(tau_rc=tau_slow),  # SLOW (100ms default)
                radius=1.5,  # Allow for combined semantic pointers
                label="Semantic State"
            )

            # Learned compression transform: 64D → 512D
            self.compression = nengo.Connection(
                self.encoder,
                self.semantic_state,
                transform=np.random.randn(self.SEMANTIC_DIM, self.VISUAL_DIM) * 0.1,
                learning_rule_type=nengo.PES(learning_rate=1e-4)
            )

            # Recurrent connection for temporal integration (narrative continuity)
            nengo.Connection(
                self.semantic_state,
                self.semantic_state,
                synapse=0.1,  # 100ms integration window
                transform=0.7  # Decay factor
            )

            # ============================================================
            # CLEAN-UP MEMORY: Stabilizes semantic pointers
            # ============================================================

            self.cleanup = nengo.Ensemble(
                n_neurons=1000,
                dimensions=self.SEMANTIC_DIM,
                neuron_type=nengo.LIF(tau_rc=0.02),  # Fast cleanup
                label="Clean-up Memory"
            )

            # Bidirectional connection for stabilization
            nengo.Connection(self.semantic_state, self.cleanup, synapse=0.01)
            nengo.Connection(self.cleanup, self.semantic_state, transform=0.3, synapse=0.05)

            # ============================================================
            # DECOMPRESSION LAYER: 512D → 64D (Fast output)
            # ============================================================

            self.decoder = nengo.Ensemble(
                n_neurons=2000,
                dimensions=self.VISUAL_DIM,
                neuron_type=nengo.LIF(tau_rc=0.02),  # Fast (20ms)
                label="Decoder"
            )

            # Learned decompression transform: 512D → 64D
            self.decompression = nengo.Connection(
                self.semantic_state,
                self.decoder,
                transform=np.random.randn(self.VISUAL_DIM, self.SEMANTIC_DIM) * 0.1,
                learning_rule_type=nengo.PES(learning_rate=1e-4)
            )

            # ============================================================
            # DREAM MODE: Noise injection for free-running generation
            # ============================================================

            def dream_noise_func(t):
                """Inject noise when in dream mode"""
                if self.get_parameter('dream_mode').value:
                    return np.random.randn(self.SEMANTIC_DIM) * 0.1
                else:
                    return np.zeros(self.SEMANTIC_DIM)

            dream_noise = nengo.Node(output=dream_noise_func)
            nengo.Connection(dream_noise, self.semantic_state, synapse=0.01)

            # ============================================================
            # ERROR ENSEMBLE: Drives learning
            # ============================================================

            self.error_ensemble = nengo.Ensemble(
                n_neurons=500,
                dimensions=self.VISUAL_DIM,
                label="Error Ensemble"
            )
            nengo.Connection(error_input, self.error_ensemble)

            # Error drives learning in both directions
            nengo.Connection(
                self.error_ensemble,
                self.compression.learning_rule,
                transform=-1,  # Minimize error
                synapse=0.01
            )
            nengo.Connection(
                self.error_ensemble,
                self.decompression.learning_rule,
                transform=-1,
                synapse=0.01
            )

            # ============================================================
            # OUTPUT NODES (Nengo → ROS2)
            # ============================================================

            # Top-down prediction to visual cortex
            topdown_output = nengo.Node(
                input=self.publish_topdown,
                size_in=self.VISUAL_DIM
            )
            nengo.Connection(self.decoder, topdown_output, synapse=0.01)

            # Semantic state monitoring
            semantic_output = nengo.Node(
                input=self.publish_semantic,
                size_in=self.SEMANTIC_DIM
            )
            nengo.Connection(self.semantic_state, semantic_output, synapse=0.01)

            # ============================================================
            # PROBES (for debugging and monitoring)
            # ============================================================

            self.semantic_probe = nengo.Probe(self.semantic_state, synapse=0.01)
            self.prediction_probe = nengo.Probe(self.decoder, synapse=0.01)

    def publish_topdown(self, t, x):
        """Callback to publish top-down predictions from Nengo to ROS2."""
        msg = Float32MultiArray()
        msg.data = x.tolist()
        self.topdown_pub.publish(msg)

    def publish_semantic(self, t, x):
        """Callback to publish semantic state for monitoring."""
        msg = Float32MultiArray()
        msg.data = x.tolist()
        self.semantic_pub.publish(msg)

    def cortex_activity_callback(self, msg: Float32MultiArray):
        """
        Receive cortex activity from the visual cortex.

        Args:
            msg: Neural activity from the visual cortex
        """
        self.cortex_activity = np.array(msg.data, dtype=np.float32)

    def error_feedback_callback(self, msg: Float32MultiArray):
        """
        Receive prediction error feedback from visual cortex for learning.

        Args:
            msg: Prediction error from visual cortex
        """
        self.prediction_error_feedback = np.array(msg.data, dtype=np.float32)
    
    def generate_prediction(self):
        """
        Main simulation loop - steps the Nengo simulator forward.

        The Nengo network automatically:
        - Compresses cortex activity to semantic pointers
        - Integrates over slow timescales
        - Decompresses to visual predictions
        - Publishes outputs via output nodes
        - Updates via PES learning from error signals

        PLUS (NEW): Monitors consciousness metrics:
        - Critical brain dynamics (branching ratio, avalanches)
        - Attribution Consciousness Index (ACI = f(Φ, κ))
        - Adaptive parameter tuning to maintain criticality
        """
        if NENGO_AVAILABLE and hasattr(self, 'sim'):
            self.sim.step()

            # ============================================================
            # CONSCIOUSNESS MONITORING (Papers Implementation)
            # ============================================================

            if CONSCIOUSNESS_MODULES_AVAILABLE and self.criticality_monitor and self.consciousness_metrics:
                # Extract current network state from probes
                if len(self.sim.data[self.semantic_probe]) > 0:
                    current_semantic = self.sim.data[self.semantic_probe][-1]

                    # Get neural activity (approximated from semantic state magnitude)
                    # In real implementation, would extract actual spike data from neurons
                    neural_activity = np.abs(current_semantic[:64])  # Use first 64 dims as proxy

                    # Create spike raster (binarized activity)
                    spike_threshold = 0.1
                    spike_raster = (neural_activity > spike_threshold).astype(float)

                    # 1. UPDATE CRITICALITY MONITOR
                    criticality_metrics = self.criticality_monitor.update(spike_raster)

                    # Publish criticality metrics
                    branching_msg = Float32()
                    branching_msg.data = float(criticality_metrics['branching_ratio'])
                    self.criticality_pub.publish(branching_msg)

                    state_msg = String()
                    state_msg.data = criticality_metrics['state']
                    self.criticality_state_pub.publish(state_msg)

                    # 2. ADAPTIVE TUNING (ConCrit Framework Implementation)
                    if self.adaptive_tuning_enabled:
                        param_name, adjustment = criticality_metrics['tuning_recommendation']

                        if param_name == 'tau_rc' and adjustment != 1.0:
                            # Adjust tau_slow to move toward criticality
                            self.tau_slow_current *= adjustment

                            # Clamp to reasonable range [50ms, 200ms]
                            self.tau_slow_current = np.clip(self.tau_slow_current, 50.0, 200.0)

                            # Log significant adjustments
                            if abs(adjustment - 1.0) > 0.01:
                                self.get_logger().info(
                                    f'Criticality tuning: {criticality_metrics["state"]} → '
                                    f'tau_slow={self.tau_slow_current:.1f}ms '
                                    f'(σ={criticality_metrics["branching_ratio"]:.3f})'
                                )

                    # 3. UPDATE CONSCIOUSNESS METRICS (ACI Framework)
                    consciousness_metrics = self.consciousness_metrics.update(
                        semantic_state=current_semantic,
                        neural_activity=neural_activity
                    )

                    # Publish ACI metrics
                    aci_msg = Float32()
                    aci_msg.data = float(consciousness_metrics['aci'])
                    self.aci_pub.publish(aci_msg)

                    prob_msg = Float32()
                    prob_msg.data = float(consciousness_metrics['consciousness_probability'])
                    self.consciousness_prob_pub.publish(prob_msg)

                    # Log consciousness state transitions
                    if consciousness_metrics['is_conscious'] and not hasattr(self, '_was_conscious'):
                        self.get_logger().info(
                            f'🧠 CONSCIOUSNESS EMERGED: ACI={consciousness_metrics["aci"]:.2f} '
                            f'(Φ={consciousness_metrics["phi"]:.3f}, κ={consciousness_metrics["kappa"]:.3f})'
                        )
                        self._was_conscious = True
                    elif not consciousness_metrics['is_conscious'] and hasattr(self, '_was_conscious'):
                        self.get_logger().info('💤 Consciousness faded')
                        delattr(self, '_was_conscious')
        else:
            # Fallback for when Nengo is not available
            pass
    
    
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
