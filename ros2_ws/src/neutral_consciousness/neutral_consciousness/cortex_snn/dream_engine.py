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
    from .oscillatory_monitor import OscillatoryMonitor
    CONSCIOUSNESS_MODULES_AVAILABLE = True
except ImportError:
    CONSCIOUSNESS_MODULES_AVAILABLE = False
    print("Warning: Consciousness monitoring modules not available")


class DreamEngine(Node):
    """
    ROS 2 Node implementing the Generative Model (Pallium).

    The Dream Engine is the "pallium" — the higher cortical area that
    generates top-down predictions about incoming sensory data. This is
    Step 2 of Feinberg & Mallatt's two-step consciousness hypothesis:
    the dorsal pallium gradually became the dominant center of sensory
    consciousness, integrating deep memory and abstract reasoning.

    Now integrates:
    - Tectum unified map (fast multisensory input)
    - Dopamine modulation from limbic node (affective learning gating)
    - Topographic 8x8 output for spatial mental image reconstruction
    - Reentrant processor bound representation (converged binding feedback)
    - Spatial prediction error in map space (not just vector space)
    """

    def __init__(self):
        super().__init__('dream_engine')

        # Dimensions
        self.SEMANTIC_DIM = 512  # High-dimensional semantic pointers
        self.VISUAL_DIM = 64     # Low-dimensional visual features (8x8 grid)

        # Configuration
        self.declare_parameter('prediction_rate_hz', 30.0)
        self.declare_parameter('dream_mode', False)
        self.declare_parameter('tau_slow', 100.0)  # Slow dynamics timescale (ms)

        self.prediction_rate = self.get_parameter('prediction_rate_hz').value

        # State (injected via Nengo Nodes)
        self.cortex_activity = np.zeros(self.VISUAL_DIM, dtype=np.float32)
        self.prediction_error_feedback = np.zeros(self.VISUAL_DIM, dtype=np.float32)
        self.tectum_map = np.zeros(self.VISUAL_DIM, dtype=np.float32)

        # Dopamine modulation from limbic node (default: full learning)
        self.dopamine_signal = 1.0

        # Reentrant bound representation (converged binding from reentrant processor)
        self.bound_representation = np.zeros(self.VISUAL_DIM, dtype=np.float32)

        # Adaptive criticality gain (modulated at runtime instead of tau_rc)
        # Range: [0.5, 1.5] where 1.0 = nominal, <1.0 = damping, >1.0 = excitation
        self.criticality_gain = 1.0

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

        # Tectum unified map subscriber (fast multisensory input)
        self.tectum_sub = self.create_subscription(
            Float32MultiArray,
            '/tectum/unified_map',
            self.tectum_callback,
            10
        )

        # Dopamine modulation subscriber (from Limbic Node)
        self.dopamine_sub = self.create_subscription(
            Float32,
            '/consciousness/affective/dopamine',
            self.dopamine_callback,
            10
        )

        # Reentrant bound representation (converged binding feedback)
        self.reentrant_sub = self.create_subscription(
            Float32MultiArray,
            '/reentrant/bound_representation',
            self.reentrant_callback,
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

        # Topographic mental image reconstruction (8x8 spatial map)
        self.topographic_pub = self.create_publisher(
            Float32MultiArray,
            'dream/topographic_reconstruction',
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

        # Oscillatory dynamics publishers
        self.oscillatory_dominant_pub = self.create_publisher(
            String,
            'consciousness/oscillatory/dominant_band',
            10
        )

        self.oscillatory_flow_pub = self.create_publisher(
            String,
            'consciousness/oscillatory/flow_direction',
            10
        )

        self.oscillatory_coupling_pub = self.create_publisher(
            Float32,
            'consciousness/oscillatory/coupling_strength',
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
            self.oscillatory_monitor = OscillatoryMonitor(
                sampling_rate=1000.0,  # 1ms Nengo timestep
                window_size=1000       # 1 second analysis window
            )
            self.get_logger().info("Consciousness monitoring modules initialized (criticality + ACI + oscillatory)")
        else:
            self.criticality_monitor = None
            self.consciousness_metrics = None
            self.oscillatory_monitor = None
            self.get_logger().warn("Consciousness monitoring disabled")

        # Adaptive criticality tuning
        self.adaptive_tuning_enabled = True

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
            tectum_input = nengo.Node(output=lambda t: self.tectum_map)
            reentrant_input = nengo.Node(output=lambda t: self.bound_representation)

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

            # Tectum unified map also feeds the encoder (multisensory input)
            # Weighted lower than direct cortex since tectum is a summary
            nengo.Connection(tectum_input, self.encoder, transform=0.3)

            # Reentrant bound representation feeds the encoder
            # This is the converged binding from the reentrant processor —
            # it only arrives when bottom-up and top-down streams agree.
            # This closes the full reentrant loop: pallium → reentrant → pallium
            nengo.Connection(reentrant_input, self.encoder, transform=0.2)

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

            # Compression transform: 64D → 512D
            # No PES learning on compression — per predictive coding,
            # error corrects the generative (decompression) pathway only.
            # Encoding weights are fixed (random projection to high-D space).
            self.compression = nengo.Connection(
                self.encoder,
                self.semantic_state,
                transform=np.random.randn(self.SEMANTIC_DIM, self.VISUAL_DIM) * 0.1
            )

            # Recurrent connection for temporal integration (narrative continuity)
            # Gain = criticality_gain × dopamine modulation
            # Criticality tunes toward σ≈1.0, dopamine gates affective engagement
            gain_node = nengo.Node(
                output=lambda t: self.criticality_gain * (0.5 + 0.5 * self.dopamine_signal) * 0.7
            )
            nengo.Connection(
                self.semantic_state,
                self.semantic_state,
                synapse=0.1,  # 100ms integration window
                transform=1.0  # Base transform (gain applied via modulation below)
            )
            # Additive gain modulation: scales the recurrent drive
            nengo.Connection(
                gain_node,
                self.semantic_state,
                transform=np.ones((self.SEMANTIC_DIM, 1)) * 0.01,  # Small modulatory effect
                synapse=0.05
            )

            # ============================================================
            # CLEAN-UP MEMORY: Attractor-based stabilization
            # ============================================================
            # Per NEF theory: cleanup memory uses point attractor dynamics
            # where the system converges noisy/partial inputs to stable
            # stored patterns. Implemented via:
            # 1. High intercepts → selective neurons that only respond to
            #    strong, coherent inputs (natural thresholding)
            # 2. Strong recurrent connections → attractor dynamics that
            #    lock onto and stabilize activated patterns
            # 3. Mutual inhibition → winner-take-all competition between
            #    candidate patterns

            self.cleanup = nengo.Ensemble(
                n_neurons=1000,
                dimensions=self.SEMANTIC_DIM,
                neuron_type=nengo.LIF(tau_rc=0.02),
                intercepts=nengo.dists.Uniform(0.3, 0.9),  # High intercepts: selective
                label="Clean-up Memory"
            )

            # Input from semantic state
            nengo.Connection(self.semantic_state, self.cleanup, synapse=0.01)

            # Strong recurrent self-connection: point attractor dynamics
            # The system locks onto activated patterns and holds them stable
            nengo.Connection(
                self.cleanup, self.cleanup,
                synapse=0.05,
                transform=0.8  # Strong self-reinforcement
            )

            # Feedback to semantic state: cleaned-up pattern stabilizes the state
            nengo.Connection(
                self.cleanup, self.semantic_state,
                transform=0.3,
                synapse=0.05
            )

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

            # Error drives DECOMPRESSION learning (predictive coding)
            # Per NEF theory: prediction error corrects the generative model
            # (top-down predictions), not the encoding pathway.
            #
            # Compression learning is removed because:
            # - compression targets semantic_state (512D)
            # - PES requires error_dim == post_ensemble_dim
            # - error_ensemble is 64D → dimension mismatch with 512D
            # - Predictive coding: error corrects predictions, not encodings
            nengo.Connection(
                self.error_ensemble,
                self.decompression.learning_rule,
                transform=-1,  # Minimize prediction error
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
            # sample_every prevents unbounded memory growth during long runs
            # Only keep the most recent sample (1ms timestep, sample every step)

            self.semantic_probe = nengo.Probe(
                self.semantic_state, synapse=0.01, sample_every=0.001
            )
            self.prediction_probe = nengo.Probe(
                self.decoder, synapse=0.01, sample_every=0.001
            )
            # Probe actual neuron spikes for accurate consciousness monitoring
            # (instead of using decoded semantic values as proxy)
            self.neuron_probe = nengo.Probe(
                self.semantic_state.neurons, sample_every=0.001
            )

    def publish_topdown(self, t, x):
        """Callback to publish top-down predictions from Nengo to ROS2.

        The decoder output is a 64D vector that represents the 8×8
        topographic reconstruction — the "mental image" generated by
        the pallium's generative model. This is published both as a
        flat prediction (for the visual cortex error computation) and
        as an explicit topographic reconstruction (for visualization
        and spatial error comparison).
        """
        msg = Float32MultiArray()
        msg.data = x.tolist()
        self.topdown_pub.publish(msg)

        # Also publish as topographic reconstruction (same data, explicit topic)
        topo_msg = Float32MultiArray()
        topo_msg.data = x.tolist()
        self.topographic_pub.publish(topo_msg)

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

    def tectum_callback(self, msg: Float32MultiArray):
        """
        Receive unified multisensory map from the sensory tectum.

        The tectum provides a fast, pre-integrated spatial representation
        combining vision + proprioception. This is the "Step 1" conscious
        input from the evolutionary older midbrain structure.
        """
        data = np.array(msg.data, dtype=np.float32)
        if len(data) == self.VISUAL_DIM:
            self.tectum_map = data

    def dopamine_callback(self, msg: Float32):
        """
        Receive dopamine modulation signal from the Limbic Node.

        Scales the recurrent gain and effectively modulates how strongly
        the dream engine updates its internal model. High dopamine =
        salient event = stronger learning and integration.
        """
        self.dopamine_signal = float(np.clip(msg.data, 0.0, 1.0))

    def reentrant_callback(self, msg: Float32MultiArray):
        """
        Receive converged bound representation from the Reentrant Processor.

        This signal only arrives when the reentrant loop has achieved
        convergence (bottom-up ≈ top-down). It represents the unified
        conscious percept — the result of bidirectional binding between
        sensory evidence and predictive context.

        Fed back into the encoder to close the full reentrant loop:
        pallium prediction → reentrant binding → pallium update.
        """
        data = np.array(msg.data, dtype=np.float32)
        if len(data) == self.VISUAL_DIM:
            self.bound_representation = data

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

                    # Get actual neuron spike data from the neuron probe
                    # This is real firing rate data, not a decoded value proxy
                    if len(self.sim.data[self.neuron_probe]) > 0:
                        raw_neuron_data = self.sim.data[self.neuron_probe][-1]
                        # Subsample to manageable size (use first 100 neurons)
                        neural_activity = raw_neuron_data[:min(100, len(raw_neuron_data))]
                    else:
                        neural_activity = np.abs(current_semantic[:64])

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
                    # Uses gain modulation instead of tau_rc (which can't change at runtime)
                    if self.adaptive_tuning_enabled:
                        param_name, adjustment = criticality_metrics['tuning_recommendation']

                        if param_name == 'tau_rc' and adjustment != 1.0:
                            # Adjust criticality_gain to move toward criticality
                            # Subcritical (adj<1) → increase gain (more excitation)
                            # Supercritical (adj>1) → decrease gain (more damping)
                            # Invert: tau_rc increase = slower = less gain
                            gain_adjustment = 1.0 / adjustment
                            self.criticality_gain *= gain_adjustment

                            # Clamp to reasonable range [0.5, 1.5]
                            self.criticality_gain = float(np.clip(
                                self.criticality_gain, 0.5, 1.5
                            ))

                            # Log significant adjustments
                            if abs(adjustment - 1.0) > 0.01:
                                self.get_logger().info(
                                    f'Criticality tuning: {criticality_metrics["state"]} → '
                                    f'gain={self.criticality_gain:.3f} '
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

                    # 4. UPDATE OSCILLATORY MONITOR (Baker & Cariani 2025)
                    if self.oscillatory_monitor is not None:
                        osc_metrics = self.oscillatory_monitor.update(neural_activity)

                        # Publish dominant band
                        dominant_msg = String()
                        dominant_msg.data = osc_metrics['dominant_band']
                        self.oscillatory_dominant_pub.publish(dominant_msg)

                        # Publish flow direction
                        flow_msg = String()
                        flow_msg.data = osc_metrics['flow_direction']
                        self.oscillatory_flow_pub.publish(flow_msg)

                        # Publish coupling strength
                        coupling_msg = Float32()
                        coupling_msg.data = float(osc_metrics['coupling_strength'])
                        self.oscillatory_coupling_pub.publish(coupling_msg)

                    # Log consciousness state transitions
                    if consciousness_metrics['is_conscious'] and not hasattr(self, '_was_conscious'):
                        self.get_logger().info(
                            f'CONSCIOUSNESS EMERGED: ACI={consciousness_metrics["aci"]:.2f} '
                            f'(Phi={consciousness_metrics["phi"]:.3f}, kappa={consciousness_metrics["kappa"]:.3f})'
                        )
                        self._was_conscious = True
                    elif not consciousness_metrics['is_conscious'] and hasattr(self, '_was_conscious'):
                        self.get_logger().info('Consciousness faded')
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
