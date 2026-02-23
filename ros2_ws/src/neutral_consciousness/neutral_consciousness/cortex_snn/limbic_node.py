"""
Limbic Node - Affective Consciousness & Valence Modulation

Implements the affective/limbic system that Feinberg & Mallatt identify
as the SECOND type of primary consciousness (alongside exteroceptive).
This node generates a global dopamine-like signal that modulates synaptic
plasticity (PES learning rates) across the entire network.

SCIENTIFIC FOUNDATION:
    Feinberg, T.E. & Mallatt, J. (2020). "Phenomenal Consciousness and
    Emergence: Eliminating the Explanatory Gap." Frontiers in Psychology,
    11, 1041. DOI: 10.3389/fpsyg.2020.01041

    Level 3 consciousness requires: "Valence coding of good and bad,
    for affective states" that "feed into premotor brain regions to
    motivate, choose, and guide movements in space."

    The biological affective system includes: habenula, basal forebrain,
    periaqueductal gray (PAG), ventral tegmental area (VTA), nucleus
    accumbens, and parts of the reticular formation.

NEO-HEBBIAN RL FOUNDATION:
    Triche, A. et al. (2022). "Exploration in neo-Hebbian reinforcement
    learning." Neural Networks, 151, 16-33.
    DOI: 10.1016/j.neunet.2022.03.021

    "Neo-Hebbian RL methods extend unsupervised Hebbian learning rules
    with value-based modulation to selectively reinforce associations."

ARCHITECTURE:
    - Receives survival metrics from Unity digital twin
    - Computes valence (pleasure/pain scalar) from weighted combination
    - Generates dopamine-like modulation signal via Nengo SNN (500 LIF)
    - Publishes dopamine signal that modulates PES learning rates globally
    - Tracks reward prediction error for intrinsic motivation (curiosity)

    The system only "cares" to update its mental images when it "feels"
    something significant — this is the missing link between reflexive
    processing and true affective consciousness.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, String
import numpy as np

try:
    import nengo
    NENGO_AVAILABLE = True
except ImportError:
    NENGO_AVAILABLE = False
    print("Warning: Nengo not installed. Running in stub mode.")


class LimbicNode(Node):
    """
    Affective modulation system implementing limbic valence computation.

    Subscribes to Unity survival metrics and computes a dopamine-like
    neuromodulatory signal that gates learning across the network.
    """

    def __init__(self):
        super().__init__('limbic_node')

        # ============================================================
        # VALENCE COMPUTATION WEIGHTS
        # ============================================================
        # These map survival metrics to a pleasure/pain scalar.
        # Positive = good (reward), Negative = bad (punishment).
        # Tuned for a robotic survival scenario in Unity.
        self.WEIGHT_BATTERY = 0.3       # High battery → positive valence
        self.WEIGHT_DAMAGE = -0.5       # Collision damage → strong negative
        self.WEIGHT_GOAL_PROX = 0.4     # Near goal → positive
        self.WEIGHT_THREAT_PROX = -0.6  # Near threat → strong negative

        # State buffers
        self.battery_level = 1.0      # [0, 1] — full by default
        self.collision_damage = 0.0   # [0, 1] — none by default
        self.goal_proximity = 0.0     # [0, 1] — 1 = at goal
        self.threat_proximity = 0.0   # [0, 1] — 1 = threat very close

        # Reward prediction error (for intrinsic motivation / curiosity)
        self.predicted_valence = 0.0
        self.reward_prediction_alpha = 0.1  # Exponential moving average rate

        # ============================================================
        # ROS2 SUBSCRIBERS (from Unity digital twin)
        # ============================================================
        self.battery_sub = self.create_subscription(
            Float32, '/unity/battery_level',
            self.battery_callback, 10
        )
        self.damage_sub = self.create_subscription(
            Float32, '/unity/collision_damage',
            self.damage_callback, 10
        )
        self.goal_sub = self.create_subscription(
            Float32, '/unity/goal_proximity',
            self.goal_callback, 10
        )
        self.threat_sub = self.create_subscription(
            Float32, '/unity/threat_distance',
            self.threat_callback, 10
        )

        # Also subscribe to prediction error magnitude for curiosity signal
        self.pred_error_sub = self.create_subscription(
            Float32MultiArray, '/neural_data/prediction_error',
            self.prediction_error_callback, 10
        )

        # ============================================================
        # ROS2 PUBLISHERS
        # ============================================================

        # Raw valence scalar [-1, +1]
        self.valence_pub = self.create_publisher(
            Float32, '/consciousness/affective/valence', 10
        )

        # Dopamine modulation signal [0, 1] (used by visual cortex, dream engine)
        self.dopamine_pub = self.create_publisher(
            Float32, '/consciousness/affective/dopamine', 10
        )

        # Reward prediction error (curiosity / surprise signal)
        self.rpe_pub = self.create_publisher(
            Float32, '/consciousness/affective/reward_prediction_error', 10
        )

        # Affective state label
        self.state_pub = self.create_publisher(
            String, '/consciousness/affective/state', 10
        )

        # ============================================================
        # BUILD NENGO MODEL
        # ============================================================
        if NENGO_AVAILABLE:
            self.build_nengo_model()
            self.sim = nengo.Simulator(self.model, dt=0.001)
            self.get_logger().info(
                "Limbic Node initialized: 500 LIF neurons, "
                "affective valence + dopamine modulation"
            )
        else:
            self.get_logger().warn("Nengo unavailable. Limbic node in stub mode.")

        # Valence computation at 30Hz (matches dream engine rate)
        self.timer = self.create_timer(1.0 / 30.0, self.compute_valence)

    def build_nengo_model(self):
        """
        Build the limbic valence SNN.

        Architecture:
        - Valence input: 1D Node (raw valence scalar from metrics)
        - VTA ensemble: 200 LIF neurons (ventral tegmental area analogue)
          Computes dopamine signal from valence + reward prediction error
        - PAG ensemble: 150 LIF neurons (periaqueductal gray analogue)
          Pain/threat urgency processing
        - Habenula ensemble: 150 LIF neurons
          Integrates negative valence, inhibits VTA when overwhelmed

        Total: 500 LIF neurons
        """
        self.model = nengo.Network(label="Limbic System")

        # Internal state: updated each compute_valence() call
        self._current_valence = 0.0
        self._current_rpe = 0.0
        self._prediction_error_magnitude = 0.0

        with self.model:
            # ============================================================
            # INPUT NODES
            # ============================================================
            valence_input = nengo.Node(output=lambda t: self._current_valence)
            rpe_input = nengo.Node(output=lambda t: self._current_rpe)
            curiosity_input = nengo.Node(
                output=lambda t: self._prediction_error_magnitude
            )

            # ============================================================
            # VTA (Ventral Tegmental Area): Dopamine production
            # ============================================================
            # The VTA is the primary dopamine source. It responds to:
            # 1. Positive valence (rewards)
            # 2. Reward prediction error (unexpected events)
            # 3. Curiosity (high prediction error = novel stimulus)
            self.vta = nengo.Ensemble(
                n_neurons=200,
                dimensions=1,
                neuron_type=nengo.LIF(tau_rc=0.02),  # 20ms dynamics
                label="VTA (Dopamine)"
            )

            # Valence → VTA: positive valence excites dopamine
            nengo.Connection(valence_input, self.vta, transform=0.6, synapse=0.01)

            # RPE → VTA: unexpected rewards/punishments boost dopamine
            # (The absolute value of RPE matters, not the sign)
            nengo.Connection(rpe_input, self.vta, transform=0.3, synapse=0.01)

            # Curiosity → VTA: novel stimuli (high prediction error) excite
            nengo.Connection(curiosity_input, self.vta, transform=0.2, synapse=0.01)

            # VTA self-sustaining dynamics (dopamine doesn't drop instantly)
            nengo.Connection(
                self.vta, self.vta,
                transform=0.5,
                synapse=0.05  # 50ms decay
            )

            # ============================================================
            # PAG (Periaqueductal Gray): Pain/threat urgency
            # ============================================================
            # Responds strongly to negative valence (damage, threat).
            # When PAG is active, it BOOSTS dopamine (fight-or-flight).
            self.pag = nengo.Ensemble(
                n_neurons=150,
                dimensions=1,
                neuron_type=nengo.LIF(tau_rc=0.01),  # 10ms — fast pain response
                intercepts=nengo.dists.Uniform(0.2, 0.8),  # Threshold for pain
                label="PAG (Pain/Urgency)"
            )

            # Negative valence → PAG (inverted: negative valence = positive PAG)
            nengo.Connection(valence_input, self.pag, transform=-1.0, synapse=0.005)

            # PAG → VTA: pain/threat increases dopamine (urgency signal)
            nengo.Connection(self.pag, self.vta, transform=0.4, synapse=0.01)

            # ============================================================
            # HABENULA: Disappointment / learned helplessness
            # ============================================================
            # Inhibits VTA when negative valence is persistent.
            # Prevents wasted energy on hopeless situations.
            self.habenula = nengo.Ensemble(
                n_neurons=150,
                dimensions=1,
                neuron_type=nengo.LIF(tau_rc=0.05),  # 50ms — slow accumulation
                label="Habenula (Inhibition)"
            )

            # Sustained negative valence → Habenula
            nengo.Connection(valence_input, self.habenula, transform=-0.5, synapse=0.1)

            # Habenula self-sustaining (accumulates over time)
            nengo.Connection(
                self.habenula, self.habenula,
                transform=0.7,
                synapse=0.1  # Slow accumulation
            )

            # Habenula → VTA: INHIBITS dopamine when persistently negative
            nengo.Connection(self.habenula, self.vta, transform=-0.3, synapse=0.02)

            # ============================================================
            # OUTPUT: Dopamine signal [0, 1]
            # ============================================================
            # The VTA output is transformed through a sigmoid-like function
            # to produce the [0, 1] dopamine modulation signal.

            dopamine_output = nengo.Node(
                input=self._publish_dopamine,
                size_in=1
            )
            nengo.Connection(self.vta, dopamine_output, synapse=0.01)

            # ============================================================
            # PROBES
            # ============================================================
            self.vta_probe = nengo.Probe(
                self.vta, synapse=0.01, sample_every=0.001
            )
            self.pag_probe = nengo.Probe(
                self.pag, synapse=0.01, sample_every=0.001
            )
            self.habenula_probe = nengo.Probe(
                self.habenula, synapse=0.01, sample_every=0.001
            )

    # ================================================================
    # ROS2 CALLBACKS
    # ================================================================

    def battery_callback(self, msg: Float32):
        self.battery_level = float(np.clip(msg.data, 0.0, 1.0))

    def damage_callback(self, msg: Float32):
        self.collision_damage = float(np.clip(msg.data, 0.0, 1.0))

    def goal_callback(self, msg: Float32):
        self.goal_proximity = float(np.clip(msg.data, 0.0, 1.0))

    def threat_callback(self, msg: Float32):
        self.threat_proximity = float(np.clip(msg.data, 0.0, 1.0))

    def prediction_error_callback(self, msg: Float32MultiArray):
        """Track prediction error magnitude for curiosity/intrinsic motivation."""
        error = np.array(msg.data, dtype=np.float32)
        self._prediction_error_magnitude = float(np.clip(
            np.linalg.norm(error) / max(len(error), 1), 0.0, 1.0
        ))

    # ================================================================
    # NENGO OUTPUT CALLBACK
    # ================================================================

    def _publish_dopamine(self, t, x):
        """
        Transform VTA output to [0, 1] dopamine signal and publish.

        Uses a sigmoid to map the potentially negative VTA activity
        to a bounded [0, 1] range suitable for learning rate modulation.
        """
        raw = float(x[0])
        # Sigmoid: maps (-inf, +inf) → (0, 1) with midpoint at 0
        dopamine = 1.0 / (1.0 + np.exp(-3.0 * raw))
        dopamine = float(np.clip(dopamine, 0.0, 1.0))

        msg = Float32()
        msg.data = dopamine
        self.dopamine_pub.publish(msg)

    # ================================================================
    # MAIN COMPUTATION LOOP
    # ================================================================

    def compute_valence(self):
        """
        Compute affective valence from Unity survival metrics and step SNN.

        Valence = weighted combination of survival signals:
          V = w1*battery - w2*damage + w3*goal_prox - w4*threat_prox

        This maps directly to Feinberg & Mallatt's affective consciousness:
        the system assigns "good" and "bad" to sensory states as a common
        currency for motivating behavior.
        """
        # Raw valence computation
        valence = (
            self.WEIGHT_BATTERY * self.battery_level
            + self.WEIGHT_DAMAGE * self.collision_damage
            + self.WEIGHT_GOAL_PROX * self.goal_proximity
            + self.WEIGHT_THREAT_PROX * self.threat_proximity
        )
        valence = float(np.clip(valence, -1.0, 1.0))

        # Reward Prediction Error (RPE)
        # RPE = actual_valence - predicted_valence
        rpe = valence - self.predicted_valence

        # Update prediction (exponential moving average)
        self.predicted_valence += self.reward_prediction_alpha * rpe

        # Store for Nengo node injection
        self._current_valence = valence
        self._current_rpe = float(np.clip(abs(rpe), 0.0, 1.0))

        # Step the Nengo simulator
        if NENGO_AVAILABLE and hasattr(self, 'sim'):
            self.sim.step()

        # Publish valence
        val_msg = Float32()
        val_msg.data = valence
        self.valence_pub.publish(val_msg)

        # Publish RPE
        rpe_msg = Float32()
        rpe_msg.data = float(rpe)
        self.rpe_pub.publish(rpe_msg)

        # Publish affective state label
        state_msg = String()
        if valence > 0.3:
            state_msg.data = "positive"
        elif valence < -0.3:
            state_msg.data = "negative"
        elif abs(rpe) > 0.2:
            state_msg.data = "surprised"
        else:
            state_msg.data = "neutral"
        self.state_pub.publish(state_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LimbicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
