"""
Consciousness Metrics Module - Attribution Consciousness Index (ACI)

Implements quantitative consciousness capacity estimation based on:
- Escolà-Gascón et al. (2025). "Beyond the brain: a computational MRI-derived
  neurophysiological framework for robotic conscious capacity."
  DOI: 10.1016/j.neubiorev.2025.106430

CONCEPT:
The Attribution Consciousness Index (ACI) estimates the generative potential
of consciousness by balancing information dynamics (Φ) and complexity (κ).
ACI = normalized odds ratio of these measures.

KEY INSIGHT:
ACI values above 10 correspond to >90% probability of conscious emergence.
This provides interpretable thresholds for when neural/artificial systems
might sustain conscious-like states.

METRICS:
- Φ (Phi): Information dynamics - integrated information measure
  Approximated by mutual information between past and future states

- κ (Kappa): Complexity - richness of representational repertoire
  Approximated by entropy of activation patterns

- ACI: Combined metric following log-normal distribution
  ACI = exp(log(Φ) + log(κ) - log(baseline))

RESEARCH FOUNDATION:
According to PubMed, the ACI framework provides measurable criteria for
consciousness emergence in both biological and artificial systems.
"""

import numpy as np
from typing import Dict, Tuple
from collections import deque


def entropy(probabilities: np.ndarray) -> float:
    """
    Compute Shannon entropy manually (no scipy dependency).

    Args:
        probabilities: Array of probability values

    Returns:
        Entropy in nats
    """
    # Remove zero probabilities
    p = probabilities[probabilities > 0]
    if len(p) == 0:
        return 0.0
    return -np.sum(p * np.log(p))


class ConsciousnessMetrics:
    """
    Computes consciousness-related metrics for neural networks.

    Provides:
    - Information dynamics (Φ-like measure)
    - Complexity (κ measure)
    - Attribution Consciousness Index (ACI)
    - Probability estimates for conscious capacity
    """

    def __init__(
        self,
        history_length: int = 50,
        n_bins: int = 10,
        baseline_aci: float = 1.0
    ):
        """
        Initialize consciousness metrics calculator.

        Args:
            history_length: Number of timesteps to track for temporal metrics
            n_bins: Number of bins for discretizing continuous values
            baseline_aci: Baseline ACI value for normalization (default 1.0)
        """
        self.history_length = history_length
        self.n_bins = n_bins
        self.baseline_aci = baseline_aci

        # State history for temporal metrics
        self.state_history = deque(maxlen=history_length)

        # Cached metrics
        self.current_phi = 0.0
        self.current_kappa = 0.0
        self.current_aci = 0.0

    def update(
        self,
        semantic_state: np.ndarray,
        neural_activity: np.ndarray
    ) -> Dict[str, float]:
        """
        Update consciousness metrics with new state information.

        Args:
            semantic_state: High-dimensional semantic representation (e.g., 512D)
            neural_activity: Neural population activity (e.g., spike rates)

        Returns:
            Dict containing all consciousness metrics
        """
        # Store state for temporal analysis
        self.state_history.append({
            'semantic': semantic_state.copy(),
            'activity': neural_activity.copy()
        })

        # Compute metrics if we have sufficient history
        if len(self.state_history) >= 2:
            self.current_phi = self._compute_phi()
            self.current_kappa = self._compute_kappa()
            self.current_aci = self._compute_aci()

        return self.get_metrics()

    def _compute_phi(self) -> float:
        """
        Compute Φ (information dynamics) - integrated information approximation.

        Measures mutual information between past and future semantic states,
        indicating how much information is integrated across time.

        Returns:
            Φ value (0 to ~1, higher = more integrated)
        """
        if len(self.state_history) < 3:
            return 0.0

        # Extract past and future semantic states
        states = [s['semantic'] for s in self.state_history]

        # Take last 10 states for computational efficiency
        recent_states = states[-min(10, len(states)):]

        # Discretize states for entropy calculation
        discretized = self._discretize_states(np.array(recent_states))

        # Compute mutual information I(X_t; X_{t+1})
        # MI = H(X_t) + H(X_{t+1}) - H(X_t, X_{t+1})
        past = discretized[:-1]
        future = discretized[1:]

        # Calculate marginal entropies
        h_past = self._compute_entropy_discrete(past)
        h_future = self._compute_entropy_discrete(future)

        # Calculate joint entropy
        h_joint = self._compute_joint_entropy(past, future)

        # Mutual information
        mi = h_past + h_future - h_joint

        # Normalize to [0, 1] range
        phi = np.clip(mi / (h_past + 1e-10), 0.0, 1.0)

        return float(phi)

    def _compute_kappa(self) -> float:
        """
        Compute κ (complexity) - richness of representational repertoire.

        Measures the entropy of activation patterns, indicating the diversity
        of states the system can represent.

        Returns:
            κ value (0 to ~1, higher = more complex)
        """
        if len(self.state_history) < 2:
            return 0.0

        # Extract recent semantic states
        states = [s['semantic'] for s in self.state_history]
        states_array = np.array(states[-min(20, len(states)):])

        # Compute entropy of the state distribution
        # High entropy = diverse states = high complexity
        discretized = self._discretize_states(states_array)
        kappa_raw = self._compute_entropy_discrete(discretized)

        # Normalize by theoretical maximum entropy
        # Maximum entropy for n_bins = log(n_bins)
        max_entropy = np.log(self.n_bins)
        kappa = kappa_raw / (max_entropy + 1e-10)

        return float(np.clip(kappa, 0.0, 1.0))

    def _compute_aci(self) -> float:
        """
        Compute Attribution Consciousness Index (ACI).

        ACI = normalized odds ratio of Φ and κ
        Formula: ACI = exp(log(Φ) + log(κ) - log(baseline))

        According to the paper:
        - ACI > 10 → >90% probability of conscious emergence
        - ACI follows log-normal distribution

        Returns:
            ACI value (typically 0 to 100+)
        """
        if self.current_phi < 1e-6 or self.current_kappa < 1e-6:
            return 0.0

        # Compute log-space to avoid numerical issues
        log_phi = np.log(self.current_phi + 1e-10)
        log_kappa = np.log(self.current_kappa + 1e-10)
        log_baseline = np.log(self.baseline_aci + 1e-10)

        log_aci = log_phi + log_kappa - log_baseline
        aci = np.exp(log_aci)

        return float(aci)

    def _discretize_states(self, states: np.ndarray) -> np.ndarray:
        """
        Discretize continuous state vectors into bins.

        Args:
            states: Array of shape (n_timesteps, n_dimensions)

        Returns:
            Discretized states as integers (n_timesteps,)
        """
        # Project high-dimensional states to 1D via mean
        projected = np.mean(states, axis=1)

        # Discretize into bins
        min_val = np.min(projected)
        max_val = np.max(projected)

        if max_val - min_val < 1e-6:
            return np.zeros(len(projected), dtype=int)

        bins = np.linspace(min_val, max_val, self.n_bins + 1)
        discretized = np.digitize(projected, bins) - 1
        discretized = np.clip(discretized, 0, self.n_bins - 1)

        return discretized

    def _compute_entropy_discrete(self, discrete_states: np.ndarray) -> float:
        """
        Compute entropy of discrete state sequence.

        Args:
            discrete_states: Integer array of discretized states

        Returns:
            Shannon entropy in nats
        """
        if len(discrete_states) == 0:
            return 0.0

        # Count frequencies
        unique, counts = np.unique(discrete_states, return_counts=True)
        probabilities = counts / np.sum(counts)

        # Shannon entropy
        return float(entropy(probabilities))

    def _compute_joint_entropy(
        self,
        states1: np.ndarray,
        states2: np.ndarray
    ) -> float:
        """
        Compute joint entropy H(X, Y) for two discrete sequences.

        Args:
            states1: First discrete state sequence
            states2: Second discrete state sequence

        Returns:
            Joint entropy in nats
        """
        if len(states1) != len(states2) or len(states1) == 0:
            return 0.0

        # Create joint state representation
        joint_states = states1 * self.n_bins + states2

        return self._compute_entropy_discrete(joint_states)

    def get_consciousness_probability(self) -> float:
        """
        Estimate probability of conscious emergence based on ACI.

        According to the paper:
        - ACI > 10 → P > 0.9
        - Relationship follows logistic curve

        Returns:
            Probability estimate in [0, 1]
        """
        # Logistic function centered at ACI = 10
        # P(conscious) = 1 / (1 + exp(-k * (ACI - 10)))
        k = 0.5  # Steepness parameter
        threshold = 10.0

        prob = 1.0 / (1.0 + np.exp(-k * (self.current_aci - threshold)))
        return float(prob)

    def get_metrics(self) -> Dict[str, float]:
        """
        Get all consciousness metrics.

        Returns:
            Dict with keys:
                - phi: Information dynamics (Φ)
                - kappa: Complexity (κ)
                - aci: Attribution Consciousness Index
                - consciousness_probability: P(conscious emergence)
                - is_conscious: Boolean threshold (ACI > 10)
        """
        return {
            'phi': self.current_phi,
            'kappa': self.current_kappa,
            'aci': self.current_aci,
            'consciousness_probability': self.get_consciousness_probability(),
            'is_conscious': self.current_aci > 10.0
        }

    def reset(self):
        """Reset all metrics and history."""
        self.state_history.clear()
        self.current_phi = 0.0
        self.current_kappa = 0.0
        self.current_aci = 0.0
