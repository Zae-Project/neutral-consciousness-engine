"""
Criticality Monitor Module - Critical Brain Dynamics

Implements monitoring and regulation of critical brain dynamics based on:
- Algom & Shriki (2025). "The ConCrit Framework: Critical brain dynamics as
  a unifying mechanistic framework for theories of consciousness."
  DOI: 10.1016/j.neubiorev.2025.106483

CONCEPT:
Neural networks operate optimally near a critical transition point between
order (frozen) and chaos (random). This module monitors the network's
criticality state and provides feedback for dynamic parameter tuning.

METRICS:
- Branching Ratio (σ): Average activation descendants per spike
  - σ < 1: Subcritical (dying out)
  - σ = 1: Critical (optimal)
  - σ > 1: Supercritical (runaway)

- Neural Avalanche Distribution: Power-law indicates criticality

RESEARCH FOUNDATION:
According to PubMed, the ConCrit framework proposes that consciousness
emerges when neural systems operate near criticality, which:
1. Enhances complexity and richness of internal representations
2. Heightens sensitivity to the system's own state
3. Supports flexible switching between conscious states
"""

import numpy as np
from collections import deque
from typing import Tuple, Optional


class CriticalityMonitor:
    """
    Monitors neural network criticality state.

    Used to detect whether the network is operating in:
    - Subcritical regime (too ordered, low responsiveness)
    - Critical regime (optimal balance, consciousness-supporting)
    - Supercritical regime (too chaotic, unstable)
    """

    def __init__(
        self,
        window_size: int = 100,
        target_branching_ratio: float = 1.0,
        tolerance: float = 0.1
    ):
        """
        Initialize the criticality monitor.

        Args:
            window_size: Number of timesteps to compute running statistics
            target_branching_ratio: Target σ for critical state (default 1.0)
            tolerance: Acceptable deviation from target (default 0.1)
        """
        self.window_size = window_size
        self.target_branching_ratio = target_branching_ratio
        self.tolerance = tolerance

        # Sliding window for activity tracking
        self.activity_history = deque(maxlen=window_size)
        self.spike_counts = deque(maxlen=window_size)

        # Avalanche tracking
        self.avalanche_sizes = []
        self.current_avalanche_size = 0
        self.in_avalanche = False

        # Criticality metrics
        self.current_branching_ratio = 1.0
        self.criticality_state = "critical"  # "subcritical", "critical", "supercritical"

    def update(self, spike_raster: np.ndarray) -> dict:
        """
        Update criticality metrics with new spike data.

        Args:
            spike_raster: Binary array of shape (n_neurons,) indicating which neurons spiked

        Returns:
            dict: Criticality metrics including branching ratio, state, and recommendations
        """
        # Count active neurons
        n_active = np.sum(spike_raster > 0)
        self.spike_counts.append(n_active)
        self.activity_history.append(spike_raster.copy())

        # Track avalanches (continuous bursts of activity)
        if n_active > 0:
            if not self.in_avalanche:
                self.in_avalanche = True
                self.current_avalanche_size = 0
            self.current_avalanche_size += n_active
        else:
            if self.in_avalanche:
                # Avalanche ended
                self.avalanche_sizes.append(self.current_avalanche_size)
                self.in_avalanche = False

        # Compute branching ratio if we have enough history
        if len(self.spike_counts) >= 2:
            self.current_branching_ratio = self._compute_branching_ratio()
            self.criticality_state = self._classify_state()

        return self.get_metrics()

    def _compute_branching_ratio(self) -> float:
        """
        Compute the branching ratio σ = ⟨A(t+1)⟩ / ⟨A(t)⟩

        This measures how activity propagates through the network.
        σ = 1 indicates critical dynamics.
        """
        if len(self.spike_counts) < 2:
            return 1.0

        spike_array = np.array(self.spike_counts)

        # Avoid division by zero
        current_activity = spike_array[:-1]
        next_activity = spike_array[1:]

        valid_indices = current_activity > 0
        if not np.any(valid_indices):
            return 1.0

        # Branching ratio: avg(next) / avg(current)
        branching = np.mean(next_activity[valid_indices]) / np.mean(current_activity[valid_indices])

        return float(branching)

    def _classify_state(self) -> str:
        """
        Classify the current criticality state.

        Returns:
            "subcritical", "critical", or "supercritical"
        """
        if self.current_branching_ratio < (self.target_branching_ratio - self.tolerance):
            return "subcritical"
        elif self.current_branching_ratio > (self.target_branching_ratio + self.tolerance):
            return "supercritical"
        else:
            return "critical"

    def get_tuning_recommendation(self) -> Tuple[str, float]:
        """
        Get parameter tuning recommendation to move toward criticality.

        Returns:
            Tuple of (parameter_name, adjustment_factor)

        Example:
            ("tau_rc", 1.1) means increase tau_rc by 10%
            ("tau_rc", 0.9) means decrease tau_rc by 10%
        """
        if self.criticality_state == "subcritical":
            # Network is too damped, needs more excitability
            # Decrease tau_rc to make neurons respond faster
            return ("tau_rc", 0.95)
        elif self.criticality_state == "supercritical":
            # Network is too excitable, needs more damping
            # Increase tau_rc to slow down responses
            return ("tau_rc", 1.05)
        else:
            # Already critical, no adjustment needed
            return ("tau_rc", 1.0)

    def get_metrics(self) -> dict:
        """
        Get current criticality metrics.

        Returns:
            dict with keys:
                - branching_ratio: Current σ value
                - state: "subcritical", "critical", or "supercritical"
                - distance_from_critical: Absolute deviation from target
                - avalanche_count: Number of avalanches recorded
                - mean_avalanche_size: Average avalanche size
                - tuning_recommendation: (param, factor) tuple
        """
        metrics = {
            "branching_ratio": self.current_branching_ratio,
            "state": self.criticality_state,
            "distance_from_critical": abs(self.current_branching_ratio - self.target_branching_ratio),
            "avalanche_count": len(self.avalanche_sizes),
            "mean_avalanche_size": np.mean(self.avalanche_sizes) if self.avalanche_sizes else 0.0,
            "tuning_recommendation": self.get_tuning_recommendation()
        }

        return metrics

    def is_consciousness_supporting(self) -> bool:
        """
        Determine if current state is likely to support consciousness.

        Based on ConCrit framework: consciousness emerges near criticality.

        Returns:
            True if state is critical, False otherwise
        """
        return self.criticality_state == "critical"

    def reset(self):
        """Reset all tracking metrics."""
        self.activity_history.clear()
        self.spike_counts.clear()
        self.avalanche_sizes.clear()
        self.current_avalanche_size = 0
        self.in_avalanche = False
        self.current_branching_ratio = 1.0
        self.criticality_state = "critical"
