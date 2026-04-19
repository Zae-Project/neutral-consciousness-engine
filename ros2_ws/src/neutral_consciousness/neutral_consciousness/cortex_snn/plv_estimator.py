"""
Phase-Locking Value estimator.

PLV is the canonical measure of phase coherence between two narrow-band
signals (Lachaux et al., 1999):

    PLV = | (1/N) * sum_t exp(i * (phi_a(t) - phi_b(t))) |

0.0 = no phase relation, 1.0 = perfect phase lock. Here we band-pass each
signal in the theta band (4-7 Hz by default) with a 4th-order Butterworth,
Hilbert-transform to recover instantaneous phase, and compute PLV over a
rolling window.

SciPy is required. If it is missing the estimator returns 0.0 and logs a
warning, so the rest of the engine keeps running.
"""

from __future__ import annotations

from collections import deque
from typing import Deque

import numpy as np

try:
    from scipy.signal import butter, filtfilt, hilbert  # type: ignore
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False


class PLVEstimator:
    """Rolling-window PLV between a cortex-rate signal and an EM reference."""

    def __init__(
        self,
        sample_rate_hz: float = 100.0,
        window_sec: float = 4.0,
        band_hz: tuple[float, float] = (4.0, 7.0),
        filter_order: int = 4,
    ) -> None:
        self.sample_rate_hz = float(sample_rate_hz)
        self.window_sec = float(window_sec)
        self.band_hz = band_hz
        self.filter_order = int(filter_order)

        self._size = max(16, int(self.window_sec * self.sample_rate_hz))
        self._a: Deque[float] = deque(maxlen=self._size)
        self._b: Deque[float] = deque(maxlen=self._size)

        if SCIPY_AVAILABLE:
            nyq = 0.5 * self.sample_rate_hz
            lo = max(1e-3, band_hz[0] / nyq)
            hi = min(0.999, band_hz[1] / nyq)
            self._b_coef, self._a_coef = butter(
                self.filter_order, [lo, hi], btype='band'
            )
        else:
            self._b_coef = self._a_coef = None

    def push(self, cortex_sample: float, em_sample: float) -> None:
        self._a.append(float(cortex_sample))
        self._b.append(float(em_sample))

    def ready(self) -> bool:
        return len(self._a) == self._size and SCIPY_AVAILABLE

    def compute(self) -> float:
        """Compute PLV over the current buffer. Returns 0.0 if not ready."""
        if not self.ready():
            return 0.0
        a = np.asarray(self._a, dtype=np.float64)
        b = np.asarray(self._b, dtype=np.float64)
        if np.std(a) < 1e-9 or np.std(b) < 1e-9:
            return 0.0
        a_bp = filtfilt(self._b_coef, self._a_coef, a)
        b_bp = filtfilt(self._b_coef, self._a_coef, b)
        phi_a = np.angle(hilbert(a_bp))
        phi_b = np.angle(hilbert(b_bp))
        plv = np.abs(np.mean(np.exp(1j * (phi_a - phi_b))))
        return float(plv)

    def reset(self) -> None:
        self._a.clear()
        self._b.clear()
