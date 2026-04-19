"""
Oscillatory Monitor Module - Temporal Brain Dynamics

Implements temporal pattern monitoring based on:
- Baker & Cariani (2025). "Time-domain brain: temporal mechanisms for brain
  functions using time-delay nets, holographic processes, radio communications,
  and emergent oscillatory sequences."
  DOI: 10.3389/fncom.2025.1540532

CONCEPT:
Information propagates through sequential oscillatory bands, where each stage
generates new emergent oscillation bands through mixing operations similar to
radio communications (Single-Sideband Carrier Suppressed - SSBCS).

OSCILLATORY CASCADE (for speech/language processing):
1. Gamma (30-100 Hz): Sound/speech detection
2. Beta (13-30 Hz): Acoustic-phonetics
3. Alpha (8-13 Hz): Phone/clusters
4. Theta (4-8 Hz): Syllables
5. Delta (0.5-4 Hz): Words/sentences

This module monitors oscillatory power across bands and detects cross-frequency
coupling, which is essential for:
- Top-down/bottom-up signal regulation
- Temporal binding of features
- Hierarchical information processing

RESEARCH FOUNDATION:
According to PubMed, temporal coding with high temporal resolution enables
holographic-like signal processing, pattern recognition, and associative
content-addressable memory.
"""

import numpy as np
from typing import Dict, List, Tuple
from collections import deque

# Simplified signal processing without scipy dependency
def welch_psd_simple(signal_data, fs, nperseg=256):
    """
    Simplified Welch's method for power spectral density.

    Args:
        signal_data: Time series data
        fs: Sampling frequency
        nperseg: Segment length

    Returns:
        freqs, psd: Frequency bins and power spectral density
    """
    # Use FFT-based PSD estimation
    n = len(signal_data)
    if n < nperseg:
        nperseg = n

    # Compute FFT
    fft_vals = np.fft.rfft(signal_data[:nperseg])
    psd = np.abs(fft_vals) ** 2 / nperseg
    freqs = np.fft.rfftfreq(nperseg, 1/fs)

    return freqs, psd


def butter_bandpass_simple(lowcut, highcut, fs, order=4):
    """
    Simplified bandpass filter (returns filter coefficients).
    For simplicity, we'll just use a window-based approach.
    """
    # Return parameters for filtering
    return {'lowcut': lowcut, 'highcut': highcut, 'fs': fs}


def apply_bandpass(signal_data, lowcut, highcut, fs):
    """
    Apply bandpass filter using FFT method.

    Args:
        signal_data: Time series to filter
        lowcut: Low frequency cutoff
        highcut: High frequency cutoff
        fs: Sampling frequency

    Returns:
        Filtered signal
    """
    # FFT-based filtering
    n = len(signal_data)
    fft_vals = np.fft.rfft(signal_data)
    freqs = np.fft.rfftfreq(n, 1/fs)

    # Create bandpass mask
    mask = (freqs >= lowcut) & (freqs <= highcut)
    fft_vals[~mask] = 0

    # Inverse FFT
    filtered = np.fft.irfft(fft_vals, n)
    return filtered


def hilbert_simple(signal_data):
    """
    Simplified Hilbert transform for analytic signal.

    Returns:
        Analytic signal (complex valued)
    """
    n = len(signal_data)
    fft_vals = np.fft.fft(signal_data)

    # Create Hilbert transform mask
    h = np.zeros(n)
    if n % 2 == 0:
        h[0] = h[n // 2] = 1
        h[1:n // 2] = 2
    else:
        h[0] = 1
        h[1:(n + 1) // 2] = 2

    analytic = np.fft.ifft(fft_vals * h)
    return analytic


class OscillatoryMonitor:
    """
    Monitors oscillatory dynamics in neural activity.

    Tracks power across multiple frequency bands and detects
    cross-frequency coupling indicative of hierarchical processing.
    """

    # Frequency bands (in Hz)
    BANDS = {
        'delta': (0.5, 4.0),
        'theta': (4.0, 8.0),
        'alpha': (8.0, 13.0),
        'beta': (13.0, 30.0),
        'gamma': (30.0, 100.0)
    }

    def __init__(
        self,
        sampling_rate: float = 1000.0,  # Hz (1ms timestep)
        window_size: int = 1000  # samples (1 second window)
    ):
        """
        Initialize oscillatory monitor.

        Args:
            sampling_rate: Sampling frequency in Hz
            window_size: Number of samples for spectral analysis
        """
        self.sampling_rate = sampling_rate
        self.window_size = window_size

        # Activity buffer for spectral analysis
        self.activity_buffer = deque(maxlen=window_size)

        # Current band powers
        self.band_powers = {band: 0.0 for band in self.BANDS.keys()}

        # Cross-frequency coupling metrics
        self.coupling_matrix = np.zeros((len(self.BANDS), len(self.BANDS)))

    def update(self, neural_activity: np.ndarray) -> Dict:
        """
        Update oscillatory metrics with new activity sample.

        Args:
            neural_activity: 1D array of neural activity (e.g., mean firing rate)

        Returns:
            Dict containing oscillatory metrics
        """
        # Add to buffer (take mean if multi-dimensional)
        if neural_activity.ndim > 1:
            activity_scalar = np.mean(neural_activity)
        else:
            activity_scalar = np.mean(neural_activity)

        self.activity_buffer.append(float(activity_scalar))

        # Compute spectral analysis if buffer is full
        if len(self.activity_buffer) >= self.window_size:
            self._compute_band_powers()
            self._compute_cross_frequency_coupling()

        return self.get_metrics()

    def _compute_band_powers(self):
        """
        Compute power in each frequency band using Welch's method.
        """
        if len(self.activity_buffer) < self.window_size:
            return

        # Convert buffer to array
        signal_data = np.array(self.activity_buffer)

        # Compute power spectral density using simplified Welch's method
        freqs, psd = welch_psd_simple(
            signal_data,
            fs=self.sampling_rate,
            nperseg=min(256, len(signal_data))
        )

        # Integrate power in each band
        for band_name, (low_freq, high_freq) in self.BANDS.items():
            # Find frequency indices
            band_mask = (freqs >= low_freq) & (freqs <= high_freq)

            if np.any(band_mask):
                # Integrate power
                band_power = np.trapz(psd[band_mask], freqs[band_mask])
                self.band_powers[band_name] = float(band_power)
            else:
                self.band_powers[band_name] = 0.0

    def _compute_cross_frequency_coupling(self):
        """
        Compute phase-amplitude coupling between frequency bands.

        Measures how the phase of slower oscillations modulates
        the amplitude of faster oscillations.
        """
        if len(self.activity_buffer) < self.window_size:
            return

        signal_data = np.array(self.activity_buffer)
        band_names = list(self.BANDS.keys())

        # Extract bandpass-filtered signals for each band
        filtered_signals = {}
        for band_name, (low_freq, high_freq) in self.BANDS.items():
            # Apply bandpass filter
            filtered = apply_bandpass(
                signal_data,
                low_freq,
                high_freq,
                self.sampling_rate
            )
            filtered_signals[band_name] = filtered

        # Compute coupling between all pairs
        for i, band1 in enumerate(band_names):
            for j, band2 in enumerate(band_names):
                if i == j:
                    self.coupling_matrix[i, j] = 0.0
                    continue

                # Simplified coupling: correlation of instantaneous amplitudes
                sig1 = filtered_signals[band1]
                sig2 = filtered_signals[band2]

                # Hilbert transform to get analytic signals
                analytic1 = hilbert_simple(sig1)
                analytic2 = hilbert_simple(sig2)

                # Instantaneous amplitudes
                amp1 = np.abs(analytic1)
                amp2 = np.abs(analytic2)

                # Correlation
                if np.std(amp1) > 1e-6 and np.std(amp2) > 1e-6:
                    coupling = np.corrcoef(amp1, amp2)[0, 1]
                    self.coupling_matrix[i, j] = abs(coupling)
                else:
                    self.coupling_matrix[i, j] = 0.0

    def get_dominant_oscillation(self) -> Tuple[str, float]:
        """
        Get the dominant oscillatory band.

        Returns:
            Tuple of (band_name, power)
        """
        if not self.band_powers:
            return ("none", 0.0)

        dominant_band = max(self.band_powers.items(), key=lambda x: x[1])
        return dominant_band

    def get_hierarchical_flow_direction(self) -> str:
        """
        Determine if information flow is bottom-up or top-down.

        Bottom-up: Fast (gamma/beta) → Slow (alpha/theta/delta)
        Top-down: Slow (alpha/theta/delta) → Fast (gamma/beta)

        Returns:
            "bottom_up", "top_down", or "balanced"
        """
        # Map bands to indices
        band_names = list(self.BANDS.keys())
        gamma_idx = band_names.index('gamma')
        beta_idx = band_names.index('beta')
        alpha_idx = band_names.index('alpha')
        theta_idx = band_names.index('theta')

        # Check gamma→alpha coupling (bottom-up)
        bottom_up_coupling = self.coupling_matrix[gamma_idx, alpha_idx]

        # Check alpha→gamma coupling (top-down)
        top_down_coupling = self.coupling_matrix[alpha_idx, gamma_idx]

        if bottom_up_coupling > top_down_coupling * 1.2:
            return "bottom_up"
        elif top_down_coupling > bottom_up_coupling * 1.2:
            return "top_down"
        else:
            return "balanced"

    def get_metrics(self) -> Dict:
        """
        Get all oscillatory metrics.

        Returns:
            Dict containing:
                - band_powers: Power in each frequency band
                - dominant_band: (name, power) of strongest oscillation
                - flow_direction: "bottom_up", "top_down", or "balanced"
                - coupling_strength: Mean cross-frequency coupling
        """
        dominant_band, dominant_power = self.get_dominant_oscillation()

        return {
            'band_powers': self.band_powers.copy(),
            'dominant_band': dominant_band,
            'dominant_power': float(dominant_power),
            'flow_direction': self.get_hierarchical_flow_direction(),
            'coupling_strength': float(np.mean(self.coupling_matrix)),
            'gamma_power': self.band_powers.get('gamma', 0.0),
            'beta_power': self.band_powers.get('beta', 0.0),
            'alpha_power': self.band_powers.get('alpha', 0.0),
            'theta_power': self.band_powers.get('theta', 0.0),
            'delta_power': self.band_powers.get('delta', 0.0)
        }

    def reset(self):
        """Reset all tracking metrics."""
        self.activity_buffer.clear()
        self.band_powers = {band: 0.0 for band in self.BANDS.keys()}
        self.coupling_matrix = np.zeros((len(self.BANDS), len(self.BANDS)))
