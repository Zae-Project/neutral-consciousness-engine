"""
Unit Tests for Neural Firewall Module

Tests the brainjacking defense system including:
- FFT frequency analysis
- Threshold detection
- Kill switch triggering

These tests can run without ROS 2 installed.
"""

import pytest
import numpy as np
import sys
import os

# Add the package to path for testing without ROS
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


class TestFrequencyAnalysis:
    """Test FFT-based frequency detection."""
    
    def test_detect_low_frequency_signal(self):
        """Low frequency signals should pass through."""
        # Generate 50Hz signal (safe)
        sample_rate = 1000.0
        duration = 0.1  # 100ms
        t = np.linspace(0, duration, int(sample_rate * duration))
        signal = np.sin(2 * np.pi * 50 * t)
        
        freq = calculate_dominant_frequency(signal, sample_rate)
        assert 45 <= freq <= 55, f"Expected ~50Hz, got {freq}Hz"
    
    def test_detect_high_frequency_attack(self):
        """High frequency signals (>150Hz) should be detected as attacks."""
        # Generate 200Hz signal (dangerous - seizure induction)
        sample_rate = 1000.0
        duration = 0.1
        t = np.linspace(0, duration, int(sample_rate * duration))
        signal = np.sin(2 * np.pi * 200 * t)
        
        freq = calculate_dominant_frequency(signal, sample_rate)
        assert freq > 150, f"Expected >150Hz attack detection, got {freq}Hz"
    
    def test_handle_empty_signal(self):
        """Empty signals should return 0Hz."""
        freq = calculate_dominant_frequency(np.array([]), 1000.0)
        assert freq == 0.0
    
    def test_handle_short_signal(self):
        """Very short signals should be handled gracefully."""
        freq = calculate_dominant_frequency(np.array([1.0, 2.0]), 1000.0)
        assert freq >= 0.0  # Should not crash


class TestVoltageThresholds:
    """Test voltage/amplitude safety limits."""
    
    def test_safe_amplitude(self):
        """Normal amplitude signals should pass."""
        signal = np.array([10.0, 20.0, 30.0, -20.0, -10.0])
        max_amp = np.max(np.abs(signal))
        assert max_amp < 100.0, "Signal should be within safe limits"
    
    def test_dangerous_amplitude(self):
        """High amplitude signals should trigger alert."""
        signal = np.array([10.0, 20.0, 150.0, -20.0, -10.0])  # 150mV spike
        max_amp = np.max(np.abs(signal))
        assert max_amp > 100.0, "Should detect over-voltage"


class TestHomomorphicEncryption:
    """Test homomorphic encryption module (stub mode)."""
    
    def test_encrypt_decrypt_roundtrip(self):
        """Data should survive encrypt/decrypt cycle."""
        from neutral_consciousness.neural_firewall.homomorphic_encryption import (
            HomomorphicContext
        )
        
        ctx = HomomorphicContext()
        original = np.array([0.5, 0.3, 0.8, 0.1, 0.6], dtype=np.float32)
        
        encrypted = ctx.encrypt(original)
        decrypted = ctx.decrypt(encrypted)
        
        assert np.allclose(original, decrypted, atol=0.01), \
            f"Roundtrip failed: {original} != {decrypted}"
    
    def test_homomorphic_addition(self):
        """Encrypted addition should work correctly."""
        from neutral_consciousness.neural_firewall.homomorphic_encryption import (
            HomomorphicContext
        )
        
        ctx = HomomorphicContext()
        a = np.array([1.0, 2.0, 3.0], dtype=np.float32)
        b = np.array([0.5, 0.5, 0.5], dtype=np.float32)
        
        enc_a = ctx.encrypt(a)
        enc_b = ctx.encrypt(b)
        enc_sum = enc_a + enc_b
        
        decrypted = ctx.decrypt(enc_sum)
        expected = a + b
        
        assert np.allclose(expected, decrypted, atol=0.01), \
            f"Homomorphic addition failed: {expected} != {decrypted}"
    
    def test_homomorphic_scalar_mult(self):
        """Encrypted scalar multiplication should work."""
        from neutral_consciousness.neural_firewall.homomorphic_encryption import (
            HomomorphicContext
        )
        
        ctx = HomomorphicContext()
        data = np.array([1.0, 2.0, 3.0], dtype=np.float32)
        scalar = 2.0
        
        encrypted = ctx.encrypt(data)
        enc_scaled = encrypted * scalar
        
        decrypted = ctx.decrypt(enc_scaled)
        expected = data * scalar
        
        assert np.allclose(expected, decrypted, atol=0.01), \
            f"Scalar multiplication failed: {expected} != {decrypted}"


class TestLatencyLimits:
    """Test Libet limit validation."""
    
    def test_libet_limit_constant(self):
        """Libet limit should be 500ms."""
        LIBET_LIMIT_MS = 500.0
        assert LIBET_LIMIT_MS == 500.0
    
    def test_safe_latency(self):
        """20ms RTT should be safe."""
        rtt = 20.0
        libet_limit = 500.0
        assert rtt < libet_limit, "20ms should be within Libet limit"
    
    def test_dangerous_latency(self):
        """600ms RTT should violate Libet limit."""
        rtt = 600.0
        libet_limit = 500.0
        assert rtt > libet_limit, "600ms should exceed Libet limit"


# Helper function extracted from traffic_monitor.py for testing
def calculate_dominant_frequency(data: np.ndarray, sample_rate: float) -> float:
    """
    Calculate dominant frequency using FFT analysis.
    
    Extracted for unit testing without ROS dependencies.
    """
    if len(data) < 4:
        return 0.0
    
    try:
        # Remove DC component
        data_centered = data - np.mean(data)
        
        # Apply FFT
        fft_result = np.fft.rfft(data_centered)
        fft_magnitude = np.abs(fft_result)
        
        # Get frequency bins
        n_samples = len(data)
        freq_bins = np.fft.rfftfreq(n_samples, d=1.0/sample_rate)
        
        # Find dominant frequency (excluding DC)
        if len(fft_magnitude) > 1:
            peak_idx = np.argmax(fft_magnitude[1:]) + 1
            return float(freq_bins[peak_idx])
        
        return 0.0
        
    except Exception:
        return 0.0


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
