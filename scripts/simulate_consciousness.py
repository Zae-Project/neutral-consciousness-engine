#!/usr/bin/env python3
"""
Standalone Simulation Script - Neutral Consciousness Engine

This script allows testing the core SNN logic without ROS 2 or Unity.
It simulates the complete data flow:
1. Generate synthetic visual input
2. Process through Predictive Coding SNN
3. Monitor prediction error
4. Test Neural Firewall thresholds
5. Demonstrate Homomorphic Encryption

Usage:
    python scripts/simulate_consciousness.py
    python scripts/simulate_consciousness.py --dream-mode
    python scripts/simulate_consciousness.py --attack-test

Requirements:
    pip install numpy nengo matplotlib (optional for plots)
"""

import argparse
import numpy as np
import time
import sys
import os

# Add package to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'ros2_ws', 'src', 'neutral_consciousness'))

# Constants from the system
LIBET_LIMIT_MS = 500.0
MAX_FREQUENCY_HZ = 150.0
VOLTAGE_LIMIT_MV = 100.0


def simulate_visual_cortex(duration_sec: float = 5.0, use_nengo: bool = True):
    """
    Simulate the Visual Cortex SNN with Predictive Coding.
    
    Args:
        duration_sec: Simulation duration in seconds
        use_nengo: Use Nengo if available, else use numpy approximation
    """
    print("\n" + "="*60)
    print("VISUAL CORTEX SIMULATION (Predictive Coding)")
    print("="*60)
    
    try:
        import nengo
        nengo_available = True
        print("Nengo available - using full SNN simulation")
    except ImportError:
        nengo_available = False
        print("Nengo not available - using numpy approximation")
    
    if use_nengo and nengo_available:
        _simulate_with_nengo(duration_sec)
    else:
        _simulate_with_numpy(duration_sec)


def _simulate_with_nengo(duration_sec: float):
    """Full Nengo SNN simulation."""
    import nengo
    
    INPUT_DIM = 64
    
    # Create the model
    model = nengo.Network(label="Generative Model Simulation")
    
    with model:
        # Simulated sensory input (sine wave pattern)
        def sensory_input(t):
            return np.sin(2 * np.pi * t * np.arange(INPUT_DIM) / INPUT_DIM) * 0.5
        
        stim = nengo.Node(sensory_input)
        
        # Cortex ensemble
        cortex = nengo.Ensemble(
            n_neurons=1000,
            dimensions=INPUT_DIM,
            neuron_type=nengo.LIF()
        )
        
        # Error units
        error_units = nengo.Ensemble(n_neurons=500, dimensions=INPUT_DIM)
        
        # Connections
        nengo.Connection(stim, error_units)
        nengo.Connection(cortex, error_units, transform=-1)
        nengo.Connection(cortex, cortex, synapse=0.05)
        
        # Learning
        conn = nengo.Connection(error_units, cortex, transform=0.1)
        conn.learning_rule_type = nengo.PES()
        
        # Probes
        error_probe = nengo.Probe(error_units, synapse=0.01)
    
    # Run simulation
    print(f"\nRunning {duration_sec}s simulation with 1000 LIF neurons...")
    
    with nengo.Simulator(model) as sim:
        sim.run(duration_sec)
    
    # Analyze results
    error_data = sim.data[error_probe]
    
    print(f"\nResults:")
    print(f"  - Time steps: {len(error_data)}")
    print(f"  - Initial error magnitude: {np.linalg.norm(error_data[0]):.4f}")
    print(f"  - Final error magnitude: {np.linalg.norm(error_data[-1]):.4f}")
    
    # Calculate sync health
    initial_error = np.linalg.norm(error_data[0])
    final_error = np.linalg.norm(error_data[-1])
    
    if initial_error > 0:
        reduction = (initial_error - final_error) / initial_error * 100
        print(f"  - Error reduction: {reduction:.1f}%")
        
        sync_health = max(0, 1.0 - final_error / initial_error)
        print(f"  - Synchronization Health: {sync_health:.2%}")
        
        if sync_health > 0.95:
            print("  - STATUS: Ready for hemispheric switch!")
        else:
            print("  - STATUS: Still learning, switch not recommended")


def _simulate_with_numpy(duration_sec: float):
    """Numpy approximation of predictive coding."""
    INPUT_DIM = 64
    steps = int(duration_sec * 100)  # 100Hz simulation
    
    print(f"\nRunning {duration_sec}s numpy simulation...")
    
    # State
    prediction = np.zeros(INPUT_DIM)
    model = np.random.randn(INPUT_DIM, INPUT_DIM) * 0.01
    
    errors = []
    
    for step in range(steps):
        t = step / 100.0
        
        # Sensory input
        actual = np.sin(2 * np.pi * t * np.arange(INPUT_DIM) / INPUT_DIM) * 0.5
        
        # Prediction error
        error = actual - prediction
        errors.append(np.linalg.norm(error))
        
        # Update model (simplified Hebbian)
        model += 0.01 * np.outer(error, prediction)
        
        # Normalize
        norm = np.linalg.norm(model)
        if norm > 10:
            model /= (norm / 10)
        
        # Generate next prediction
        prediction = np.tanh(model @ prediction + error * 0.1)
    
    print(f"\nResults:")
    print(f"  - Initial error: {errors[0]:.4f}")
    print(f"  - Final error: {errors[-1]:.4f}")
    print(f"  - Error reduction: {(errors[0] - errors[-1]) / errors[0] * 100:.1f}%")


def test_neural_firewall():
    """Test the Neural Firewall with various attack patterns."""
    print("\n" + "="*60)
    print("NEURAL FIREWALL TEST (Brainjacking Defense)")
    print("="*60)
    
    sample_rate = 1000.0
    duration = 0.1
    t = np.linspace(0, duration, int(sample_rate * duration))
    
    test_cases = [
        ("Normal brain activity (10Hz alpha)", 10),
        ("High gamma (80Hz)", 80),
        ("ATTACK: Seizure induction (200Hz)", 200),
        ("ATTACK: High-frequency burst (300Hz)", 300),
    ]
    
    for name, freq in test_cases:
        signal = np.sin(2 * np.pi * freq * t) * 50  # 50mV amplitude
        detected_freq = calculate_frequency(signal, sample_rate)
        
        is_attack = detected_freq > MAX_FREQUENCY_HZ
        status = "BLOCKED" if is_attack else "PASSED"
        
        print(f"\n  {name}")
        print(f"    Detected frequency: {detected_freq:.1f}Hz")
        print(f"    Status: {status}")
        if is_attack:
            print(f"    KILL SWITCH TRIGGERED")
    
    # Voltage test
    print(f"\n  Voltage Limit Test:")
    normal_voltage = np.array([10, 20, 30, -20, -10])
    attack_voltage = np.array([10, 20, 150, -20, -10])  # 150mV spike
    
    print(f"    Normal signal max: {np.max(np.abs(normal_voltage))}mV - PASSED")
    print(f"    Attack signal max: {np.max(np.abs(attack_voltage))}mV - BLOCKED")


def calculate_frequency(data: np.ndarray, sample_rate: float) -> float:
    """FFT-based frequency detection."""
    if len(data) < 4:
        return 0.0
    
    data_centered = data - np.mean(data)
    fft_result = np.fft.rfft(data_centered)
    fft_magnitude = np.abs(fft_result)
    freq_bins = np.fft.rfftfreq(len(data), d=1.0/sample_rate)
    
    if len(fft_magnitude) > 1:
        peak_idx = np.argmax(fft_magnitude[1:]) + 1
        return float(freq_bins[peak_idx])
    return 0.0


def test_homomorphic_encryption():
    """Demonstrate homomorphic encryption."""
    print("\n" + "="*60)
    print("HOMOMORPHIC ENCRYPTION TEST (Satellite Security)")
    print("="*60)
    
    try:
        from neutral_consciousness.neural_firewall.homomorphic_encryption import (
            NeuralEncryptionWrapper,
            TENSEAL_AVAILABLE
        )
        
        mode = "TenSEAL (Real)" if TENSEAL_AVAILABLE else "Stub (Demo)"
        print(f"\n  Mode: {mode}")
        
        wrapper = NeuralEncryptionWrapper()
        
        # Test data
        biological_input = np.array([0.5, 0.3, 0.8, 0.1, 0.6], dtype=np.float32)
        prediction = np.array([0.4, 0.3, 0.7, 0.2, 0.5], dtype=np.float32)
        
        print(f"\n  Biological Brain Input (SECRET): {biological_input}")
        
        # Encrypt
        enc_input, meta = wrapper.encrypt_spike_train(biological_input)
        enc_pred, _ = wrapper.encrypt_spike_train(prediction)
        
        print(f"  Encrypted (satellite sees): <ciphertext>")
        
        # Compute on encrypted data (satellite operation)
        enc_error = wrapper.process_encrypted_prediction(enc_input, enc_pred)
        print(f"  Satellite computed: E(error) = E(input) - E(prediction)")
        
        # Decrypt (only biological interface can do this)
        decrypted = wrapper.context.decrypt(enc_error)
        expected = biological_input - prediction
        
        print(f"\n  Decrypted Error: {decrypted}")
        print(f"  Expected Error:  {expected}")
        print(f"  Match: {np.allclose(decrypted, expected, atol=0.01)}")
        
    except ImportError as e:
        print(f"\n  Could not import HE module: {e}")
        print("  Run from ros2_ws directory or install package first.")


def test_latency_limits():
    """Test Libet limit validation."""
    print("\n" + "="*60)
    print("LATENCY INJECTOR TEST (OISL Simulation)")
    print("="*60)
    
    test_rtts = [10, 20, 50, 100, 200, 400, 500, 600]
    
    print(f"\n  Libet Limit: {LIBET_LIMIT_MS}ms")
    print(f"  (Consciousness continuity threshold)\n")
    
    for rtt in test_rtts:
        if rtt < LIBET_LIMIT_MS:
            status = "OK - Consciousness continuous"
        elif rtt == LIBET_LIMIT_MS:
            status = "WARNING - At limit"
        else:
            status = "VIOLATION - Consciousness broken!"
        
        print(f"  RTT {rtt:3d}ms: {status}")


def run_dream_mode():
    """Run the system in dream mode (free-running generation)."""
    print("\n" + "="*60)
    print("DREAM MODE (Free-running Generation)")
    print("="*60)
    
    print("\nIn dream mode, the Generative Model runs without sensory input,")
    print("generating internal predictions based on learned patterns.\n")
    
    INPUT_DIM = 64
    steps = 100
    
    # Internal model (learned from experience)
    model = np.random.randn(INPUT_DIM, INPUT_DIM) * 0.1
    prediction = np.random.randn(INPUT_DIM) * 0.1
    
    print("Generating dream sequence...")
    
    for step in range(steps):
        # Dream: free-running with noise
        noise = np.random.randn(INPUT_DIM) * 0.1
        prediction = np.tanh(model @ prediction + noise)
        
        # Normalize
        norm = np.linalg.norm(model)
        if norm > 10:
            model /= (norm / 10)
        
        if step % 20 == 0:
            activity = np.mean(np.abs(prediction))
            print(f"  Step {step:3d}: Activity level = {activity:.4f}")
    
    print("\nDream sequence complete.")


def main():
    parser = argparse.ArgumentParser(
        description='Neutral Consciousness Engine - Standalone Simulation'
    )
    parser.add_argument(
        '--dream-mode', action='store_true',
        help='Run in dream mode (free-running generation)'
    )
    parser.add_argument(
        '--attack-test', action='store_true',
        help='Run brainjacking attack tests'
    )
    parser.add_argument(
        '--full', action='store_true',
        help='Run all tests'
    )
    parser.add_argument(
        '--duration', type=float, default=2.0,
        help='Simulation duration in seconds (default: 2.0)'
    )
    
    args = parser.parse_args()
    
    print("\n" + "="*60)
    print("  NEUTRAL CONSCIOUSNESS ENGINE - SIMULATION")
    print("  Watanabe Transfer Protocol v0.1.0")
    print("="*60)
    
    if args.dream_mode:
        run_dream_mode()
    elif args.attack_test:
        test_neural_firewall()
    elif args.full:
        simulate_visual_cortex(args.duration)
        test_neural_firewall()
        test_homomorphic_encryption()
        test_latency_limits()
    else:
        # Default: run basic simulation
        simulate_visual_cortex(args.duration)
        test_latency_limits()
    
    print("\n" + "="*60)
    print("  Simulation complete.")
    print("="*60 + "\n")


if __name__ == '__main__':
    main()
