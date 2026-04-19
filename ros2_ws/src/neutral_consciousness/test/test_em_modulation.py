"""
Unit tests for the transmissive-layer additions:

- Ambient EM driver frequency content (`em_driver.generate_schumann_window`)
- PLV estimator: trivial-echo, noise null, phase-jitter (anti-rubber-stamp),
  not-ready semantics.
- Ephaptic scalar-field block: stability, no self-oscillation under silent
  drive (only when Nengo is available).

Tests load the target modules directly from file paths via importlib to
avoid triggering the top-level `neutral_consciousness` package `__init__`
(which imports rclpy and therefore cannot be imported in a non-ROS
environment).
"""

from __future__ import annotations

import importlib.util
import os
import sys
import types

import numpy as np
import pytest


# Stub out ROS modules so em_driver.py can be imported in a non-ROS environment.
# Only `generate_schumann_window` is used from em_driver in these tests — the
# ROS-dependent `EMDriverNode` is not touched.
def _ensure_ros_stubs() -> None:
    if 'rclpy' not in sys.modules:
        rclpy_stub = types.ModuleType('rclpy')
        rclpy_stub.init = lambda *a, **kw: None  # type: ignore[attr-defined]
        rclpy_stub.shutdown = lambda *a, **kw: None  # type: ignore[attr-defined]
        rclpy_stub.spin = lambda *a, **kw: None  # type: ignore[attr-defined]
        sys.modules['rclpy'] = rclpy_stub

        node_stub = types.ModuleType('rclpy.node')

        class _Node:
            def __init__(self, *a, **kw): pass
            def declare_parameter(self, *a, **kw):
                return types.SimpleNamespace(value=a[1] if len(a) > 1 else None)
            def get_parameter(self, *a, **kw):
                return types.SimpleNamespace(value=None)
            def create_publisher(self, *a, **kw): return types.SimpleNamespace(publish=lambda *x, **y: None)
            def create_subscription(self, *a, **kw): return None
            def create_timer(self, *a, **kw): return None
            def create_service(self, *a, **kw): return None
            def get_logger(self): return types.SimpleNamespace(
                info=lambda *a, **kw: None,
                warn=lambda *a, **kw: None,
                error=lambda *a, **kw: None,
                fatal=lambda *a, **kw: None,
            )
            def destroy_node(self): pass
        node_stub.Node = _Node
        sys.modules['rclpy.node'] = node_stub

    if 'std_msgs' not in sys.modules:
        std = types.ModuleType('std_msgs')
        msg = types.ModuleType('std_msgs.msg')
        class _Msg:
            def __init__(self): self.data = None
        msg.Float32MultiArray = _Msg
        msg.Float32 = _Msg
        msg.Bool = _Msg
        msg.String = _Msg
        sys.modules['std_msgs'] = std
        sys.modules['std_msgs.msg'] = msg


_ensure_ros_stubs()


def _load_module(mod_name: str, rel_path: str):
    here = os.path.dirname(__file__)
    abs_path = os.path.normpath(
        os.path.join(
            here, '..', 'neutral_consciousness', 'cortex_snn', rel_path,
        )
    )
    spec = importlib.util.spec_from_file_location(mod_name, abs_path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[mod_name] = module
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


em_driver = _load_module('em_driver_under_test', 'em_driver.py')
plv_mod = _load_module('plv_estimator_under_test', 'plv_estimator.py')

try:
    import nengo  # noqa: F401
    ephaptic_mod = _load_module('ephaptic_under_test', 'ephaptic_coupling.py')
    NENGO_AVAILABLE = True
except ImportError:
    ephaptic_mod = None
    NENGO_AVAILABLE = False


SAMPLE_RATE_HZ = 1000.0


# ---------------------------------------------------------------------------
# EM driver
# ---------------------------------------------------------------------------

def test_em_driver_frequency_peak_at_schumann_fundamental():
    """FFT of a 10 s Schumann window peaks within ±0.1 Hz of 7.83 Hz."""
    sig = em_driver.generate_schumann_window(
        duration_sec=10.0, sample_rate_hz=SAMPLE_RATE_HZ, amp_mv=5.0
    )
    spectrum = np.abs(np.fft.rfft(sig - np.mean(sig)))
    freqs = np.fft.rfftfreq(len(sig), d=1.0 / SAMPLE_RATE_HZ)
    peak_freq = freqs[np.argmax(spectrum)]
    assert abs(peak_freq - em_driver.SCHUMANN_HARMONICS_HZ[0]) < 0.1, (
        f"dominant peak {peak_freq:.2f} Hz is not the Schumann fundamental"
    )


def test_em_driver_amplitude_well_below_firewall_cap():
    sig = em_driver.generate_schumann_window(
        duration_sec=2.0, sample_rate_hz=SAMPLE_RATE_HZ, amp_mv=5.0
    )
    assert np.max(np.abs(sig)) < 100.0, "amplitude must stay below firewall cap"


# ---------------------------------------------------------------------------
# PLV estimator
# ---------------------------------------------------------------------------

@pytest.mark.skipif(not plv_mod.SCIPY_AVAILABLE, reason="scipy unavailable")
def test_plv_trivial_echo_is_nearly_one():
    """If cortex == EM, PLV must be ≈ 1.0 (upper-bound sanity check)."""
    plv = plv_mod.PLVEstimator(sample_rate_hz=100.0, window_sec=4.0)
    t = np.arange(0, 8.0, 1.0 / 100.0)
    ref = np.sin(2 * np.pi * 6.0 * t)
    for v in ref:
        plv.push(v, v)
    value = plv.compute()
    assert value > 0.98, f"trivial-echo PLV should be ≈ 1.0, got {value:.3f}"


@pytest.mark.skipif(not plv_mod.SCIPY_AVAILABLE, reason="scipy unavailable")
def test_plv_under_white_noise_is_below_gate_threshold():
    """Uncorrelated noise streams must yield PLV below the 0.80 gate
    threshold. The noise floor for a 4-s window at 100 Hz band-passed to
    4–7 Hz is roughly 0.3–0.5 (only ~16–28 cycles in the passband), so we
    check against the operational threshold rather than an absolute zero."""
    plv = plv_mod.PLVEstimator(sample_rate_hz=100.0, window_sec=4.0)
    rng = np.random.default_rng(0)
    for _ in range(800):
        plv.push(rng.normal(), rng.normal())
    value = plv.compute()
    assert value < 0.8, f"noise PLV must stay below the 0.8 gate, got {value:.3f}"


@pytest.mark.skipif(not plv_mod.SCIPY_AVAILABLE, reason="scipy unavailable")
def test_plv_phase_jittered_is_intermediate():
    """Phase-jittered θ-band signal must produce a PLV strictly between
    the noise floor and 1.0 — anti-rubber-stamp guard."""
    plv = plv_mod.PLVEstimator(sample_rate_hz=100.0, window_sec=4.0)
    t = np.arange(0, 8.0, 1.0 / 100.0)
    rng = np.random.default_rng(1)
    ref = np.sin(2 * np.pi * 6.0 * t)
    jitter = 0.8 * np.sin(2 * np.pi * 0.5 * t + rng.uniform(0, 2 * np.pi))
    noisy = np.sin(2 * np.pi * 6.0 * t + jitter)
    for r, n in zip(ref, noisy):
        plv.push(n, r)
    value = plv.compute()
    assert 0.1 < value < 0.98, (
        f"phase-jittered PLV should be intermediate, got {value:.3f}"
    )


def test_plv_not_ready_returns_zero():
    plv = plv_mod.PLVEstimator(sample_rate_hz=100.0, window_sec=4.0)
    plv.push(0.1, 0.2)
    assert plv.compute() == 0.0


# ---------------------------------------------------------------------------
# Ephaptic coupling (Nengo-only)
# ---------------------------------------------------------------------------

@pytest.mark.skipif(not NENGO_AVAILABLE, reason="nengo unavailable")
def test_ephaptic_block_builds_and_steps():
    """Build a tiny cortex + ephaptic block and step for 500 ms — no NaNs,
    and the cortex stays bounded."""
    import nengo as ng

    state = ephaptic_mod.EphapticState()
    state.env_value = 0.0

    net = ng.Network()
    with net:
        cortex = ng.Ensemble(n_neurons=100, dimensions=4)
        ng.Connection(cortex, cortex, synapse=0.05)
        ephaptic_mod.build_ephaptic_field(cortex, 4, state)
        probe = ng.Probe(cortex, synapse=0.01)

    with ng.Simulator(net, dt=0.001, progress_bar=False) as sim:
        sim.run(0.5)

    data = sim.data[probe]
    assert not np.any(np.isnan(data)), "cortex produced NaNs"
    assert np.max(np.abs(data)) < 5.0, (
        "cortex representation exploded under ephaptic drive"
    )


@pytest.mark.skipif(not NENGO_AVAILABLE, reason="nengo unavailable")
def test_ephaptic_no_self_oscillation_when_env_silent():
    """Silent environment + no sensory input ⇒ cortex must stay bounded
    (no runaway self-oscillation). Nengo ensembles have non-zero baseline
    decoding even with zero input due to random tuning curves, so the
    assertion is boundedness (within the default radius=1.0 envelope),
    not decay to zero."""
    import nengo as ng

    state = ephaptic_mod.EphapticState()
    state.env_value = 0.0

    net = ng.Network()
    with net:
        cortex = ng.Ensemble(n_neurons=100, dimensions=4)
        ng.Connection(cortex, cortex, synapse=0.05)
        ephaptic_mod.build_ephaptic_field(cortex, 4, state)
        probe = ng.Probe(cortex, synapse=0.05)

    with ng.Simulator(net, dt=0.001, progress_bar=False) as sim:
        sim.run(2.0)

    tail = sim.data[probe][-500:]
    assert np.max(np.abs(tail)) < 1.5, (
        "cortex self-oscillated under silent env; self_gain likely too high"
    )
    # And no NaNs / blow-up.
    assert not np.any(np.isnan(tail)), "cortex produced NaNs under silent env"
