"""
Ephaptic coupling block for the cortex ensemble.

Motivation: the peer-reviewed transmissive-brain-function literature
(Rouleau & Cimino, NeuroSci 3:440-456, 2022; Anastassiou et al., 2011)
reports that neurons respond to ambient and self-generated EM fields as
weak as 0.5 mV/mm. This block simulates that mechanism at the population
level: the cortex ensemble's own rate produces a scalar "self-field" that
is summed with a firewall-cleared exogenous EM sample, low-pass filtered,
and fed back as a uniform additive bias to every neuron in the ensemble.

This is intentionally a caricature: real ephaptic effects are local (nearest
neighbours) and spatially structured, not population-wide. The scalar-field
version is what is needed to (a) entrain the ensemble, (b) keep the loop
stable, and (c) produce a meaningful PLV — a direct input would force the
phase-locking value to 1.0 and turn the second convergence criterion into a
rubber stamp.
"""

import numpy as np

try:
    import nengo
    NENGO_AVAILABLE = True
except ImportError:
    NENGO_AVAILABLE = False


# Scalar-field gain defaults. Interpreted in the cortex's normalised
# representation space, not in absolute mV. The `0.5 mV/mm-equivalent`
# language in the whitepaper refers to the biological floor this tries
# to imitate, not the literal Nengo value.
DEFAULT_SELF_GAIN = 0.02   # recurrent population feedback; must be << 1
DEFAULT_ENV_GAIN = 0.05    # exogenous EM drive
DEFAULT_FIELD_TAU = 0.02   # 20 ms low-pass on the combined field


class EphapticState:
    """Mutable container for the current EM-field sample.

    `visual_cortex.py` updates `env_value` whenever a new firewall-cleared
    `/environment/em_field` message arrives; the Nengo Node reads it on
    each simulator step.
    """

    def __init__(self) -> None:
        self.env_value: float = 0.0
        self.enabled: bool = True

    def read(self) -> float:
        return self.env_value if self.enabled else 0.0


def build_ephaptic_field(
    cortex,
    n_dimensions: int,
    state: EphapticState,
    self_gain: float = DEFAULT_SELF_GAIN,
    env_gain: float = DEFAULT_ENV_GAIN,
    tau: float = DEFAULT_FIELD_TAU,
):
    """Wire an ephaptic-style scalar field into an existing cortex ensemble.

    Must be called inside a `with nengo.Network():` block. The ensemble
    `cortex` must already exist.

    Args:
        cortex: the `nengo.Ensemble` to modulate.
        n_dimensions: the representational dimension of the ensemble.
        state: shared `EphapticState` whose `env_value` is updated from
            the firewall-cleared EM topic by the containing node.
        self_gain: recurrent self-field gain; keep well below 1.0 to
            avoid self-oscillation.
        env_gain: exogenous EM drive gain.
        tau: low-pass time constant on the combined field, in seconds.

    Returns:
        the `nengo.Node` exposing the scalar field (useful for probing).
    """
    if not NENGO_AVAILABLE:
        raise RuntimeError("Nengo unavailable; cannot build ephaptic field")

    # Environmental field node: reads the latest firewall-cleared sample.
    env_node = nengo.Node(output=lambda t: state.read())

    # Population rate proxy: decode the cortex's own mean activation.
    # `dimensions=1` with `transform=[[1/n_dim]*n_dim]` averages the
    # representational state into a single scalar self-field.
    self_probe = nengo.Ensemble(n_neurons=50, dimensions=1)
    nengo.Connection(
        cortex,
        self_probe,
        transform=[[1.0 / n_dimensions] * n_dimensions],
        synapse=tau,
    )

    # Combined scalar field node.
    field_node = nengo.Node(size_in=1, label="ephaptic_field")
    nengo.Connection(env_node, field_node, transform=[[env_gain]], synapse=tau)
    nengo.Connection(self_probe, field_node, transform=[[self_gain]], synapse=tau)

    # Broadcast the scalar uniformly to every representational dimension
    # of the cortex. Additive bias — modulates excitability, does not
    # substitute for sensory input.
    broadcast = [[1.0]] * n_dimensions
    nengo.Connection(field_node, cortex, transform=broadcast, synapse=tau)

    return field_node
