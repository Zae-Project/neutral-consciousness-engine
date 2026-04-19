# Transmissive Layer: Ambient EM Modulation

## Why this exists

The default Neutral Consciousness Engine cortex is a *productive* model: a
Nengo spiking network that minimises prediction error (Friston free-energy
framing). That is necessary, but the peer-reviewed transmissive-brain
literature argues it is not sufficient. Neurons respond to ambient and
self-generated electromagnetic fields as weak as 0.5 mV/mm via **ephaptic
coupling**, and mammalian θ/α rhythms show measurable phase coherence with
the Earth's Schumann cavity modes at 7.83 Hz and its first four harmonics
(14.3, 20.8, 27.3, 33.8 Hz).

If a synthetic hemisphere only replicates the productive computation, the
Shadow-Mode prediction-error gate (`sync_health ≥ 0.95`) will systematically
over-estimate synthesis with the biological hemisphere: the biological side
is still consuming a transmissive input the synthetic side cannot see. Any
hemispheric switch admitted on productive agreement alone risks dissociation.

This module adds a minimal, mechanistic transmissive channel and a second
convergence criterion — Phase-Locking Value (PLV) — so the gate can only
open when both the productive error AND the transmissive θ-band coherence
are satisfied simultaneously.

## Components

### 1. Ambient EM driver (`cortex_snn/em_driver.py`)

Publishes a deterministic scalar EM waveform on `/environment/em_field_raw`
at 1 kHz (`std_msgs/Float32MultiArray`, one channel). Default mode
`schumann` emits the fundamental + harmonics with decaying weights
`[1.0, 0.5, 0.3, 0.2, 0.15]` at 5 mV peak — two orders of magnitude below
the Neural Firewall's 100 mV over-voltage cap.

Modes:
- `schumann` — Schumann fundamental + four harmonics (default).
- `noise` — zero-mean Gaussian; used as the falsification / null case.
- `silent` — emits 0.0; used for the ephaptic-ablation run.

### 2. Neural Firewall integration (`neural_firewall/traffic_monitor.py`)

The firewall subscribes to `/environment/em_field_raw`, runs the same
amplitude and FFT checks it runs on the satellite neural stream, and
republishes safe samples on `/environment/em_field`. A malicious publisher
spoofing a 200 Hz seizure harmonic or an over-voltage drive is rejected at
the same gate that protects the biological interface — the transmissive
channel cannot become a brainjacking side-door.

### 3. Ephaptic coupling (`cortex_snn/ephaptic_coupling.py`)

Wires the firewall-cleared EM sample into the cortex **only** through a
weak, low-passed scalar field. The field is constructed from (a) the
cortex's own decoded population rate (self-probe, `self_gain = 0.02`) and
(b) the exogenous EM drive (`env_gain = 0.05`), low-passed with a 20 ms
time constant. The resulting scalar is broadcast uniformly as an additive
bias to every representational dimension of the cortex ensemble.

Crucially, the EM signal is never a direct input to the cortex. A direct
input would force PLV to 1.0 and turn the second gate criterion into a
rubber stamp. Forcing the signal through a weak scalar field means
entrainment has to *emerge* from the network's dynamics — which is exactly
the claim the transmissive literature makes about the biology.

This is deliberately a caricature of the biology: real ephaptic effects are
spatially local (nearest-neighbour), not population-wide. A spatial-grid
version would be a straightforward extension if the scalar version proves
too coarse.

### 4. Phase-Locking Value estimator (`cortex_snn/plv_estimator.py`)

Maintains a 4-second rolling buffer of the cortex's decoded scalar rate and
the firewall-cleared EM sample, band-pass filters both at 4–7 Hz (4th-order
Butterworth, zero-phase via `filtfilt`), Hilbert-transforms to recover
instantaneous phase, and computes

    PLV = | mean_t exp(i (φ_cortex(t) − φ_EM(t))) |

(Lachaux et al., 1999). Published on `/transmissive_sync/plv`
(`std_msgs/Float32`) at ~10 Hz. 0 = no phase relation, 1 = perfect lock.

### 5. Dual-criterion hemispheric gate (`tests/split_brain_test.py`)

`/trigger_hemispheric_switch` now requires BOTH

- `sync_health ≥ 0.95` (productive agreement), AND
- `plv ≥ 0.80` (transmissive θ-band coherence)

to be sustained for 3.0 consecutive seconds. The sustained-gate state is
published on `/transmissive_sync/gate_ready` (`std_msgs/Bool`) for
observability. A single instantaneous fluctuation does not open the gate,
and a productive match without transmissive entrainment is rejected, as is
transmissive coincidence without predictive agreement.

## Falsification design

A good second criterion must be possible to fail. The test plan in
`test/test_em_modulation.py` encodes:

- **Trivial-echo** (cortex == EM) must produce PLV ≈ 1.0 (upper bound).
- **Uncorrelated white noise** must produce PLV well below 0.4 (null).
- **Phase-jittered θ-band signal** must produce PLV strictly between the
  noise floor and 1.0 — this is the anti-rubber-stamp test.
- **Ephaptic ablation** (`ephaptic_enabled=false`) must leave the gate
  never able to open, because the environment has no path into the cortex.
- **Spoofed 200 Hz EM driver** must trip the firewall kill switch.

## References

- Rouleau, N. & Cimino, N. (2022). *A Transmissive Theory of Brain
  Function: Implications for Free Will, Consciousness, and Mental Health.*
  NeuroSci 3(3): 440–456.
- Rouleau, N. & Levin, M. (forthcoming). *Multiple Realizability of
  Sentience in Living Systems and Beyond.* eNeuro / Consciousness
  Engineering collection.
- Anastassiou, C. A. et al. (2011). *Ephaptic coupling of cortical
  neurons.* Nature Neuroscience 14: 217–223.
- Lachaux, J.-P. et al. (1999). *Measuring phase synchrony in brain
  signals.* Human Brain Mapping 8: 194–208.
- Pycroft, L. et al. (2016). *Brainjacking: Implant Security Issues in
  Invasive Neuromodulation.* World Neurosurgery 92: 454–462.
