"""
Cortex SNN Subpackage

Contains the Spiking Neural Network core components:
- visual_cortex: Isomorphic topographic visual processing (8x8 grid, predictive coding)
- dream_engine: Generative Model / Pallium (SPA, slow dynamics, top-down predictions)
- sensory_tectum: Fast multisensory convergence (tectum, Step 1 consciousness)
- limbic_node: Affective valence modulation (dopamine-like learning gating)

Architecture (Feinberg & Mallatt Two-Step Model):
  Step 1 (Tectum): 2,000 LIF neurons, 5ms tau_rc, multisensory convergence
  Step 2 (Pallium): 10,000 LIF neurons, 100ms tau_rc, semantic compression
  Visual Cortex: 3,200 LIF neurons, isomorphic 8x8 topographic grid
  Limbic Node: 500 LIF neurons, VTA + PAG + Habenula affective computation

Consciousness Monitoring (unchanged):
- criticality_monitor: ConCrit branching ratio (Algom & Shriki 2025)
- consciousness_metrics: ACI / Phi / Kappa (Escola-Gascon et al. 2025)
- oscillatory_monitor: Frequency band analysis (Baker & Cariani 2025)
"""

from .visual_cortex import VisualCortexNode
from .dream_engine import DreamEngine
from .sensory_tectum import SensoryTectum
from .limbic_node import LimbicNode

__all__ = ['VisualCortexNode', 'DreamEngine', 'SensoryTectum', 'LimbicNode']
