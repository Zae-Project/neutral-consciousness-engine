"""
Cortex SNN Subpackage

Contains the Spiking Neural Network core components:
- visual_cortex: Visual processing with Nengo (Predictive Coding)
- dream_engine: Generative Model logic (Free-running prediction)

Architecture:
- 1000 LIF neurons (Cortex) for prediction
- 500 LIF neurons (Error Units) for surprise signal
- PES learning rule for model adaptation
"""

from .visual_cortex import VisualCortexNode
from .dream_engine import DreamEngine

__all__ = ['VisualCortexNode', 'DreamEngine']
