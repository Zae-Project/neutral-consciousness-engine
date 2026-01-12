"""
Neural Firewall Subpackage

Security middleware for monitoring neural traffic and preventing
unauthorized access or "brainjacking" during consciousness operations.

Modules:
- traffic_monitor: Real-time FFT analysis, spike rate monitoring, kill switch
- homomorphic_encryption: CKKS-based encrypted neural processing for satellite link
- latency_injector: OISL delay simulation with Libet limit validation

Safety Features:
- Frequency analysis (seizure prevention: <150Hz)
- Voltage limiting (excitotoxicity prevention: <100mV)
- Kill switch with heartbeat for Unity disconnect
- Homomorphic encryption for satellite data protection

References:
- Pycroft et al. (2016) - Brainjacking defenses
- Nguyen et al. (2025) - Hybrid TEE for BCI
"""

from .traffic_monitor import NeuralFirewall
from .homomorphic_encryption import (
    HomomorphicContext,
    NeuralEncryptionWrapper,
    EncryptedVector,
    TENSEAL_AVAILABLE
)
from .latency_injector import LatencyInjector

__all__ = [
    'NeuralFirewall',
    'HomomorphicContext', 
    'NeuralEncryptionWrapper',
    'EncryptedVector',
    'LatencyInjector',
    'TENSEAL_AVAILABLE'
]
