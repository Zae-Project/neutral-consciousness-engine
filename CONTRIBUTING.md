# Contributing to Neutral Consciousness Engine

This guide provides instructions for setting up, building, and contributing to the Neutral Consciousness Engine project.

## Overview

The Neutral Consciousness Engine is a hybrid **ROS 2 + Unity + Python SNN** system implementing the "Generative Model" hypothesis. A Spiking Neural Network (SNN) simulates a 3D reality to predict sensory inputs.

### Architecture

- **ROS 2 (Humble)** backend for SNN logic (Python/C++)
- **Unity 6** frontend for the 3D "Virtual Body" and environment
- **Nengo/BindsNET** for the neuromorphic core
- **Neural Firewall** middleware for security

## Repository Structure

```text
neutral-consciousness-engine/
|
|- .gitignore                  # Combined Unity + ROS 2 + Python gitignore
|- README.md                   # Documentation linking to the hypothesis
|- LICENSE                     # MIT License
|
|- docs/                       # Architecture diagrams & Whitepaper
|   |- whitepaper.md           # The Medical Hypothesis logic
|   |- ARCHITECTURE.md         # Technical architecture details
|
|- ros2_ws/                    # The "Mind" (ROS 2 Workspace)
|   |- src/
|       |- neutral_consciousness/   # Main Python Package
|       |   |- package.xml
|       |   |- setup.py
|       |   |- launch/
|       |   |   |- master_system.launch.py  # Launches Unity + SNN
|       |   |- neutral_consciousness/
|       |       |- __init__.py
|       |       |- cortex_snn/          # The SNN Core
|       |       |   |- visual_cortex.py # Nengo Visual processing
|       |       |   |- dream_engine.py  # Generative Model logic
|       |       |- neural_firewall/     # Security Middleware
|       |           |- traffic_monitor.py
|       |
|       |- ros_tcp_endpoint/   # Unity<->ROS Communication Bridge
|           # (Clone from Unity-Technologies/ROS-TCP-Endpoint)
|
|- unity_project/              # The "Body" (Unity Project)
    |- Assets/
    |   |- RosSettings/        # ROS Connection Config
    |   |- Scripts/
    |   |   |- RosBridgeClient.cs
    |   |- Scenes/
    |       |- VirtualLab.unity
    |- Packages/
    |- ProjectSettings/
```

## Prerequisites

- Git
- Python 3.10+
- ROS 2 Humble
- Unity 6 (for frontend development)
- Nengo library

## Setup

### 1. Clone the Repository

```bash
git clone https://github.com/Zae-Project/neutral-consciousness-engine.git
cd neutral-consciousness-engine
```

### 2. ROS 2 Workspace Setup

```bash
cd ros2_ws
colcon build
source install/setup.bash
```

### 3. Install Python Dependencies

```bash
pip install nengo numpy
```

### 4. Unity Setup (Optional)

For frontend development, install the ROS-TCP-Connector in Unity:
- Open Unity Package Manager
- Add package from git URL:
  `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector`

## Running the System

### Launch the Full System

```bash
ros2 launch neutral_consciousness master_system.launch.py
```

This starts:
1. ROS-TCP Endpoint (Bridge to Unity)
2. Neutral Consciousness Engine (SNN)
3. Optional: Unity build if available

## Core Components

### Visual Cortex (Predictive Coding)

The `visual_cortex.py` implements the Generative Model architecture:

- **Sensory Layer:** Represents raw input from camera/sensors
- **Prediction Layer:** 1000 LIF neurons representing the cortex
- **Error Units:** Fire only when Reality differs from Prediction
- **Learning:** Hebbian learning via PES rule to minimize prediction error

Key equation: `Error = Input - Prediction`

This architecture transmits only prediction errors, minimizing bandwidth for satellite links.

### Neural Firewall (Security)

The `traffic_monitor.py` implements brainjacking defense:

- **Frequency Analysis:** Detects induced gamma synchrony (>150Hz)
- **Voltage Limiting:** Prevents excitotoxicity (>100mV equivalent)
- **Kill Switch:** Physical disconnect upon attack detection

Safety thresholds are based on biological limits (gamma waves rarely exceed 100Hz).

### Encryption Strategy

The system uses a **Hybrid TEE Architecture** to balance security and latency:

- **Identity Handshake:** Homomorphic Encryption for authentication
- **Neural Stream:** AES-256 (<1ms latency) for high-speed spike transmission
- **Neural Firewall:** Packet inspection after decryption

This approach keeps total latency under the 500ms "Libet Buffer" required for seamless consciousness.

## Contributing

### Branch Protection Workflow

1. **Fork** the repository or create a **feature branch**
2. **Make changes** with clear, focused commits
3. **Test thoroughly** before submitting
4. **Submit a pull request** with a clear description
5. **Address review feedback** promptly

### Code Style

- Use clear, readable code with comments
- Include type hints and docstrings
- Follow ROS 2 Python style conventions
- Add meaningful tests for new functionality

### Documentation

When adding features, update:
- Code docstrings
- `docs/ARCHITECTURE.md` for architectural changes
- `docs/whitepaper.md` for hypothesis-related changes

## Scientific Context

This system implements the "Generative Model" of consciousness, where the brain is hypothesized to run a predictive simulation of reality. If prediction error is zero, the system perfectly understands its environment, minimizing the bandwidth required for satellite communication.

For details on the underlying hypothesis, see `docs/whitepaper.md`.

## References

1. Friston, K. (2010). The free-energy principle. *Nature Reviews Neuroscience*.
2. Pycroft, L., et al. (2016). Brainjacking: Implant Security. *World Neurosurgery*.
3. Watanabe, M. (2021). The neutral consciousness framework.

## License

MIT License. See LICENSE file for details.
