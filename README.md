# The Neutral Consciousness Engine

**Consciousness Substrate Transfer via Spiking Neural Networks**

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/)
[![Python 3.10+](https://img.shields.io/badge/Python-3.10%2B-blue)](https://www.python.org/)
[![Zae Project](https://img.shields.io/badge/Zae-Project-orange)](https://github.com/Zae-Project)

---

## Table of Contents

- [Overview](#overview)
- [Scientific Foundation](#scientific-foundation)
- [Core Architecture](#core-architecture)
- [System Components](#system-components)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Usage Examples](#usage-examples)
- [ROS 2 Topics & Services](#ros-2-topics--services)
- [Documentation](#documentation)
- [Research Resources](#research-resources)
- [Contributing](#contributing)
- [License](#license)

---

## Overview

This repository implements the **"Generative Model"** for the Watanabe Transfer Protocol - a theoretical framework for seamless consciousness substrate transfer. It serves as the **"Neutral Consciousness"** synthetic hemisphere required for hemisphere integration.

### What is the Watanabe Transfer Protocol?

A theoretical framework for interfacing biological neural networks with artificial substrates, ensuring continuity of consciousness during transfer. Based on the principle that consciousness can integrate between biological and synthetic hemispheres through a corpus callosum-like interface.

**IMPORTANT DISCLAIMER:** This project is **inspired by** and builds upon the research approach of Professor Masataka Watanabe, particularly his work on interhemispheric transfer in split-brain patients. **This is NOT his official work.** We are independent researchers exploring consciousness substrate transfer based on principles from his publications.

### Project Goals

1. **Scientific Exploration**: Investigate whether consciousness can operate on synthetic substrates
2. **Predictive Coding**: Implement generative models that minimize bandwidth via prediction error
3. **Security**: Ensure neural data cannot be hijacked or compromised
4. **Validation**: Create falsifiable tests for the hemisphere integration hypothesis

---

## Scientific Foundation

This project builds upon established neuroscience and consciousness research:

### Primary Inspiration

**Watanabe, M., et al. (2014)**  
*"Interhemispheric transfer of visual information in split-brain patients"*  
Neuropsychologia, 63, 133-142  
DOI: [10.1016/j.neuropsychologia.2014.08.025](https://doi.org/10.1016/j.neuropsychologia.2014.08.025)  
**Relevance**: Empirical basis for corpus callosum interface and validation protocol

**Watanabe's Neutral Consciousness Framework** (referenced in project documentation)  
**Relevance**: Theoretical foundation for substrate-independent consciousness

### Supporting Research

**Split-Brain Research:**
- **Sperry, R.W. (1968).** "Hemisphere deconnection and unity in conscious awareness." *American Psychologist*, 23(10), 723-733.
- **Gazzaniga, M.S. (2005).** "Forty-five years of split-brain research." *Nature Reviews Neuroscience*, 6(8), 653-659.

**Consciousness Timing:**
- **Libet, B., et al. (1983).** "Time of conscious intention to act." *Brain*, 106(3), 623-642.
- **Schurger, A., et al. (2012).** "An accumulator model for spontaneous neural activity." *PNAS*, 109(42), E2904-E2913.

**Predictive Coding:**
- **Friston, K. (2010).** "The free-energy principle: a unified brain theory?" *Nature Reviews Neuroscience*, 11(2), 127-138.
- **Rao, R.P., & Ballard, D.H. (1999).** "Predictive coding in the visual cortex." *Nature Neuroscience*, 2(1), 79-87.

**Neuromorphic Computing:**
- **Eliasmith, C., et al. (2012).** "A large-scale model of the functioning brain." *Science*, 338(6111), 1202-1205.
- **Davies, M., et al. (2018).** "Loihi: A neuromorphic manycore processor." *IEEE Micro*, 38(1), 82-99.

**Security:**
- **Pycroft, L., et al. (2016).** "Brainjacking: Implant security issues in invasive neuromodulation." *World Neurosurgery*, 92, 454-462.

### Comprehensive Bibliography

For a complete list of 100+ researchers, 50+ papers, and 35+ books, see:
- **[Zae Project Bibliography](https://github.com/Zae-Project/zae-docs/blob/main/reference/bibliography.md)**
- **[Researchers Directory](https://github.com/Zae-Project/zae-docs/blob/main/reference/researchers-directory.md)**

---

## Core Architecture

### The Mind-Body System

```
┌─────────────────────────────────────────────────────────────────┐
│                   NEUTRAL CONSCIOUSNESS ENGINE                   │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌───────────────────┐          ┌───────────────────┐           │
│  │    THE BODY       │          │     THE MIND      │           │
│  │   (Unity 6)       │◄────────►│   (ROS 2/Nengo)   │           │
│  │   Digital Twin    │  TCP/IP  │   Spiking Neural  │           │
│  │                   │  Bridge  │     Network       │           │
│  └───────────────────┘          └───────────────────┘           │
│          │                              │                        │
│          │                              ▼                        │
│          │                     ┌───────────────────┐            │
│          │                     │ Neural Firewall   │            │
│          │                     │  (Hybrid TEE)     │            │
│          │                     └───────────────────┘            │
│          │                              │                        │
│          ▼                              ▼                        │
│  ┌────────────────────────────────────────────────┐             │
│  │         ROS 2 Topic Infrastructure             │             │
│  └────────────────────────────────────────────────┘             │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### Four Core Pillars

1. **The Brain (Generative Model)**
   - Spiking Neural Network built with Nengo
   - Implements **Predictive Coding**: predicts sensory input, only transmits prediction error
   - Minimizes bandwidth for OISL (Optical Inter-Satellite Link) compatibility
   - Based on Free Energy Principle (Friston, 2010)

2. **The Body (Digital Twin)**
   - Physics-compliant avatar in Unity 6
   - Serves as somatosensory anchor
   - Provides sensory feedback to the synthetic hemisphere

3. **The Bridge (Corpus Callosum Interface)**
   - Low-latency TCP/IP communication
   - Simulates biological corpus callosum
   - ROS-TCP-Endpoint for Unity ↔ ROS 2 integration

4. **The Guardian (Neural Firewall)**
   - **Hybrid TEE** (Trusted Execution Environment)
   - AES-256 for real-time neural streams
   - Homomorphic Encryption for identity handshakes
   - Active "Brainjacking" defenses (Pycroft et al., 2016)

---

## System Components

### ROS 2 Nodes

| Node | Executable | Purpose | Key Topics |
|------|-----------|---------|------------|
| **Visual Cortex** | `cortex_node` | Predictive coding SNN | `/neural_data/prediction_error`, `/synchronization_health` |
| **Dream Engine** | `dream_node` | Generative model | `dream/prediction`, `dream/prediction_error` |
| **Neural Firewall** | `firewall_node` | Brainjacking defense | `firewall/kill_switch`, `/verified_neural_stream` |
| **Latency Injector** | `latency_injector_node` | OISL delay simulation | `/neural_stream/delayed` |
| **Homomorphic Encryption** | `he_node` | Satellite security | `/satellite_uplink/encrypted` |
| **Split Brain Test** | `split_brain_node` | Uni-hemispheric protocol | `/conscious_output/unified_field` |

### Unity Components

| Component | Purpose |
|-----------|---------|
| **RosBridgeClient.cs** | Publishes camera feed, receives kill switch |
| **VirtualCamera** | Provides visual sensory input |
| **AvatarController** | Digital twin body representation |

---

## Installation

### Prerequisites

**Required:**
- **Operating System**: Windows 10/11, Ubuntu 22.04, or macOS
- **ROS 2**: Humble Hawksbill (LTS)
- **Python**: 3.10 or higher
- **Unity**: Unity 6 (or Unity 2022 LTS)

**Optional:**
- **Nengo**: For SNN simulation (installed via pip)
- **TenSEAL**: For real homomorphic encryption (currently using mock)

### Step-by-Step Installation

#### 1. Install ROS 2 Humble

**Windows:**
See detailed guide: [docs/ROS2_INSTALLATION_WINDOWS.md](docs/ROS2_INSTALLATION_WINDOWS.md)

**Linux (Ubuntu 22.04):**
```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 apt repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble
sudo apt update
sudo apt install ros-humble-desktop python3-colcon-common-extensions
```

#### 2. Clone the Repository

```bash
git clone https://github.com/Zae-Project/neutral-consciousness-engine.git
cd neutral-consciousness-engine
```

#### 3. Install Python Dependencies

**Using virtual environment (recommended):**
```bash
python3 -m venv venv
source venv/bin/activate  # Linux/Mac
# or: venv\Scripts\activate  # Windows

pip install -r requirements.txt
```

#### 4. Clone ROS-TCP-Endpoint

```bash
cd ros2_ws/src
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
cd ../..
```

#### 5. Build the ROS 2 Workspace

```bash
cd ros2_ws
colcon build --packages-select neutral_consciousness
source install/setup.bash  # Linux/Mac
# or: call install\setup.bat  # Windows
```

#### 6. Install Unity (Optional for testing)

Download Unity Hub: https://unity.com/download

Install Unity 6 or Unity 2022 LTS through Unity Hub

---

## Quick Start

### 1. Test Individual Node

```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash  # Linux
# or: call C:\dev\ros2_humble\ros2-windows\local_setup.bat  # Windows

# Source workspace
cd ros2_ws
source install/setup.bash

# Run a single node
ros2 run neutral_consciousness cortex_node
```

Expected output:
```
[INFO] [visual_cortex_snn]: Generative Model (Predictive Coding) Initialized.
```

### 2. Launch Full System

```bash
ros2 launch neutral_consciousness master_system.launch.py
```

**With custom parameters:**
```bash
ros2 launch neutral_consciousness master_system.launch.py \
    round_trip_time_ms:=50.0 \
    dream_mode:=true \
    encryption_enabled:=true
```

### 3. Verify Topic Communication

```bash
# List all topics
ros2 topic list

# Monitor prediction error
ros2 topic echo /neural_data/prediction_error

# Check synchronization health
ros2 topic echo /synchronization_health
```

---

## Usage Examples

### Example 1: Running Visual Cortex Alone

```bash
# Terminal 1: Launch cortex
ros2 run neutral_consciousness cortex_node

# Terminal 2: Publish test image (mock)
ros2 topic pub /unity/camera/raw sensor_msgs/msg/Image \
    "header: {frame_id: 'camera'}, height: 480, width: 640, encoding: 'rgb8', data: []"

# Terminal 3: Monitor output
ros2 topic echo /neural_data/prediction_error
```

### Example 2: Testing Dream Mode

```bash
# Terminal 1: Launch dream engine
ros2 run neutral_consciousness dream_node --ros-args -p dream_mode:=true

# Terminal 2: Watch predictions
ros2 topic echo /dream/prediction

# Terminal 3: Toggle dream mode
ros2 topic pub /dream/enable std_msgs/msg/Bool "data: false"
```

### Example 3: Triggering Split-Brain Switch

```bash
# Terminal 1: Launch split brain test
ros2 run neutral_consciousness split_brain_node

# Terminal 2: Check synchronization health
ros2 topic echo /synchronization_health

# Terminal 3: Trigger switch (requires health > 0.95)
ros2 service call /trigger_hemispheric_switch std_srvs/srv/Trigger
```

### Example 4: Testing Neural Firewall

```bash
# Terminal 1: Launch firewall
ros2 run neutral_consciousness firewall_node

# Terminal 2: Send malicious pattern (high frequency)
ros2 topic pub /satellite_uplink/incoming_data std_msgs/msg/Float32MultiArray \
    "data: [100.0, 100.0, 100.0, 100.0]"  # Over voltage

# Terminal 3: Check kill switch
ros2 topic echo /firewall/kill_switch
```

---

## ROS 2 Topics & Services

### Published Topics

| Topic | Type | Publisher | Description |
|-------|------|-----------|-------------|
| `/neural_data/prediction_error` | `Float32MultiArray` | Visual Cortex | Prediction error signal (surprise) |
| `/synchronization_health` | `Float32` | Visual Cortex | Health metric (0.0-1.0) |
| `dream/prediction` | `Float32MultiArray` | Dream Engine | Current prediction |
| `dream/prediction_error` | `Float32MultiArray` | Dream Engine | Dream mode error |
| `/verified_neural_stream` | `Float32MultiArray` | Neural Firewall | Safe, verified data |
| `firewall/kill_switch` | `Bool` | Neural Firewall | Emergency disconnect signal |
| `/neural_stream/delayed` | `Float32MultiArray` | Latency Injector | Latency-simulated data |
| `/satellite_uplink/encrypted` | `Float32MultiArray` | HE Node | Encrypted outgoing data |
| `/conscious_output/unified_field` | `Image` | Split Brain Test | Unified visual field |

### Subscribed Topics

| Topic | Type | Subscriber | Description |
|-------|------|------------|-------------|
| `unity/camera/raw` | `Image` | Visual Cortex | Raw camera feed from Unity |
| `cortex/visual/activity` | `Float32MultiArray` | Dream Engine | Cortical activity |
| `dream/enable` | `Bool` | Dream Engine | Toggle dream mode |
| `/satellite_uplink/incoming_data` | `Float32MultiArray` | Neural Firewall | Incoming satellite data |
| `/neural_stream/outgoing` | `Float32MultiArray` | HE Node | Data to encrypt |
| `/camera/left_eye` | `Image` | Split Brain Test | Biological hemisphere |
| `/camera/right_eye` | `Image` | Split Brain Test | Synthetic hemisphere |

### Services

| Service | Type | Provider | Description |
|---------|------|----------|-------------|
| `/trigger_hemispheric_switch` | `Trigger` | Split Brain Test | Switch from Shadow to Active mode |

---

## Documentation

### Core Documentation

- **[Installation Guide (Windows)](docs/ROS2_INSTALLATION_WINDOWS.md)** - Complete ROS 2 Humble setup for Windows
- **[Whitepaper](docs/whitepaper.md)** - Scientific hypothesis and validation protocol
- **[Architecture](docs/ARCHITECTURE.md)** - System design and data flow
- **[Watanabe Protocol](docs/WATANABE_PROTOCOL.md)** - Detailed protocol specification (coming soon)
- **[Contributing Guidelines](CONTRIBUTING.md)** - How to contribute with scientific rigor (coming soon)

### API Documentation

- **[ROS 2 Topics Reference](docs/ROS2_TOPICS.md)** - Complete topic specification (coming soon)
- **[Node Parameters](docs/NODE_PARAMETERS.md)** - Configuration options (coming soon)
- **[Unity Integration](docs/UNITY_INTEGRATION.md)** - Unity-ROS bridge guide (coming soon)

---

## Research Resources

### Zae Project Bibliography

For comprehensive research materials supporting this project:

**[Zae Project Bibliography](https://github.com/Zae-Project/zae-docs/blob/main/reference/bibliography.md)**
- **100+ Key Researchers** - Leading scientists in consciousness, BCIs, neuromorphic computing, WBE
- **50+ Foundational Papers** - Seminal publications with full citations
- **35+ Essential Books** - Organized by topic with reading recommendations
- **Research Institutions & Labs** - Major centers advancing consciousness research
- **Conference & Journal Directory** - Community engagement opportunities

**[Researchers Directory](https://github.com/Zae-Project/zae-docs/blob/main/reference/researchers-directory.md)**
- **40+ Researcher Profiles** - Current affiliations, research focus, contact information
- **20+ Major Labs** - Leading research institutions
- **Industry Leaders** - Neuralink, Kernel, Paradromics, Synchron, Blackrock Neurotech

### Relevant Research Domains

- **Consciousness Studies**: Chalmers, Nagel, Koch, Tononi, Dehaene, Seth
- **Neuromorphic Computing**: Mead, Boahen, Eliasmith, Furber
- **Computational Neuroscience**: Sejnowski, Gerstner, Izhikevich
- **Whole Brain Emulation**: Sandberg, Bostrom, Koene, Hayworth
- **Information Theory**: Shannon, Kurzweil, Tegmark, Aaronson

---

## Contributing

We welcome contributions that maintain scientific rigor and professional standards.

### Contribution Guidelines

1. **Scientific Citations Required**: All claims must be backed by peer-reviewed research
2. **Clear Attribution**: Distinguish between established science and speculative exploration
3. **Reproducibility**: Code must be testable and results reproducible
4. **Documentation**: Every feature must be documented with scientific rationale

See [CONTRIBUTING.md](CONTRIBUTING.md) for detailed guidelines (coming soon).

---

## Project Status

**Current Version**: 0.1.0  
**ROS 2 Distribution**: Humble Hawksbill  
**Python**: 3.10+  
**Unity**: Unity 6 (compatible with Unity 2022 LTS)

**Development Stage**: Research & Prototyping

---

## Related Projects

Part of the **Zae Project** ecosystem:

- **[arkspace-core](https://github.com/Zae-Project/arkspace-core)** - Satellite infrastructure layer
- **[brain-emulation](https://github.com/Zae-Project/brain-emulation)** - Corpus callosum BCI interface
- **[zae-docs](https://github.com/Zae-Project/zae-docs)** - Unified documentation and research

---

## Support

- **Issues**: [GitHub Issues](https://github.com/Zae-Project/neutral-consciousness-engine/issues)
- **Discussions**: [GitHub Discussions](https://github.com/Zae-Project/neutral-consciousness-engine/discussions)
- **Organization**: [Zae Project](https://github.com/Zae-Project)

---

## License

MIT License - see [LICENSE](LICENSE) for details.

---

## Citation

If you use this software in your research, please cite:

```bibtex
@software{neutral_consciousness_engine,
  author = {Zae Project Team},
  title = {Neutral Consciousness Engine: SNN-based Generative Model for Consciousness Substrate Transfer},
  year = {2026},
  url = {https://github.com/Zae-Project/neutral-consciousness-engine},
  note = {Inspired by Watanabe's hemisphere integration research}
}
```

And reference the foundational research:

```bibtex
@article{watanabe2014interhemispheric,
  title={Interhemispheric transfer of visual information in split-brain patients},
  author={Watanabe, M. and others},
  journal={Neuropsychologia},
  volume={63},
  pages={133--142},
  year={2014},
  doi={10.1016/j.neuropsychologia.2014.08.025}
}
```

---

*Built with scientific rigor. Inspired by curiosity. Driven by the question: Can consciousness transcend its substrate?*
