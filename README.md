# The Neutral Consciousness Engine

**Consciousness Substrate Transfer via Spiking Neural Networks**

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/)
[![Python 3.10+](https://img.shields.io/badge/Python-3.10%2B-blue)](https://www.python.org/)

---

## Overview

This repository implements the **"Generative Model"** for the Watanabe Transfer Protocol - a theoretical framework for seamless consciousness substrate transfer. It serves as the **"Neutral Consciousness"** synthetic hemisphere required for hemisphere integration.

**IMPORTANT DISCLAIMER:** This project is **inspired by** and builds upon the research approach of Professor Masataka Watanabe, particularly his work on interhemispheric transfer in split-brain patients. **This is NOT his official work.** We are independent researchers exploring consciousness substrate transfer based on principles from his publications.

### Scientific Foundation

**Key Research References:**
- **Watanabe, M., et al. (2014).** "Interhemispheric transfer of visual information in split-brain patients." *Neuropsychologia*, 63, 133-142. DOI: 10.1016/j.neuropsychologia.2014.08.025
- **Watanabe's Neutral Consciousness Framework** (referenced in project documentation)
- **Sperry, R.W. (1968).** "Hemisphere deconnection and unity in conscious awareness." *American Psychologist*, 23(10), 723-733.
- **Libet, B. (1983).** Unconscious cerebral initiative. *Behavioral and Brain Sciences*.
- **Friston, K. (2010).** The free-energy principle. *Nature Reviews Neuroscience*.

## Core Architecture

*   **The Brain (Generative Model):** A Spiking Neural Network (SNN) built with Nengo/ROS 2. Unlike traditional AI, it implements **Predictive Coding**: the cortex predicts sensory input and only transmits the *prediction error* (Error = Input - Prediction), minimizing bandwidth for OISL compatibility.
*   **The Body (Digital Twin):** A physics-compliant avatar in Unity 6, serving as the somatosensory anchor.
*   **The Bridge (Corpus Callosum):** A low-latency TCP handshake simulating the biological interface.
*   **The Guardian (Neural Firewall):** A **Hybrid TEE** (Trusted Execution Environment) middleware. It combines AES-256 for real-time streams with Homomorphic Encryption for identity handshakes. It includes active "Brainjacking" defenses that monitor for malicious stimulation patterns (e.g., high-frequency seizure induction).

## Directory Structure

```
neutral-consciousness-engine/
│
├── ros2_ws/                    # The "Mind" (ROS 2 Workspace)
│   └── src/
│       ├── neutral_consciousness/   # Main Python Package
│       └── ros_tcp_endpoint/        # Unity<->ROS Communication Bridge
│
└── unity_project/              # The "Body" (Unity Project)
    ├── Assets/
    │   ├── RosSettings/
    │   ├── Scripts/
    │   └── Scenes/
    └── Packages/
```

## Getting Started

### Prerequisites

- ROS 2 Humble
- Unity 6
- Python 3.10+
- Nengo or BindsNET

### Installation

1. Clone this repository:
   ```bash
   git clone https://github.com/venturaEffect/neutral-consciousness-engine.git
   cd neutral-consciousness-engine
   ```

2. Install Python dependencies:
   ```bash
   pip install -r requirements.txt
   ```

3. Clone the ROS-TCP-Endpoint (Unity-ROS Bridge):
   ```bash
   cd ros2_ws/src
   git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
   cd ../..
   ```

4. Build the ROS 2 workspace:
   ```bash
   cd ros2_ws
   colcon build
   source install/setup.bash  # Linux/Mac
   # or: call install/setup.bat  # Windows
   ```

5. Install Unity ROS Hub:
   - Open Unity Package Manager
   - Add package from git URL: `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector`

### Running the System

Launch the entire consciousness system with one command:
```bash
ros2 launch neutral_consciousness master_system.launch.py
```

## Documentation

See the [docs/whitepaper.md](docs/whitepaper.md) for the full Medical Hypotheses logic.

## Research Resources

For comprehensive research materials supporting this project, see the **[Zae Project Bibliography](https://github.com/Zae-Project/zae-docs/blob/main/reference/bibliography.md)** - a centralized repository containing:

- **100+ Key Researchers** - Leading scientists in consciousness studies, SNNs, neuromorphic computing, and WBE
- **50+ Foundational Papers** - Seminal publications with full citations
- **35+ Essential Books** - Organized by topic with reading recommendations
- **Research Institutions & Labs** - Major centers advancing consciousness research
- **Conference & Journal Directory** - Community engagement opportunities

**Relevant Sections for Neutral Consciousness Engine:**
- Consciousness Studies (Chalmers, Nagel, Koch, Tononi, Dehaene, Seth)
- Neuromorphic Computing (Mead, Boahen, Eliasmith, Furber)
- Computational Neuroscience (Sejnowski, Gerstner, Izhikevich)
- Whole Brain Emulation (Sandberg, Bostrom, Koene, Hayworth)
- Information Theory (Shannon, Kurzweil, Tegmark, Aaronson)

Also see the **[Researchers Directory](https://github.com/Zae-Project/zae-docs/blob/main/reference/researchers-directory.md)** for detailed profiles and contact information.

## License

MIT License - see [LICENSE](LICENSE) for details.
