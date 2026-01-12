# The Neutral Consciousness Engine

This repository implements the **"Generative Model"** described in the *Medical Hypotheses* submission. It serves as the **"Neutral Consciousness"** substrate required for the **Watanabe Transfer Protocol**.

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

## License

MIT License - see [LICENSE](LICENSE) for details.
