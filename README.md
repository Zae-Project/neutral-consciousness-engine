# The Neutral Consciousness Engine

This repository implements the **"Generative Model"** described in the *Medical Hypotheses* submission. It serves as the **"Neutral Consciousness"** substrate required for the **Watanabe Transfer Protocol**.

## Core Architecture

*   **The Brain:** A Spiking Neural Network (SNN) built with Nengo/ROS 2.
*   **The Body:** A physics-compliant avatar in Unity 6.
*   **The Bridge:** A low-latency TCP handshake simulating the Corpus Callosum interface.
*   **The Guardian:** A Neural Firewall to prevent "Brainjacking" during transfer.

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
   git clone https://github.com/YOUR_USERNAME/neutral-consciousness-engine.git
   cd neutral-consciousness-engine
   ```

2. Build the ROS 2 workspace:
   ```bash
   cd ros2_ws
   colcon build
   source install/setup.bash
   ```

3. Install Unity ROS Hub:
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
