### **1. Master Instruction Prompt**

**Context:**
"We are building a hybrid **ROS 2 + Unity + Python SNN** repository named `neutral-consciousness-engine`. This system implements the 'Generative Model' hypothesis, where a Spiking Neural Network (SNN) simulates a 3D reality to predict sensory inputs.

**Objective:**
Initialize a public GitHub repository with a robust directory structure that supports:
1.  **ROS 2 (Humble)** backend for the SNN logic (Python/C++).
2.  **Unity 6** frontend for the 3D 'Virtual Body' and environment.
3.  **Nengo/BindsNET** for the neuromorphic core.
4.  A **Neural Firewall** middleware for security.

Please follow the structure and file contents below to generate the repository."

---

### **2. Repository Directory Structure**

Tell the Copilot to generate this exact folder tree. It separates the "Mind" (ROS 2) from the "Body" (Unity).

```text
neutral-consciousness-engine/
│
├── .gitignore                  # Combined Unity + ROS 2 + Python gitignore
├── README.md                   # Documentation linking to the hypothesis
├── LICENSE                     # MIT License
│
├── docs/                       # Architecture diagrams & Whitepaper
│   └── whitepaper.md           # The Medical Hypothesis logic
│
├── ros2_ws/                    # The "Mind" (ROS 2 Workspace)
│   ├── src/
│   │   ├── neutral_consciousness/   # Main Python Package
│   │   │   ├── package.xml
│   │   │   ├── setup.py
│   │   │   ├── launch/
│   │   │   │   └── master_system.launch.py  # Launches Unity + SNN
│   │   │   └── neutral_consciousness/
│   │   │       ├── __init__.py
│   │   │       ├── cortex_snn/          # The SNN Core
│   │   │       │   ├── visual_cortex.py # Nengo Visual processing
│   │   │       │   └── dream_engine.py  # Generative Model logic
│   │   │       └── neural_firewall/     # Security Middleware
│   │   │           └── traffic_monitor.py
│   │   │
│   │   └── ros_tcp_endpoint/   # Unity<->ROS Communication Bridge
│   │       # (Clone from Unity-Technologies/ROS-TCP-Endpoint)
│
└── unity_project/              # The "Body" (Unity Project)
    ├── Assets/
    │   ├── RosSettings/        # ROS Connection Config
    │   ├── Scripts/
    │   │   └── RosBridgeClient.cs
    │   └── Scenes/
    │       └── VirtualLab.unity
    ├── Packages/
    └── ProjectSettings/
```

---

### **3. Essential Configuration Files**

Provide these snippets to the Copilot so the system works immediately after cloning.

#### **A. The Combined `.gitignore`**
*Merging Unity and Python/ROS build artifacts is critical to avoid a messy repo.*

**Prompt:** "Create a root `.gitignore` file that merges standard Unity, Python, and ROS 2 exclusions."

```gitignore
# --- ROS 2 ---
install/
build/
log/
*/__pycache__/

# --- Unity ---
/[Ll]ibrary/
/[Tt]emp/
/[Oo]bj/
/[Bb]uild/
/[Bb]uilds/
/[Ll]ogs/
/[Uu]ser[Ss]ettings/
*.csproj
*.unityproj
*.sln
*.suo
*.tmp
*.user
*.userprefs
*.pidb
*.booproj
*.svd
*.pdb
*.mdb
*.opendb
*.VC.db

# --- Python / Virtual Env ---
venv/
env/
.env
```

#### **B. The Master Launch File (`master_system.launch.py`)**
*This script allows you to start the entire "Consciousness" with one command.*

**Prompt:** "Generate the `ros2_ws/src/neutral_consciousness/launch/master_system.launch.py` file using `ExecuteProcess` to start the Unity executable and `Node` to start the SNN."

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # 1. Start the ROS-TCP Endpoint (Bridge to Unity)
        Node(
            package='ros_tcp_endpoint',
            executable='default_server_endpoint',
            parameters=[{'ROS_IP': '0.0.0.0'}, {'ROS_TCP_PORT': 10000}],
        ),

        # 2. Start the Neutral Consciousness Engine (SNN)
        Node(
            package='neutral_consciousness',
            executable='cortex_node',  # You will define this entry point
            name='cortex_core',
            output='screen'
        ),

        # 3. (Optional) Auto-launch Unity Build if available
        # ExecuteProcess(
        #     cmd=['./unity_project/Builds/Linux/NeutralBody.x86_64'],
        #     output='screen'
        # )
    ])
```

#### **C. The Neural Firewall Stub (`traffic_monitor.py`)**
*This is the cybersecurity component you requested.*

**Prompt:** "Create a basic `traffic_monitor.py` for the Neural Firewall module that monitors spike rates."

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class NeuralFirewall(Node):
    def __init__(self):
        super().__init__('neural_firewall')
        # Threshold: Max spikes per second before "Kill Switch" triggers
        self.SPIKE_RATE_LIMIT = 200.0 
        
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'incoming_neural_data',
            self.monitor_traffic,
            10)

    def monitor_traffic(self, msg):
        spike_rate = sum(msg.data) / len(msg.data) # Simplified metric
        
        if spike_rate > self.SPIKE_RATE_LIMIT:
            self.get_logger().fatal(f'SECURITY ALERT: Spike rate {spike_rate}Hz exceeds safety limits!')
            self.trigger_kill_switch()

    def trigger_kill_switch(self):
        # Logic to sever connection to Satellite/Unity
        self.get_logger().info('KILL SWITCH ACTIVATED: Connection Severed.')
        # In a real scenario, this would send a hardware interrupt signal
```

---

### **4. Next Steps for You**

1.  **Initialize the Repo:** Run these commands (or ask Copilot to do it):
    ```bash
    mkdir neutral-consciousness-engine
    cd neutral-consciousness-engine
    git init
    # (Create the folders and files from Section 2 & 3)
    ```
2.  **Install Unity ROS Hub:** Inside your Unity Project, you must install the [ROS-TCP-Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector).
    *   *Action:* Open Unity Package Manager -> Add package from git URL -> `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector`
3.  **Documentation:** Paste the following into your `README.md` to align with the hypothesis:

> ## The Neutral Consciousness Engine
>
> This repository implements the **"Generative Model"** described in the *Medical Hypotheses* submission. It serves as the **"Neutral Consciousness"** substrate required for the **Watanabe Transfer Protocol**.
>
> ### Core Architecture
> *   **The Brain:** A Spiking Neural Network (SNN) built with Nengo/ROS 2.
> *   **The Body:** A physics-compliant avatar in Unity 6.
> *   **The Bridge:** A low-latency TCP handshake simulating the Corpus Callosum interface.
> *   **The Guardian:** A Neural Firewall to prevent "Brainjacking" during transfer.