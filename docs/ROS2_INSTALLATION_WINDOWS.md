# ROS 2 Humble Installation Guide for Windows

**Last Updated:** January 15, 2026  
**Target Platform:** Windows 10/11 64-bit  
**ROS 2 Distribution:** Humble Hawksbill (LTS)

---

## Prerequisites

### System Requirements

- **OS:** Windows 10 version 20H2 or Windows 11
- **Architecture:** x64 (64-bit) only
- **RAM:** Minimum 4GB (8GB+ recommended)
- **Disk Space:** ~10GB for ROS 2 + dependencies
- **Admin Rights:** Required for installation

### Required Software

1. **Visual Studio 2019 (with C++ Build Tools)**
   - Download: [Visual Studio 2019 Community](https://visualstudio.microsoft.com/downloads/)
   - Required components during installation:
     - Desktop development with C++
     - Windows 10 SDK (latest)
   - Alternative: [Build Tools for Visual Studio 2019](https://visualstudio.microsoft.com/downloads/#build-tools-for-visual-studio-2019)

2. **Chocolatey** (Package Manager for Windows)
   - Installation instructions: https://chocolatey.org/install
   - Open PowerShell as Administrator and run:
     ```powershell
     Set-ExecutionPolicy Bypass -Scope Process -Force; [System.Net.ServicePointManager]::SecurityProtocol = [System.Net.ServicePointManager]::SecurityProtocol -bor 3072; iex ((New-Object System.Net.WebClient).DownloadString('https://community.chocolatey.org/install.ps1'))
     ```

3. **Python 3.10**
   - Recommended via Chocolatey:
     ```powershell
     choco install -y python310
     ```
   - Verify:
     ```powershell
     python --version  # Should show Python 3.10.x
     ```

---

## Installation Steps

### Step 1: Install Dependencies via Chocolatey

Open PowerShell as Administrator:

```powershell
choco install -y git cmake curl wget

# Install additional ROS 2 dependencies
choco install -y vcredist2015
```

### Step 2: Download ROS 2 Humble Binary Package

1. Visit: [ROS 2 Humble Windows Releases](https://github.com/ros2/ros2/releases)
2. Download the latest **Windows x64** binary package:
   ```
   ros2-humble-YYYYMMDD-windows-release-amd64.zip
   ```
   (e.g., `ros2-humble-20220523-windows-release-amd64.zip`)

### Step 3: Extract ROS 2

1. Create installation directory:
   ```powershell
   mkdir C:\dev\ros2_humble
   ```

2. Extract the downloaded `.zip` file to `C:\dev\ros2_humble\`
   - You should have: `C:\dev\ros2_humble\ros2-windows\`

### Step 4: Install Python Dependencies

Open Command Prompt (NOT PowerShell) as Administrator:

```cmd
pip install -U colcon-common-extensions vcstool
pip install -U rosdep
```

### Step 5: Set up ROS 2 Environment

Create a shortcut to launch ROS 2 environment easily:

1. Open Command Prompt
2. Navigate to:
   ```cmd
   cd C:\dev\ros2_humble\ros2-windows
   ```

3. Run the setup script:
   ```cmd
   call local_setup.bat
   ```

4. Verify installation:
   ```cmd
   ros2 doctor
   ```
   Should show: `All 5 checks passed`

---

## Environment Setup (Every Session)

**IMPORTANT:** You must source the ROS 2 setup file in EVERY new terminal:

```cmd
call C:\dev\ros2_humble\ros2-windows\local_setup.bat
```

### Creating a Permanent Shortcut

1. Create a batch file `ros2_env.bat`:
   ```batch
   @echo off
   call C:\dev\ros2_humble\ros2-windows\local_setup.bat
   cmd /k
   ```

2. Save it to `C:\Users\YourUsername\Desktop\`
3. Double-click to launch a ROS 2-ready terminal

---

## Neutral Consciousness Engine Setup

### Step 1: Clone the Repository

```cmd
cd C:\Users\YourUsername\OneDrive\Escritorio
git clone https://github.com/Zae-Project/neutral-consciousness-engine.git
cd neutral-consciousness-engine
```

### Step 2: Install Python Dependencies

Create a virtual environment (recommended):

```cmd
python -m venv venv
venv\Scripts\activate
pip install -r requirements.txt
```

### Step 3: Build the ROS 2 Workspace

```cmd
cd ros2_ws
colcon build --packages-select neutral_consciousness
```

Expected output: `Finished <<< neutral_consciousness`

### Step 4: Source the Workspace

```cmd
call install\setup.bat
```

### Step 5: Verify Node Availability

```cmd
ros2 pkg list | findstr neutral
```

Should show: `neutral_consciousness`

```cmd
ros2 run neutral_consciousness cortex_node --help
```

Should show node information (even if Nengo not installed yet)

---

## Testing the Installation

### Quick Test: Launch Individual Node

```cmd
# Ensure ROS 2 environment is sourced
call C:\dev\ros2_humble\ros2-windows\local_setup.bat

# Navigate to workspace
cd C:\Users\YourUsername\OneDrive\Escritorio\neutral-consciousness-engine\ros2_ws
call install\setup.bat

# Launch a test node
ros2 run neutral_consciousness firewall_node
```

Expected: Node starts and logs initialization

### Full System Test (requires Unity)

```cmd
ros2 launch neutral_consciousness master_system.launch.py
```

**NOTE:** This will fail if Unity is not running and ROS-TCP-Endpoint is not installed.

---

## Common Issues & Solutions

### Issue 1: "colcon: command not found"

**Solution:** Install colcon:
```cmd
pip install -U colcon-common-extensions
```

### Issue 2: "Python 3.10 not found"

**Solution:** Ensure Python is in PATH:
```cmd
where python
```
Should show: `C:\Python310\python.exe`

If not, add to PATH manually or reinstall via Chocolatey.

### Issue 3: "Visual Studio compiler not found"

**Solution:** Ensure Visual Studio 2019 C++ Build Tools are installed.

Verify:
```cmd
"C:\Program Files (x86)\Microsoft Visual Studio\2019\BuildTools\VC\Auxiliary\Build\vcvarsall.bat" x64
```

### Issue 4: "rclpy module not found"

**Solution:** ROS 2 environment not sourced. Run:
```cmd
call C:\dev\ros2_humble\ros2-windows\local_setup.bat
```

### Issue 5: "Nengo import failed"

**Solution:** Install Nengo in your Python environment:
```cmd
pip install nengo
```

---

## Next Steps

1. **Install Unity 6**: Download from [Unity Hub](https://unity.com/download)
2. **Clone ROS-TCP-Endpoint**:
   ```cmd
   cd ros2_ws\src
   git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
   cd ..\..
   colcon build
   ```

3. **Run the Full System**:
   - Launch ROS 2: `ros2 launch neutral_consciousness master_system.launch.py`
   - Open Unity project in `unity_project/`
   - Press Play in Unity Editor

---

## Resources

- **Official ROS 2 Humble Docs**: https://docs.ros.org/en/humble/
- **ROS 2 Windows Installation**: https://docs.ros.org/en/humble/Installation/Windows-Install-Binary.html
- **Troubleshooting**: https://docs.ros.org/en/humble/How-To-Guides/Installation-Troubleshooting.html
- **Zae Project GitHub**: https://github.com/Zae-Project
- **Unity ROS Integration**: https://github.com/Unity-Technologies/Unity-Robotics-Hub

---

## Support

For issues specific to the Neutral Consciousness Engine:
- **Issues**: https://github.com/Zae-Project/neutral-consciousness-engine/issues
- **Discussions**: https://github.com/Zae-Project/neutral-consciousness-engine/discussions

For ROS 2 general issues:
- **ROS Answers**: https://answers.ros.org/
- **ROS Discourse**: https://discourse.ros.org/

---

*This guide is maintained by the Zae Project Team.*
