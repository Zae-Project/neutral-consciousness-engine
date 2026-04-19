# Neutral Consciousness Engine - System Architecture

This document describes the complete system architecture, ROS 2 topic connectivity, and data flow for the Neutral Consciousness Engine.

## Overview

The system implements a "Mind-Body" architecture for consciousness simulation:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        NEUTRAL CONSCIOUSNESS ENGINE                         │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────────────┐          ┌─────────────────────┐                  │
│  │     THE BODY        │          │      THE MIND       │                  │
│  │   (Unity 6)         │◄────────►│    (ROS 2/Nengo)    │                  │
│  │   Digital Twin      │  TCP/IP  │   Spiking Neural    │                  │
│  │                     │  Bridge  │      Network        │                  │
│  └─────────────────────┘          └─────────────────────┘                  │
│           │                                  │                              │
│           │                                  │                              │
│           ▼                                  ▼                              │
│  ┌─────────────────────┐          ┌─────────────────────┐                  │
│  │   ROS Bridge        │          │   Neural Firewall   │                  │
│  │   Client (C#)       │          │   (Hybrid TEE)      │                  │
│  └─────────────────────┘          └─────────────────────┘                  │
│                                              │                              │
│                                              ▼                              │
│                                   ┌─────────────────────┐                  │
│                                   │  Satellite Uplink   │                  │
│                                   │  (OISL Simulation)  │                  │
│                                   └─────────────────────┘                  │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

## ROS 2 Nodes

### 1. Visual Cortex (`cortex_node`)
**File:** `cortex_snn/visual_cortex.py`

Implements True Predictive Coding using Nengo SNN. Also hosts the transmissive layer: the firewall-cleared ambient EM field is wired into the cortex via the ephaptic-coupling scalar field, and the rolling θ-band PLV between cortex rate and EM reference is published for the hemispheric gate.

| Direction | Topic | Type | Description |
|-----------|-------|------|-------------|
| Subscribe | `unity/camera/raw` | `sensor_msgs/Image` | Raw camera feed from Unity |
| Subscribe | `/environment/em_field` | `std_msgs/Float32MultiArray` | Firewall-cleared ambient EM (drives the ephaptic field) |
| Publish | `/neural_data/prediction_error` | `std_msgs/Float32MultiArray` | Prediction error signal |
| Publish | `/synchronization_health` | `std_msgs/Float32` | Sync health (0.0-1.0) |
| Publish | `/transmissive_sync/plv` | `std_msgs/Float32` | θ-band Phase-Locking Value (0.0-1.0) |

### 2. Dream Engine (`dream_node`)
**File:** `cortex_snn/dream_engine.py`

Generative Model for prediction and "dreaming".

| Direction | Topic | Type | Description |
|-----------|-------|------|-------------|
| Subscribe | `cortex/visual/activity` | `std_msgs/Float32MultiArray` | Cortex activity input |
| Subscribe | `dream/enable` | `std_msgs/Bool` | Toggle dream mode |
| Publish | `dream/prediction` | `std_msgs/Float32MultiArray` | Current prediction |
| Publish | `dream/prediction_error` | `std_msgs/Float32MultiArray` | Prediction error |

### 3. Neural Firewall (`firewall_node`)
**File:** `neural_firewall/traffic_monitor.py`

Brainjacking defense system with real-time FFT analysis.

| Direction | Topic | Type | Description |
|-----------|-------|------|-------------|
| Subscribe | `/satellite_uplink/incoming_data` | `std_msgs/Float32MultiArray` | Incoming satellite data |
| Subscribe | `/environment/em_field_raw` | `std_msgs/Float32MultiArray` | Raw ambient EM drive (transmissive input) |
| Publish | `/verified_neural_stream` | `std_msgs/Float32MultiArray` | Verified safe data |
| Publish | `/environment/em_field` | `std_msgs/Float32MultiArray` | Firewall-cleared ambient EM |
| Publish | `firewall/kill_switch` | `std_msgs/Bool` | Emergency disconnect signal |

**Safety Thresholds:**
- Max Frequency: 150 Hz (seizure prevention)
- Voltage Limit: 100 mV (excitotoxicity prevention)

### 3b. Ambient EM Driver (`em_driver_node`)
**File:** `cortex_snn/em_driver.py`

Exogenous transmissive-layer driver. Publishes a deterministic Schumann-cavity reference (7.83 Hz + first four harmonics) at 1 kHz, 5 mV peak. Routed through the Neural Firewall before reaching the cortex.

| Direction | Topic | Type | Description |
|-----------|-------|------|-------------|
| Publish | `/environment/em_field_raw` | `std_msgs/Float32MultiArray` | Raw ambient EM samples |

**Parameters:**
- `em_mode`: `schumann` (default) | `noise` | `silent`
- `publish_rate_hz`: 1000.0
- `output_voltage_mv`: 5.0

### 4. Latency Injector (`latency_injector_node`)
**File:** `neural_firewall/latency_injector.py`

Simulates OISL (Optical Inter-Satellite Link) delays.

| Direction | Topic | Type | Description |
|-----------|-------|------|-------------|
| Subscribe | `/neural_stream/generated` | `std_msgs/Float32MultiArray` | SNN output |
| Publish | `/neural_stream/delayed` | `std_msgs/Float32MultiArray` | Delayed output to Unity |

**Parameters:**
- `round_trip_time_ms`: Default 20ms (configurable)
- Libet Limit: 500ms (consciousness continuity threshold)

### 5. Homomorphic Encryption (`he_node`)
**File:** `neural_firewall/homomorphic_encryption.py`

Secure neural data processing using CKKS scheme.

| Direction | Topic | Type | Description |
|-----------|-------|------|-------------|
| Subscribe | `/neural_stream/outgoing` | `std_msgs/Float32MultiArray` | Data to encrypt |
| Subscribe | `/satellite_downlink/encrypted` | `std_msgs/Float32MultiArray` | Encrypted responses |
| Publish | `/satellite_uplink/encrypted` | `std_msgs/Float32MultiArray` | Encrypted data |
| Publish | `/neural_stream/decrypted` | `std_msgs/Float32MultiArray` | Decrypted responses |
| Publish | `/encryption/status` | `std_msgs/String` | System status |

### 6. Split Brain Test (`split_brain_node`)
**File:** `tests/split_brain_test.py`

Implements the Uni-hemispheric Subjective Protocol with dual-criterion gate.

| Direction | Topic | Type | Description |
|-----------|-------|------|-------------|
| Subscribe | `/camera/left_eye` | `sensor_msgs/Image` | Biological hemisphere input |
| Subscribe | `/camera/right_eye` | `sensor_msgs/Image` | Synthetic hemisphere input |
| Subscribe | `/synchronization_health` | `std_msgs/Float32` | Productive convergence metric |
| Subscribe | `/transmissive_sync/plv` | `std_msgs/Float32` | Transmissive convergence metric |
| Publish | `/conscious_output/unified_field` | `sensor_msgs/Image` | Unified visual field |
| Publish | `/transmissive_sync/gate_ready` | `std_msgs/Bool` | Sustained dual-criterion gate |
| Service | `/trigger_hemispheric_switch` | `std_srvs/Trigger` | Switch hemisphere mode |

**Gate:** `sync_health ≥ 0.95` AND `plv ≥ 0.80`, sustained for 3 s.

**Modes:**
- **Shadow Mode:** Synthetic hemisphere learning only (gated output)
- **Active Mode:** Synthetic hemisphere merged with biological

## Future Work

### Reserved: Biophoton / UPE Optical Side-Channel

Topic `/environment/biophoton_channel` is reserved for the ultraweak-photon-emission (UPE) optical pathway described in Rouleau & Cimino (2022) but not yet implemented. It would carry a non-visual optical reference (analogous to the current EM driver) for substrates that may couple via Opsin3/5 or axonal light-guiding. Deferred until the scalar EM loop is validated end-to-end.

## Unity Components

### RosBridgeClient (`RosBridgeClient.cs`)

| Direction | Topic | Type | Description |
|-----------|-------|------|-------------|
| Publish | `unity/camera/raw` | `sensor_msgs/Image` | Camera feed @ 30Hz |
| Subscribe | `firewall/kill_switch` | `std_msgs/Bool` | Emergency disconnect |

**Configuration:**
- Default ROS IP: 127.0.0.1
- Default Port: 10000
- Image Resolution: 640x480

## Data Flow Diagram

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                              DATA FLOW                                        │
└──────────────────────────────────────────────────────────────────────────────┘

                    SENSORY INPUT PATH (Unity → SNN)
                    ================================
                    
    ┌─────────┐    unity/camera/raw    ┌──────────────┐
    │  Unity  │ ─────────────────────► │ Visual       │
    │ Camera  │                        │ Cortex SNN   │
    └─────────┘                        └──────┬───────┘
                                              │
                                              │ /neural_data/prediction_error
                                              ▼
                                       ┌──────────────┐
                                       │    Dream     │
                                       │   Engine     │
                                       └──────────────┘


                    SATELLITE PATH (with Security)
                    ==============================
                    
    ┌──────────────┐   /neural_stream/outgoing   ┌──────────────┐
    │ Visual       │ ──────────────────────────► │ Homomorphic  │
    │ Cortex       │                             │ Encryption   │
    └──────────────┘                             └──────┬───────┘
                                                        │
                                                        │ /satellite_uplink/encrypted
                                                        ▼
    ┌──────────────┐   /satellite_uplink/        ┌──────────────┐
    │   Neural     │ ◄─────incoming_data──────── │  Satellite   │
    │  Firewall    │                             │  (Simulated) │
    └──────┬───────┘                             └──────────────┘
           │
           │ /verified_neural_stream
           ▼
    ┌──────────────┐   /neural_stream/generated  ┌──────────────┐
    │   Latency    │ ◄────────────────────────── │ Verified     │
    │  Injector    │                             │ Stream       │
    └──────┬───────┘                             └──────────────┘
           │
           │ /neural_stream/delayed
           ▼
    ┌──────────────┐
    │    Unity     │
    │   Avatar     │
    └──────────────┘


                    SAFETY LOOP (Kill Switch)
                    =========================
                    
    ┌──────────────┐   firewall/kill_switch      ┌──────────────┐
    │   Neural     │ ──────────────────────────► │    Unity     │
    │  Firewall    │       (Bool: True)          │ Disconnect   │
    └──────────────┘                             └──────────────┘
```

## Launch Configuration

### Full System Launch

```bash
ros2 launch neutral_consciousness master_system.launch.py
```

### Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `round_trip_time_ms` | 20.0 | OISL latency simulation |
| `dream_mode` | false | Start Dream Engine in dream mode |
| `em_driver_enabled` | true | Enable the ambient EM driver node |
| `em_mode` | schumann | EM waveform: `schumann` \| `noise` \| `silent` |
| `ephaptic_enabled` | true | (reserved) Toggle for disabling the ephaptic block in ablation runs |

### Individual Node Launch

```bash
# Visual Cortex only
ros2 run neutral_consciousness cortex_node

# Neural Firewall only
ros2 run neutral_consciousness firewall_node

# Dream Engine with parameters
ros2 run neutral_consciousness dream_node --ros-args -p dream_mode:=true

# Latency Injector with custom RTT
ros2 run neutral_consciousness latency_injector_node --ros-args -p round_trip_time_ms:=50.0
```

## Scientific References

1. **Predictive Coding:** Friston, K. (2010). The free-energy principle. *Nature Reviews Neuroscience*.
2. **Split-Brain Research:** Sperry, R. W. (1968). Hemisphere deconnection. *American Psychologist*.
3. **Libet's Delay:** Libet, B. (1985). Unconscious cerebral initiative. *Behavioral and Brain Sciences*.
4. **Brainjacking:** Pycroft, L., et al. (2016). Implant Security. *World Neurosurgery*.
5. **Hybrid TEE:** Nguyen, T., et al. (2025). Hybrid TEE for BCI Latency Optimization. *IEEE TBME*.
6. **Transmissive Theory:** Rouleau, N., & Cimino, N. (2022). A Transmissive Theory of Brain Function. *NeuroSci* 3(3): 440–456.
7. **Phase-Locking Value:** Lachaux, J.-P., et al. (1999). Measuring phase synchrony in brain signals. *Human Brain Mapping* 8: 194–208.

## Version

- **Engine Version:** 0.1.0
- **ROS 2 Distribution:** Humble
- **Unity Version:** 6

---

*For the full scientific hypothesis, see [whitepaper.md](whitepaper.md)*
