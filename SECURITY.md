# Security Policy

## Project Status

Neutral Consciousness Engine is a **safety-critical research project** implementing consciousness substrate transfer protocols. Security is paramount due to the nature of neural data handling and brainjacking defense.

**Current Phase**: Research & Prototyping (pre-v1.0)

---

## Supported Versions

| Version | Supported          | Status |
| ------- | ------------------ | ------ |
| main    | :white_check_mark: | Active development |
| < 1.0   | :warning:          | Pre-release, experimental security |

---

## Reporting a Vulnerability

**⚠️ CRITICAL**: For neural firewall vulnerabilities or "brainjacking" attack vectors, report immediately.

### How to Report

**Email**: [security@todosloscobardesdelvalle.com](mailto:security@todosloscobardesdelvalle.com)

**Subject**: `[SECURITY-CRITICAL] NCE - Brief Description`

### Priority Classification

| Type | Priority | Example |
|------|----------|---------|
| **Brainjacking Attack Vector** | CRITICAL | Neural firewall bypass, frequency attack |
| **Encryption Weakness** | CRITICAL | Homomorphic encryption flaw, key exposure |
| **ROS 2 Security** | HIGH | Topic hijacking, service spoofing |
| **Input Validation** | HIGH | Malformed neural data, buffer overflow |
| **Information Disclosure** | MEDIUM | Log leakage, timing attacks |
| **DoS/Resource Exhaustion** | MEDIUM | Memory exhaustion, CPU spike |

---

## Threat Model

### System Context

The Neutral Consciousness Engine implements:
- **The Mind**: ROS 2-based spiking neural network
- **The Body**: Unity 6 digital twin
- **The Bridge**: TCP/IP corpus callosum interface
- **The Guardian**: Neural Firewall with hybrid TEE

### Assets to Protect

1. **Neural Data Integrity**: Spike trains must not be corrupted
2. **Consciousness Continuity**: <50ms latency must be maintained
3. **User Safety**: No excitotoxicity or seizure induction
4. **Privacy**: Neural patterns are sensitive biometric data
5. **System Availability**: Consciousness transfer cannot tolerate downtime

### Threat Actors

| Actor | Capability | Motivation | Likelihood |
|-------|------------|------------|------------|
| **Nation State** | Advanced persistent threat | Espionage, sabotage | Low (research phase) |
| **Cybercriminals** | Ransomware, data theft | Financial | Low (no value yet) |
| **Researchers** | Protocol analysis | Validation | High |
| **Insiders** | Full system access | Sabotage, curiosity | Medium |
| **Script Kiddies** | Automated tools | Disruption | Medium |

### Attack Vectors

#### 1. Brainjacking (CRITICAL)

**Definition**: Malicious manipulation of neural data to induce harm.

**Scientific Foundation**:
> Pycroft, L., et al. (2016). "Brainjacking: Implant Security Issues in Invasive Neuromodulation." *World Neurosurgery*, 92, 454-462.
> DOI: [10.1016/j.wneu.2016.05.010](https://doi.org/10.1016/j.wneu.2016.05.010)

**Attack Types**:
- **High-Frequency Attack**: >150 Hz gamma synchrony to induce seizures
- **Over-Voltage Attack**: >100 mV spikes causing excitotoxicity
- **Timing Attack**: Spike injection at Libet's 350ms window
- **Pattern Injection**: Malicious spike patterns to disrupt cognition

**Defenses**:
- Neural Firewall with frequency analysis (traffic_monitor.py)
- Voltage limiting and spike amplitude capping
- Kill switch with physical disconnect
- Redundant safety monitoring

#### 2. Encryption Attacks

**Threat**: Compromise of Hybrid TEE architecture.

**Attack Types**:
- **Key Extraction**: Steal AES-256 keys from memory
- **Homomorphic Attack**: Exploit CKKS encryption weaknesses
- **Man-in-the-Middle**: Intercept neural stream during handshake
- **Replay Attack**: Re-inject recorded neural patterns

**Defenses**:
- AES-256-GCM for neural stream (<1ms latency)
- TenSEAL homomorphic encryption for identity handshake
- Secure key storage (future: hardware security module)
- Nonce-based replay protection (future)

#### 3. ROS 2 Security

**Threat**: DDS (Data Distribution Service) lacks built-in authentication.

**Attack Types**:
- **Topic Hijacking**: Publish malicious data to `/neural_data/prediction_error`
- **Service Spoofing**: Fake `/cortex/dream_mode_switch` service
- **Eavesdropping**: Sniff neural data on unencrypted DDS topics
- **Resource Exhaustion**: Flood topics with high-frequency messages

**Defenses**:
- ROS 2 Security (SROS2) with permissions (future)
- Network isolation (localhost-only deployment)
- Topic whitelist and validation
- Rate limiting on critical topics

#### 4. Unity Bridge Attacks

**Threat**: TCP/IP bridge between ROS 2 and Unity is unauthenticated.

**Attack Types**:
- **Sensor Spoofing**: Send fake camera data to Visual Cortex
- **Desynchronization**: Inject latency to break <50ms requirement
- **Command Injection**: Manipulate Unity scene state
- **Data Exfiltration**: Extract proprioceptive feedback

**Defenses**:
- Mutual TLS authentication (future)
- Message signing with HMAC (future)
- Latency monitoring and kill switch
- Input validation on all Unity messages

---

## Known Vulnerabilities

### Current Implementation Limitations

#### 🔴 CRITICAL

1. **No Authentication on ROS 2 Topics**
   - **Status**: Known issue, accepted risk for research phase
   - **Impact**: Any process can publish to neural topics
   - **Mitigation**: Deploy only on trusted, isolated systems
   - **Fix Target**: v0.9 (SROS2 integration)

2. **Limited Kill Switch Testing**
   - **Status**: Kill switch implemented but not stress-tested
   - **Impact**: May not trigger fast enough under attack
   - **Mitigation**: Conservative safety thresholds
   - **Fix Target**: v0.8 (comprehensive attack simulation)

3. **Plaintext Neural Data on Localhost**
   - **Status**: ROS 2 topics unencrypted on loopback
   - **Impact**: Local eavesdropping possible
   - **Mitigation**: Ensure no untrusted processes on system
   - **Fix Target**: v1.0 (encrypted DDS transport)

#### 🟠 HIGH

4. **No Rate Limiting on Neural Topics**
   - **Status**: DoS attack possible via topic flooding
   - **Impact**: CPU exhaustion, latency spike
   - **Mitigation**: Monitor system resources
   - **Fix Target**: v0.9

5. **Unity Connection Lacks Auth**
   - **Status**: Any Unity instance can connect
   - **Impact**: Fake digital twin could feed malicious data
   - **Mitigation**: Bind to localhost only
   - **Fix Target**: v0.9

#### 🟡 MEDIUM

6. **Limited Input Validation**
   - **Status**: Some ROS message shapes not validated
   - **Impact**: Malformed messages could crash nodes
   - **Mitigation**: Exception handling in callbacks
   - **Fix Target**: v0.8

---

## Secure Deployment Guidelines

### Minimum Security Requirements

✅ **Must Have**:
1. Network isolation (no internet exposure)
2. Trusted execution environment
3. Updated dependencies (no known CVEs)
4. Audit logging enabled
5. Kill switch tested

❌ **Must NOT**:
1. Deploy on public networks
2. Use default ROS 2 domain ID (0)
3. Disable neural firewall
4. Ignore latency warnings
5. Run with elevated privileges unnecessarily

### Deployment Checklist

#### Network Isolation
```bash
# Verify no external network access
ifconfig | grep inet  # Should only see 127.0.0.1

# Set ROS 2 to localhost only
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=$(shuf -i 1-101 -n 1)  # Random domain ID
```

#### Firewall Configuration
```bash
# Ubuntu/Debian firewall
sudo ufw enable
sudo ufw default deny incoming
sudo ufw default deny outgoing
sudo ufw allow from 127.0.0.1
```

#### ROS 2 Security (SROS2)
```bash
# Generate security keys (future)
ros2 security create_keystore ~/sros2_keys
ros2 security create_key ~/sros2_keys /neutral_consciousness_engine

# Set environment variables
export ROS_SECURITY_KEYSTORE=~/sros2_keys
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce
```

---

## Security Features

### Implemented

✅ **Neural Firewall** (`neural_firewall/traffic_monitor.py`)
- Frequency analysis (>150 Hz detection)
- Voltage limiting (>100 mV threshold)
- Kill switch with redundant publishing (3x)
- Health monitoring heartbeat

✅ **Homomorphic Encryption** (in development)
- TenSEAL CKKS scheme for identity handshake
- Computation on encrypted neural data
- Future: Secure multi-party computation

✅ **Latency Monitoring** (`latency_injector.py`)
- Tracks round-trip time (RTT)
- Alerts if >50ms (Libet buffer requirement)
- Automatic degradation if latency unsafe

✅ **Exception Handling**
- Try-catch blocks in critical callbacks
- Graceful degradation on errors
- Error logging with context

### Planned (Security Roadmap)

#### v0.8 (Hardening Phase)
- [ ] Comprehensive input validation for all ROS messages
- [ ] Attack simulation test suite
- [ ] Security documentation and threat modeling
- [ ] Audit logging for all critical events

#### v0.9 (Authentication Phase)
- [ ] SROS2 integration with topic permissions
- [ ] Unity bridge authentication (mutual TLS)
- [ ] Rate limiting on neural topics
- [ ] Session management and replay protection

#### v1.0 (Production-Ready)
- [ ] Hardware Security Module (HSM) integration
- [ ] Encrypted DDS transport
- [ ] Intrusion detection system
- [ ] Formal security audit
- [ ] Compliance with medical device standards (if applicable)

---

## Incident Response

### If You Detect an Attack

1. **Immediate**: Trigger kill switch
   ```bash
   ros2 topic pub /kill_switch std_msgs/msg/Bool "{data: true}"
   ```

2. **Isolate**: Disconnect network
   ```bash
   sudo ifconfig eth0 down
   ```

3. **Document**: Save logs
   ```bash
   ros2 bag record -a -o incident_$(date +%Y%m%d_%H%M%S)
   ```

4. **Report**: Email security@todosloscobardesdelvalle.com

5. **Analyze**: Review ROS logs
   ```bash
   cat ~/.ros/log/*/rosout.log | grep "ERROR\|WARN"
   ```

---

## Testing & Validation

### Security Test Suite

Run security tests:
```bash
# Unit tests for neural firewall
pytest test/test_neural_firewall.py -v

# Attack simulation (coming soon)
pytest test/security/test_brainjacking_defense.py

# Fuzzing tests (coming soon)
pytest test/security/test_fuzzing.py
```

### Penetration Testing

Planned penetration testing:
- **v0.8**: Internal team testing
- **v0.9**: External security researcher review
- **v1.0**: Professional penetration test

---

## Responsible Disclosure

### Timeline

1. **Report Received**: Acknowledge within 24 hours (critical) or 48 hours (others)
2. **Triage**: Assess severity within 48 hours
3. **Fix Development**:
   - Critical: 7 days
   - High: 30 days
   - Medium: 60 days
4. **Public Disclosure**: 90 days after patch, or coordinated earlier

### Recognition

Security researchers will be:
- Credited in CONTRIBUTORS.md
- Acknowledged in release notes
- Listed in Security Hall of Fame
- Co-authors on security papers (if applicable)

---

## References

### Security Research

**Brainjacking**:
- Pycroft et al. (2016). World Neurosurgery, 92, 454-462.
- Denning et al. (2009). "Neurosecurity." *IEEE Technology and Society Magazine*.

**Medical Device Security**:
- FDA (2014). "Content of Premarket Submissions for Management of Cybersecurity in Medical Devices."
- IEC 62443: Industrial automation and control systems security

**Consciousness Safety**:
- Libet, B. (1983). "Time of conscious intention to act." *Brain*, 106(3), 623-642.
- Used for latency budget (500ms Libet buffer)

---

## Contact

**Security Issues**: security@todosloscobardesdelvalle.com

**General Questions**: [GitHub Discussions](https://github.com/Zae-Project/neutral-consciousness-engine/discussions)

**Bug Reports**: [GitHub Issues](https://github.com/Zae-Project/neutral-consciousness-engine/issues)

---

**Last Updated**: January 22, 2026
**Version**: 1.0.0
**Next Security Review**: March 2026
