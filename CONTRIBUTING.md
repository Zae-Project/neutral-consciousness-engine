# Contributing to Neutral Consciousness Engine

Thank you for your interest in contributing to the Neutral Consciousness Engine! This project maintains strict scientific and professional standards to ensure all work is grounded in legitimate research.

---

## Table of Contents

1. [Code of Conduct](#code-of-conduct)
2. [Scientific Citation Requirements](#scientific-citation-requirements)
3. [How to Contribute](#how-to-contribute)
4. [Documentation Standards](#documentation-standards)
5. [Code Standards](#code-standards)
6. [Testing Requirements](#testing-requirements)
7. [Pull Request Process](#pull-request-process)

---

## Code of Conduct

### Our Commitment

This project is dedicated to advancing consciousness research through:
- **Scientific rigor**: All claims must be backed by peer-reviewed research
- **Professional standards**: Code and documentation must be production-quality
- **Ethical responsibility**: Safety and ethics are paramount
- **Intellectual honesty**: Clear distinction between established science and speculation

### Unacceptable Behavior

- Pseudoscience or unsupported claims
- Plagiarism or uncited use of others' work
- Misrepresentation of research findings
- Code that violates safety principles
- Disrespectful or unprofessional conduct

---

## Scientific Citation Requirements

### ⚠️ MANDATORY: All Scientific Claims Must Be Cited

**Every** statement about neuroscience, consciousness, or technical capabilities must include:

1. **Primary Source Citation**
   ```markdown
   **Author, A., et al. (Year).** "Title of paper." *Journal Name*, Volume(Issue), Pages.
   DOI: [link]
   ```

2. **Relevance Statement**
   ```markdown
   **Relevance**: Brief explanation of why this research supports the claim
   ```

### Example - Correct Citation

```markdown
## Predictive Coding in Visual Cortex

The system implements predictive coding based on:

**Rao, R.P., & Ballard, D.H. (1999).**  
"Predictive coding in the visual cortex: a functional interpretation of some
extra-classical receptive-field effects."  
*Nature Neuroscience*, 2(1), 79-87.  
DOI: [10.1038/4580](https://doi.org/10.1038/4580)

**Relevance**: Demonstrates that V1 neurons compute prediction error, validating
our implementation of Error = Input - Prediction in visual_cortex.py
```

### Example - Incorrect (Missing Citation)

```markdown
## Predictive Coding in Visual Cortex

❌ The brain uses predictive coding to minimize surprise.

(No citation - unacceptable!)
```

### Watanabe Attribution

When referencing Professor Watanabe's work, **always include**:

```markdown
**IMPORTANT**: This work is inspired by Professor Masataka Watanabe's research
approach. This is NOT his official work. We are independent researchers applying
principles from his publications.
```

**Required Watanabe Citation:**
```markdown
**Watanabe, M., et al. (2014).**  
"Interhemispheric transfer of visual information in split-brain patients."  
*Neuropsychologia*, 63, 133-142.  
DOI: [10.1016/j.neuropsychologia.2014.08.025](https://doi.org/10.1016/j.neuropsychologia.2014.08.025)
```

### Distinguishing Fact from Speculation

**Use these tags to clarify status:**

- `[ESTABLISHED]`: Peer-reviewed, replicated science
- `[THEORETICAL]`: Proposed but not yet validated
- `[SPECULATIVE]`: Exploratory ideas requiring validation
- `[UNPROVEN]`: No empirical support yet

**Example:**

```markdown
## Consciousness Substrate Independence

[THEORETICAL] This project explores whether consciousness can operate on
synthetic substrates. While split-brain research [ESTABLISHED] shows hemisphere
independence, substrate transfer remains [UNPROVEN].
```

---

## How to Contribute

### Types of Contributions

We welcome contributions in:

1. **Documentation**
   - Improving clarity
   - Adding scientific references
   - Creating tutorials
   - Fixing errors

2. **Code**
   - Bug fixes
   - Performance improvements
   - New ROS 2 nodes
   - Test coverage

3. **Research**
   - Literature reviews
   - Scientific validation
   - Experimental protocols
   - Data analysis

4. **Testing**
   - Integration tests
   - Performance benchmarks
   - Safety validation
   - Edge case identification

### Before You Start

1. **Check existing issues**: Avoid duplicate work
2. **Read documentation**: Understand the project architecture
3. **Review scientific foundation**: Know the research basis
4. **Discuss major changes**: Open an issue for discussion first

---

## Documentation Standards

### Structure

All documentation must include:

1. **Title and Overview**
   - Clear, descriptive title
   - 1-2 sentence summary
   - Estimated reading time

2. **Scientific Foundation**
   - Primary research citations
   - Relevance statements
   - Distinction between fact and speculation

3. **Technical Details**
   - Implementation specifics
   - Code examples
   - Usage instructions

4. **References**
   - Complete bibliography
   - DOI links
   - Access information

### Formatting

**Use Markdown with:**
- Headers: `#`, `##`, `###`
- Code blocks: ` ```python ` with language specified
- Tables: For structured data
- Lists: For sequential information
- Links: Always use descriptive text, not bare URLs

**Example:**
```markdown
See [Friston (2010)](https://doi.org/10.1038/nrn2787) for free energy principle.

❌ Don't: https://doi.org/10.1038/nrn2787
```

### Required Sections

Every documentation file must have:

```markdown
# Title

**Brief Description**

---

## Overview
[What this document covers]

## Scientific Foundation
[Research basis with citations]

## Technical Details
[Implementation specifics]

## References
[Complete bibliography]

---

**Last Updated**: [Date]  
**Version**: [Semantic version]  
**Status**: [Draft/Review/Final]
```

---

## Code Standards

### Python Code

**Style Guide**: Follow PEP 8

**Required Elements:**

1. **Module Docstring**
   ```python
   """
   Module Name - Brief Description
   
   Longer description of module purpose.
   
   SCIENTIFIC FOUNDATION:
   - Author, A. (Year). Citation with DOI
   - Relevance statement
   
   NOTE: Watanabe attribution if applicable
   """
   ```

2. **Function Docstrings**
   ```python
   def compute_prediction_error(input_signal, prediction):
       """
       Compute prediction error as per Rao & Ballard (1999).
       
       Args:
           input_signal (np.ndarray): Actual sensory input
           prediction (np.ndarray): Model prediction
           
       Returns:
           np.ndarray: Prediction error (Input - Prediction)
           
       References:
           Rao & Ballard (1999). Predictive coding in visual cortex.
           Nature Neuroscience, 2(1), 79-87.
       """
       return input_signal - prediction
   ```

3. **Inline Comments**
   ```python
   # Libet's 500ms buffer (Libet et al., 1983)
   LIBET_LIMIT_MS = 500.0
   ```

### ROS 2 Nodes

**Required Documentation:**

1. **Node Description**: What the node does
2. **Topics Published**: With message types
3. **Topics Subscribed**: With message types
4. **Services**: With service types
5. **Parameters**: With defaults and ranges
6. **Scientific Basis**: Citations for algorithms

**Example:**
```python
"""
Visual Cortex Node - Predictive Coding SNN

Implements predictive coding (Rao & Ballard, 1999) using Nengo SNNs.

Topics Published:
- /neural_data/prediction_error (Float32MultiArray): Prediction error signal
- /synchronization_health (Float32): Health metric (0.0-1.0)

Topics Subscribed:
- unity/camera/raw (Image): Visual input from Unity

Parameters:
- input_dim (int): Input vector dimensionality [default: 64]

References:
- Rao, R.P., & Ballard, D.H. (1999). Nature Neuroscience, 2(1), 79-87.
"""
```

### Safety-Critical Code

**Code involving neural data, encryption, or safety must:**

1. **Include Threat Model**: What attacks are you defending against?
2. **Reference Security Research**: Cite Pycroft et al. (2016) for brainjacking
3. **Implement Defense in Depth**: Multiple layers of protection
4. **Fail Securely**: Default to safe state on error

**Example:**
```python
def inspect_packet(self, msg):
    """
    Inspect neural data for brainjacking signatures (Pycroft et al., 2016).
    
    Threat Model:
    - High-frequency seizure induction (>150 Hz)
    - Over-voltage excitotoxicity (>100 mV)
    
    Defense: Frequency analysis + amplitude limiting + kill switch
    
    References:
    - Pycroft et al. (2016). Brainjacking. World Neurosurgery, 92, 454-462.
    """
    if self.calculate_frequency(data) > self.MAX_FREQUENCY_HZ:
        self.trigger_kill_switch("Brainjacking attack detected")
```

---

## Testing Requirements

### Unit Tests

**Required for:**
- All new functions
- All bug fixes
- All algorithmic changes

**Test Structure:**
```python
import pytest
import numpy as np
from neutral_consciousness.cortex_snn import visual_cortex

def test_prediction_error_computation():
    """
    Test prediction error follows Rao & Ballard (1999) definition.
    
    Validates: Error = Input - Prediction
    """
    input_signal = np.array([1.0, 2.0, 3.0])
    prediction = np.array([0.9, 2.1, 2.8])
    expected_error = np.array([0.1, -0.1, 0.2])
    
    error = visual_cortex.compute_prediction_error(input_signal, prediction)
    
    assert np.allclose(error, expected_error, atol=0.01)
```

### Integration Tests

**Required for:**
- New ROS 2 nodes
- Topic communication
- System-level features

**Test Structure:**
```python
def test_cortex_dream_integration():
    """
    Test Visual Cortex → Dream Engine integration.
    
    Validates prediction error flows correctly between nodes.
    """
    # Launch nodes
    # Publish test data
    # Verify topic communication
    # Check prediction accuracy
```

### Performance Tests

**Required for:**
- Latency-critical code
- Real-time constraints
- Resource usage

**Example:**
```python
def test_latency_within_libet_limit():
    """
    Verify total RTT < 200ms (Libet buffer requirement).
    
    References:
    - Libet et al. (1983). Brain, 106(3), 623-642.
    """
    start = time.time()
    # Simulate full pipeline
    end = time.time()
    latency_ms = (end - start) * 1000
    
    assert latency_ms < 200, f"Latency {latency_ms}ms exceeds Libet limit"
```

---

## Pull Request Process

### Before Submitting

- [ ] All tests pass locally
- [ ] Code follows style guide
- [ ] Documentation updated
- [ ] Scientific citations included
- [ ] No merge conflicts with main

### PR Title Format

```
[Type]: Brief description

Examples:
[Docs]: Add predictive coding references to Visual Cortex
[Feature]: Implement STDP learning rule
[Fix]: Correct latency calculation in injector node
[Test]: Add integration tests for Neural Firewall
```

### PR Description Template

```markdown
## Summary
Brief description of changes

## Scientific Basis
Citations supporting this work

## Changes Made
- Bullet list of specific changes

## Testing
How this was tested

## Related Issues
Closes #123
```

### Review Checklist

Reviewers will verify:

- [ ] **Scientific Citations**: All claims cited
- [ ] **Code Quality**: Follows standards
- [ ] **Tests**: Adequate coverage
- [ ] **Documentation**: Clear and complete
- [ ] **Safety**: No security vulnerabilities
- [ ] **Ethics**: Follows ethical guidelines

### Approval Requirements

- **2 approvals** for code changes
- **1 approval + scientific validation** for algorithmic changes
- **3 approvals** for safety-critical changes

---

## Getting Help

### Resources

- **Documentation**: [docs/](docs/)
- **Examples**: [examples/](examples/) (coming soon)
- **Issues**: [GitHub Issues](https://github.com/Zae-Project/neutral-consciousness-engine/issues)
- **Discussions**: [GitHub Discussions](https://github.com/Zae-Project/neutral-consciousness-engine/discussions)

### Research Support

- **Bibliography**: [Zae Project Bibliography](https://github.com/Zae-Project/zae-docs/blob/main/reference/bibliography.md)
- **Researchers**: [Researchers Directory](https://github.com/Zae-Project/zae-docs/blob/main/reference/researchers-directory.md)

### Contact

- **General Questions**: Open a GitHub Discussion
- **Bug Reports**: Open a GitHub Issue
- **Security Issues**: Email (see SECURITY.md) - coming soon

---

## Recognition

Contributors will be:
- Listed in CONTRIBUTORS.md
- Acknowledged in release notes
- Co-authors on academic publications (if applicable)

---

## License

By contributing, you agree that your contributions will be licensed under the MIT License.

---

**Thank you for helping advance consciousness research with scientific rigor!**

*Last Updated*: January 15, 2026  
*Version*: 1.0.0
