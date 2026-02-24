from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'neutral_consciousness'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'nengo', 'numpy'],
    zip_safe=True,
    maintainer='Neutral Consciousness Team',
    maintainer_email='info@theconsciousness.ai',
    description='The Neutral Consciousness Engine - SNN-based Generative Model for the Watanabe Transfer Protocol',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Core SNN Nodes
            'cortex_node = neutral_consciousness.cortex_snn.visual_cortex:main',
            'dream_node = neutral_consciousness.cortex_snn.dream_engine:main',
            'tectum_node = neutral_consciousness.cortex_snn.sensory_tectum:main',
            'limbic_node = neutral_consciousness.cortex_snn.limbic_node:main',
            'reentrant_node = neutral_consciousness.cortex_snn.reentrant_processor:main',

            # Neural Firewall Nodes
            'firewall_node = neutral_consciousness.neural_firewall.traffic_monitor:main',
            'latency_injector_node = neutral_consciousness.neural_firewall.latency_injector:main',
            'he_node = neutral_consciousness.neural_firewall.homomorphic_encryption:main',

            # Test Nodes
            'split_brain_node = neutral_consciousness.tests.split_brain_test:main',
        ],
    },
)
