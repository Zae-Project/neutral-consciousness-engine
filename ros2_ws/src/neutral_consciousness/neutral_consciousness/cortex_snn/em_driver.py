"""
Ambient EM Driver

Publishes a simulated ambient electromagnetic field on
`/environment/em_field_raw` as a single-channel Float32MultiArray. The
default waveform is the Schumann cavity fundamental (7.83 Hz) plus the
first four harmonics (14, 20, 26, 33 Hz), which overlap the EEG
theta / alpha / beta bands.

This is purely in-silico: it does NOT model a physical field. It is the
shared exogenous reference that the biological and synthetic hemispheres
must co-entrain to for the Uni-hemispheric Subjective Protocol to admit a
hemispheric switch. The signal is routed through the Neural Firewall before
reaching the cortex, so a malicious publisher injecting seizure-inducing
harmonics is rejected at the same gate as any other neural stream.

Parameters:
    em_mode (str): one of 'schumann', 'noise', 'silent'. Default 'schumann'.
    publish_rate_hz (float): output sample rate. Default 1000.0.
    output_voltage_mv (float): peak amplitude in mV-equivalent units.
        Kept well below the 100 mV firewall cap. Default 5.0.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np


SCHUMANN_HARMONICS_HZ = (7.83, 14.3, 20.8, 27.3, 33.8)


class EMDriverNode(Node):
    def __init__(self):
        super().__init__('em_driver')

        self.declare_parameter('em_mode', 'schumann')
        self.declare_parameter('publish_rate_hz', 1000.0)
        self.declare_parameter('output_voltage_mv', 5.0)
        self.declare_parameter('noise_seed', 0)

        self.mode = self.get_parameter('em_mode').value
        self.rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.amp_mv = float(self.get_parameter('output_voltage_mv').value)
        self._rng = np.random.default_rng(
            int(self.get_parameter('noise_seed').value)
        )

        self.pub = self.create_publisher(
            Float32MultiArray, '/environment/em_field_raw', 10
        )

        self._t = 0.0
        self._dt = 1.0 / self.rate_hz
        self.timer = self.create_timer(self._dt, self._tick)

        self.get_logger().info(
            f'EM Driver started: mode={self.mode} rate={self.rate_hz:.0f}Hz '
            f'amp={self.amp_mv:.1f}mV'
        )

    def _sample(self, t: float) -> float:
        if self.mode == 'silent':
            return 0.0
        if self.mode == 'noise':
            return float(self._rng.normal(0.0, self.amp_mv))
        # 'schumann' — fundamental + harmonics with decaying weights
        weights = np.array([1.0, 0.5, 0.3, 0.2, 0.15])
        weights = weights / np.sum(weights)
        val = 0.0
        for w, f in zip(weights, SCHUMANN_HARMONICS_HZ):
            val += w * np.sin(2.0 * np.pi * f * t)
        return float(self.amp_mv * val)

    def _tick(self):
        sample = self._sample(self._t)
        msg = Float32MultiArray()
        msg.data = [sample]
        self.pub.publish(msg)
        self._t += self._dt


def generate_schumann_window(
    duration_sec: float,
    sample_rate_hz: float = 1000.0,
    amp_mv: float = 5.0,
) -> np.ndarray:
    """Offline helper used by unit tests.

    Returns a 1-D array of Schumann samples over `duration_sec`.
    """
    n = int(duration_sec * sample_rate_hz)
    t = np.arange(n) / sample_rate_hz
    weights = np.array([1.0, 0.5, 0.3, 0.2, 0.15])
    weights = weights / weights.sum()
    signal = np.zeros(n)
    for w, f in zip(weights, SCHUMANN_HARMONICS_HZ):
        signal += w * np.sin(2.0 * np.pi * f * t)
    return amp_mv * signal


def main(args=None):
    rclpy.init(args=args)
    node = EMDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
