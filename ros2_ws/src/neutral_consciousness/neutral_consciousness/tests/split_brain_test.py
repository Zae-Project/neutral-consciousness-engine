"""
Split Brain Test Node

Implements the "Uni-hemispheric Subjective Protocol".
managed the transition from "Shadow Mode" to full "Hemispheric Transfer".

Logic:
- Subscribes to Biological (Left) and Machine (Right) inputs.
- Shadow Mode: GATES the machine output (learning only).
- Active Mode: MERGES machine output with biological output.
- Switch gate (dual-criterion, hysteresis):
    sync_health >= 0.95  AND  plv >= 0.8
  Both must be sustained for GATE_HYSTERESIS_SEC consecutive seconds
  before the switch is admitted. A single fluctuation is not enough —
  a productive-only synthesis that lacks transmissive entrainment is
  rejected, and a transmissive coincidence without predictive agreement
  is equally rejected.
- `/transmissive_sync/gate_ready` (std_msgs/Bool) exposes the sustained
  gate state for observability.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool
from std_srvs.srv import Trigger


SYNC_HEALTH_THRESHOLD = 0.95
PLV_THRESHOLD = 0.8
GATE_HYSTERESIS_SEC = 3.0
GATE_TICK_HZ = 10.0


class SplitBrainTest(Node):
    def __init__(self):
        super().__init__('split_brain_test')

        # State
        self.shadow_mode = True
        self.sync_health = 0.0
        self.plv = 0.0

        # Hysteresis: count how long both criteria have been simultaneously
        # satisfied. Reset to zero the moment either drops below threshold.
        self._gate_sustained_sec = 0.0
        self._gate_ready = False

        # Subscribers
        self.left_eye_sub = self.create_subscription(
            Image, '/camera/left_eye', self.process_bio_input, 10
        )  # Biological
        self.right_eye_sub = self.create_subscription(
            Image, '/camera/right_eye', self.process_machine_input, 10
        )  # Synthetic

        self.health_sub = self.create_subscription(
            Float32, '/synchronization_health', self.update_health, 10
        )
        self.plv_sub = self.create_subscription(
            Float32, '/transmissive_sync/plv', self.update_plv, 10
        )

        # Publisher (The Unified Conscious Field)
        self.unified_field_pub = self.create_publisher(
            Image, '/conscious_output/unified_field', 10
        )

        # Gate-ready publisher so external observers can tell whether a
        # `/trigger_hemispheric_switch` call would currently succeed.
        self.gate_ready_pub = self.create_publisher(
            Bool, '/transmissive_sync/gate_ready', 10
        )

        # Service
        self.switch_srv = self.create_service(
            Trigger, '/trigger_hemispheric_switch', self.handle_switch
        )

        # Gate tick — updates sustained-time bookkeeping.
        self._gate_dt = 1.0 / GATE_TICK_HZ
        self.gate_timer = self.create_timer(self._gate_dt, self._gate_tick)

        self.get_logger().info(
            "Split Brain Test Node Initialized. Mode: SHADOW (Gated). "
            f"Gate requires sync>={SYNC_HEALTH_THRESHOLD} AND "
            f"plv>={PLV_THRESHOLD} for {GATE_HYSTERESIS_SEC}s."
        )

    def update_health(self, msg):
        self.sync_health = msg.data

    def update_plv(self, msg):
        self.plv = msg.data

    def _gate_tick(self):
        both_ok = (
            self.sync_health >= SYNC_HEALTH_THRESHOLD
            and self.plv >= PLV_THRESHOLD
        )
        if both_ok:
            self._gate_sustained_sec += self._gate_dt
        else:
            self._gate_sustained_sec = 0.0

        prev = self._gate_ready
        self._gate_ready = self._gate_sustained_sec >= GATE_HYSTERESIS_SEC

        if self._gate_ready != prev:
            self.get_logger().info(
                f"Transmissive gate {'OPEN' if self._gate_ready else 'CLOSED'} "
                f"(sync={self.sync_health:.2f}, plv={self.plv:.2f})"
            )

        ready_msg = Bool()
        ready_msg.data = self._gate_ready
        self.gate_ready_pub.publish(ready_msg)

    def process_bio_input(self, msg):
        # Biological side is always active/published (until death)
        # For this test, we might just pass it through or merge
        # But instructions verify logic, so let's just log or re-publish
        # Simple implementation: Re-publish as part of unified field
        if self.shadow_mode:
            # Only bio is driving
            self.unified_field_pub.publish(msg)

    def process_machine_input(self, msg):
        if self.shadow_mode:
            # GATED: Do not affect the output
            return
        else:
            # ACTIVE: This would merge with bio input
            # For this test, we publish it (simulating the synthetic hemisphere taking control or joining)
            self.unified_field_pub.publish(msg)

    def handle_switch(self, request, response):
        if not self._gate_ready:
            response.success = False
            response.message = (
                f"ABORT: Transmissive gate not open "
                f"(sync={self.sync_health:.2f} [>= {SYNC_HEALTH_THRESHOLD}?], "
                f"plv={self.plv:.2f} [>= {PLV_THRESHOLD}?], "
                f"sustained={self._gate_sustained_sec:.1f}s / "
                f"{GATE_HYSTERESIS_SEC}s)"
            )
            self.get_logger().error(response.message)
        else:
            self.shadow_mode = not self.shadow_mode
            status = "ACTIVE" if not self.shadow_mode else "SHADOW"
            response.success = True
            response.message = f"Hemispheric Switch Success. New Mode: {status}"
            self.get_logger().warn(f"SWITCH TRIGGERED: {status} MODE ENGAGED")

        return response


def main(args=None):
    rclpy.init(args=args)
    node = SplitBrainTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
