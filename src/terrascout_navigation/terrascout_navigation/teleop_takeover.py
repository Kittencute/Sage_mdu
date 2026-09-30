#!/usr/bin/env python3


from functools import partial

import rclpy
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import TwistStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from fleet_common import signals
from fleet_common.qos import latched_qos

CANCEL_SERVICES = ("navigate_to_pose/_action/cancel_goal",)
LOCK_PERIOD_S = 0.1
OPERATOR_ACTIVE_WINDOW = Duration(seconds=1.0)


def detect_rising_edge(prev: bool, curr: bool) -> bool:
    return curr and not prev


class TeleopTakeoverNode(Node):
    def __init__(self):
        super().__init__("teleop_takeover")

        self._takeover_pub = self.create_publisher(Bool, "teleop_takeover", latched_qos())
        self._takeover_pub.publish(Bool(data=False))
        self._estop_lock_pub = self.create_publisher(Bool, "estop_lock", 1)
        self._autonomy_lock_pub = self.create_publisher(Bool, "autonomy_lock", 1)

        self._cancel_clients = [self.create_client(CancelGoal, name) for name in CANCEL_SERVICES]

        self._estop_engaged = False
        self._manual = False
        self._last_operator_command = None

        self.create_subscription(TwistStamped, "cmd_vel_joy", self._operator_cb, 10)
        self.create_subscription(Bool, "estop", self._estop_cb, 10)
        self.create_service(Trigger, "resume_autonomy", self._resume_cb)
        self.create_timer(LOCK_PERIOD_S, self._publish_locks)

        self.get_logger().info(f"teleop_takeover ready: cancel services={CANCEL_SERVICES}")

    def _operator_cb(self, msg: TwistStamped) -> None:
        self._last_operator_command = self.get_clock().now()
        linear, angular = msg.twist.linear, msg.twist.angular
        if any((linear.x, linear.y, linear.z, angular.x, angular.y, angular.z)):
            self._enter_manual("Operator took control")

    def _estop_cb(self, msg: Bool) -> None:
        engaged = detect_rising_edge(self._estop_engaged, msg.data)
        self._estop_engaged = msg.data
        if engaged:
            self._enter_manual("Soft e-stop engaged")
        self._publish_locks()

    def _publish_locks(self) -> None:
        self._estop_lock_pub.publish(Bool(data=self._estop_engaged))
        self._autonomy_lock_pub.publish(Bool(data=self._manual))

    def _resume_cb(self, request, response):
        response.success = False
        if self._estop_engaged:
            response.message = "the soft e-stop is engaged"
            return response
        if (
            self._last_operator_command is not None
            and self.get_clock().now() - self._last_operator_command < OPERATOR_ACTIVE_WINDOW
        ):
            response.message = "operator commands arrived within the last second"
            return response
        self.get_logger().warning("Autonomy resumed; manual control released")
        self._manual = False
        self._takeover_pub.publish(Bool(data=False))
        self._publish_locks()
        response.success = True
        response.message = "autonomy resumed"
        return response

    def _enter_manual(self, cause: str) -> None:
        if self._manual:
            return
        self.get_logger().warning(f"{cause} — cancelling autonomous navigation")
        self._manual = True
        self._publish_locks()
        self._takeover_pub.publish(Bool(data=True))
        for client in self._cancel_clients:
            if not client.service_is_ready():
                raise RuntimeError(
                    f"cancel {client.srv_name} failed: cause: the service is not ready"
                )
            client.call_async(CancelGoal.Request()).add_done_callback(
                partial(self._check_cancel, client.srv_name)
            )

    def _check_cancel(self, service: str, future) -> None:
        code = future.result().return_code
        if code != CancelGoal.Response.ERROR_NONE:
            raise RuntimeError(f"cancel {service} failed: cause: return code {code}")


def main():
    signals.init()
    node = TeleopTakeoverNode()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
