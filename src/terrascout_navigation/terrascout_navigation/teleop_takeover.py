#!/usr/bin/env python3


from functools import partial

import rclpy
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import TwistStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from fleet_interfaces.srv import ControlMission

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

        self._service_group = ReentrantCallbackGroup()
        self._client_group = ReentrantCallbackGroup()

        self._takeover_pub = self.create_publisher(Bool, "teleop_takeover", latched_qos())
        self._estop_lock_pub = self.create_publisher(Bool, "estop_lock", 1)
        self._autonomy_lock_pub = self.create_publisher(Bool, "autonomy_lock", 1)
        self._cancel_clients = [
            self.create_client(CancelGoal, name, callback_group=self._client_group)
            for name in CANCEL_SERVICES
        ]
        self._mission_control = self.create_client(
            ControlMission, "control_mission", callback_group=self._client_group
        )

        self._estop_engaged = False
        # A clean boot has no mission to restore, so permit mission submission.
        # mission_server separately requests a lock when it restores a mission.
        self._manual = False
        self._last_operator_command = None
        self._startup_cancel_sent = False
        self._startup_cancel_done = False
        self._takeover_pub.publish(Bool(data=False))

        self.create_subscription(TwistStamped, "cmd_vel_joy", self._operator_cb, 10)
        self.create_subscription(Bool, "estop", self._estop_cb, 10)
        self.create_subscription(Bool, "teleop_takeover", self._takeover_state_cb, latched_qos())
        self.create_subscription(Bool, "mission_restore_lock", self._restore_lock_cb, latched_qos())
        self.create_service(
            Trigger, "resume_autonomy", self._resume_cb,
            callback_group=self._service_group,
        )
        self.create_timer(LOCK_PERIOD_S, self._publish_locks)
        self.create_timer(LOCK_PERIOD_S, self._cancel_navigation_on_startup)

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

    def _takeover_state_cb(self, msg: Bool) -> None:
        if msg.data and not self._manual:
            self._enter_manual("Mission server requested a safe startup lock")

    def _restore_lock_cb(self, msg: Bool) -> None:
        if msg.data and not self._manual:
            self._enter_manual("A suspended mission was restored; operator resume required")

    def _publish_locks(self) -> None:
        self._estop_lock_pub.publish(Bool(data=self._estop_engaged))
        self._autonomy_lock_pub.publish(Bool(data=self._manual))

    def _cancel_navigation_on_startup(self) -> None:
        if self._startup_cancel_sent or not self._cancel_clients:
            return
        if not all(client.service_is_ready() for client in self._cancel_clients):
            return
        for client in self._cancel_clients:
            client.call_async(CancelGoal.Request()).add_done_callback(
                partial(self._check_startup_cancel, client.srv_name)
            )
        self._startup_cancel_sent = True

    def _check_startup_cancel(self, service: str, future) -> None:
        self._check_cancel(service, future)
        self._startup_cancel_done = True

    async def _resume_cb(self, request, response):
        response.success = False
        if not self._startup_cancel_done:
            response.message = "startup navigation cancellation is not confirmed; remain stopped"
            return response
        if self._estop_engaged:
            response.message = "the soft e-stop is engaged"
            return response
        if (
            self._last_operator_command is not None
            and self.get_clock().now() - self._last_operator_command < OPERATOR_ACTIVE_WINDOW
        ):
            response.message = "operator commands arrived within the last second"
            return response
        if self._manual:
            if not self._mission_control.service_is_ready():
                response.message = "mission control service is unavailable; manual control remains active"
                return response
            mission_request = ControlMission.Request()
            mission_request.request = ControlMission.Request.RESUME
            try:
                mission_response = await self._mission_control.call_async(mission_request)
            except Exception as error:
                response.message = f"mission resume request failed; manual control remains active: {error}"
                return response
            if not mission_response.accepted and mission_response.reason != "no active mission":
                response.message = mission_response.reason
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
