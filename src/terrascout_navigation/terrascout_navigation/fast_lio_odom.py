#!/usr/bin/env python3
"""Convert FAST-LIO2 IMU/body odometry to Nav2 odometry and TF."""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster

from fleet_common import signals
from fleet_common.params import declare

ODOM_FRAME = "odom"
BASE_FRAME = "base_link"
ROOT_FRAME = "vrtk_link"
IMU_Z_IN_BASE = 0.15
IMU_Z_IN_ROOT = 0.48


def _rotate(q, vector):
    """Rotate vector by quaternion q=(x,y,z,w)."""
    x, y, z, w = q
    vx, vy, vz = vector
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + y * tz - z * ty,
        vy + w * ty + z * tx - x * tz,
        vz + w * tz + x * ty - y * tx,
    )


def _multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


class FastLioOdomAdapter(Node):
    def __init__(self):
        super().__init__("fast_lio_odom_adapter")
        self._publisher = self.create_publisher(Odometry, "odom", 20)
        self._tf = TransformBroadcaster(self)
        self._previous = None
        self._filtered_twist = None
        self._alpha = float(
            declare(
                self,
                "velocity_smoothing_alpha",
                rclpy.Parameter.Type.DOUBLE,
                "Weight of the new velocity estimate; 1 disables smoothing",
                0.01,
                1.0,
            )
        )
        self._subscription = self.create_subscription(
            Odometry,
            "fast_lio/odometry",
            self._on_odometry,
            qos_profile_sensor_data,
        )
        self.get_logger().warning(
            "FAST-LIO2 provides Nav2 odometry; inspect drift before physical use"
        )

    def _on_odometry(self, source: Odometry) -> None:
        if (source.header.frame_id, source.child_frame_id) != ("camera_init", "body"):
            self.get_logger().error(
                f"Expected FAST-LIO frames camera_init -> body; received "
                f"{source.header.frame_id} -> {source.child_frame_id}"
            )
            return
        pose = source.pose.pose
        q = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        norm = math.sqrt(sum(value * value for value in q))
        position = (pose.position.x, pose.position.y, pose.position.z)
        if not math.isfinite(norm) or norm < 1e-8 or not all(map(math.isfinite, position)):
            self.get_logger().error("Ignoring invalid FAST-LIO pose")
            return
        q = tuple(value / norm for value in q)
        base = _rotate(q, (0.0, 0.0, -IMU_Z_IN_BASE))
        root = _rotate(q, (0.0, 0.0, -IMU_Z_IN_ROOT))
        base_position = tuple(position[i] + base[i] for i in range(3))
        root_position = tuple(position[i] + root[i] for i in range(3))
        stamp = source.header.stamp.sec + source.header.stamp.nanosec * 1e-9

        output = Odometry()
        output.header = source.header
        output.header.frame_id = ODOM_FRAME
        output.child_frame_id = BASE_FRAME
        output.pose.pose.position.x, output.pose.pose.position.y, output.pose.pose.position.z = base_position
        (output.pose.pose.orientation.x, output.pose.pose.orientation.y,
         output.pose.pose.orientation.z, output.pose.pose.orientation.w) = q
        output.pose.covariance = list(source.pose.covariance)

        if self._previous is None:
            self._previous = (stamp, base_position, q)
            return
        old_stamp, old_position, old_q = self._previous
        dt = stamp - old_stamp
        if not 0.0 < dt <= 0.5:
            self._previous = (stamp, base_position, q)
            self._filtered_twist = None
            return
        speed = math.dist(base_position, old_position) / dt
        if speed > 3.0:
            self.get_logger().error(f"Ignoring FAST-LIO odometry jump ({speed:.2f} m/s)")
            self._previous = (stamp, base_position, q)
            self._filtered_twist = None
            return
        world_velocity = tuple((base_position[i] - old_position[i]) / dt for i in range(3))
        inverse_q = (-q[0], -q[1], -q[2], q[3])
        linear = _rotate(inverse_q, world_velocity)
        dq = _multiply((-old_q[0], -old_q[1], -old_q[2], old_q[3]), q)
        if dq[3] < 0.0:
            dq = tuple(-v for v in dq)
        vnorm = math.sqrt(sum(dq[i] * dq[i] for i in range(3)))
        angular = (0.0, 0.0, 0.0) if vnorm < 1e-8 else tuple(
            dq[i] * (2.0 * math.atan2(vnorm, dq[3]) / (vnorm * dt)) for i in range(3)
        )
        twist = [*linear, *angular]
        if self._filtered_twist is not None:
            twist = [self._alpha * n + (1.0 - self._alpha) * old
                     for n, old in zip(twist, self._filtered_twist)]
        self._filtered_twist = twist
        (output.twist.twist.linear.x, output.twist.twist.linear.y,
         output.twist.twist.linear.z, output.twist.twist.angular.x,
         output.twist.twist.angular.y, output.twist.twist.angular.z) = twist
        output.twist.covariance = list(source.twist.covariance)
        self._publisher.publish(output)

        transform = TransformStamped()
        transform.header = output.header
        transform.child_frame_id = ROOT_FRAME
        (transform.transform.translation.x, transform.transform.translation.y,
         transform.transform.translation.z) = root_position
        transform.transform.rotation = output.pose.pose.orientation
        self._tf.sendTransform(transform)
        self._previous = (stamp, base_position, q)


def main():
    signals.init()
    node = FastLioOdomAdapter()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
