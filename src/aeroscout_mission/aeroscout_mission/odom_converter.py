#!/usr/bin/env python3


import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleGlobalPosition, VehicleOdometry
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import NavSatFix, NavSatStatus
from tf2_ros import TransformBroadcaster

from fleet_common import signals

INPUT_TOPIC = "fmu/out/vehicle_odometry"
OUTPUT_TOPIC = "odom"
ODOM_FRAME = "odom"
BASE_FRAME = "base_link"
COVARIANCE_DIAGONAL = (0.01, 0.01, 0.01, 0.001, 0.001, 0.001)


class OdomConverter(Node):
    def __init__(self):
        super().__init__("odom_converter")

        self.subscription = self.create_subscription(
            VehicleOdometry,
            INPUT_TOPIC,
            self.listener_callback,
            qos_profile_sensor_data,
        )
        self.publisher = self.create_publisher(Odometry, OUTPUT_TOPIC, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self._global_sub = self.create_subscription(
            VehicleGlobalPosition,
            "fmu/out/vehicle_global_position",
            self._global_position_callback,
            qos_profile_sensor_data,
        )
        self._navsat_pub = self.create_publisher(NavSatFix, "navsat", 10)

        self._world_transform = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        self._body_transform = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])

        self.get_logger().info(
            f"odom_converter: {INPUT_TOPIC} -> {OUTPUT_TOPIC} ({ODOM_FRAME} -> {BASE_FRAME})"
        )

    @staticmethod
    def _cov6x6(diag):
        cov = np.zeros((6, 6))
        np.fill_diagonal(cov, diag)
        return cov.flatten().tolist()

    def _to_enu(self, vec):
        return self._world_transform @ np.asarray(vec)

    def _to_flu(self, vec):
        return self._body_transform @ np.asarray(vec)

    def _sample_stamp(self, timestamp_us, timestamp_sample_us):
        now = self.get_clock().now()
        if now.nanoseconds == 0:
            return None
        age = Duration(nanoseconds=(int(timestamp_us) - int(timestamp_sample_us)) * 1000)
        return (now - age).to_msg()

    @staticmethod
    def _rotation_ned_frd(q_px4):
        return R.from_quat([q_px4[1], q_px4[2], q_px4[3], q_px4[0]]).as_matrix()

    def _quat_to_enu_flu(self, R_ned_frd):
        R_enu_flu = self._world_transform @ R_ned_frd @ self._body_transform
        q_enu = R.from_matrix(R_enu_flu).as_quat()
        if q_enu[3] < 0:
            q_enu = [-x for x in q_enu]
        return q_enu

    def listener_callback(self, msg: VehicleOdometry):
        values = [*msg.position, *msg.q, *msg.velocity, *msg.angular_velocity]
        if not np.all(np.isfinite(values)):
            raise RuntimeError(
                f"convert {INPUT_TOPIC} failed: cause: non-finite value in position "
                f"{list(msg.position)}, q {list(msg.q)}, velocity {list(msg.velocity)} or "
                f"angular velocity {list(msg.angular_velocity)}"
            )
        if msg.pose_frame != VehicleOdometry.POSE_FRAME_NED:
            raise RuntimeError(
                f"convert {INPUT_TOPIC} failed: cause: unsupported pose frame {msg.pose_frame}"
            )
        stamp = self._sample_stamp(msg.timestamp, msg.timestamp_sample)
        if stamp is None:
            return
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = ODOM_FRAME
        odom.child_frame_id = BASE_FRAME

        pos_enu = self._to_enu(msg.position)
        odom.pose.pose.position.x = float(pos_enu[0])
        odom.pose.pose.position.y = float(pos_enu[1])
        odom.pose.pose.position.z = float(pos_enu[2])

        R_ned_frd = self._rotation_ned_frd(msg.q)
        q_enu = self._quat_to_enu_flu(R_ned_frd)
        odom.pose.pose.orientation.x = q_enu[0]
        odom.pose.pose.orientation.y = q_enu[1]
        odom.pose.pose.orientation.z = q_enu[2]
        odom.pose.pose.orientation.w = q_enu[3]

        if msg.velocity_frame == VehicleOdometry.VELOCITY_FRAME_NED:
            vel = self._to_flu(R_ned_frd.T @ np.asarray(msg.velocity))
        elif msg.velocity_frame == VehicleOdometry.VELOCITY_FRAME_BODY_FRD:
            vel = self._to_flu(msg.velocity)
        else:
            raise RuntimeError(
                f"convert {INPUT_TOPIC} failed: cause: unsupported velocity frame "
                f"{msg.velocity_frame}"
            )
        odom.twist.twist.linear.x = float(vel[0])
        odom.twist.twist.linear.y = float(vel[1])
        odom.twist.twist.linear.z = float(vel[2])

        av = self._to_flu(msg.angular_velocity)
        odom.twist.twist.angular.x = float(av[0])
        odom.twist.twist.angular.y = float(av[1])
        odom.twist.twist.angular.z = float(av[2])

        odom.pose.covariance = self._cov6x6(COVARIANCE_DIAGONAL)
        odom.twist.covariance = self._cov6x6(COVARIANCE_DIAGONAL)

        self.publisher.publish(odom)

        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = ODOM_FRAME
        t.child_frame_id = BASE_FRAME
        t.transform.translation.x = odom.pose.pose.position.x
        t.transform.translation.y = odom.pose.pose.position.y
        t.transform.translation.z = odom.pose.pose.position.z
        t.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

    def _global_position_callback(self, msg: VehicleGlobalPosition):
        values = [msg.lat, msg.lon, msg.eph, msg.epv]
        if msg.alt_valid:
            values.append(msg.alt_ellipsoid)
        if not np.all(np.isfinite(values)):
            raise RuntimeError(
                "convert fmu/out/vehicle_global_position failed: cause: non-finite value in "
                f"lat {msg.lat}, lon {msg.lon}, alt_ellipsoid {msg.alt_ellipsoid}, eph {msg.eph} "
                f"or epv {msg.epv}"
            )
        stamp = self._sample_stamp(msg.timestamp, msg.timestamp_sample)
        if stamp is None:
            return
        fix = NavSatFix()
        fix.header.stamp = stamp
        fix.header.frame_id = BASE_FRAME
        fix.latitude = float(msg.lat)
        fix.longitude = float(msg.lon)
        fix.altitude = float(msg.alt_ellipsoid) if msg.alt_valid else float("nan")
        fix.status.status = (
            NavSatStatus.STATUS_FIX
            if msg.lat_lon_valid and not msg.dead_reckoning
            else NavSatStatus.STATUS_NO_FIX
        )
        fix.status.service = NavSatStatus.SERVICE_GPS
        eph2 = float(msg.eph) ** 2
        epv2 = float(msg.epv) ** 2
        fix.position_covariance = [
            eph2,
            0.0,
            0.0,
            0.0,
            eph2,
            0.0,
            0.0,
            0.0,
            epv2,
        ]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self._navsat_pub.publish(fix)


def main(args=None):
    signals.init(args=args)
    node = OdomConverter()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
