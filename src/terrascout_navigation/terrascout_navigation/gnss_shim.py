#!/usr/bin/env python3

import copy
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_srvs.srv import SetBool
from tf2_ros import StaticTransformBroadcaster

from fleet_common import signals
from fleet_common.params import declare
from fleet_common.qos import latched_qos

FRAME_GLOBAL = "map"
FRAME_ODOM = "odom"
FRAME_ROOT = "vrtk_link"


class GnssShimNode(Node):
    def __init__(self):
        super().__init__("gnss_shim")

        self.odom_enu_pub = self.create_publisher(Odometry, "fixposition/odometry_enu", 10)
        self.odom_smooth_pub = self.create_publisher(Odometry, "fixposition/odometry_smooth", 10)
        self.llh_pub = self.create_publisher(NavSatFix, "fixposition/odometry_llh", 10)
        self._localization_fault = False
        self._latest_fix: NavSatFix | None = None
        self._fault_service = self.create_service(
            SetBool, "gnss_shim/set_localization_fault", self._set_localization_fault
        )

        self.datum_pub = self.create_publisher(NavSatFix, "fixposition/datum", latched_qos())

        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        self._odom_sub = self.create_subscription(Odometry, "odom_gt", self._odom_cb, 10)
        self._navsat_sub = self.create_subscription(
            NavSatFix, "navsat", self._navsat_cb, qos_profile_sensor_data
        )

        self._map_yaw_offset = math.radians(
            declare(
                self,
                "map_yaw_offset_deg",
                Parameter.Type.DOUBLE,
                "Counter-clockwise yaw from geographic east to map +X, in degrees",
                -180.0,
                180.0,
                "0 for an ENU-aligned map; set only from measured axis calibration",
            )
        )
        self._simulated_position_std_m = declare(
            self,
            "simulated_position_std_m",
            Parameter.Type.DOUBLE,
            "Configured nominal position standard deviation for Gazebo NavSatFix messages",
            0.01,
            100.0,
            "simulation-only fallback used when Gazebo does not populate covariance",
        )
        self._publish_static_map_odom()
        self._publish_datum()
        self.get_logger().info("Publishing TF")

    def _publish_static_map_odom(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = FRAME_GLOBAL
        t.child_frame_id = FRAME_ODOM
        # odom is ENU; map +X is rotated by map_yaw_offset from east.
        # The map<-odom TF therefore carries the inverse rotation.
        t.transform.rotation.z = math.sin(-self._map_yaw_offset / 2.0)
        t.transform.rotation.w = math.cos(-self._map_yaw_offset / 2.0)
        self.static_tf_broadcaster.sendTransform(t)

    def _publish_datum(self):
        datum = NavSatFix(
            latitude=declare(
                self,
                "datum_latitude",
                Parameter.Type.DOUBLE,
                "Latitude of the map origin, in degrees",
                -90.0,
                90.0,
            ),
            longitude=declare(
                self,
                "datum_longitude",
                Parameter.Type.DOUBLE,
                "Longitude of the map origin, in degrees",
                -180.0,
                180.0,
            ),
            altitude=declare(
                self,
                "datum_altitude",
                Parameter.Type.DOUBLE,
                "Altitude of the map origin above the ellipsoid, in metres",
                -500.0,
                9000.0,
            ),
        )
        datum.header.stamp = self.get_clock().now().to_msg()
        self.datum_pub.publish(datum)
        self.get_logger().info(
            f"Datum published: lat={datum.latitude:.6f}, lon={datum.longitude:.6f}"
        )

    def _odom_cb(self, msg):
        stamp = msg.header.stamp

        smooth = Odometry()
        smooth.header.stamp = stamp
        smooth.header.frame_id = FRAME_ODOM
        smooth.child_frame_id = FRAME_ROOT
        smooth.pose = msg.pose
        smooth.twist = msg.twist
        self.odom_smooth_pub.publish(smooth)

        enu = Odometry()
        enu.header.stamp = stamp
        enu.header.frame_id = FRAME_GLOBAL
        enu.child_frame_id = FRAME_ROOT
        enu.pose = msg.pose
        enu.twist = msg.twist
        self.odom_enu_pub.publish(enu)

    def _navsat_cb(self, msg):
        fix = NavSatFix()
        fix.header = msg.header
        fix.status = msg.status
        fix.latitude = msg.latitude
        fix.longitude = msg.longitude
        fix.altitude = msg.altitude
        fix.position_covariance = list(msg.position_covariance)
        fix.position_covariance_type = msg.position_covariance_type
        if fix.position_covariance_type == NavSatFix.COVARIANCE_TYPE_UNKNOWN:
            variance = self._simulated_position_std_m**2
            fix.position_covariance = [
                variance, 0.0, 0.0,
                0.0, variance, 0.0,
                0.0, 0.0, variance,
            ]
            fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self._latest_fix = fix
        if self._localization_fault:
            self._publish_invalid_fix()
        else:
            self.llh_pub.publish(fix)

    def _set_localization_fault(self, request, response):
        """Inject an invalid NavSatFix in simulation, or resume normal forwarding."""
        self._localization_fault = request.data
        if self._localization_fault:
            self._publish_invalid_fix()
            response.success = True
            response.message = "simulation localization fault injected (NavSatFix status=NO_FIX)"
            self.get_logger().warning(response.message)
        else:
            if self._latest_fix is not None:
                self.llh_pub.publish(self._latest_fix)
            response.success = True
            response.message = "simulation localization fault cleared; forwarding Gazebo fixes"
            self.get_logger().info(response.message)
        return response

    def _publish_invalid_fix(self):
        fix = copy.deepcopy(self._latest_fix) if self._latest_fix is not None else NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.status.status = NavSatStatus.STATUS_NO_FIX
        self.llh_pub.publish(fix)


def main():
    signals.init()
    node = GnssShimNode()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
