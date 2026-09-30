#!/usr/bin/env python3


import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

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

        self.datum_pub = self.create_publisher(NavSatFix, "fixposition/datum", latched_qos())

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        self._odom_sub = self.create_subscription(Odometry, "odom_gt", self._odom_cb, 10)
        self._navsat_sub = self.create_subscription(
            NavSatFix, "navsat", self._navsat_cb, qos_profile_sensor_data
        )

        self._publish_static_map_odom()
        self._publish_datum()
        self.get_logger().info("Publishing TF")

    def _publish_static_map_odom(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = FRAME_GLOBAL
        t.child_frame_id = FRAME_ODOM
        t.transform.rotation.w = 1.0
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

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = FRAME_ODOM
        t.child_frame_id = FRAME_ROOT
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

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
        self.llh_pub.publish(msg)


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
