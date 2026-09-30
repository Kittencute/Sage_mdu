#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from fleet_common import signals
from fleet_common.params import declare


class TwistStamper(Node):
    def __init__(self):
        super().__init__("twist_stamper")
        self.frame_id = declare(
            self,
            "frame_id",
            rclpy.Parameter.Type.STRING,
            "Frame the stamped velocity command is expressed in",
            constraints="a non-empty TF frame name",
        )
        if not self.frame_id:
            raise ValueError("configure twist_stamper failed: cause: frame_id is empty")
        self.publisher = self.create_publisher(TwistStamped, "cmd_vel_out", 10)
        self.create_subscription(Twist, "cmd_vel_in", self.stamp, 10)

    def stamp(self, twist):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.twist = twist
        self.publisher.publish(msg)


def main():
    signals.init()
    node = TwistStamper()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
