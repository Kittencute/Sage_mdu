#!/usr/bin/env python3

from __future__ import annotations

import argparse
import math
import sys


def _finite(value: float, name: str) -> float:
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite, got {value}")
    return value


def _positive(value: float, name: str) -> float:
    _finite(value, name)
    if not value > 0.0:
        raise ValueError(f"{name} must be positive, got {value}")
    return value


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Create, read, update, or delete a keepout zone on a TerraScout unit."
    )
    parser.add_argument(
        "--namespace",
        required=True,
        help="ROS namespace of the vehicle, such as terrascout1",
    )
    parser.add_argument("--label", default="keepout", help="Label used in logs and status")
    parser.add_argument("--longitude", type=float, default=0.0, help="WGS84 longitude in degrees")
    parser.add_argument("--latitude", type=float, default=0.0, help="WGS84 latitude in degrees")
    parser.add_argument("--x-length", type=float, default=0.0, help="Zone length along x axis in map frame (m)")
    parser.add_argument("--y-length", type=float, default=0.0, help="Zone length along y axis in map frame (m)")
    parser.add_argument("--yaw-deg", type=float, default=0.0, help="Clockwise yaw in map frame (degrees)")
    parser.add_argument(
        "--clear",
        action="store_true",
        help="Delete the zone for --label",
    )
    parser.add_argument(
        "--read",
        action="store_true",
        help="Read and print the zone for --label",
    )
    args = parser.parse_args()

    if args.clear and args.read:
        parser.error("--clear and --read are mutually exclusive")

    try:
        if not args.clear and not args.read:
            _finite(args.latitude, "latitude")
            _finite(args.longitude, "longitude")
            _positive(args.x_length, "x-length")
            _positive(args.y_length, "y-length")
            _finite(args.yaw_deg, "yaw-deg")
    except ValueError as exc:
        parser.error(str(exc))

    import rclpy
    from fleet_interfaces.srv import GetKeepoutZone, SetKeepoutZone

    rclpy.init()
    node = rclpy.create_node("keepout_zone", namespace=args.namespace)
    if args.read:
        get_client = node.create_client(GetKeepoutZone, "get_keepout_zone")
        set_client = None
    else:
        set_client = node.create_client(SetKeepoutZone, "set_keepout_zone")
        get_client = None

    try:
        if args.read:
            print("waiting for keepout server")
            while not get_client.wait_for_service(timeout_sec=1.0):
                pass

            request = GetKeepoutZone.Request()
            request.label = args.label

            future = get_client.call_async(request)
            rclpy.spin_until_future_complete(node, future)
            response = future.result()
            if response is None:
                print("read keepout zone failed: cause: no service response", file=sys.stderr)
                return 1
            if not response.found:
                print(f"read keepout zone failed: cause: {response.reason}", file=sys.stderr)
                return 1
            print(
                "keepout zone read: "
                f"label={args.label} "
                f"lat={response.latitude:.7f} "
                f"lon={response.longitude:.7f} "
                f"x_length={response.x_length:.2f} "
                f"y_length={response.y_length:.2f} "
                f"yaw_deg={response.yaw_deg:.1f} "
                f"active={response.active}"
            )
            return 0

        print("waiting for keepout server")
        while not set_client.wait_for_service(timeout_sec=1.0):
            pass

        request = SetKeepoutZone.Request()
        request.label = args.label
        request.latitude = args.latitude
        request.longitude = args.longitude
        request.x_length = args.x_length
        request.y_length = args.y_length
        request.yaw_deg = args.yaw_deg
        request.clear = args.clear

        future = set_client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        response = future.result()
        if response is None:
            print("set keepout zone failed: cause: no service response", file=sys.stderr)
            return 1
        if not response.accepted:
            print(f"set keepout zone failed: cause: {response.reason}", file=sys.stderr)
            return 1
        action = "deleted" if args.clear else "upserted"
        print(f"keepout zone {action}: {response.reason}")
        return 0
    except KeyboardInterrupt:
        print("interrupted", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
