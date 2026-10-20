#!/usr/bin/env python3
from __future__ import annotations
import argparse
import math
import sys
import time
try:
    from fleet_interfaces.msg import MissionCommand, MissionState
except ModuleNotFoundError:
    # Test environments may not have generated ROS interfaces available.
    class MissionCommand:
        TAKEOFF = 1
        LAND = 2
        WAYPOINT = 3
        HOME = 4
    class MissionState:
        MISSION_FINISHED = 2
TAKEOFF = MissionCommand.TAKEOFF
LAND = MissionCommand.LAND
WAYPOINT = MissionCommand.WAYPOINT
HOME = MissionCommand.HOME
MISSION_FINISHED = MissionState.MISSION_FINISHED


def spin_until_future(spin_once, node, future, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while not future.done() and time.monotonic() < deadline:
        spin_once(node, timeout_sec=min(0.1, max(0.0, deadline - time.monotonic())))
    return future.done()


def build_plan_items(
    coords: list[tuple[float, float, float]],
    altitude: float,
    hold: float | None = None,
    include_land: bool = True,
    home_only: bool = False,
) -> list[dict]:
    if home_only:
        return [{"id": 1, "kind": HOME}]
    items: list[dict] = [{"id": 1, "kind": TAKEOFF, "altitude_agl": altitude}]
    for index, (lat, lon, alt) in enumerate(coords, start=2):
        item = {
            "id": index,
            "kind": WAYPOINT,
            "latitude": lat,
            "longitude": lon,
            "altitude_agl": alt,
        }
        if hold is not None:
            item["hold_s"] = hold
        items.append(item)
    items.append({"id": len(items) + 1, "kind": HOME})
    if include_land:
        items.append({"id": len(items) + 1, "kind": LAND})
    return items
def parse_coords(raw: list[str], altitude: float) -> list[tuple[float, float, float]]:
    coords: list[tuple[float, float, float]] = []
    for entry in raw:
        parts = entry.split(",")
        if len(parts) == 2:
            coords.append((float(parts[0]), float(parts[1]), altitude))
        elif len(parts) == 3:
            coords.append((float(parts[0]), float(parts[1]), float(parts[2])))
        else:
            raise ValueError(f"parse coordinate '{entry}' failed: cause: expected lat,lon[,alt]")
    return coords
def parse_home(home_arg: str | None) -> tuple[float, float] | None:
    if home_arg is None:
        return None
    parsed = parse_coords([home_arg], 0.0)[0]
    return (parsed[0], parsed[1])
def order_waypoints(
    coords: list[tuple[float, float, float]],
    mode: str,
    home: tuple[float, float],
) -> list[tuple[float, float, float]]:
    if mode != "unordered" or len(coords) < 2:
        return coords
    remaining = list(coords)
    ordered: list[tuple[float, float, float]] = []
    current_lat, current_lon = home
    while remaining:
        next_index = min(
            range(len(remaining)),
            key=lambda i: _distance_sq_m(current_lat, current_lon, remaining[i][0], remaining[i][1]),
        )
        waypoint = remaining.pop(next_index)
        ordered.append(waypoint)
        current_lat, current_lon = waypoint[0], waypoint[1]
    return ordered
def _distance_sq_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    meters_per_deg_lat = 111_320.0
    mean_lat_rad = math.radians((lat1 + lat2) * 0.5)
    meters_per_deg_lon = meters_per_deg_lat * math.cos(mean_lat_rad)
    dy = (lat2 - lat1) * meters_per_deg_lat
    dx = (lon2 - lon1) * meters_per_deg_lon
    return dx * dx + dy * dy
def main() -> int:
    parser = argparse.ArgumentParser(description="Send a mission to a vehicle's mission server.")
    parser.add_argument(
        "--namespace", required=True, help="ROS namespace of the vehicle, such as aeroscout1"
    )
    parser.add_argument("waypoints", nargs="*", help="GPS coordinates as lat,lon[,alt_agl]")
    parser.add_argument(
        "--altitude",
        type=float,
        default=10.0,
        help="Altitude above home ground (m) for takeoff and waypoints without one",
    )
    parser.add_argument("--hold", type=float, default=None, help="Hold at each waypoint (s)")
    destination_group = parser.add_mutually_exclusive_group()
    destination_group.add_argument("--return", dest="return_start", action="store_true", help="Return to mission start (default for waypoint missions)")
    destination_group.add_argument("--home", action="store_true", help="Go to the saved Home location after waypoints or immediately")
    parser.add_argument("--sethome", metavar="LAT,LON", help="Save persistent Home coordinates (no mission sent)")
    parser.add_argument("--clear", action="store_true", help="Stop active mission and clear displayed waypoints")
    parser.add_argument("--add", action="store_true", help="Add one waypoint to the active mission")
    route_group = parser.add_mutually_exclusive_group()
    route_group.add_argument(
        "--ordered",
        dest="routing_mode",
        action="store_const",
        const="ordered",
        help="Visit waypoints in the exact order provided (default).",
    )
    route_group.add_argument(
        "--unordered",
        dest="routing_mode",
        action="store_const",
        const="unordered",
        help="Reorder waypoints greedily from home by nearest next waypoint.",
    )
    parser.set_defaults(routing_mode="ordered")
    parser.add_argument("--mission-id", type=int, default=1)
    parser.add_argument(
        "--home-only",
        action="store_true",
        help="End mission at HOME and skip LAND command.",
    )
    args = parser.parse_args()
    try:
        coords = parse_coords(args.waypoints, args.altitude)
        new_home = parse_home(args.sethome)
    except ValueError as exc:
        parser.error(str(exc))
    if args.add and (len(coords) != 1 or args.clear or args.sethome or args.home or args.return_start):
        parser.error("--add requires exactly one waypoint and no destination/management flags")
    if args.clear and (coords or args.home or args.return_start or args.sethome):
        parser.error("--clear cannot be combined with other options")
    if args.sethome and (coords or args.home or args.return_start):
        parser.error("--sethome is a standalone command")
    if not (coords or args.home or args.return_start or args.clear or args.sethome):
        parser.error("provide waypoints or --return/--home/--sethome/--clear")
    import rclpy
    from fleet_interfaces.action import ExecuteMission
    from fleet_interfaces.msg import MissionCommand, MissionPlan
    from geographic_msgs.msg import GeoPoint
    from rclpy.action import ActionClient
    from sensor_msgs.msg import NavSatFix
    from std_srvs.srv import Trigger
    from std_msgs.msg import String
    import json
    from rcl_interfaces.srv import SetParameters
    from rcl_interfaces.msg import Parameter as RosParameter, ParameterValue, ParameterType
    from fleet_common.qos import latched_qos
    rclpy.init()
    node = rclpy.create_node("send_mission", namespace=args.namespace)
    client = ActionClient(node, ExecuteMission, "execute_mission")
    outcome = {"status": None, "reason": ""}
    readiness = {"ready": False, "reason": None}
    def on_state(message):
        if not message.ready and message.not_ready_reason != readiness["reason"]:
            print(f"waiting: mission server not ready: {message.not_ready_reason}")
        readiness["ready"] = message.ready
        readiness["reason"] = message.not_ready_reason
    node.create_subscription(
        MissionState,
        "mission_state",
        on_state,
        latched_qos(),
    )
    try:
        if args.add:
            publisher = node.create_publisher(String, "add_mission_waypoint", 10)
            # Wait for the server subscriber so the command is not silently lost.
            for _ in range(50):
                if publisher.get_subscription_count():
                    break
                rclpy.spin_once(node, timeout_sec=0.1)
            if not publisher.get_subscription_count():
                print("add failed: mission server subscriber unavailable", file=sys.stderr)
                return 1
            lat, lon, alt = coords[0]
            msg = String()
            msg.data = json.dumps({"lat": lat, "lon": lon, "alt": alt, "mode": args.routing_mode})
            publisher.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.2)
            print("waypoint addition requested (check mission_server logs for acceptance)")
            return 0
        print("waiting for the mission server state...", flush=True)
        state_deadline = time.monotonic() + 15.0
        while readiness["reason"] is None and time.monotonic() < state_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if readiness["reason"] is None:
            print(
                "send mission failed: cause: no mission_state received within 15 s; "
                "check mission_server and ROS namespace",
                file=sys.stderr,
            )
            return 1
        if not readiness["ready"]:
            print("waiting up to 30 s for mission server readiness...", flush=True)
            ready_deadline = time.monotonic() + 30.0
            while not readiness["ready"] and time.monotonic() < ready_deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            if not readiness["ready"]:
                print(
                    "send mission failed: cause: mission server did not become ready within "
                    f"30 s: {readiness['reason']}",
                    file=sys.stderr,
                )
                return 1
        if args.clear:
            clear_client = node.create_client(Trigger, "clear_mission")
            if not clear_client.wait_for_service(timeout_sec=5.0):
                print("clear mission failed: clear_mission service unavailable", file=sys.stderr)
                return 1
            future = clear_client.call_async(Trigger.Request())
            if not spin_until_future(rclpy.spin_once, node, future, 5.0):
                print("clear mission failed: service response timed out after 5 s", file=sys.stderr)
                return 1
            response = future.result()
            if response is None or not response.success:
                print(f"clear mission failed: {response.message if response else 'no response'}", file=sys.stderr)
                return 1
            print(response.message)
            return 0
        if args.sethome:
            setter = node.create_client(SetParameters, "mission_server/set_parameters")
            if not setter.wait_for_service(timeout_sec=5.0):
                print("sethome failed: mission_server/set_parameters unavailable", file=sys.stderr)
                return 1
            value = f"{new_home[0]:.9f},{new_home[1]:.9f}"
            param = RosParameter(name="saved_home", value=ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=value))
            future = setter.call_async(SetParameters.Request(parameters=[param]))
            if not spin_until_future(rclpy.spin_once, node, future, 5.0):
                print("sethome failed: service response timed out after 5 s", file=sys.stderr)
                return 1
            result = future.result()
            if result is None or not result.results[0].successful:
                print("sethome failed: parameter rejected", file=sys.stderr)
                return 1
            print(f"saved Home: {value}")
            return 0
        target_service = "get_home_point" if args.home else "get_current_point"
        target = node.create_client(Trigger, target_service)
        if not target.wait_for_service(timeout_sec=5.0):
            print(f"{target_service} service unavailable", file=sys.stderr)
            return 1
        future = target.call_async(Trigger.Request())
        if not spin_until_future(rclpy.spin_once, node, future, 5.0):
            print(f"{target_service} timed out waiting for a response", file=sys.stderr)
            return 1
        response = future.result()
        if response is None or not response.success:
            print(f"target unavailable: {response.message if response else 'no response'}", file=sys.stderr)
            return 1
        try:
            home = parse_home(response.message)
        except ValueError:
            print(f"home unavailable: invalid home data '{response.message}'", file=sys.stderr)
            return 1
        print(f"mission target: {home[0]:.7f},{home[1]:.7f}")
        coords = order_waypoints(coords, args.routing_mode, home)
        if args.routing_mode == "unordered":
            print("waypoints reordered with --unordered based on nearest-next from home")
        plan = MissionPlan()
        plan.id = args.mission_id
        for item in build_plan_items(
            coords,
            args.altitude,
            args.hold,
            include_land=not args.home_only,
            home_only=(len(coords) == 0),
        ):
            command = MissionCommand()
            command.id = item["id"]
            command.kind = item["kind"]
            command.latitude = item.get("latitude", 0.0)
            command.longitude = item.get("longitude", 0.0)
            command.altitude_agl = item.get("altitude_agl", 0.0)
            command.hold_s = item.get("hold_s", 0.0)
            command.yaw_deg = float("nan")
            plan.commands.append(command)
        plan.home = GeoPoint(latitude=home[0], longitude=home[1], altitude=0.0)
        print("waiting for the mission server to be ready for missions")
        if not client.wait_for_server(timeout_sec=10.0):
            print("send mission failed: execute_mission action server unavailable after 10 s", file=sys.stderr)
            return 1
        goal = ExecuteMission.Goal()
        goal.plan = plan
        def on_feedback(message):
            feedback = message.feedback
            print(f"command {feedback.command_id} status {feedback.status}")
        send = client.send_goal_async(goal, feedback_callback=on_feedback)
        if not spin_until_future(rclpy.spin_once, node, send, 15.0):
            print("send mission failed: timed out waiting 15 s for mission acceptance", file=sys.stderr)
            return 1
        handle = send.result()
        if handle is None or not handle.accepted:
            print(
                "send mission failed: cause: the mission server rejected the plan", file=sys.stderr
            )
            return 1
        print(f"mission {plan.id} accepted with {len(plan.commands)} commands")
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future)
        result = result_future.result().result
        outcome["status"] = result.status
        outcome["reason"] = result.reason
    except KeyboardInterrupt:
        print("interrupted", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    if outcome["status"] == MISSION_FINISHED:
        print(f"mission {plan.id} finished")
        return 0
    print(
        f"mission {plan.id} ended with status {outcome['status']}: {outcome['reason']}",
        file=sys.stderr,
    )
    return 1
if __name__ == "__main__":
    sys.exit(main())
