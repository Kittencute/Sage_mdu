#!/usr/bin/env python3

from __future__ import annotations

import argparse
import math
import sys

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
    home_group = parser.add_mutually_exclusive_group()
    home_group.add_argument(
        "--home",
        default=None,
        help="Set mission HOME target explicitly as lat,lon.",
    )
    home_group.add_argument(
        "--origin",
        action="store_true",
        help=(
            "Use spawn/datum coordinate as mission HOME target. "
            "If no waypoints are provided, runs a home-only mission."
        ),
    )
    home_group.add_argument(
        "--setorigin",
        default=None,
        help="Set origin/home target explicitly as lat,lon.",
    )
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
        home = parse_home(args.home or args.setorigin)
    except ValueError as exc:
        parser.error(str(exc))

    if not coords and not (args.home or args.origin or args.setorigin):
        parser.error("at least one waypoint or one of --home/--origin/--setorigin is required")

    if not coords and args.setorigin:
        print("setorigin received without waypoints: origin updated for this command, no mission sent")
        return 0

    if args.home is not None:
        home_mode = "home"
    elif args.setorigin is not None:
        home_mode = "setorigin"
    else:
        home_mode = "origin"

    import rclpy
    from fleet_interfaces.action import ExecuteMission
    from fleet_interfaces.msg import MissionCommand, MissionPlan
    from geographic_msgs.msg import GeoPoint
    from rclpy.action import ActionClient
    from sensor_msgs.msg import NavSatFix

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
        if home_mode == "origin":
            datum_fixes: list[NavSatFix] = []
            datum = node.create_subscription(
                NavSatFix, "fixposition/datum", datum_fixes.append, latched_qos()
            )
            print("waiting for spawn/datum position on fixposition/datum for home")
            while not datum_fixes:
                rclpy.spin_once(node)
            node.destroy_subscription(datum)
            fix = datum_fixes[0]
            home = (fix.latitude, fix.longitude)
            print(f"home at spawn/datum position {home[0]:.7f},{home[1]:.7f}")
        else:
            assert home is not None
            print(f"home set to {home[0]:.7f},{home[1]:.7f}")

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
            home_only=(len(coords) == 0 and args.origin),
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
        while not readiness["ready"]:
            rclpy.spin_once(node)
        client.wait_for_server()
        goal = ExecuteMission.Goal()
        goal.plan = plan

        def on_feedback(message):
            feedback = message.feedback
            print(f"command {feedback.command_id} status {feedback.status}")

        send = client.send_goal_async(goal, feedback_callback=on_feedback)
        rclpy.spin_until_future_complete(node, send)
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
