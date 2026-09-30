#!/usr/bin/env python3

import argparse
import json
import sys
import time

import paho.mqtt.client as mqtt

CMD_NAV_TAKEOFF = 0
CMD_NAV_LAND = 1
CMD_NAV_WAYPOINT = 2

STATUS_FINISHED = 3
STATUS_NAMES = {
    0: "NOT_ASSIGNED",
    1: "NOT_STARTED",
    2: "RUNNING",
    3: "FINISHED",
    4: "STOPPED",
    5: "FAILED",
    6: "ABORTED",
}

MISSION_ID = int(time.time())


def make_mission(vehicle_id: int) -> dict:
    return {
        "id": MISSION_ID,
        "vehicleId": vehicle_id,
        "homeLocation": {"latitude": 47.397787, "longitude": 8.545594, "altitude": 0.0},
        "commands": [
            {
                "id": 0,
                "commandType": CMD_NAV_TAKEOFF,
                "startTime": 0,
                "endTime": 5,
                "params": [0.0, 0.0, 47.397787, 8.545594, 0.0],
            },
            {
                "id": 1,
                "commandType": CMD_NAV_WAYPOINT,
                "startTime": 5,
                "endTime": 30,
                "params": [0.0, 1.0, 0.0, 0.0, 47.397787, 8.545594, 0.0],
            },
            {
                "id": 2,
                "commandType": CMD_NAV_WAYPOINT,
                "startTime": 30,
                "endTime": 55,
                "params": [0.0, 1.0, 0.0, 0.0, 47.397787, 8.545660, 0.0],
            },
            {
                "id": 3,
                "commandType": CMD_NAV_WAYPOINT,
                "startTime": 55,
                "endTime": 80,
                "params": [0.0, 1.0, 0.0, 0.0, 47.397742, 8.545660, 0.0],
            },
            {
                "id": 4,
                "commandType": CMD_NAV_LAND,
                "startTime": 80,
                "endTime": 90,
                "params": [0.0, 0.0, 0.0, 47.397787, 8.545594, 0.0],
            },
        ],
    }


def elapsed(start: float) -> str:
    dt = time.time() - start
    minutes = int(dt) // 60
    seconds = int(dt) % 60
    return f"[{minutes:02d}:{seconds:02d}]"


def main() -> int:
    parser = argparse.ArgumentParser(description="MADUM end-to-end test mission")
    parser.add_argument("--vehicle-id", type=int, default=2001)
    parser.add_argument("--broker", type=str, default="localhost:1883")
    parser.add_argument("--timeout", type=int, default=300)
    args = parser.parse_args()

    host, _, port_str = args.broker.partition(":")
    port = int(port_str) if port_str else 1883

    mission = make_mission(args.vehicle_id)
    num_commands = len(mission["commands"])

    start = time.time()
    finished_cmds: set[int] = set()
    mission_sent = False
    bridge_ready = False

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)

    def on_connect(client, userdata, flags, rc, properties=None):
        nonlocal bridge_ready
        if rc != 0:
            print(f"{elapsed(start)} Connection failed: rc={rc}")
            return
        print(f"{elapsed(start)} Connected to {host}:{port}")
        client.subscribe("MADUM/vehicle/summary/#", qos=1)
        client.subscribe("MADUM/vehicle/stateVector/#", qos=1)
        client.subscribe("MADUM/vehicle/missionUpdate/#", qos=1)
        client.subscribe("MADUM/vehicle/alarm/#", qos=1)

    def on_message(client, userdata, msg):
        nonlocal mission_sent, bridge_ready

        topic = msg.topic
        try:
            payload = json.loads(msg.payload)
        except (json.JSONDecodeError, UnicodeDecodeError):
            print(f"{elapsed(start)} << {topic}: (unparseable)")
            return

        if "summary" in topic:
            vid = payload.get("id", "?")
            name = payload.get("name", "?")
            print(f"{elapsed(start)} << {topic}: id={vid} name={name}")
            if not bridge_ready and vid == args.vehicle_id:
                bridge_ready = True
                send_mission(client)

        elif "stateVector" in topic:
            loc = payload.get("location", {})
            lat = loc.get("latitude", 0)
            lon = loc.get("longitude", 0)
            spd = payload.get("speed", 0)
            bat = payload.get("battery", 0)
            print(
                f"{elapsed(start)} << stateVector: lat={lat:.3f} lon={lon:.3f} spd={spd:.1f} bat={bat:.1f}"
            )
            if not bridge_ready:
                bridge_ready = True
                print(f"{elapsed(start)} Bridge detected via stateVector. Sending mission...")
                send_mission(client)

        elif "missionUpdate" in topic and payload.get("missionId") == MISSION_ID:
            cmd_id = payload.get("commandId", "?")
            status = payload.get("statusCode", -1)
            status_name = STATUS_NAMES.get(status, f"UNKNOWN({status})")
            print(f"{elapsed(start)} << missionUpdate: cmd={cmd_id} {status_name}")
            if status == STATUS_FINISHED:
                finished_cmds.add(cmd_id)
                if len(finished_cmds) == num_commands:
                    print(
                        f"{elapsed(start)} Mission {MISSION_ID} complete. All {num_commands}/{num_commands} commands FINISHED."
                    )
                    client.disconnect()

        elif "alarm" in topic:
            code = payload.get("alarmCode", "?")
            message = payload.get("message", "")
            print(f"{elapsed(start)} << alarm: code={code} msg={message}")

        else:
            print(f"{elapsed(start)} << {topic}: {json.dumps(payload)[:120]}")

    def send_mission(client):
        nonlocal mission_sent
        if mission_sent:
            return
        mission_sent = True
        topic = f"MADUM/middleware/missionSend/{args.vehicle_id}"
        print(
            f"{elapsed(start)} Bridge ready. Sending test mission {MISSION_ID} ({num_commands} commands)..."
        )
        print(f"{elapsed(start)} >> {topic}")
        client.publish(topic, json.dumps(mission), qos=2)

    client.on_connect = on_connect
    client.on_message = on_message

    try:
        client.connect(host, port, keepalive=60)
    except (ConnectionRefusedError, OSError) as e:
        print(f"{elapsed(start)} Failed to connect to {host}:{port}: {e}")
        return 1

    deadline = time.time() + args.timeout
    client.loop_start()

    try:
        while time.time() < deadline:
            if len(finished_cmds) == num_commands:
                client.loop_stop()
                return 0
            time.sleep(0.1)
    except KeyboardInterrupt:
        print(f"\n{elapsed(start)} Interrupted.")
        client.loop_stop()
        return 1

    client.loop_stop()
    print(
        f"{elapsed(start)} Timeout after {args.timeout}s. {len(finished_cmds)}/{num_commands} commands finished."
    )
    return 1


if __name__ == "__main__":
    sys.exit(main())
