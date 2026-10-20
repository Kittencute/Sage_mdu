# SAGE

[![CI](https://github.com/SLaRCLabs/sage/actions/workflows/ci.yml/badge.svg)](https://github.com/SLaRCLabs/sage/actions/workflows/ci.yml)

Fleet management for ground and aerial robots on ROS 2 Jazzy.

SAGE runs TerraScout ground robots and AeroScout drones in Gazebo simulation or on hardware. Each robot runs as one unit, rendered from a Python fleet definition, and shares its topics with the station over Zenoh.

Status: research software. The simulation is verified end to end; the physical TerraScout awaits bench checks; there is no physical AeroScout yet.

## Recent development updates

- **Forest simulation:** terrain generation keeps tree placements inside the 90 m × 90 m terrain and updates tree elevations to follow the generated heightmap. Small and medium pine model variants are available alongside the existing trees, and the forest world and Foxglove view use the updated scene.
- **Navigation and obstacle handling:** ground segmentation publishes clouds in `base_footprint`, whose origin is at ground level (unlike the elevated `base_link`). The non-ground obstacle cloud is filtered from 0.0 m to +1.5 m in that frame; its input `minZ` stays wide at -1.5 m, and the separate raw cloud remains available for costmap ray-clearing. Points already classified as ground are not restored by this height filter, so root classification still requires validation. Current experimental Nav2 tuning checks path validity at 2 Hz, updates/publishes local costmap at 8/5 Hz and global at 3/2 Hz, uses 0.9 m local and 1.0 m global inflation, and allows 1.0 m/s forward speed. Collision Monitor has outer 30%-slowdown, middle 60%-slowdown, and inner stop polygons. Validate behavior and controller timing in simulation before relying on these experimental settings.
- **Missions and Foxglove:** mission handling supports ordered/unordered routes, adding and clearing waypoints, Home/return behavior, and mission drafts from map-frame Foxglove clicks. Failed mission legs can be tracked and recovered by the mission server.
- **Positioning and setup:** geodesy and GNSS/map alignment were updated, and local setup notes cover machine-specific ROS domain and Gazebo partition values.

The -1.0 m lower obstacle cutoff, route stability at 1 Hz, inflation tuning, root traversal, mission backtracking, and controller timing still require scenario testing. A successful build does not verify those behaviors. Recent runs show repeated Collision Monitor stops and MPPI failures; inspect the obstacle cloud at the stopped pose and validate low-obstacle safety before relying on this cutoff.

## Install

Requirements:

- x86_64 Linux with Docker, the Compose plugin and GNU Make
- Python 3 with PyYAML
- `vcs` (vcstool or vcs2l)
- For simulation: an NVIDIA GPU with the NVIDIA Container Toolkit, and an X server
- About 12 GB of disk for the simulation

Build the simulation:

```sh
make use TARGET=sim
make build
```

## Usage

### Start and stop

```sh
make up
make down
```

`make up` runs in the foreground. Gazebo opens with both robots, and every process of the `world`, `terrascout1`, `aeroscout1` and `station` units logs `ready`.

### Record and compare FAST-LIO2 odometry

Use the `sim-record` target to record simulation topics to MCAP, including FAST-LIO2 odometry and Gazebo ground truth (`/terrascout1/fast_lio/odometry` and `/terrascout1/odom_gt`). Recording is not enabled by the default `sim` target.

```sh
# Select and start the recording-enabled simulation
make use TARGET=sim-record
make up
```

Let the robot run the route you want to evaluate. To stop, either press **Ctrl+C** in the terminal running `make up` and wait for it to return to the prompt, or run `make down` from another terminal. Wait for the stack and recorder to stop normally so the bag metadata is written. Bags are stored under `.sage/run/1/recordings/terrascout1/` (the `1` is the SAGE instance; change it if using `SAGE_INSTANCE` other than 1). Rosbag splits output at 60-second intervals.

The mission tester creates a timestamped `mission_YYYYMMDD_HHMMSS` directory under `src/terrascout_mission_tester/mission_results/` when it launches a preset route. It stores the selected ordered route in `waypoints.json`, launch details in `metadata.json`, and client output in `mission_client.log`. The client log does not prove that the mission was accepted or completed. After stopping the stack normally, run the single post-processing entrypoint from the workspace root. It selects the newest completed bag, reuses a results bundle already associated with that exact recording (or a tester bundle created during that bag's recording), and otherwise creates a bundle with the same timestamp format from the bag's start time. It does not choose an unrelated folder just because that folder was modified most recently. Both analyses write to the same selected bundle:

```sh
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 src/terrascout_navigation/terrascout_navigation/scripts/process_mission_recording.py --show-trajectory
```

`--show-trajectory` opens the route plot after saving it. The comparator writes `YYYYMMDDTHHMMSS_fast_lio_error.csv`, `YYYYMMDDTHHMMSS_position_error.png`, `YYYYMMDDTHHMMSS_orientation_error.png`, and `YYYYMMDDTHHMMSS_trajectory_comparison.png` into the bundle. The filename timestamp is the bag recording start converted to Europe/Stockholm; rosbag folder timestamps are treated as UTC. The comparison aligns trajectories at the first matched sample, so its errors measure relative drift, not absolute localization accuracy.

The report writes `mission_events.csv`, `mission_report.json`, and—if absent—a `waypoints.json` snapshot reconstructed from recorded waypoint GeoJSON. It selects the latest positive mission ID by default; pass `--mission-id` to choose another mission. MIS-20 scoring uses `/terrascout1/odom_gt`, the recorded geodetic datum and completed waypoint points, plus the Nav2 footprint polygon. It reports each destination's minimum planar vehicle-edge distance and checks the 20-minute limit. This computation assumes ground-truth `odom` axes align with `map` axes and uses the configured simulation footprint; a simulation pass does not establish an assessed-trial or physical-robot pass. For MIS-22 it also calculates odometry path length, transitions to zero commanded velocity, teleoperation takeovers, and safety-stop activations when those streams are present. Check the JSON `available` and `not_recorded_or_not_computed` fields for remaining gaps such as order-change reasons and faults.

The interactive tester in `src/terrascout_mission_tester/terrascout_menu.py` has four route presets and uses `waypoints.json`. WP7–WP20 are generated local simulation points; verify them in the world before use. Consecutive legs in the current presets were checked to stay below 44 m, leaving margin under the mission server's 45 m leg limit.

### Watch in Foxglove

In [Foxglove](https://foxglove.dev), choose Open connection, then Foxglove WebSocket, enter the unit's address, and import the matching layout from `.sage/render/sim.1/layouts/`.

| Unit | Connection |
| --- | --- |
| TerraScout | `ws://127.0.0.1:8765` |
| AeroScout | `ws://127.0.0.1:8766` |
| Station | `ws://127.0.0.1:8767` |

### Open a shell in a unit

```sh
make shell SERVICE=unit-terrascout1
```

The commands below run in this shell.

### Keepout CRUD

```sh
# Create
ros2 run fleet_common keepout_zone.py --namespace terrascout1 --label test_zone --longitude 8.545594 --latitude 47.397742 --x-length 4 --y-length 4
# Read
ros2 run fleet_common keepout_zone.py --namespace terrascout1 --label test_zone --read
# Update (same label, new values)
ros2 run fleet_common keepout_zone.py --namespace terrascout1 --label test_zone --longitude 8.545620 --latitude 47.397760 --x-length 6 --y-length 3 --yaw-deg 10
# Delete
ros2 run fleet_common keepout_zone.py --namespace terrascout1 --label test_zone --clear
```

For a colored semi-opaque keepout zone volume in Foxglove, add a 3D panel layer for `/terrascout1/keepout_zone_marker` (message type `visualization_msgs/msg/Marker`).

### Send a mission

```sh
# AeroScout: Send a mission with an altitude of 10 meters
ros2 run fleet_common send_mission.py --namespace aeroscout1 --altitude 10 47.39785,8.54566

# TerraScout: Send one waypoint and return to the mission starting position
ros2 run fleet_common send_mission.py --namespace terrascout1 --return 47.39781,8.545594

# TerraScout: Send one waypoint and then go to the saved Home position
ros2 run fleet_common send_mission.py --namespace terrascout1 --home 47.39781,8.545594

# TerraScout - Ordered (default): visit 5 waypoints and return to mission start
ros2 run fleet_common send_mission.py --namespace terrascout1 --ordered --return 47.39779,8.54564 47.39788,8.54581 47.39760,8.54595 47.39754,8.54545 47.39793,8.54542

# TerraScout - Unordered: reorder 5 waypoints and return to mission start
ros2 run fleet_common send_mission.py --namespace terrascout1 --unordered --return 47.39779,8.54564 47.39788,8.54581 47.39760,8.54595 47.39754,8.54545 47.39793,8.54542

# TerraScout - Home: navigate directly to the saved Home position
ros2 run fleet_common send_mission.py --namespace terrascout1 --home

# TerraScout - Set Home coordinates (does not start a mission)
ros2 run fleet_common send_mission.py --namespace terrascout1 --sethome 47.397742,8.545594

# TerraScout - Clear: stop the active mission and clear its visualization
ros2 run fleet_common send_mission.py --namespace terrascout1 --clear

# Add a waypoint without changing the current navigation target
ros2 run fleet_common send_mission.py --namespace terrascout1 --add --ordered 47.39790,8.54570

# Add a waypoint and reorder the remaining route
ros2 run fleet_common send_mission.py --namespace terrascout1 --add --unordered 47.39790,8.54570
```

Mission command behavior:

- Default mission sequence: TAKEOFF -> WAYPOINT(s) -> HOME/RETURN -> LAND
- `--return` returns to the robot's position when the current mission was started
- `--home` returns to the saved Home position after completing the waypoints
- `--sethome lat,lon` sets a new Home position without starting a mission
- `--clear` cancels the active mission and clears its waypoints from the server and Foxglove
- `--ordered` keeps the waypoint sequence exactly as provided (default)
- `--unordered` reorders waypoints using a nearest-next route
- `--add --ordered lat,lon` adds a waypoint after the remaining waypoints in the active mission
- `--add --unordered lat,lon` adds a waypoint and reorders the remaining unvisited waypoints

### Build a mission by clicking in Foxglove

TerraScout's Foxglove 3D Scene panel can collect map-frame clicks as a waypoint draft. This uses the 3D Scene viewport (with its fixed frame set to `map`); the 2D geographic map panel is for viewing and does not collect mission clicks.

1. Open the TerraScout Foxglove connection and the SAGE TerraScout layout.
2. In the 3D Scene panel, click each desired location in order. Numbered cyan markers show the collected draft.
3. Add a Foxglove **Service Call** panel and call one of these empty `std_srvs/srv/Trigger` services:
	- `/terrascout1/mission_waypoints/send_ordered` to visit points in click order.
	- `/terrascout1/mission_waypoints/send_unordered` to reorder all clicked points using nearest-next routing.
4. The mission returns to the robot's position when the mission is submitted, matching `--return`.

Use `/terrascout1/mission_waypoints/clear` to discard the draft before submitting. For another robot, replace `terrascout1` with its namespace. Foxglove submits the same `ExecuteMission` action plan used by `send_mission.py`; no shell command needs to be typed. The bridge must be running, and the click frame must be `map`.

### Take over and hand back

Driving from the Foxglove Teleop panel locks autonomy, cancels the active Nav2 goal and suspends the mission at its current destination. Use the explicit resume service to return control; it rejects while the e-stop/operator input is active, localization is invalid or stale, or navigation is unavailable. A fresh path is computed before Nav2 receives a movement goal.

```sh
ros2 service call /terrascout1/resume_autonomy std_srvs/srv/Trigger
```

Mission state and keepout-zone definitions are atomically saved under
`/run/sage/terrascout1/` (the unit's mounted run directory). After a mission-server restart the mission is
restored suspended; autonomy starts locked and requires the explicit resume
service. The interactive tester includes suspend, state-inspection and restart
checklist entries for MIS-16/MIS-17.

### Soft e-stop

```sh
ros2 topic pub --once /terrascout1/estop std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /terrascout1/estop std_msgs/msg/Bool "{data: false}"
ros2 service call /terrascout1/resume_autonomy std_srvs/srv/Trigger
```

### Inspect a target

```sh
make render TARGET=sim
ros2 param describe /terrascout1/mission_server max_leg_length_m
```

`make render` writes what every unit runs to `.sage/render/<target>.1/`.

## Development

Develop in the dev container, which uses the image `make build` produces and gives an editor ROS 2, the workspace, and Python and C++ code intelligence without ROS on the host.

```sh
make build
```

Open the repository in VS Code and run Dev Containers: Reopen in Container. Keep running `make build`, `make up`, `make down` and `make shell` on the host; the dev container reads their output and does not join the fleet's network.

`ros2` in the dev container sees no running unit and warns that it cannot reach a Zenoh router. To work with the running fleet, open a host terminal and run `make shell SERVICE=unit-terrascout1`.

## Configuration

Physical robots read site values from `site.yaml` in the repository root.

```sh
cp src/fleet_config/site.example.yaml site.yaml
make build TARGET=terrascout1
make up TARGET=terrascout1
```

Before the first drive, set the command timeout of every CubeMars motor in the CubeMars configuration tool.

## Project layout

| Path | Contents |
| --- | --- |
| `src/fleet_config/fleet_config/classes/` | What each unit runs |
| `src/fleet_config/fleet_config/targets.py` | Targets |
| `third_party.repos`, `patches/` | Pinned third-party sources and local changes |
| `runs/` | Run records |

## Troubleshooting

SAGE is fail-stop: any process that exits stops the whole project, and nothing is restarted. Each `make up` writes a run record to `runs/<time>-<target>.<instance>/` with the render, the commit, the image ids, `compose.log`, and every unit's state and logs.

## Citation

See `CITATION.cff`.

## Maintainer

Emil Persson, Mälardalen University, emil.persson@mdu.se

## License

MIT © Emil Persson. Third-party components and their licenses are listed in `THIRD_PARTY_NOTICES`.
