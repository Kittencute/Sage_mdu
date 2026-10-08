# SAGE

[![CI](https://github.com/SLaRCLabs/sage/actions/workflows/ci.yml/badge.svg)](https://github.com/SLaRCLabs/sage/actions/workflows/ci.yml)

Fleet management for ground and aerial robots on ROS 2 Jazzy.

SAGE runs TerraScout ground robots and AeroScout drones in Gazebo simulation or on hardware. Each robot runs as one unit, rendered from a Python fleet definition, and shares its topics with the station over Zenoh.

Status: research software. The simulation is verified end to end; the physical TerraScout awaits bench checks; there is no physical AeroScout yet.

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

# TerraScout: Send a simple mission with one waypoint
ros2 run fleet_common send_mission.py --namespace terrascout1 --home 47.397742,8.545594 47.39781,8.545594

# TerraScout - Ordered (default): follows waypoint order exactly (5 waypoints)
ros2 run fleet_common send_mission.py --namespace terrascout1 --ordered --home 47.397742,8.545594 47.39778,8.54562 47.39786,8.54578 47.39795,8.54592 47.39803,8.54576 47.39790,8.54558

# TerraScout - Unordered: auto-reorder by nearest-next from home target (same 5 waypoints)
ros2 run fleet_common send_mission.py --namespace terrascout1 --unordered --home 47.397742,8.545594 47.39778,8.54562 47.39786,8.54578 47.39795,8.54592 47.39803,8.54576 47.39790,8.54558

# TerraScout - Home (go to dynamic spawn/datum home)
ros2 run fleet_common send_mission.py --namespace terrascout1 --origin

# TerraScout - Set origin/home target explicitly
ros2 run fleet_common send_mission.py --namespace terrascout1 --setorigin 47.397742,8.545594 
```

Mission command behavior:

- Default list is: TAKEOFF -> WAYPOINT(s) -> HOME -> LAND
- `--home lat,lon` means: after waypoints, go back to this coordinate
- `--origin` means: go to the original spawn/datum home position
- `--setorigin lat,lon` means: set a new coordinate to use as that origin/home target
- `--ordered` keeps your waypoint sequence (default)
- `--unordered` reorders waypoints to a nearest-next route from selected home target

### Take over and hand back

Driving from the Foxglove Teleop panel puts the TerraScout in manual control and aborts its mission.

```sh
ros2 service call /terrascout1/resume_autonomy std_srvs/srv/Trigger
```

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
