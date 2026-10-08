# Local SAGE Setup Notes

This document contains machine-specific setup notes and fixes for running the SAGE TerraScout simulation.

Some values are specific to each development computer. Do not blindly copy machine-specific values between computers.

---

## 1. ROS Domain ID

File:

```text
src/fleet_config/fleet_config/model.py
```

Set a different ROS domain ID for each computer to keep ROS 2 communication isolated.

### dator10

```python
@property
def domain_id(self) -> int:
    return 133
```
Example:

| Computer | ROS Domain ID |
|---|---:|
| dator10 | `133` |
| dator11 | `30` |

---

## 2. Gazebo Partition

File:

```text
src/fleet_config/fleet_config/model.py
```

SAGE originally generates the Gazebo partition from the SAGE instance:

```python
@property
def gz_partition(self) -> str:
    return f"sage_{self.index}"
```

This causes a problem when two computers on the same network both run the same SAGE instance.

For example, both computers using from the git repo:

```text
sage_1
```

can cause their Gazebo simulations to discover and interact with each other.

This can result in behavior such as controlling the TerraScout on one computer and seeing the TerraScout on the other computer move as well.

Use a unique machine name as part of the Gazebo partition.

### dator10

```python
@property
def gz_partition(self) -> str:
    return f"dator10_sage_{self.index}"
```
The resulting partitions for SAGE instance 1 are:

| Computer | Gazebo Partition |
|---|---|
| dator10 | `dator10_sage_1` |
| dator11 | `dator11_sage_1` |

---

## 3. Foxglove Station Port

Use the Station Foxglove WebSocket address:

```text
ws://127.0.0.1:8767
```

## 4. TerraScout Simulation Model and Topics

When adding, removing, or modifying simulated TerraScout components or sensors, multiple files may need to be updated.

The main files are:

```text
src/terrascout_description/urdf/scout.urdf.xacro
src/terrascout_description/config/gz_bridge.yaml
src/terrascout_mission/config/simulation/foxglove_bridge.yaml
```

### `scout.urdf.xacro`

File:

```text
src/terrascout_description/urdf/scout.urdf.xacro
```

Contains the TerraScout robot model and simulated components/sensors.

For example, simulated cameras and depth sensors are defined here.

Changes here may add or remove:

- sensors
- cameras
- depth cameras
- links
- Gazebo plugins
- sensor parameters
- simulation topics

---

### `gz_bridge.yaml`

File:

```text
src/terrascout_description/config/gz_bridge.yaml
```

Contains ROS 2 ↔ Gazebo topic bridges.

When a sensor or simulated component publishes a Gazebo topic that is needed by ROS 2, its bridge should be configured here.

If a sensor is removed from the model, check whether its bridge should also be removed.

---

### `foxglove_bridge.yaml`

File:

```text
src/terrascout_mission/config/simulation/foxglove_bridge.yaml
```

Controls which ROS topics are exposed through the Foxglove bridge.

When adding a new simulated sensor, its topics may need to be added here before they appear in Foxglove.

These topics should also use the appropriate QoS configuration where required.

---

## 5. Quick Checklist for a New Development PC

Before running the simulation on another computer on the same network:

1. Choose a unique ROS domain ID.

2. Set it in:

   ```python
   @property
   def domain_id(self) -> int:
       return <unique_domain_id>
   ```

3. Choose a unique machine name for the Gazebo partition.

4. Set:

   ```python
   @property
   def gz_partition(self) -> str:
       return f"<machine_name>_sage_{self.index}"
   ```

5. Build the project if necessary:

   ```bash
   make build
   ```

6. Start the simulation:

   ```bash
   make up
   ```

7. Verify the actual Gazebo partition:

   ```bash
   docker exec sage1-world-1 sh -c 'pid=$(pgrep -f "gz sim" | head -1); tr "\0" "\n" < /proc/$pid/environ | grep GZ_PARTITION'
   ```

8. Verify that controlling the TerraScout on one development computer does not move the simulation on another computer.

---

## Important

The two settings that must be unique between development computers are:

```python
@property
def domain_id(self) -> int:
    return 133
```

and:

```python
@property
def gz_partition(self) -> str:
    return f"dator10_sage_{self.index}"
```

This keeps the ROS 2 and Gazebo simulations separated between development computers.

---

## 6. Keepout-Zone Implementation Map

Files changed for the keepout-zone implementation:

```text
src/fleet_common/CMakeLists.txt
src/fleet_common/fleet_common/keepout_zone.py

src/fleet_interfaces/CMakeLists.txt
src/fleet_interfaces/srv/GetKeepoutZone.srv
src/fleet_interfaces/srv/SetKeepoutZone.srv

src/fleet_config/fleet_config/classes/terrascout.py

src/terrascout_mission/CMakeLists.txt
src/terrascout_mission/package.xml
src/terrascout_mission/terrascout_mission/keepout_geometry.py
src/terrascout_mission/terrascout_mission/keepout_server.py
src/terrascout_mission/test/test_keepout_geometry.py
src/terrascout_mission/config/simulation/foxglove_bridge.yaml
src/terrascout_mission/config/physical/foxglove_bridge.yaml

src/terrascout_navigation/config/nav2.yaml
src/terrascout_navigation/config/navigate_to_pose.xml
```

### fleet_common

#### `src/fleet_common/CMakeLists.txt`

Installs keepout CLI scripts into the ROS package runtime path (`lib/fleet_common`) so they can be run through `ros2 run`.

#### `src/fleet_common/fleet_common/keepout_zone.py`

User-facing CLI and service client for keepout CRUD-style operations:

- create/update (upsert): call `set_keepout_zone`
- read: call `get_keepout_zone`
- delete: call `set_keepout_zone` with `--clear`

It validates numeric inputs for non-read, non-delete operations.

---

### fleet_interfaces

#### `src/fleet_interfaces/CMakeLists.txt`

Registers keepout interfaces for ROSIDL generation.

#### `src/fleet_interfaces/srv/SetKeepoutZone.srv`

Service contract for create/update/delete behavior:

- request contains label + geometry fields + `clear`
- response returns `accepted` and `reason`

#### `src/fleet_interfaces/srv/GetKeepoutZone.srv`

Service contract for read behavior:

- request contains `label`
- response returns `found`, `reason`, zone geometry, and `active`

---

### fleet_config

#### `src/fleet_config/fleet_config/classes/terrascout.py`

Adds and configures keepout server process startup for TerraScout.

This is what wires keepout server execution into the normal unit lifecycle.

---

### terrascout_mission

#### `src/terrascout_mission/CMakeLists.txt`

Installs keepout runtime node and keepout unit tests as part of package build/testing.

#### `src/terrascout_mission/package.xml`

Declares dependencies required by keepout publication and server runtime (for example geometry message support).

#### `src/terrascout_mission/terrascout_mission/keepout_server.py`

Runtime keepout server node. Responsibilities:

- lock map datum from GNSS shim
- serve `set_keepout_zone` and `get_keepout_zone`
- manage in-memory zones by label
- publish zone polygon to collision-monitor topic
- publish sampled zone points to costmap obstacle `PointCloud2` topic
- periodically republish active zone points for late subscribers

Startup contract:

- starts empty (no active keepout zone)
- publishes an empty keepout cloud by default, and non-empty keepout data after a set/update request

#### `src/terrascout_mission/terrascout_mission/keepout_geometry.py`

Pure geometry helpers used by server logic and tests.

#### `src/terrascout_mission/test/test_keepout_geometry.py`

Regression tests for deterministic rectangle generation and geometry validation.

#### `src/terrascout_mission/config/simulation/foxglove_bridge.yaml`

Whitelists keepout topics so simulation Foxglove can visualize keepout polygon/point data.

#### `src/terrascout_mission/config/physical/foxglove_bridge.yaml`

Physical deployment equivalent of simulation whitelist so the same keepout topics are exposed.

---

### terrascout_navigation

#### `src/terrascout_navigation/config/nav2.yaml`

Adds keepout obstacle source to global and local costmap obstacle layers.

Important integration points:

- include keepout source in `observation_sources`
- use `PointCloud2`
- point topic to `/<robot_namespace>/keepout_obstacle_points`

Without this, Nav2 does not consume keepout points even if server publishing works.

---

## 7. Runtime Support and Environment Setup

These files are not keepout feature logic, but they must be valid for simulation/runtime verification:

### `src/fleet_config/fleet_config/model.py`

Defines machine/environment parameters such as ROS domain and Gazebo partitioning.

If these are wrong, simulation isolation breaks and keepout validation may occur in the wrong ROS/Gazebo context.

### `src/fleet_config/fleet_config/targets.py`

Defines runnable targets and composition used by `make use`, `make build`, and `make up`.

This selects which runtime graph is rendered and therefore whether keepout-enabled TerraScout stack is launched.

### `src/terrascout_description/config/gz_bridge.yaml`

Controls Gazebo↔ROS topic bridging for simulation sensors and robot interfaces.

Not keepout-specific, but incorrect bridge setup can block dependent runtime behavior and make system-level keepout verification misleading.

---

## 8. Recent Keepout and Navigation Tuning Changes

This section summarizes recent behavior changes made after the initial keepout integration.

### `src/terrascout_mission/terrascout_mission/keepout_server.py`

- continues publishing keepout data periodically, including empty keepout cloud when no zone is active
- publishes a dedicated Foxglove marker topic (`keepout_zone_marker`) for a persistent colored keepout overlay
- on keepout delete, requests immediate local and global Nav2 costmap clear services

Result:

- keepout deletion is reflected faster in Nav2 and in Foxglove costmap views
- keepout visualization is available as an explicit overlay topic, independent of costmap windowing

### `src/terrascout_navigation/config/nav2.yaml`

- keepout obstacle source enabled in both global and local obstacle layers
- keepout obstacle range increased for farther zones:
    - global `keepout_mark.obstacle_max_range: 1000.0`
    - local `keepout_mark.obstacle_max_range: 200.0`
- obstacle layer height filters are currently:
    - `max_obstacle_height: 2.5`
    - `min_obstacle_height: -0.3`

Result:

- keepout zones are consumed at longer distances than before, but costmap display is still bounded by rolling-window behavior

### `src/terrascout_navigation/config/navigate_to_pose.xml`

- uses `PipelineSequence` with `RateController hz="0.2"`
- replanning path check is done through `IsPathValid` and `GlobalUpdatedGoal`
- keeps standard local/global costmap clearing recovery actions

Result:

- replanning is conservative (low frequency) and depends on path validity/goal updates, which reduces churn but can delay reaction to fast-changing blockage

### `src/fleet_config/fleet_config/classes/terrascout.py`

- keepout server launch params now include:
    - `marker_topic: keepout_zone_marker`
    - `publish_period_s: 0.2`

Result:

- keepout geometry and marker updates are republished faster for Foxglove and late subscribers

### `src/terrascout_mission/config/simulation/foxglove_bridge.yaml`
### `src/terrascout_mission/config/physical/foxglove_bridge.yaml`

- both simulation and physical Foxglove bridges whitelist `/<robot_namespace>/keepout_zone_marker`

Result:

- Foxglove can render a dedicated colored keepout overlay layer directly from marker messages

---

## 9. Mission CLI Changes (Code-Wise)

This section is only about mission command-line behavior and related code updates.

### Scope note

- `mission_server.py` was not changed for this feature set.
- Behavior changed in mission sending client code (`send_mission.py`), tests, and docs.

### Files changed

- `src/fleet_common/fleet_common/send_mission.py`
- `src/fleet_common/test/test_mission_json.py`
- `README.md`

### What changed in `send_mission.py`

- Routing mode support:
  - `--ordered` keeps waypoint order as typed.
  - `--unordered` reorders with nearest-next logic from selected home target.
- Home target argument model:
  - `--home lat,lon`: explicit return coordinate after waypoint execution.
  - `--origin`: use dynamic spawn/datum coordinate from `fixposition/datum`.
  - `--setorigin lat,lon`: set explicit origin/home target using alternate naming.
- Mission dispatch behavior:
  - Normal path: `TAKEOFF -> WAYPOINT(s) -> HOME -> LAND`.
  - `--origin` with no waypoints: sends home-only mission.
  - `--setorigin lat,lon` with no waypoints: no mission dispatched (no movement).

### What changed in tests

`src/fleet_common/test/test_mission_json.py` now covers:

- ordered vs unordered waypoint behavior
- home-only plan generation behavior
- explicit home/origin coordinate parsing

### What changed in docs

`README.md` mission examples and behavior notes were updated so operators can see:

- which argument sets explicit home (`--home`)
- which argument uses spawn/datum (`--origin`)
- how `--setorigin` behaves with and without waypoints