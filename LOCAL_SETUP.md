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

## 5. Record and Compare FAST-LIO2 Odometry

Use the `sim-record` target to record MCAP bags containing FAST-LIO2 odometry and Gazebo ground truth. The default `sim` target does not enable recording.

From the workspace root, build and start the recording-enabled simulation:

```sh
make use TARGET=sim-record
make build
make up
```

Run the mission or route you want to evaluate while the simulation is running. To finish the recording, press **Ctrl+C** in the terminal running `make up` and wait for the stack to stop and the prompt to return. Alternatively, run `make down` from a second terminal. Do not kill the recorder/container forcefully; it needs to shut down normally to write the bag metadata.

The bag is saved under `.sage/run/1/recordings/terrascout1/` for SAGE instance 1. Use the matching instance directory if you started with `SAGE_INSTANCE` set to another number. After the stack has stopped, run the comparison from the workspace root:

```sh
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 src/terrascout_navigation/terrascout_navigation/scripts/process_mission_recording.py --show-trajectory
```

The comparison script is `compare_fast_lio_gt.py` (there is no `compare_odom_mcap.py` file). It reads `/terrascout1/fast_lio/odometry` and `/terrascout1/odom_gt`, then writes its timestamped error CSV and three plots into the same `mission_results/mission_*` bundle as the mission report. It does not use or create a separate `fast_lio_error_results/` directory. The filename prefix uses Swedish local time (Europe/Stockholm), converting rosbag folder timestamps from UTC. `--show-trajectory` opens the overlay after saving it. Its errors are relative trajectory drift after first-pose alignment, not absolute localization error.

The interactive tester at `src/terrascout_mission_tester/terrascout_menu.py` offers four route presets using `src/terrascout_mission_tester/waypoints.json`. WP7–WP20 are generated simulation coordinates, not recovered historical values. The global costmap is a 110 m × 110 m rolling window. Mission legs/home distance may be up to 200 m; each NavigateToPose request is split into intermediate targets no farther than 35% of the smaller global window dimension (38.5 m for the current config). Nav2 plans each segment against the current costmap; this is not a guarantee that an obstacle-blocked segment is reachable. Validate long routes in simulation.

Current experimental navigation tuning is in `src/terrascout_navigation/config/nav2.yaml` and `config/navigate_to_pose.xml`: local costmap update/publish rates are 10/8 Hz; global rates are 3/2 Hz; local/global inflation radii are 0.9/1.0 m with scaling factors 5.0/6.0; MPPI maximum forward speed is 1.0 m/s and forward acceleration limit is 2.5 m/s². Angular response has also been reduced experimentally. Collision Monitor uses outer 30%-slowdown, middle 60%-slowdown, and inner stop polygons. These settings have not established the cause of the reported startup turnaround. Compare initial robot heading, first goal, planned path, and `cmd_vel_nav`/`cmd_vel_auto` in the same `map` frame before making further controller changes. Validate obstacle detection, slow/stop activation, controller timing, and recovery in simulation; these settings cannot prevent collisions if obstacles are absent from the input cloud.

---

**Mission route length:** the fleet configuration permits mission legs and home distances up to 200 m. The rolling global costmap is 110 m × 110 m; the mission server stages each longer destination through intermediate goals sized at 38.5 m under the current configuration. Every segment still requires a valid Nav2 path, and the mission destination is not marked complete until its actual goal is reached. Confirm rendered parameters and test long routes, blocked segments, cancellation, and backtracking. Increasing costmap dimensions increases memory and planning workload.

### Mission ordering and keepout updates

- `ordered` missions preserve the submitted waypoint sequence, including after a keepout-zone update. The next waypoint remains the next eligible destination in that order.
- `unordered` missions optimize the full remaining route after a keepout-zone update, using straight-line geographic distance and the configured mission start/return point. Destinations inside keepout zones are excluded as failed before reordering.
- If an updated keepout covers the active waypoint, the mission server cancels/stops that active navigation goal before marking the waypoint failed and continuing under the configured waypoint-failure policy. Future blocked waypoints are skipped.
- Nav2 replans each active leg using its costmaps, independently of the mission-level order optimizer. The optimizer does not score Nav2 path lengths or establish reachability; a selected route can still contain an unreachable leg.

For validation, test both modes with a keepout covering (1) the active destination and (2) a future destination. Confirm ordered missions retain their remaining sequence, unordered missions update their selected order when needed, and blocked commands appear as failed in mission state/visualization. A successful build does not prove live Nav2 obstacle updates or cancellation timing; verify these in simulation.

### MPR-09 policy proposal — project-owner approval required

This is a draft for review, not evidence of approval. Before assessed trials, the project owner must approve the policy and the team must record the approver, date, and approved revision here or in the trial procedure.

1. **Ordering:** an ordered mission follows its submitted sequence. An unordered mission selects the remaining route with the shortest complete straight-line tour from the current position through eligible destinations and back to mission start. Recalculate the unordered remainder after a keepout update makes a destination invalid. This is a geographic-distance heuristic; it does not compare Nav2 planned path lengths and does not prove reachability.
2. **Keepout updates:** apply each accepted named-zone add/update/removal to Nav2. If the active destination lies inside a new zone, cancel its goal, wait for cancellation, mark that command failed with the zone/reason, then continue. Mark future destinations inside the zone failed before selecting a new unordered order. Ordered routes retain their remaining order.
3. **Unreachable destination / recovery period:** allow the configured Nav2 behavior-tree recoveries to complete, then make one mission-layer retry of the same waypoint after the configured 2.0 s delay (`waypoint_retry_limit: 1`). If that attempt also fails, record the command ID, outcome, and failure reason; mark it failed and continue to the next eligible positional command when `continue_after_waypoint_failure` is true. If no eligible destination remains, fail the mission rather than silently treating the route as complete. The approved procedure must confirm that this retry window is the intended recovery period.
4. **Restoration:** after operator takeover or process restart, keep autonomy suspended. Resume only after explicit operator request, valid/fresh localization, navigation readiness, and a fresh successful path to the current destination. Do not automatically resume or navigate home while localization is invalid.

**Approval record:** `Pending — obtain project-owner approval before marking MPR-09 or MIS-15 complete.` Run and retain tests for each rule, then conduct scripted simulation checks and the required assessed-trial validation.

**MIS-06 simulation evidence (2026-10-23):** In `runs/20261023T083815Z-sim-record.1/recordings/terrascout1/rosbag2_2026_10_23-08_38_21/`, the first recorded invalid localization fix was at 55.300 s. Ground-truth odometry fell below 0.02 m/s by 55.588 s, approximately 0.29 s after the invalid fix, within the 2 s simulation limit. This does not establish physical-robot compliance or complete the required assessed fault-trial recording; retain those as outstanding verification.

For simulation fault injection, call `/terrascout1/gnss_shim/set_localization_fault` with `std_srvs/srv/SetBool` (`data: true` to inject, `data: false` to clear). Localization loss suspends the mission; resuming requires restored valid localization, navigation readiness, a fresh path, and an explicit operator request.

## 6. Quick Checklist for a New Development PC

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

## 7. Keepout-Zone Implementation Map

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

## 7. TerraScout Forest Navigation Tuning

Current experimental navigation settings are in:

```text
src/terrascout_navigation/config/ground_seg.yaml
src/terrascout_navigation/config/nav2.yaml
src/terrascout_navigation/config/navigate_to_pose.xml
patches/ground_segmentation_ros2/tf-failure-stops-node.patch
```

### Robot-relative obstacle heights

The segmentation node transforms LiDAR points into `base_footprint` and publishes the obstacle cloud in that frame. This frame is the ground-level robot origin; `base_link` is elevated 0.33 m above it. The obstacle filter keeps non-ground Z values from **0.0 m to +1.5 m** relative to `base_footprint`. The segmentation input crop remains wider (`minZ: -1.5 m`) so it does not discard valid returns first. The raw cloud is retained as the separate costmap clearing/raytracing source. Points classified as ground are removed before this height filter and cannot be recovered by lowering its minimum.

Nav2's obstacle-layer min/max height checks happen after observations are transformed into each costmap's global frame (`map` or `odom`). Those source limits are intentionally wide; do not use them to implement a robot-relative height band. Collision Monitor uses `base_footprint` as its base frame to match the segmentation cloud and applies the matching 0.0 m / +1.5 m filter.

Ground plane fitting uses the baseline `groundInlierThreshold` of 0.15 m; this tolerance can still classify near-plane root returns as ground. Validate `ground_points` and `obstacle_points` at a root and test low rocks/curbs with the `base_footprint` height reference before relying on this configuration.

### Costmap inflation and replanning

- Both inflation radii are 0.6 m, the smallest value previously accepted by the footprint validator (circumscribed footprint about 0.596 m).
- Global cost scaling is 3.0; local cost scaling is 5.0. The lower global value makes costs decay more gradually through the inflation band, while the local layer keeps the steeper 5.0 decay for tighter tracking.
- The behavior tree checks path validity at 1 Hz and computes a new path when the current path is invalid or the goal changes.
- Follow failure recovery attempts a 1 m reverse at 0.15 m/s with a 12 s allowance, clears both costmaps, attempts one 90-degree turn, then replans even if the spin times out. The fallback has a wait and a slow 0.25 m forward nudge; Collision Monitor can still block motion.
- The latest stuck run timed out on backup/spin and Collision Monitor blocked the nudge. It also reported controller rates as low as about 7 Hz against a 20 Hz target; extra retries alone are not a fix.

These are configuration choices to evaluate, not confirmed performance improvements. Validate clear, narrow, blocked, and root-obstacle routes with Collision Monitor enabled. Compare costmaps and obstacle-cloud data before changing several values at once.


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

## 9. Mission and Foxglove Waypoint Changes

### Mission server Home behavior and crash fix

#### `src/terrascout_mission/terrascout_mission/mission_server.py`

- Renamed the navigation `GetParameters` service client field from `_parameters` to `_navigation_parameters_client`. `_parameters` is reserved internally by the ROS 2 `Node`; overwriting it caused `get_parameter("saved_home")` to crash the server when `--home` was used.
- Captures the first valid GPS fix as the dynamic spawn Home fallback. An explicitly configured `saved_home` from `--sethome` takes priority; otherwise the first valid startup position is returned by `get_home_point`.
- Hosts the Foxglove click collector in-process so it uses the existing mission server executable and cannot cause startup failure because of separate executable discovery.

#### `src/fleet_common/fleet_common/send_mission.py`

- `--home` asks the mission server for its saved Home and sends a home-only mission.
- `--sethome LAT,LON` updates the mission server's `saved_home` parameter without sending a mission.
- Invalid Home response text is handled as a clean client error rather than an uncaught coordinate parsing exception.

### Foxglove click-to-mission workflow

Files involved:

- `src/terrascout_mission/terrascout_mission/foxglove_mission_input.py`
- `src/terrascout_mission/terrascout_mission/mission_server.py`
- `src/fleet_common/fleet_common/geodesy.py`
- `src/terrascout_mission/config/simulation/foxglove_bridge.yaml`
- `src/terrascout_mission/config/physical/foxglove_bridge.yaml`
- `src/terrascout_mission/layouts/terrascout.json`
- `src/terrascout_mission/package.xml`
- `README.md`

The collector subscribes to `/<robot_namespace>/clicked_point` (`geometry_msgs/PointStamped`) from the Foxglove 3D Scene panel. It only accepts clicks in the configured `map` frame and converts map-frame ENU coordinates to GPS using the GNSS datum. Each accepted click is retained as a draft waypoint and visualized as a numbered marker on `mission_waypoint_markers`.

The inverse geodesy helper returns coordinates in `(latitude, longitude, altitude)` order. `foxglove_mission_input.py` must preserve that order when storing click coordinates; swapping latitude and longitude creates invalid points thousands of kilometres away. The mission server validates leg lengths and will reject such a mission.

Foxglove Trigger services operate on the draft:

- `/<robot_namespace>/mission_waypoints/clear` clears the draft.
- `/<robot_namespace>/mission_waypoints/send_ordered` submits all points in click order.
- `/<robot_namespace>/mission_waypoints/send_unordered` submits all points after nearest-next ordering.

Both send services submit one `ExecuteMission` goal with `TAKEOFF -> WAYPOINT(s) -> HOME -> LAND`; the return Home in that mission is the robot's GPS position when submitted. Draft points remain visible if the mission server rejects the goal. The existing `send_mission.py` plan and waypoint-ordering helpers are reused. The 2D geographic map panel is display-only; waypoint clicks are made in the 3D Scene with its fixed frame set to `map`.

The Trigger service response is asynchronous with respect to the action: `success: true` means the goal was queued for `ExecuteMission`, not that the mission server accepted it or that navigation started. Check `mission_server` logs and `/terrascout1/mission_state` for acceptance/progress. A rejected goal is reported in the mission-server log; its click draft remains available to retry or clear.

### Build/startup validation

- `make build` passes all workspace tests (139 tests, 0 failures).
- The Foxglove collector is attached to `mission_server` rather than launched as a separate executable. This avoids ROS executable lookup failure under `--symlink-install` when a newly added Python source file lacks executable permissions.
- After rebuilding, `make up` was verified with all four simulation containers remaining up and `mission_server` reporting `ready for missions`.
- During a live Foxglove test, the mission server rejected a draft because the earlier click-conversion code had latitude and longitude reversed; the log showed a first leg of about 4,939,886 m against the 45 m limit. The current collector unpacks `enu_to_llh()` as `lat, lon, _` and therefore fixes that specific rejection. Rebuild and restart after source edits before testing.

### Other mission tester and launch changes

- `src/terrascout_mission_tester/terrascout_menu.py` sends all configured JSON waypoints for Ordered/Unordered missions instead of asking the operator to select a subset. Single-waypoint selection remains for adding a waypoint or setting saved Home. The menu no longer includes the old return-to-start option 6 or y/N confirmations.
- `Makefile` adds `make exec COMMAND=...` to run a command in the active unit container with the ROS workspace overlay sourced.
- `terrascout_menu.py` checks whether host ROS can resolve `fleet_common` and falls back to the unit-container execution path when needed. The stack must be running for that fallback.

---

## 10. Nav2 Progress and Recovery Tuning

The 2026-10-13 simulation log showed repeated `controller_server: Failed to make progress`, intermittent missed 20 Hz control-loop deadlines, optimizer failures, and eventual navigation failure at mission command 3. A single failed mission waypoint must not be silently skipped: it may be an obstacle, a bad map/route, or a real safety issue. The mission should either reach that waypoint, fail with its identity, or allow an operator to choose an alternate point.

### `src/terrascout_navigation/config/nav2.yaml`

- Relaxed `SimpleProgressChecker` from a 5 s / 1.0 m requirement to 8 s / 0.25 m, with 0.25 rad angular progress. This detects a stalled robot sooner while allowing cautious movement.
- Enabled bounded reverse velocity in MPPI (`vx_min: -0.25`) and the velocity smoother (`min_velocity[0]: -0.25`). The existing `PreferForwardCritic` still favors forward motion; reverse is available when useful rather than being forced.

### `src/terrascout_navigation/config/navigate_to_pose.xml`

- After `FollowPath` fails, the first contextual recovery action backs straight up 1 m (0.15 m/s, 12 s allowance), then clears both costmaps and computes a fresh path to the same goal.
- The collision monitor remains active during reverse. If it blocks the maneuver or the backup action times out, Nav2 proceeds to its bounded recovery cycle; it does not force motion through an obstacle.

### `src/terrascout_mission/terrascout_mission/mission_server.py`

- Nav2 remains responsible for geometric obstacle avoidance and its behavior-tree recovery cycle. If Nav2 ultimately aborts a `WAYPOINT`, the mission server waits 2 seconds and retries the exact same target once by default (`waypoint_retry_limit: 1`, `waypoint_retry_delay_s: 2.0`). Both parameters are configurable at startup; retries are bounded to 0–3 and 0–30 seconds.
- A successful retry finishes the original command normally. If both attempts are aborted and a later positional command exists, the mission marks the failed command, returns to the last reached point, then continues to the next planned positional command. It does not invent a substitute coordinate. If backtracking fails, or there is no later target, the mission ends as failed.
- `continue_after_waypoint_failure` enables that bounded fallback. It runs only after Nav2 reports `STATUS_ABORTED` and the same-target retry is exhausted; cancellation does not trigger it. The current recovery uses a single contextual spin; the outer fallback uses backup, costmap clearing, wait, and a slow forward nudge without repeated spins.
- `map_yaw_offset_deg` is a single fleet setting shared by simulation `map→odom` TF and mission/Foxglove GPS conversions. It defaults to 0 (the world is ENU with zero heading); only change it after measuring +X/+Y against known east/north motion. Do not compensate by rotating only the Foxglove camera.
- The global costmap explicitly uses `map` and `base_link`; the rolling local costmap uses `odom` and `base_link`.

### Validation required

- The policy/configuration changes require a fresh build and simulation run before relying on them operationally.
- Verify parameter/XML loading; test an unobstructed route, temporary obstacle, 1 m backup, same-target retry, return to last reached point, continuation, and a final blocked waypoint that ends as mission failure.
- Monitor actual command velocities, progress-checker resets, planner/controller failures, and controller loop-rate warnings. The loop-rate warnings may indicate simulation/CPU scheduling load; do not further loosen stall detection or increase MPPI workload without measurement.