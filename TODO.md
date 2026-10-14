# TODO / Project Changes

## 3D map
- [x] Forest terrain: generate terrain mesh and heightmap.
    - [x] Tree heights: align objects with terrain.
    - [x] Robot spawn: use terrain elevation.
    - [x] Forest scene: update world and Foxglove view.
- [ ] Map orientation: align 2D view with 3D navigation.
    - [x] Yaw alignment: share map-to-ENU offset across GPS and TF.
    - [x] Costmap frames: use `map` / `base_link`.
    - [ ] Goal alignment: measure offset and verify map clicks against robot pose.
- [ ] Forest spacing: finish spacing trees for true-size meshes.
    - [ ] Tree overlap: resolve duplicates while preserving clearings.
    - [x] Terrain bounds: keep tree positions within 90 m × 90 m and refresh terrain-following Z.

## Keepout zones
- [x] Keepout zones: add Nav2 integration.
    - [x] Zone services: set, get, and clear.
    - [x] Zone markers: publish obstacle points and Foxglove markers.
    - [x] Zone clearing: fix clear behavior and tune costmap display.
- [ ] Foxglove zones: create keepouts by clicking in the map.

## Navigation
- [x] Nav2 recovery: improve planning and recovery behavior.
    - [x] Costmaps: tune obstacle layers, inflation, and replanning.
    - [x] Stall detection: detect stalls sooner and allow bounded reverse motion.
    - [x] Path failure: clear both costmaps and replan to the same goal.
    - [x] Waypoint retry: retry once after Nav2 recovery.
    - [x] Mission recovery: mark failures, backtrack to last reached point, then continue.
- [ ] Nav2 routes: test clear, blocked, narrow, reverse, retry, and failure cases.
- [ ] Mission backtracking: fix and retest return to the last reached point.
- [ ] Controller timing: test reduced MPPI workload against loop-rate drops and optimizer resets.
- [ ] Controller tuning: improve recovery and motion.
    - [ ] MPPI settings: verify reduced batch size and iteration count; tune path inversion and angular limits.
    - [ ] RotationShim: verify inversion tolerances and reverse speed.
    - [x] Recovery protocol: try a short reverse, then continue to clear and replan if safety blocks it.
    - [x] Robot-relative obstacle filtering: crop the published obstacle cloud to -0.5 m through +1.5 m in `base_link`; preserve the wider raw cloud for clearing.
    - [ ] Inflation test: verify 0.6 m radius and 5.0 cost scaling in both costmaps on root and narrow routes.
    - [ ] Recovery tests: test reversing, turning, and obstacle recovery with collision monitoring enabled.

## Missions and waypoints
- [x] Mission routing: support ordered/unordered routes, Home, and return.
    - [x] Mission commands: add `--home`, `--sethome`, `--return`, `--clear`, and `--add`.
    - [x] Home fallback: use the first valid GPS position.
- [x] Waypoint management: update missions without restarting.
    - [x] Route updates: clear/add points and draw from robot start.
    - [x] Home return: distinguish Home from Origin; return without new coordinates.
- [ ] Failed waypoint: edit or replace it without resending the mission.
- [ ] Waypoint order: verify routing order behavior.
- [ ] Failure continuation: validate end-to-end mission recovery.
- [ ] Mission range rejection: compare submitted waypoint order and GPS datum when legs unexpectedly exceed 45 m.

## Foxglove
- [x] Foxglove missions: create missions from the 3D Scene.
    - [x] Map clicks: convert ENU positions to GPS.
    - [x] Mission preview: number, clear, and submit ordered/unordered points.
    - [x] Bridge setup: configure permissions, layout, and coordinates.

## Tester, teleop, and setup
- [x] Mission tester: add menu and sample waypoints.
    - [x] Tester commands: send configured points from containers.
- [x] Teleop controls: add hold-to-move and resume autonomy.
- [ ] Mission buttons: add teleop shortcuts for common missions.
- [x] Setup docs: document Nav2, missions, keepouts, and Foxglove.
- [ ] Tester exit: investigate code 137 and identify the cause.
