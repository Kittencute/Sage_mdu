# TODO

## Mission Replanner
- [x] Coordinate Nav2 behavior-tree recovery with a bounded mission-layer retry of the same aborted waypoint
- [x] Handle unreachable waypoints explicitly: fail with the original command ID after retries; never silently skip or fabricate an alternate target
- [ ] Add an operator workflow to edit/replace a failed waypoint without resending the entire mission
- [ ] Investigate waypoint ordering

## Nav2 recovery and route quality
- [x] Relax the progress checker so slow motion/recovery does not abort after only 5 seconds
- [x] Allow bounded reverse motion through both MPPI and the velocity smoother
- [x] Clear both costmaps and compute a fresh path after FollowPath fails
- [x] After Nav2 exhausts recovery, wait briefly and retry the exact failed waypoint once by default
- [ ] Rebuild and verify XML/parameter loading, then test blocked, narrow, reverse, retry, and explicit failure routes in simulation
- [ ] Review controller loop-rate warnings separately; lower simulation CPU load or tune MPPI only after profiling confirms it is the bottleneck

## Mission Sending
- [x] Fix the missing UI waypoint line from the start position
- [x] Add an option to mission waypoints without restarting
    - [x] Clear / reset mission
    - [x] Return (like Go Home, but without adding coordinates)
    - [x] Use Home instead of Origin
- [x] Enable waypoint selection by clicking on the map in Foxglove
- [x] Investigate how to support ordered and unordered waypoints in Foxglove

## Teleop
- [x] Support continuous movement while holding a direction button
- [ ] Add buttons for predefined missions (Go Home, Return to Origin, etc.)
- [ ] Add support for creating keepout zones by clicking on the map
- [x] Add a button to resume autonomy (`/terrascout1/resume_autonomy`)