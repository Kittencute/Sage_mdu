# TODO

## Mission Replanner
- [ ] Combine path and mission planning so failed navigation can be reported to the mission layer with waypoint context
- [ ] Handle unreachable waypoints explicitly; never silently skip one—allow a clear failure or operator-selected alternate waypoint
- [ ] Investigate waypoint ordering

## Nav2 recovery and route quality
- [x] Relax the progress checker so slow motion/recovery does not abort after only 5 seconds
- [x] Allow bounded reverse motion through both MPPI and the velocity smoother
- [x] Clear both costmaps and compute a fresh path after FollowPath fails
- [ ] Rebuild and verify XML/parameter loading, then test blocked and narrow routes in simulation
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