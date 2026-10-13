# TODO

## Mission Replanner
- [ ] Combine path and mission planning
- [ ] Handle unreachable waypoints
- [ ] Investigate waypoint ordering

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