# TODO / Project Changes

## World and maps
- [x] Forest environment
  - [x] Terrain and vegetation
    - [x] Terrain mesh
    - [x] Terrain-following trees
    - [x] Forest visualization
  - [x] Robot placement
    - [x] Elevated spawn
  - [ ] Forest layout
    - [x] Terrain bounds
    - [ ] Resolve tree overlap
- [ ] Map alignment
  - [x] Shared yaw offset
  - [x] Costmap frames
  - [ ] Verify goal alignment

## Perception and costmaps
- [x] Obstacle processing
  - [x] Robot-relative height filter
    - [x] Trial bounds: 0.0 m to +1.5 m in `base_footprint`
  - [x] Ground-segmentation crop
    - [x] Keep wide `minZ` at -1.5 m
  - [x] Preserve raw cloud for clearing
- [ ] Perception validation
  - [ ] Ground rejection
  - [ ] Root detection
    - [ ] Validate baseline groundInlierThreshold 0.15 on roots and uneven terrain
    - [ ] Validate ground-level height reference on roots, rocks, and curbs
  - [ ] Confirm low roots are ignored without chassis contact
  - [ ] Inspect cloud at stopped pose
- [x] Costmap setup
  - [x] Inflation radius: 0.6 m (global and local)
  - [x] Scaling: 3.0 global / 5.0 local
- [ ] Costmap validation
  - [ ] Test 0.6 m inflation
  - [ ] Compare global/local scaling
  - [ ] Inspect Collision Monitor trigger
- [x] Keepout zones
  - [x] Nav2 integration
    - [x] Zone services
    - [x] Zone visualization
    - [x] Zone clearing
  - [ ] Create zones by clicking

## Planning and control
- [x] Planner configuration
  - [x] Obstacle sources
  - [x] Path validity checks at 1 Hz
  - [x] Clear costmaps and replan
- [ ] Route testing
  - [ ] Clear route
  - [ ] Blocked route
  - [ ] Narrow route
  - [ ] Mission backtracking
- [ ] MPPI controller
  - [ ] Measure controller load against 20 Hz target
  - [ ] Investigate observed 7 Hz loop
  - [ ] Check optimizer resets
  - [ ] Verify MPPI settings
  - [ ] Check inversion limits
  - [ ] Verify reverse speed
- [x] Recovery behavior
  - [x] Bounded 1 m reverse
  - [x] Clear both costmaps
  - [x] Attempt one slower 90-degree spin
  - [x] Replan if spin fails
  - [x] Protected 0.25 m forward nudge at 0.25 m/s
- [ ] Recovery testing
  - [ ] Test reverse and spin
  - [ ] Test Collision-Monitor-blocked nudge
  - [ ] Diagnose root collision
  - [ ] Test retry and terminal-failure behavior

## Mission management
- [x] Mission routing
  - [x] Ordered and unordered missions
  - [x] Home and return
  - [x] Mission CLI options
- [x] Waypoint updates
  - [x] Clear and add points
  - [x] Return to saved Home
- [ ] Mission validation
  - [ ] Verify waypoint order
  - [ ] Test failure continuation
  - [ ] Check mission range
  - [ ] Edit failed waypoint

## Operator interface
- [x] Foxglove mission workflow
  - [x] Click-built missions
  - [x] ENU/GPS conversion
  - [x] Preview and submit
  - [x] Bridge configuration
  - [ ] Click-built keepouts
- [x] Mission tester
  - [x] Container commands
  - [x] Sample waypoint menu
- [x] Teleoperation handoff
- [ ] Mission shortcuts

## Reliability and documentation
- [ ] Diagnose exit 137
- [x] Setup documentation
- [ ] Re-run and record navigation trials
  - [x] Crossed the low root with the -0.10 m cutoff
  - [ ] Diagnose later open-space stop at waypoint 5
    - [ ] Collision Monitor repeatedly triggered PolygonStop
    - [ ] Backup and spin later timed out; one earlier recovery succeeded
    - [ ] Controller loop dropped as low as 9.6 Hz
    - [ ] Inspect obstacle points and Collision Monitor state at the stopped pose
  - [ ] Validate root classification and low-obstacle behavior in `base_footprint`
