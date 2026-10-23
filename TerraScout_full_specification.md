## Implementation and Verification Status

- `[x]` **Tested IRL** — Implemented and successfully tested on the physical robot.
- `[x*]` **Tested in Simulation** — Implemented and tested in simulation, but not yet fully verified on the physical robot.
- `[x?]` **Implemented / Unconfirmed** — Functionality may be implemented or partially implemented, but has not been sufficiently tested.
- `[ ]` **Not Implemented** — Functionality has not been implemented or tested.

**Note:** These markers track implementation and verification progress. Formal requirement acceptance may additionally require documented evidence and project-owner approval.

# Detecting and Locating Objects with TerraScout
## SLaRC-PERCEPTION
*26 August 2026, SLaRC*

## 3 Goals and Objectives
By the end of the course TerraScout will publish two streams. One reports the vehicle’s localisation status. The other contains records of the stationary objects around the vehicle. Each object record carries identifying imagery, class and attributes, three-dimensional position, classification confidence and position uncertainty. The project will measure both streams against surveyed reference data rather than accept them on inspection. Five objectives define the work.
- [ ] 1. Measure pose error against a surveyed reference while the vehicle is stationary and while it is moving, evaluate the effect of losing RTK corrections and publish the resulting localisation status.
- [ ] 2. Validate and record the existing front-camera stream, calibrate it to the vehicle’s coordinate system and synchronise image and pose timestamps.
- [ ] 3. Photograph, classify and locate objects from a configurable class set, including the markers and representative allied and enemy objects used in the ARES-inspired benchmark.
- [ ] 4. Combine repeated observations of the same stationary object and publish the resulting records through the vehicle’s ROS 2 interface.
- [ ] 5. Evaluate detection, classification, positioning, repeated-observation handling and processing delay on reserved data and in outdoor trials, then provide a labelled dataset and reusable evaluation scripts. Choice of detector and position-estimation method is left to the team. The team may also replace or remount the camera, but only where measurement shows that the existing arrangement cannot reach the approved targets.

## 5 Requirements

### 5.1 Project Requirements
- [x] **PPR-01** Before selecting a perception method, the team shall measure the existing camera stream, document the available sensors and explain how their measurements relate to the vehicle’s position and coordinate system.
- [x] **PPR-02** If the team proposes replacing or remounting the camera, the team shall show why the existing arrangement cannot meet the approved targets, and shall obtain the project owner’s approval before making the change.
- [ ] **PPR-03** The team shall reserve the final evaluation data and keep them separate from all data used for training, tuning and development.
- [ ] **PPR-04** The team shall obtain reference measurements of object positions and vehicle poses in the output coordinate system, and document the method and uncertainty. Before final evaluation, the project owner shall confirm that the reference-measurement method is accurate enough to determine whether the targets in PPR-06 are met.
- [ ] **PPR-05** The reserved evaluation data shall include images with no target and images containing lookalike objects that are not targets.
- [ ] **PPR-06** Before the final evaluation, the team shall propose targets for detection rate, false-positive rate, classification accuracy, position error, localisation error, correct grouping of repeated observations and processing delay, and shall obtain the project owner’s approval of those targets.
- [ ] **PPR-07** The team shall obtain the project owner’s approval of a safety procedure before the first trial on the vehicle.
- [ ] **PPR-08** The team shall report every attempted trial, including failed and aborted trials.
- [ ] **PPR-09** At the start of the project, the team and the project owner shall set a delivery date for the object-record interface definition and sample messages. The team shall deliver both by that date.
- [x] **PPR-10** The team shall use published work to justify its design and evaluation choices.

### 5.2 Product Requirements
Base requirements are mandatory. Extra requirements apply only if time allows.
- [ ] **PER-01** Base. Every published object record shall carry identity, provenance and measurement fields. The identity and provenance fields shall include an object identifier, the first and latest observation times and an observation count. The measurement fields shall include the coordinate system, class and attributes, three-dimensional position, classification confidence, position uncertainty and a reference to identifying imagery.
- [ ] **PER-02** Base. Each record shall give the object’s position twice. One position shall be relative to the vehicle at the time of the latest observation (ARES REQ-270). The other shall be a current estimate in the vehicle’s map coordinate system.
- [ ] **PER-03** Base. The class set shall be configurable and shall include people, vehicles, a casualty target and the four supply symbols from ARES Task 1, together with at least one allied and one enemy model from Task 2, selected with the project owner’s approval. Extra. The class set shall include the complete Task 2 model set in the attached rules.
- [ ] **PER-04** Base. The perception system shall provide a configurable confidence threshold and shall not use detections falling below that threshold to create or update an object record.
- [ ] **PER-05** Base. A published object record that cannot be assigned to a configured class shall be labelled as unknown.
- [ ] **PER-06** Base. The perception system shall retain an identifying source image or crop for every observation used in a published record (ARES REQ-250 and Task 2), and associate that image with the object identifier and observation time.
- [ ] **PER-07** Base. The localisation status shall be published at a constant, configurable rate and shall contain the type and validity of the latest position fix, estimated pose uncertainty and age of the latest RTK correction.
- [ ] **PER-08** Base. When incoming RTK corrections stop, the localisation status shall report within 2 s that the pose estimate is unreliable.
- [ ] **PER-09** Base. Images, calibration data, vehicle poses, detections, observations and any sensor data used to estimate object positions shall be recordable using a common time base.
- [ ] **PER-10** Base. A single interface validator shall accept both the sample records and records produced by the completed system.
- [ ] **PER-11** Base. The team shall verify the camera calibration and every conversion between coordinate systems used to estimate object positions, and report the remaining errors.
- [ ] **PER-12** Base. The outdoor evaluation shall include target distances of 2 m, 5 m and 10 m. Frontal and oblique views shall be tested in sunlit and shaded conditions. Testing shall include a stationary and a moving vehicle together with unobstructed and partly occluded targets. Each condition shall be represented, but the evaluation need not cover every possible combination. Before the evaluation, the team shall set and then meet a minimum number of observations for each reported condition.
- [ ] **PER-13** Base. Repeated observations assigned to the same stationary object shall update one record. Observations of different objects shall not be combined into one record.
- [ ] **PER-14** Base. The team shall measure the time from image capture to the corresponding object-record creation or update.
- [ ] **PER-15** Extra. In the shared outdoor trial, an enemy record shall create a keepout area and an allied record shall not. Where the new area blocks the current path, the vehicle shall revise its remaining mission and shall keep the nearest vehicle edge at least 2 m from the enemy object.

## 6 Success Criteria
The course project is complete once every project requirement is met and every product requirement has a documented pass or fail. The system is accepted once every Base product requirement passes and every criterion below is satisfied. Performance is judged against the targets approved under PPR-06. A failed Extra requirement does not block acceptance.
- [ ] 1. Pose error against the surveyed reference is reported both with the vehicle stationary and with it moving. When RTK corrections are deliberately interrupted, the published status flags the pose as unreliable within 2 s.
- [ ] 2. Detection rate, false-positive rate, classification accuracy, position error, correct grouping of repeated observations and processing delay are reported for the conditions in PER-12. The report also compares the stated uncertainty with the measured error, and each measure meets its approved target.
- [ ] 3. Every observation behind a published benchmark record has an identifying photograph, and in the
[ ] **PER-13** trials the proportion of correctly grouped observations meets the target approved under
[ ] **PPR-06**.
- [ ] 4. A researcher outside the team evaluates a different detection method using only the delivered data, scripts and instructions.
- [ ] 5. The sample object records and records from the completed perception system pass the same interface validation.

# Planning and Completing Missions with TerraScout
## SLaRC-MISSION
*26 August 2026, SLaRC*

## 3 Goals and Objectives
By the end of the course TerraScout will plan, adapt and finish missions of four unordered destinations while keepout areas change during execution. TerraScout will follow approved rules when a destination cannot be reached and restore the mission safely after an interruption. Every destination will have a recorded outcome. Five objectives define the work.
- [x*] 1. Establish how reliably the current system follows an ordered route, maintains communication, supports manual takeover and records a run.
- [x*] 2. Develop and validate mission input for unordered destinations, a final destination and initial keepout areas, then select a feasible order using paths from the existing navigation system.
- [x*] 3. Add, update and remove keepout areas during execution, requesting new paths and reordering remaining destinations when access changes.
- [x?] 4. Apply an approved policy to unreachable destinations and restore interrupted missions in a safe suspended state.
- [ ] 5. Evaluate mission, safety and failure responses through repeated field trials, an endurance trial and deliberate interruptions.

## 5 Requirements

### 5.1 Project Requirements
- [x] **MPR-01** The team shall record the baseline described in Objective 1 before modifying any capability included in that baseline.
- [ ] **MPR-02** The team shall obtain the project owner’s approval of a safety procedure before the first assessed trial.
- [x*] **MPR-03** The assessed mission shall contain a start, four unordered destinations, a final destination and approved initial keepout areas. The team and project owner shall approve the test area and record each destination and keepout area in map coordinates. The initial mission and scripted update sequence shall remain fixed across assessed trials.
- [ ] **MPR-04** Before each assessed trial, the team shall define the mission, keepout updates, obstacles, interruptions, injected faults, weather, procedure and measurements.
- [x*] **MPR-05** The team shall report every attempted trial, including failed and aborted trials.
- [ ] **MPR-06** For every trial, the team shall record temperature and exposure to rain, dust or mud, and compare them with ARES REQ-150, REQ-160 and REQ-170. Environmental compliance is not an acceptance criterion.
- [ ] **MPR-07** The team shall verify runtime keepout behaviour using approved scripted updates that do not require operator input during autonomous movement.
- [x] **MPR-08** For every mechanical change, the team shall measure the affected capability before and after the change.
- [Pending] **MPR-09** Before assessed trials, the team shall document the rules for ordering, keepout updates, unreachable destinations and restoration, and obtain approval from the project owner.
- [x] **MPR-10** The team shall use published work to justify its design and evaluation choices.

### 5.2 Product Requirements
Base requirements are mandatory. Extra requirements apply only if time allows.
- [ ] **MIS-01** Base. When the physical emergency stop is activated, the vehicle shall override all software commands, stop within 1 s and remain stopped until a reset procedure is run (ARES REQ-340).
- [ ] **MIS-02** Base. An indicator lamp shall show whether the vehicle is in standby, manual or autonomous mode and shall be clearly visible from 10 m in sunlight (ARES REQ-100, REQ-110 and REQ-230).
- [ ] **MIS-03** Base. A visible warning shall begin at least 5 s before autonomous movement and remain active throughout that movement (ARES REQ-350 and REQ-360).
- [x*] **MIS-04** Base. The vehicle shall not exceed 1 m/s when moving forwards or backwards on flat ground or slopes (ARES REQ-180).
- [x*] **MIS-05** Base. During autonomous motion, the vehicle shall slow as it approaches an obstacle in its path and shall stop before reaching the obstacle. Before assessed trials, the team shall obtain approval from the project owner for the slowing and stopping distances.
- [x*] **MIS-06** Base. When the published localisation status reports that the pose estimate is unreliable, the vehicle shall stop autonomous motion within 2 s and report a localisation fault.
- [x*] **MIS-07** Base. Manual driving and takeover commands shall have priority over autonomous commands.
- [ ] **MIS-08** Base. If the command link is lost while the vehicle is under manual control, the vehicle shall stop within 1 s and report the loss.
- [ ] **MIS-09** Base. If the operator telemetry link is lost during autonomous motion, the vehicle shall stop after a configurable timeout with a default of 10 s and report the loss.
- [x*] **MIS-10** Base. The vehicle shall accept four unordered destinations, a final destination and named circular or polygonal keepout areas in map coordinates. The vehicle shall reject an invalid mission and report the reason.
- [x*] **MIS-11** Base. The vehicle shall use path lengths to determine which destinations are reachable and select their order. The vehicle shall publish the mission’s overall status, the selected order, the status and outcome of each destination, and the reason for every change of order.
- [x*] **MIS-12** Base. During a normal assessed mission, the vehicle shall execute without control input except for a safety intervention (ARES REQ-330). The explicit resume command in an interruption trial is permitted.
- [Pending] **MIS-13** Base. During execution, the vehicle shall accept automated updates that add or change a named circular or polygonal keepout area and its required clearance, or remove an existing area.
- [x*] **MIS-14** Base. When a keepout update blocks the current path, the vehicle shall request a new path. Where that update changes the reachability or path length of any remaining destination, the vehicle shall also select a new order.
- [x*] **MIS-15** Base. If no valid path to a destination is available after the approved recovery period, the vehicle shall apply the policy approved under MPR-09. The vehicle shall report the destination, outcome and reason, and proceed to the next destination unless the policy requires the mission to stop.
- [x*] **MIS-16** Base. Whenever the mission or any destination changes status, the vehicle shall preserve the mission identifier, the status of each destination, the keepout areas, the selected order and any failure reasons.
- [x*] **MIS-17** Base. After manual takeover or a process restart, the vehicle shall remain stopped and restore the mission in a suspended state. Resuming shall require an explicit command, valid localisation and new paths.
- [ ] **MIS-18** Base. The vehicle shall maintain control and telemetry at a range of 100 m (ARES REQ-200) and when natural terrain blocks the line of sight (ARES REQ-210). A separate test shall evaluate operation with other wireless networks active (ARES REQ-080).
- [x*] **MIS-19** Base. The vehicle shall support at least 20 min of continuous mission operation (ARES REQ- 370).
- [x*] **MIS-20** Base. In an assessed trial, the nearest vehicle edge shall pass within 1 m of each completed destination, following the Task 1 scoring distance, and the mission shall finish within 20 min.
- [x*] **MIS-21** Base. On a common time base, the vehicle shall record mission state, order, paths, keepout areas, commands, mode, faults, interventions, position, localisation and obstacle data.
- [x*] **MIS-22** Base. A script shall summarise each order change and destination outcome, including the reason. The script shall also report minimum distances, completion time, distance driven, stops, interventions and faults.
- [x?] **MIS-23** Extra. Given a bounded search area and required spacing, the vehicle shall generate and execute a patrol through that area while respecting all active keepout areas and the Base safety requirements.
- [x?] **MIS-24** Extra. A SLaRC-MISSION adapter shall map configured classes or attributes in live SLaRC- PERCEPTION records to keepout areas sized for position uncertainty and vehicle dimensions. In the shared outdoor trial, an enemy record shall create a keepout area and an allied record shall not. Where the new area blocks the current path, the vehicle shall revise its remaining mission and shall keep the nearest vehicle edge at least 2 m from the enemy object.

## 6 Success Criteria
The course project is complete once every project requirement is met and every product requirement has a documented pass or fail. The system is accepted for research trials once every Base product requirement passes and every criterion below is satisfied. A failed Extra requirement does not block acceptance.
- [x*] 1. The team attempts at least five assessed missions, of which three or more pass. In each passing trial, the vehicle reports a feasible order, visits every destination within 1 m, finishes within 20 min and receives no control input.
- [x*] 2. Scripted updates add, change and remove the specified keepout areas. When an update blocks a path, the vehicle requests another path and, if reachability or path lengths change, selects a new order. Every required clearance is maintained.
- [x*] 3. When one destination cannot be reached, the vehicle applies the approved policy and reports the destination, outcome and reason. The vehicle then proceeds to the next destination unless the policy requires the mission to stop.
- [x*] 4. After manual takeover and a deliberate restart, the vehicle remains stopped and restores the suspended mission. Resuming requires an explicit resume command, valid localisation and new paths.
- [ ] 5. Recordings of deliberate emergency-stop, obstacle, localisation, manual-link and telemetry-link faults show the vehicle stopping as required.
- [ ] 6. Someone who did not observe the trial identifies one injected fault from the trial’s recording and summary alone.
- [ ] 7. A researcher outside the team runs the mission from the operating procedure alone.
