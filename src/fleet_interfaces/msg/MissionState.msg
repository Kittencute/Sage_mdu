uint8 COMMAND_WAITING = 0
uint8 COMMAND_RUNNING = 1
uint8 COMMAND_PAUSED = 2
uint8 COMMAND_FINISHED = 3
uint8 COMMAND_FAILED = 4
uint8 COMMAND_ABORTED = 5

uint8 MISSION_IDLE = 0
uint8 MISSION_RUNNING = 1
uint8 MISSION_PAUSED = 2
uint8 MISSION_FINISHED = 3
uint8 MISSION_FAILED = 4
uint8 MISSION_ABORTED = 5

builtin_interfaces/Time stamp
# identifier of the mission in MissionPlan.id
int32 mission_id
# MissionCommand.id of the command being executed
int32 current_command_id
# MissionCommand.id of every command in the plan
int32[] command_ids
# COMMAND_* value of each command, in the order of command_ids
uint8[] command_status
# Failure reason for each command; aligned with command_ids, empty if none
string[] command_failure_reasons
# Reason for the most recent selected-order change; empty if unchanged
string order_change_reason
# MISSION_* value
uint8 mission_status

# Most recent mission-level failure or suspension reason, empty when none
string failure_reason

bool ready
string not_ready_reason
