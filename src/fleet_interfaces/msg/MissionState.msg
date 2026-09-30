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
# MISSION_* value
uint8 mission_status

bool ready
string not_ready_reason
