uint8 UNKNOWN = 0
uint8 TAKEOFF = 1
uint8 LAND = 2
uint8 WAYPOINT = 3
uint8 HOME = 4
uint8 CAMERA_IMAGE = 5
uint8 VIDEO_START = 6
uint8 VIDEO_STOP = 7

# identifier of the command within its plan
int32 id
uint8 kind
# WGS 84, degrees
float64 latitude
float64 longitude
# metres above the altitude of the vehicle's local position origin
float64 altitude_agl
# time to hold at the position, in seconds
float64 hold_s
# distance from the position at which it counts as reached, in metres
float64 acceptance_radius_m
# heading in degrees
float64 yaw_deg
# additional command-specific values, meaning depends on kind
float64[] parameters
