std_msgs/Header header

# Mission state machine: idle, preflight, takeoff, outbound, descend, release, relaunch,
# ascend, return, land, complete, failed.
string state
string vehicle
string dropoff
# outbound | return | empty
string leg
float64 leg_remaining
uint32 waypoint
uint32 waypoint_count
bool avoiding
string threat
# Last notable event, human readable.
string detail

# Vehicle as the mission sees it.
bool connected
bool armed
string flight_mode
float64 battery
geometry_msgs/Point position
string payload
