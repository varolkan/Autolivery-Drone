uint8 UNKNOWN=0
uint8 ATTACHED=1
uint8 RELEASED=2

std_msgs/Header header
uint8 state
# Gripper backend reporting (simulated, mavlink_gripper, servo, electromagnet, winch).
string gripper_type
# True when `state` comes from a sensor rather than being inferred from commands.
bool sensed
