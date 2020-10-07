# Desired State of the Advanced Driver Assistance Systems

std_msgs/Header header

uint8 NONE = 0
uint8 LEFT = 1
uint8 RIGHT = 2

bool lkas               # Lane Keeping Assist System, 0 = inactive  or 1 = active
uint8 lane_change_req   # Lane change request for left and right

bool cc                 # Cruise Control, 0 = inactive  or 1 = active
float32 cmd_velocity    # Command Velocity for Cruise Controller [m/s]
