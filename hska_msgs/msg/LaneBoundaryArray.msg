# Contains an array of lane boundaries contained in a section or segment of road

std_msgs/Header header

uint32 min_look_ahead   # [m]
uint32 max_look_ahead   # [m]

uint32 img_width        # [pixel]
uint32 img_height       # [pixel]

float32 pixel_to_m

hska_msgs/LaneBoundary[] boundaries
