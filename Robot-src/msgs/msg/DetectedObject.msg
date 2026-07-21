std_msgs/Header header

geometry_msgs/Point position  # x, y in mm (z always 0 for 2D detection)
float32 orientation           # rotation in degrees (0-360)
string color                  # detected color name
int32 object_id               # Unique ID for tracking
