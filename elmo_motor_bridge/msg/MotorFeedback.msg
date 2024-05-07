# include path: elmo_motor_bridge/msg/motor_feedback.hpp

std_msgs/Header header

int64   position # rad
int64   velocity # rad/s
float64 current  # A

int32 op_mode