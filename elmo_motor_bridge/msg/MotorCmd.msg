# include path: elmo_motor_bridge/msg/motor_cmd.hpp

int32   id       # select the motor
int32   op_mode  # select operational mode
float64 position # unit: rad
float64 velocity # unit: rad/s
float64 current  # unit: A

# op_mode:
# OPM402_CYCLIC_SYNC_POSITION_MODE = 8,
# OPM402_CYCLIC_SYNC_VELOCITY_MODE = 9,
# OPM402_CYCLIC_SYNC_TORQUE_MODE = 10,

