uint8 STAGE_UNDEFINED=0
uint8 STAGE_ARM_RAMP=1
uint8 STAGE_CHASSIS_RAMP=2
uint8 STAGE_TRACKING=3

uint8 stage_id
string label
uint32 start_index
uint32 end_index
builtin_interfaces/Duration start_time_from_start
builtin_interfaces/Duration end_time_from_start
builtin_interfaces/Duration duration
float32 progress
