# Operating mode for the whole-body controller
uint8 MODE_IDLE=0
uint8 MODE_ARM_RAMP=1
uint8 MODE_BASE_RAMP=2
uint8 MODE_TRACKING=3
uint8 MODE_HOLD=4
uint8 MODE_FAULT=5

uint8 mode
uint32 active_target_id
builtin_interfaces/Time stamp
string mode_label
