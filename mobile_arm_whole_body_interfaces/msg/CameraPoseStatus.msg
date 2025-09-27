# Matching identifier for the processed target
uint32 id

# Tracking lifecycle status
uint8 STATUS_NEW=0
uint8 STATUS_ACTIVE=1
uint8 STATUS_COMPLETE=2
uint8 STATUS_FAILED=3

uint8 status
string message
# Optional timestamp for this status update
builtin_interfaces/Time stamp
