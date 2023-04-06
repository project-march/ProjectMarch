std_msgs/Header header
#
# The type of instruction.
int8 gait_type
#
# All possible instruction gait_types.
int32 SIT = 0
int32 STAND = 1
int32 WALK = 2
int32 STEP_CLOSE = 3
int32 FORCE_UNKNOWN = 4
int32 ERROR  = 5

# Unique identifier for the input device sending this message. See `Alive`.
string id
