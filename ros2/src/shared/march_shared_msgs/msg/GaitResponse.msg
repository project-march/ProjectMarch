std_msgs/Header header
#
# The type of instruction.
int32 gait_type
#
# All possible instruction gait_types.
int32 SIT = 0
int32 STAND = 2
int32 WALK = 4
int32 FORCE_UNKNOWN = 5
int32 ERROR  = 7