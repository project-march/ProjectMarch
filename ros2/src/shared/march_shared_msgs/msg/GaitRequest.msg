std_msgs/Header header
#
# The type of instruction.
int8 gait_type
#
# All possible instruction gait_types.
int8 SIT = 0
int8 STAND = 2
int8 WALK = 4
int8 FORCE_UNKNOWN = 5
int8 ERROR  = 7

# If the type is a WALK this is the name of the WALK.
string gait_name

# Unique identifier for the input device sending this message. See `Alive`.
string id
