#ExoMode.msg
std_msgs/Header header
int8 mode
string node_type
int8 previous_mode

# Constant types of modes
int8 SIT = 0
int8 STAND = 1
int8 WALK = 2
int8 BOOTUP = 3
int8 ERROR = 4
int8 SIDEWAYSRIGHT = 5
int8 LARGEWALK = 6
int8 SMALLWALK = 7
int8 ASCENDING = 8
int8 DESCENDING = 9
int8 VARIABLESTEP = 10
int8 VARIABLEWALK = 11
int8 HIGHSTEP1 = 12
int8 HIGHSTEP2 = 13
int8 HIGHSTEP3 = 14
int8 HINGE = 15
int8 SIDEWAYSLEFT = 16

string CARTESIAN = "cartesian" 
string JOINT_ANGLES = "joint_angles" 
string TEST_JOINTS = "test_joints" 
string TEST_SETUP = "test_setup" 

