"""Author: Andrew Hutani, MIX"""

import json
import os
import rospkg

script_dir = os.path.dirname(os.path.realpath(__file__))

json_dir = os.path.join(script_dir, '../../../march_mode_machine/generate/modes.json')
json_dir = os.path.normpath(json_dir)


# Open the JSON file
with open(json_dir) as f:
    data = json.load(f)



# Create a list to store all modes
all_modes = []

# Add each mode to the list
for inner_list in data:
    for mode in inner_list:
        all_modes.append(mode)

# Sort the list by the 'ExoMode' value
all_modes.sort(key=lambda mode: mode['ExoMode'])

output_dir = os.path.join(script_dir, '../msg')
print(output_dir)
with open(os.path.join(output_dir, 'ExoMode.msg'), 'w') as f:
    f.write("#ExoMode.msg\n")
    f.write("std_msgs/Header header\n")
    f.write("int8 mode\n")
    f.write("string node_type\n\n")

    f.write("# Constant types of modes\n")
    for mode in all_modes:
        name = mode['name']
        value = mode['ExoMode']
        f.write(f'int8 {name.upper()} = {value}\n')
    
    f.write('\nstring CARTESIAN = "cartesian" \n')
    f.write('string JOINT_ANGLES = "joint_angles" \n')
    f.write('string TEST_JOINTS = "test_joints" \n')
    f.write('string TEST_SETUP = "test_setup" \n\n')