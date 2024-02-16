"""Author: Andrew Hutani, MIX"""

import json
import os

# Get the directory of this script
script_dir = os.path.dirname(os.path.realpath(__file__))

# Open the JSON file
with open(os.path.join(script_dir, 'modes.json'), 'r') as f:
    data = json.load(f)

# Get the output directory
output_dir = os.path.join(script_dir, '../include/march_mode_machine')

# Create a list to store all modes
all_modes = []

# Add each mode to the list
for inner_list in data:
    for mode in inner_list:
        all_modes.append(mode)

# Sort the list by the 'exoMode' value
all_modes.sort(key=lambda mode: mode['exoMode'])

# Open the output file
with open(os.path.join(output_dir, 'exo_mode.hpp'), 'w') as f:
    f.write("#pragma once\n#include <string>\n#include <vector>\n\n")

    f.write("// This is a generated file. Do not edit.\n\n")

    # Write the enum definition
    f.write('enum class exoMode {\n')

    # Write each enum member
    for mode in all_modes:
        name = mode['name']
        value = mode['exoMode']
        f.write(f'    {name} = {value},\n')

    # Write the closing brace for the enum
    f.write('};\n\n')

    # Write the toString function
    f.write("inline std::string toString(exoMode state) {\n")
    f.write("    switch (state) {\n")
    for mode in all_modes:
        name = mode['name']
        f.write(f"        case exoMode::{name}: return \"{name}\";\n")
    f.write("        default: return \"Unknown\";\n")
    f.write("    }\n")
    f.write("}\n\n")

    # Write the operator overload for output stream
    f.write("inline std::ostream& operator<<(std::ostream& os, exoMode state) {\n")
    f.write("    os << toString(state);\n")
    f.write("    return os;\n")
    f.write("}\n")