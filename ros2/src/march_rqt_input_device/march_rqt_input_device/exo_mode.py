import json
from enum import Enum
import os
import rclpy

class NamedEnum(Enum):
    def __str__(self):
        return self.name

def create_enum_from_json(json_filename):
    # Get the directory of the current script
    script_dir = os.path.dirname(os.path.realpath(__file__))

    # Construct the full path to the JSON file
    json_file = os.path.join(script_dir, json_filename)

    with open(json_file, 'r') as f:
        data = json.load(f)

    # Flatten the list of lists into a single list
    data = [item for sublist in data for item in sublist]

    # Create a dictionary where the keys are the names and the values are the exoModes
    data_dict = {item['name']: item['ExoMode'] for item in data}

    return NamedEnum('ExoMode', data_dict)

layout_path = os.path.join("../../march_mode_machine/", "generate", "modes.json")
ExoMode = create_enum_from_json(layout_path)

    # def __str__(self):
    #     return self.name