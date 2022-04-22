#!/usr/bin/env python3
"""Updates the starting positions of the specified gaits.

Author: Wolf Nederpel, MVI.
Date: 25 - 08 - 2021
Usage: Set the right old and new stand positions, fix the path from which to change
    and run `./fix_start_positions_gait.py`.
"""
from pathlib import Path
import yaml
import copy

old_stand_position_start = {
    "left_ankle": {
        "position": 0.0,
        "time_from_start": 0,
        "velocity": 0.0,
    },
    "left_hip_aa": {
        "position": 0.0349,
        "time_from_start": 0,
        "velocity": 0.0,
    },
    "left_hip_fe": {
        "position": -0.1745,
        "time_from_start": 0,
        "velocity": 0.0,
    },
    "left_knee": {
        "position": 0.0,
        "time_from_start": 0,
        "velocity": 0.0,
    },
    "right_ankle": {
        "position": 0.0,
        "time_from_start": 0,
        "velocity": 0.0,
    },
    "right_hip_aa": {
        "position": 0.0349,
        "time_from_start": 0,
        "velocity": 0.0,
    },
    "right_hip_fe": {
        "position": -0.1745,
        "time_from_start": 0,
        "velocity": 0.0,
    },
    "right_knee": {
        "position": 0.0,
        "time_from_start": 0,
        "velocity": 0.0,
    },
}
old_stand_position_end = copy.deepcopy(old_stand_position_start)

new_stand_position_start = {
    "left_ankle": {
        "position": 0.0524,
        "time_from_start": 0,
        "velocity": 0.0,
    },
    "left_hip_aa": {
        "position": 0.0349,
        "time_from_start": 0,
        "velocity": 0.0,
    },
    "left_hip_fe": {
        "position": -0.1745,
        "time_from_start": 0,
        "velocity": 0.0,
    },
    "left_knee": {
        "position": 0.0,
        "time_from_start": 0,
        "velocity": 0.0,
    },
    "right_ankle": {
        "position": 0.0524,
        "time_from_start": 0,
        "velocity": 0.0,
    },
    "right_hip_aa": {
        "position": 0.0349,
        "time_from_start": 0,
        "velocity": 0.0,
    },
    "right_hip_fe": {
        "position": -0.1745,
        "time_from_start": 0,
        "velocity": 0.0,
    },
    "right_knee": {
        "position": 0.0,
        "time_from_start": 0,
        "velocity": 0.0,
    },
}
new_stand_position_end = copy.deepcopy(new_stand_position_start)


# actually for dicts in dicts
def almost_equal_nested_dict(dict1: dict, dict2: dict) -> bool:
    """Todo: Add docstrings."""
    result = True
    for key_outer, value in dict1.items():
        for key_inner, true_value in value.items():
            result = result and (round(true_value, 4) == round(dict2[key_outer][key_inner], 4))
    return result


amount_of_start_positions_set = 0
amount_of_end_positions_set = 0
paths_that_failed = []
for path in Path(
    "../ros2/src/gaits/march_gait_files/airgait_vi/",
).rglob("*.subgait"):
    try:
        with open(path, "r") as subgait_file:
            print(path)
            content = yaml.full_load(subgait_file)
            final_time = content["duration"]
            current_start_position = {}
            current_end_position = {}

            for joint, setpoint_list in content["joints"].items():
                current_start_position[joint] = setpoint_list[0]
                current_end_position[joint] = setpoint_list[-1]

                old_stand_position_end[joint]["time_from_start"] = final_time
                new_stand_position_end[joint]["time_from_start"] = final_time

            if almost_equal_nested_dict(
                current_start_position,
                old_stand_position_start,
            ):
                for joint in content["joints"].keys():
                    content["joints"][joint][0] = new_stand_position_start[joint]  # noqa: E501
                amount_of_start_positions_set += 1

            if almost_equal_nested_dict(
                current_end_position,
                old_stand_position_end,
            ):
                for joint in content["joints"].keys():
                    content["joints"][joint][-1] = new_stand_position_end[joint]  # noqa: E501
                amount_of_end_positions_set += 1
            subgait_file.close()

        with open(path, "w") as subgait_file:
            yaml.dump(content, subgait_file)
    except Exception as e:  # noqa: B902 PIE786
        paths_that_failed.append(path)
        print(e)

print(f"The paths {paths_that_failed} failed.")
print(f"There were {amount_of_start_positions_set} start positions changed")
print(f"There were {amount_of_end_positions_set} end positions changed")
