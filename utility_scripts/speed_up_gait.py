#!/usr/bin/env python3
# Author: Wolf Nederpel
# Date: 25 - 08 - 2021  # noqa: E800
# Description: Speeds up selected gaits
# Usage: set the subgaits to speed up either by a suffix and prefix or by giving the entire name,
# optionally set the neww extension and description and run ./speed_up_gait.py
import os
import yaml

speed_up_factor = 1.1

directory_path = "../ros2/src/gaits/march_gait_files/airgait_vi/"
gait_name = "walk"
common_prefix = "MVI_" + gait_name.replace("_", "") + "_"
common_suffix = "_v9"
subgaits = [
    "left_close",
    "left_swing",
    "right_close",
    "right_open",
    "right_swing",
    "left_open",
]
versions_to_speed_up = {}
for subgait_name in subgaits:
    versions_to_speed_up[subgait_name] = common_prefix + subgait_name.replace("_", "") + common_suffix
# versions_to_speed_up = {  # noqa: E800
#     "left_close": "MVI_walk_leftclose_v9",  # noqa: E800
#     "left_swing": "MVI_walk_leftswing_v9",  # noqa: E800
#     "right_close": "MVI_walk_rightclose_v9",  # noqa: E800
#     "right_open": "MVI_walk_rightopen_v9",  # noqa: E800
#     "right_swing": "MVI_walk_rightswing_v9",  # noqa: E800
# }  # noqa: E800

new_version_extension = "v100"
new_description = "v9 but faster"

paths_that_failed = []
subgait_suffix = ".subgait"

for subgait_name, version in versions_to_speed_up.items():
    read_path = os.path.join(
        directory_path,
        gait_name,
        subgait_name,
        version + subgait_suffix,
    )
    try:
        with open(read_path, "r") as subgait_file:
            print(read_path)
            content = yaml.full_load(subgait_file)
            content["duration"] = round(content["duration"] / speed_up_factor)
            for joint in content["joints"]:
                for index, setpoint in enumerate(content["joints"][joint]):
                    content["joints"][joint][index]["time_from_start"] = round(
                        setpoint["time_from_start"] / speed_up_factor,
                    )
                    content["joints"][joint][index]["velocity"] = round(
                        setpoint["velocity"] * speed_up_factor,
                        4,
                    )
        if new_version_extension != "":
            new_version_name = version[: version.rfind("_") + 1] + new_version_extension  # noqa: E501
        else:
            new_version_name = version
        if new_description != "":
            content["description"] = new_description
        write_path = os.path.join(
            directory_path,
            gait_name,
            subgait_name,
            new_version_name + subgait_suffix,
        )
        with open(write_path, "w") as subgait_file:
            yaml.dump(content, subgait_file)
    except Exception as e:  # noqa: B902 PIE786
        paths_that_failed.append(read_path)
        print(e)
print(f"The paths {paths_that_failed} failed.")
