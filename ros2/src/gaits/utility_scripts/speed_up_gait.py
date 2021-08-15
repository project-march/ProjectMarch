#!/usr/bin/env python3
import os
import yaml

speed_up_percentage = 1.1

gait_to_speed_up = "../march_gait_files/airgait_vi/walk"
common_prefix = "MVI_walk_"
common_suffix = "_v9"
gaits = ["left_close", "left_swing", "right_close", "right_open", "right_swing"]
versions_to_speed_up = {}
for gait_name in gaits:
    versions_to_speed_up[gait_name] = (
        common_prefix + gait_name.replace("_", "") + common_suffix
    )
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
    read_path = os.path.join(gait_to_speed_up, subgait_name, version + subgait_suffix)
    try:
        with open(read_path, "r") as subgait_file:
            print(read_path)
            content = yaml.full_load(subgait_file)
            content["duration"] = round(content["duration"] / 1.1)
            for joint in content["joints"]:
                for index, setpoint in enumerate(content["joints"][joint]):
                    content["joints"][joint][index]["time_from_start"] = round(
                        setpoint["time_from_start"] / 1.1
                    )
                    content["joints"][joint][index]["velocity"] = round(
                        setpoint["velocity"] * 1.1, 4
                    )
        if new_version_extension != "":
            new_version_name = version[: version.rfind("_") + 1] + new_version_extension
        if new_description != "":
            content["description"] = new_description
        write_path = os.path.join(
            gait_to_speed_up, subgait_name, new_version_name + subgait_suffix
        )
        with open(write_path, "w") as subgait_file:
            yaml.dump(content, subgait_file)
    except Exception as e:  # noqa: B902 PIE786
        paths_that_failed.append(read_path)
        print(e)
print(f"the paths {paths_that_failed} failed.")
