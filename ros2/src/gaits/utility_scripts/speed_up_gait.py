#!/usr/bin/env python3
from pathlib import Path
import yaml

speed_up_percentage = 1.1

gait_to_speed_up = "../march_gait_files/airgait_vi/walk"
versions_to_speed_up = {
    "left_close": "MVI_walk_leftclose_v9",
    "left_swing": "MVI_walk_leftswing_v9",
    "right_close": "MVI_walk_rightclose_v9",
    "right_open": "MVI_walk_rightopen_v9",
    "right_swing": "MVI_walk_rightswing_v9",
}

paths_that_failed = []


for subgait_name, version in versions_to_speed_up.items():
    path = Path(gait_to_speed_up + subgait_name + version)
    try:
        with open(path, "r") as subgait_file:
            print(path)
            content = yaml.full_load(subgait_file)
            content["duration"] = content["duration"] / 1.1
            for joint in content["joints"]:
                for index, setpoint in enumerate(content["joints"][joint]):
                    content["joints"][joint][index]["time_from_start"] = (
                        setpoint["time_from_start"] / 1.1
                    )
                    content["joints"][joint][index]["velocity"] = (
                        setpoint["velocity"] * 1.1
                    )

        with open(path, "w") as subgait_file:
            yaml.dump(content, subgait_file)
    except Exception as e:  # noqa: B902 PIE786
        paths_that_failed.append(path)
        print(e)
print(f"the paths {paths_that_failed} failed.")
