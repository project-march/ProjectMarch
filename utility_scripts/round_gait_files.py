#!/usr/bin/env python3
from pathlib import Path
import yaml

desired_digits = 4
paths_that_failed = []

for path in Path(
        "../ros2/src/gaits/march_gait_files/airgait_vi/",
).rglob("*.subgait"):
    try:
        with open(path, "r") as subgait_file:
            print(path)
            content = yaml.full_load(subgait_file)
            for joint in content["joints"]:
                for index, setpoint in enumerate(content["joints"][joint]):
                    content["joints"][joint][index]["position"] = round(
                        setpoint["position"],
                        desired_digits,
                    )
                    content["joints"][joint][index]["velocity"] = round(
                        setpoint["velocity"],
                        desired_digits,
                    )
            subgait_file.close()

        with open(path, "w") as subgait_file:
            yaml.dump(content, subgait_file)
    except Exception as e:  # noqa: B902 PIE786
        paths_that_failed.append(path)
        print(e)
print(f"the paths {paths_that_failed} failed.")
