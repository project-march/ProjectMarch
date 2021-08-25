#!/usr/bin/env python3
# Author: Wolf Nederpel
# Date: 25-8-2021
# Description: Removes a selected field from the subgait files under the specified path
# usage: set the right field to remove, fix the path from which to remove and run ./remove_field_from_subgait.py
from pathlib import Path
import yaml

field_to_remove = "version"

paths_that_failed = []
for path in Path(
    "../ros2/src/gaits/march_gait_files/airgait_vi/",
).rglob("*.subgait"):
    try:
        with open(path, "r") as subgait_file:
            print(path)
            content = yaml.full_load(subgait_file)
            if "version" in content:
                content.pop(field_to_remove)
            subgait_file.close()

        with open(path, "w") as subgait_file:
            yaml.dump(content, subgait_file)
    except Exception as e:  # noqa: B902 PIE786
        paths_that_failed.append(path)
        print(e)

print(f"The paths {paths_that_failed} failed.")
