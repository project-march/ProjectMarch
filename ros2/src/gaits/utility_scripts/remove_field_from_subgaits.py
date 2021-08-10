#!/usr/bin/env python3
from pathlib import Path
import yaml

field_to_remove = "version"

paths_that_failed = []
for path in Path("/home/pmarch/march/ros1/src").rglob("*.subgait"):
    try:
        file = open(path, "r")
        print(path)
        content = yaml.full_load(file)
        if "version" in content:
            content.pop(field_to_remove)
        file.close()
        file = open(path, "w")
        yaml.dump(content, file)
    except Exception as e:  # noqa: B902
        paths_that_failed.append(path)
        print(e)

print(f"the paths {paths_that_failed} failed.")
