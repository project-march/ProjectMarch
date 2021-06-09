#!/usr/bin/env python
from setuptools import setup
from glob import glob
import os

package_name = "march_smartglasses_bridge"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Project MARCH",
    maintainer_email="software@projectmarch.nl",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "march_smartglasses_bridge = march_smartglasses_bridge.bridge:main"
        ],
    },
)
