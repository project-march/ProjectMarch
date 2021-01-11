#!/usr/bin/env python
from setuptools import setup
from glob import glob
import os

package_name = "march_rqt_gait_version_tool"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        (
            os.path.join("share", "ament_index", "resource_index", "packages"),
            [os.path.join("resource", package_name)],
        ),
        (os.path.join("share", package_name), ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.launch")),
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.launch.py")),
        ),
        (os.path.join("share", package_name), ["plugin.xml"]),
        (
            os.path.join("share", package_name),
            [os.path.join("resource", "gait_selection.ui")],
        ),
        (
            os.path.join("share", package_name),
            [os.path.join("resource", "parametric_pop_up.ui")],
        ),
        (
            os.path.join("share", package_name, "resource", "img"),
            glob(os.path.join("resource", "img", "*.png")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Project March",
    maintainer_email="software@projectmarch.nl",
    description="An RQT plugin for changing the gait version during runtime.",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "gait_version_tool = march_rqt_gait_version_tool.gait_version_tool_plugin:main"
        ],
    },
)
