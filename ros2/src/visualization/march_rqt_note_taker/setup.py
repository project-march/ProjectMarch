#!/usr/bin/env python
from setuptools import setup
from glob import glob
import os

package_name = "march_rqt_note_taker"

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
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name), [os.path.join("plugin.xml")]),
        (
            os.path.join("share", package_name),
            [os.path.join("resource", "note_taker.ui")],
        ),
        (
            os.path.join("share", package_name, "resource", "img"),
            glob("resource/img/*.png"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Project March",
    maintainer_email="software@projectmarch.nl",
    description="An RQT plugin for monitoring log messages and writing logs.",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["note_taker = march_rqt_note_taker.notes_plugin:main"],
    },
)
