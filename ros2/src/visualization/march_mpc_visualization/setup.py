#!/usr/bin/env python
from setuptools import setup
from glob import glob
import os

package_name = "march_mpc_visualization"

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
            glob(os.path.join("launch", "*.launch.py")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Project March",
    maintainer_email="software@projectmarch.nl",
    description="This package contains a Flask application that streams the estimations from march_acado_mpc to a host",
    license="GPLv3",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "march_mpc_visualization= march_mpc_visualization.mpc_visualization_node:main"
        ],
    },
)
