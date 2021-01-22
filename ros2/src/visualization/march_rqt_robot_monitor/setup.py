"""Setup file for the march_rqt_robot_monitor package."""

from setuptools import setup
from glob import glob
import os

package_name = "march_rqt_robot_monitor"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name, f"{package_name}.diagnostic_analyzers"],
    data_files=[
        (
            os.path.join("share", "ament_index", "resource_index", "packages"),
            [os.path.join("resource", package_name)],
        ),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Project March",
    maintainer_email="software@projectmarch.nl",
    description="An RQT plugin for monitoring log messages and writing logs.",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [f"{package_name}_node = {package_name}.updater:main"],
    },
)
