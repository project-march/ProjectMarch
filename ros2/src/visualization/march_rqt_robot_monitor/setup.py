from distutils.core import setup

from setuptools import setup
from glob import glob
import os

package_name = 'march_rqt_robot_monitor'

d = generate_distutils_setup(
    packages=[
        "march_rqt_robot_monitor",
        "march_rqt_robot_monitor.diagnostic_analyzers",
    ],
    package_dir={"": "src"},
    scripts=["scripts/march_rqt_robot_monitor_node"],
)
