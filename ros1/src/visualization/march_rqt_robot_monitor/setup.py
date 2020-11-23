#!/usr/bin/env python
from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['march_rqt_robot_monitor', 'march_rqt_robot_monitor.diagnostic_analyzers'],
    package_dir={'': 'src'},
    scripts=['scripts/march_rqt_robot_monitor_node'],
)

setup(**d)
