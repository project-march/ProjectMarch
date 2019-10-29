## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['ros_test_tutorial'],
    package_dir={'': 'doc/development/ros_test_tutorial/src'})

setup(**setup_args)
