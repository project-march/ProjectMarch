# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
import os

from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup

setup_args = generate_distutils_setup(
    packages=["march_parameter_server"],
    package_dir={"": "src"},
    scripts=[os.path.join("scripts", "march_parameter_server_node")],
)

setup(**setup_args)
