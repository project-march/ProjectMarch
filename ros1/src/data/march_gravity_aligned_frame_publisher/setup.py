# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup

setup_args = generate_distutils_setup(
    packages=["march_gravity_aligned_frame_publisher"],
    package_dir={"": "src"},
    scripts=["scripts/march_gravity_aligned_frame_publisher"],
)

setup(**setup_args)
