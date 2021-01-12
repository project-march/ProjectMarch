# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup

setup_args = generate_distutils_setup(
    packages=[
        "march_utility",
        "march_utility.gait",
        "march_utility.exceptions",
        "march_utility.foot_classes",
        "march_utility.utilities",
    ],
    package_dir={"": "src"},
)

setup(**setup_args)
