import os
from glob import glob

from setuptools import setup

package_name = "march_simulation"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob(os.path.join("launch", "*.launch.py"))),
        (os.path.join("share", package_name, "obstacles"), glob(os.path.join("obstacles", "*.xacro"))),
        (os.path.join("share", package_name, "worlds"), glob(os.path.join("worlds", "*.world"))),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Project March",
    maintainer_email="software@projectmarch.nl",
    description="The simulation world in which the exoskeleton can be placed",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            "to_world_transform = march_simulation.to_world_transform:main",
            "spawn_obstacle = march_simulation.spawn_obstacle:main",
            "set_obstacle_dimensions = march_simulation.set_obstacle_dimensions:main",
        ],
    },
)
