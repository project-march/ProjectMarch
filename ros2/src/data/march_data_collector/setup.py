import os
from glob import glob

from setuptools import setup

package_name = "march_data_collector"


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
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Project March",
    maintainer_email="software@projectmarch.nl",
    description="Package that is responsible for computing the CoM using the IMU data "
    "and computing the capture point for balance purposes",
    license="TODO: License declaration",
    tests_require=[],
    entry_points={
        "console_scripts": [
            "data_collector = march_data_collector.data_collector_node:main"
        ],
    },
)
