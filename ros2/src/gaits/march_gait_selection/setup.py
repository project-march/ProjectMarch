"""Author: Unknown."""

import os
from glob import glob, iglob
from setuptools import setup

package_name = "march_gait_selection"


def data_files():
    """Generates the list of data files necessary for gait selection.

    The gait and subgait files required for testing are taken from the ros1 directory to decrease duplication.
    """
    test_gait_files_source = "test/testing_gait_files"
    data = [
        (
            os.path.join("share", "ament_index", "resource_index", "packages"),
            [os.path.join("resource", package_name)],
        ),
        (os.path.join("share", package_name), ["package.xml"]),
        (
            os.path.join("share", package_name, "test", "resources"),
            [os.path.join(test_gait_files_source, "default.yaml")],
        ),
        (
            os.path.join("share", package_name, "test", "resources"),
            [os.path.join(test_gait_files_source, "realsense_gaits.yaml")],
        ),
        (os.path.join("share", package_name, "launch"), glob(os.path.join("launch", "*.launch.py"))),
        (
            os.path.join("share", package_name, "position_queue"),
            glob(os.path.join("config", "position_queue.yaml")),
        ),
    ]
    for filename in iglob(os.path.join(test_gait_files_source, "**", "*.subgait"), recursive=True):
        data.append(
            (
                os.path.join(
                    "share",
                    package_name,
                    "test",
                    "resources",
                    os.path.dirname(os.path.relpath(filename, test_gait_files_source)),
                ),
                [filename],
            )
        )
    for filename in iglob(os.path.join(test_gait_files_source, "**", "*.gait"), recursive=True):
        data.append(
            (
                os.path.join(
                    "share",
                    package_name,
                    "test",
                    "resources",
                    os.path.dirname(os.path.relpath(filename, test_gait_files_source)),
                ),
                [filename],
            )
        )
    return data


setup(
    name=package_name,
    version="0.0.0",
    packages=[
        package_name,
        "march_gait_selection.state_machine",
        "march_gait_selection.gaits",
        "march_gait_selection.dynamic_interpolation",
        "march_gait_selection.dynamic_interpolation.cybathlon_obstacle_gaits",
    ],
    data_files=data_files(),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Project March",
    maintainer_email="software@projectmarch.nl",
    description="Package that is responsible for receiving input from the input device, "
    "determining whether a request is valid and executing the gait",
    license="TODO: License declaration",
    tests_require=["pytest", "unittest"],
    entry_points={
        "console_scripts": ["march_gait_selection = march_gait_selection.gait_selection_node:main"],
    },
)
