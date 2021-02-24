import os
from glob import glob

from setuptools import setup, find_packages

package_name = "march_robot_information"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(),
    data_files=[
        (
            os.path.join("share", "ament_index", "resource_index", "packages"),
            [os.path.join("resource", package_name)],
        ),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        # (
        #     os.path.join("share", package_name, "test", "launch_test"),
        #     glob("test/launch_test/*.py"),
        # ),
        (
            os.path.join("share", package_name, "test", "unittest"),
            glob("test/unittest/*.py"),
        ),
        (
            os.path.join("share", package_name, "test", "resource"),
            glob("test/resource/*.*"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Project March",
    maintainer_email="software@projectmarch.nl",
    description="This node contains information about the march robot such as "
    "the joint names in its node parameters "
    "and makes this information public to other nodes.",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            f"{package_name} = {package_name}.robot_information_node:main",
        ],
    },
    tests_require=["pytest"],
)
