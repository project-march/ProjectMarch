from setuptools import setup
from glob import glob
import os

package_name = "march_gait_preprocessor"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Project MARCH",
    maintainer_email="software@projectmarch.nl",
    description="Publishes fake foot locations",
    license="TODO: License declaration",
    tests_require=["pytest", "unittest"],
    entry_points={
        "console_scripts": [
            "march_gait_preprocessor = march_gait_preprocessor.gait_preprocessor_node:main"
        ]
    },
)
