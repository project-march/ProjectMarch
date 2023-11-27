import os
from glob import glob
from setuptools import setup

package_name = "bezier_visualization"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    scripts=["bezier_visualization/bezier_visualization_node.py"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("launch/*launch.[pxy][yma]*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Marco",
    maintainer_email="marco2771@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["bezier_visualization_node = bezier_visualization.bezier_visualization_node:main"],
    },
)
