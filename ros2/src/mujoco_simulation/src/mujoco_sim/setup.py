import os
from glob import glob
from setuptools import setup

package_name = "mujoco_sim"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    scripts=[
        "mujoco_sim/low_level_control/low_level_controller.py",
        "mujoco_sim/low_level_control/controller_position.py",
        "mujoco_sim/low_level_control/controller_torque.py",
        "mujoco_sim/sensor_data_extraction.py",
        "mujoco_sim/aie_passive_force/aie_passive_force.py",
    ],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("urdf/*")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name), glob("launch/*launch.[pxy][yma]*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Sahand",
    maintainer_email="sahandwagemakers@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["mujoco_sim_node = mujoco_sim.mujoco_sim_node:main"],
    },
)
