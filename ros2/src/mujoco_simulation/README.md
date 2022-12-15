## Mujoco Simulation

This repository is a sub-repository of the <b><a href="https://gitlab.com/project-march/march">Project March repository</a></b>.
It contains the ROS2 workspace for running the <b>Mujoco simulation-to-High level control</b> loop. It is mainly used for testing high-level controllers on a simulated exoskeleton in Mujoco. This allows for rapid prototyping of these controllers before the physical exoskeleton is operational.

<b>Please note that once this repository is deemed feature-complete, it will be integrated into the main March repository as a mock for the hardware interface.</b>

## How to Build


- To install required ROS environment, please follow the [Installation ROS and tools](https://docs.projectmarch.nl/doc/getting_started/install_ros_and_tools.html) instructions.

- Then, to set up the workspace, copy-paste the following instructions line by line:

```
$ pip3 install mujoco
$ source /opt/ros/foxy/setup.bash
$ git clone https://gitlab.com/project-march/sideprojects/Mujoco-simulation --recurse-submodules
$ cd Mujoco-simulation
$ colcon build --packages-select acados
$ colcon build --packages-select acados_solver
$ colcon build
$ source install/setup.bash
```
