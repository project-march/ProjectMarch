# Mujoco Simulation
</p>
This repository is a sub-repository of the <b><a href="https://gitlab.com/project-march/march">Project March repository</h></b>.
It contains the ROS2 workspace for running the <b>Mujoco simulation-to-High level control</b> loop. It is mainly used for testing high-level controllers on a simulated exoskeleton in Mujoco. This allows for rapid prototyping of these controllers before the physical exoskeleton is operational.

<b>Please note that once this repository is deemed feature-complete, it will be integrated into the main March repository as a mock for the hardware interface.</b>

## How to Build
</p>

- To install required ROS enviroment, please follow the [Install ROS and tools](https://docs.projectmarch.nl/doc/getting_started/install_ros_and_tools.html) instructions.

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

## Workspace structure overview
</p>
Only the relevant files are shown in this structure overview:

```bash
📦Mujoco-simulation
 ┣ 📂acados*
 ┣ 📦acados_solver
 ┃ ┣ 📂src
 ┃ ┃ ┗ 📜solver_node.cpp
 ┃ ┣ 📝CMakeLists.txt
 ┃ ┗ 🔧package.xml
 ┣ 📦mujoco-interfaces
 ┃ ┣ 📂msg
 ┃ ┃ ┣ 📃MujocoDataControl.msg
 ┃ ┃ ┣ 📃MujocoDataRequest.msg
 ┃ ┃ ┣ 📃MujocoDataSensing.msg
 ┃ ┃ ┣ 📃MujocoDataState.msg
 ┃ ┃ ┗ 📃MujocoSetControl.msg
 ┃ ┣ 📂srv
 ┃ ┃ ┗ 📜ReadMujoco.srv
 ┃ ┣ 📝CMakeLists.txt
 ┃ ┗ 🔧package.xml
 ┣ 📦mujoco_reader
 ┃ ┣ 📂mujoco_reader
 ┃ ┃ ┗ 📜mujoco_reader_node.py
 ┃ ┣ 🔧package.xml
 ┃ ┣ 🔧setup.cfg
 ┃ ┗ 📝setup.py
 ┣ 📦mujoco_sim
 ┃ ┣ 📂config
 ┃ ┃ ┗ 📃low_level_controller_tunings.yaml
 ┃ ┣ 📂launch
 ┃ ┃ ┗ 📜mujoco_sim_launch.py
 ┃ ┣ 📂mujoco_sim
 ┃ ┃ ┣ 📂low_level_control
 ┃ ┃ ┃ ┣ controller_position.py
 ┃ ┃ ┃ ┣ controller_torque.py
 ┃ ┃ ┃ ┗ low_level_controller.py
 ┃ ┃ ┣ 📜mujoco_sim_node.py
 ┃ ┃ ┗ 📜mujoco_visualize.py
 ┃ ┣ 🔧package.xml
 ┃ ┣ 🔧setup.cfg
 ┃ ┗ 📝setup.py
 ┣ 📦mujoco_writer
 ┃ ┣ 📂mujoco_writer
 ┃ ┃ ┗ 📜mujoco_writer_node.py
 ┃ ┣ 🔧package.xml
 ┃ ┣ 🔧setup.cfg
 ┃ ┗ 📝setup.py
 ┣ 📦robot_description
 ┃ ┣ 📂src
 ┃ ┃ ┗ 📜mujoco_reader_node.py
 ┃ ┣ 📂urdf
 ┃ ┃ ┣ 📃march.xml
 ┃ ┃ ┗ 📃march7.urdf
 ┃ ┣ 📝CMakeLists.txt
 ┃ ┗ 🔧package.xml
 ┗ 📋README.md
```
*submodule of the repository

### <b>📦 acados</b>
The solver library used for high-level control. For more information, visit the repository <a>https://github.com/acados/acados</a>.

### <b>📦 acados_solver</b>
The package which performs the high-level control problem.

### <b>📦 mujoco_interfaces</b>
The package containing all used custom messages and services in this package.

### <b>📦 mujoco_reader</b>
The package containing the node which obtains data from the Mujoco Simulation.

### <b>📦 mujoco_sim</b>
The package containing the node which performs the Mujoco simulation loop.

### <b>📦 mujoco_writer</b>
The package containing the node which  writes data to the Mujoco simulation.

### <b>📦 robot_description</b>
The package containnig the models to be used in the Mujoco simulation.
