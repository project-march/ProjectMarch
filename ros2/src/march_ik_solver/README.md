# MARCH Inverse Kinematics Solver

## Overiew

This package is an implementation of the pseudo-inverse Jacobian-based inverse kinematics using task priority control. In principle, it solves the inverse kinematics by converging towards the desired task configurations based on the current joint configuration. 

The Inverse Kinematics Solver Node (IK Solver Node) is enacted when it receives desired feet positions from the Gait Planning Node given that the State Estimator Node updates the joint state beforehand. Using Complete Orthogonal Decomposition, it calculates the pseudo-inverse of the Jacobian which is then used to transform the weighted error between the desired and the current workspace positions into the desired joint velocity. This joint velocity is integrated over some timestep to obtain the desired position. The solving of the inverse kinematics is iterated as many times as possible within two states. The iterations is terminated once convergence, max number of iterations, or the maximum timeout are reached. The IK Solver Node then finally publishes the desired joint positions to the Hardware Interface via Joint Trajectory Controller.

The flow of the IK Solver Node integrated with the other relevant modules can be seen in the sequence diagram below.

![Sequence diagram of IK Solver](./docs/imgs/seq_diagram_iks.png)



**Keywords**: inverse kinematics, task priority control

### License

The source is released under GNU[GNU General Public License v3.0 and above](https://gitlab.com/project-march/march/-/blob/main/LICENSE.md)

**Author: Alexander James Becoy <br/>
Affiliation: [Project MARCH](https://www.projectmarch.nl)<br/>
Maintainers:**

- **Alexander James Becoy, alexanderjames.becoy@projectmarch.nl**

The MARCH State Estimator package has been tested under ROS2 Foxy on Ubuntu 20.04.

## Installation

### Building from Source

**Dependencies**

- [Robot Operatibg System 2 (ROS2)](https://docs.ros.org/en/foxy/index.html) (middleware for robotics),
- [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page) (linear algebra library)
- [Boost](https://www.boost.org/) (support for linear algebra library)

**Building**

To build from source, clone the latest version from this repository into MARCH's ROS2 workspace and compile the package using

```Linux
cd ~/march/ros2
rosdep install --from-paths . --ignore-src
mba
source install/local_setup.bash
```

### Unit Tests

Run the unit tests with

```Linux
colcon test --packages-select march_ik_solver && colcon test-result --verbose
```

or when using the MARCH aliases

```Linux
mbt march_ik_solver
```

## Usage

You can try out the inverse kinematics solver by launching the simulation.

```Linux
ros2 launch march_launch march.launch.py
```

Then press or type the available gait modes in the newly launched Input Device GUI / Terminal.

## Config files

- **ik_solver.yaml**: Contains the parameters for the IKSolver and Tasks. _See Nodes > ik\_solver\_node > Parameters_.

## Launch files

- **ik_solver.launch.py**: Launch the IK Solver Node with the parameters given in the `ik_solver.yaml`.

## Nodes

### ik_solver_node

TODO: Provide description of this node

**Subscribed Topics**

- `/state_estimation/state` ([march_shared_msgs/StateEstimation](https://gitlab.com/project-march/march/-/blob/dev/ros2/src/shared/march_shared_msgs/msg/StateEstimation.msg?ref_type=heads))
    [Joint state](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/JointState.html), filtered [IMU](https://docs.ros2.org/foxy/api/sensor_msgs/msg/Imu.html), [feet poses](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Pose.html), and [stance leg](https://docs.ros2.org/foxy/api/std_msgs/msg/UInt64.html).
- `/ik_solver/buffer/input` ([march_shared_msgs/IKSFootPositions](https://gitlab.com/project-march/march/-/blob/dev/ros2/src/shared/march_shared_msgs/msg/IksFootPositions.msg?ref_type=heads))
    [header](https://docs.ros2.org/latest/api/std_msgs/msg/Header.html), and [left foot position and right foot position in body reference frame](https://docs.ros2.org/latest/api/geometry_msgs/msg/Point.html).

**Published Topic**

- `/joint_trajectory_controller/joint_trajectory` ([trajectory_msgs/JointTrajectory](https://docs.ros2.org/foxy/api/trajectory_msgs/msg/JointTrajectory.html))

**Client**

- `/state_estimation/get_node_position` ([march_shared_msgs/srv/GetNodePosition](https://gitlab.com/project-march/march/-/blob/dev/ros2/src/shared/march_shared_msgs/srv/GetNodePosition.srv?ref_type=heads))
- `/state_estimation/get_node_jacobian` ([march_shared_msgs/srv/GetNodeJacobian](https://gitlab.com/project-march/march/-/blob/dev/ros2/src/shared/march_shared_msgs/srv/GetNodeJacobian.srv?ref_type=heads))

**Parameters**

- `state_estimator_time_offset`: Time offset between two states from State Estimator Node in seconds.
- `joint_trajectory_controller_period`: Time to arrive at desired joint positions from the current joint positions in seconds.
- `convergence_threshold`: Obtained desired joint positions is accepted if the corresponding error norm is below this threshold.
- `max_iterations`: Max number of iteration of solving the inverse kinematics.
- `integral_dt`: Timestep for integrating the joint velocity to joint position.
- `joint`:
    - `names`: List of names of the joints in this order. Additionally, in order of left to right, and from base to foot.
    - `limits`: 
        - `positions`: List of `upper` and `lower` joint positions limits to the corresponding joint name in degrees.
        - `velocities`:  List of `upper` and `lower` joint velocities limits to the corresponding joint name in degrees.
- `task_names`: List of names of the task in this order. First task has the lowest priority. The list is also used to iterate over the following parameters.
- _`arbitrary task name`_:
    - `nodes`: List of names of interested nodes to obtain their positions and Jacobians from the Robot Description Node. E.g. `left_ankle` and `right_ankle` for Motion Task.
    - `m`: The workspace dimension size in unsigned integer.
    - `n`: The configuration dimension size in unsigned integer.
    - `kp`: List of proportional gains corresponding to each element in the workspace vector in double.
    - `kd`: List of derivative gains corresponding to each element in the workspace vector in double.
    - `ki`: List of integral gains corresponding to each element in the workspace vector in double.
    - `damp_coeff`: The damping coefficient to ensure singularity-robustness when calculating the pseudo-inverse in double.

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://gitlab.com/project-march/march/-/issues).

## Credit

Credits to [leggedrobotics](https://github.com/leggedrobotics/ros_best_practices/tree/main) for providing [a template to write the README of a custom ROS package](https://github.com/leggedrobotics/ros_best_practices/tree/main/ros_package_template).