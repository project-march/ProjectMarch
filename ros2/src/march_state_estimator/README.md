# MARCH State Estimator

## Overiew

TODO: Write overview description of this package.

**Keywords**: state estimation, robot description, URDF parser, kinematics, dynamics, GiNaC, symbolic expressions

### License

The source is released under a TODO: Insert license.

**Author: Alexander James Becoy, Alexander Andonov <br/>
Affiliation: [Project MARCH](https://www.projectmarch.nl)<br/>
Maintainers:**

- **Alexander James Becoy, alexanderjames.becoy@projectmarch.nl**
- **Alexander Andonov, alexander.andonov@projectmarch.nl**

The MARCH State Estimator package has been tested under ROS2 Foxy on Ubuntu 20.04.



## Installation

### Installation from Packages

**IMPORTANT.** We need to install the necessary deb packages:

```Linux
sudo apt-get update
sudo apt-get install libmpfr-dev libgmp-dev libboost-all-dev
```

### Building GiNaC

This package makes full use of a C++ library called **GiNaC**, which is not readily available in **ROS**/**ROS2**. Therefore, it is required to build this library separately.

First, update the git submodules.

```Linux
cd ~/march
git submodule update --recursive --init
```

Go to the GiNaC folder and create a build folder build the GiNaC library.

```Linux
mkdir build && cd build
cmake ..
make
```

This is not strictly necessary, but it is strongly recommended to test the library.

```Linux
make check
```

**Installing GiNaC**

Finally, complete the build make by installing the make file.

```Linux
sudo make install
```

**Uninstalling GiNaC**

**NOTE:** To remove GiNaC, it is mandatory to clean the generated files before deleting the library.

```Linux
cd ~/march/ros2/src/libraries/ginac
make clean
make uninstall
```

### Building from Source

**Dependencies**

- [Robot Operatibg System 2 (ROS2)](https://docs.ros.org/en/foxy/index.html) (middleware for robotics),
- [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page) (linear algebra library)

**Building**

To build from source, clone the latest version from this repository into MARCH's ROS2 workspace and compile the package using

```Linux
cd ~/march/ros2
rosdep install --from-paths . --ignore-src
colcon build
```

### Unit Tests

Run the unit tests with

```Linux
TODO: Provide command line for unit testing
```

### Static code analysis

Run the static code analysis with

```Linux
TODO: Provide command line for static code analysis
```

## Usage

TODO: Describe the quickest way to run this software separately

```Linux
TODO: ROS2 command line
```

## Config files

Config file folder/set 1

- **config_file_1.yaml** TODO: Shortly explain the content of this config file

## Launch files

- **state_estimator.launch.py**: TODO: shortly explain what is launched (e.g. standard simulation, simulation with gdb, ...)
    Argument set 1
        - `argument_1` Short description (e.g. as commented in launch file). Default: `default_value`.

## Nodes

### state_estimator_node

TODO: Provide description of this node

**Subscribed Topics**

- `/joint_state` ([sensor_msgs/JointState](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/JointState.html))
    The measured state of each joint in the robot which consists the joint position, joint velocity and joint effort.

**Published Topics**

- ...

**Services**

- ...

**Parameters**

- ...

### kalman_filter_node

...

### robot_description_node

...

## Troubleshooting

### GiNaC is empty, missing files, or has faulty files

If the folder that contains GiNaC is empty, is missing files, or faulty files, it is best to delete it, redownload, and reinstall it.

To delete and redownload it, follow the following instruction:

```
cd path/to/ros2_workspace/src/libraries
rm -rf ./ginac
git submodule update --init --recursive
```

To reinstall, follow the installation procedure describing above.


## Bugs & Feature Requests

Please report bugs and request features using the Issue Tracker (TODO: Provide link to issue page).

## Credit

Credits to [leggedrobotics](https://github.com/leggedrobotics/ros_best_practices/tree/main) for providing [a template to write the README of a custom ROS package](https://github.com/leggedrobotics/ros_best_practices/tree/main/ros_package_template).