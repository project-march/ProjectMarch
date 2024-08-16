
.. _march_hardware_interface-label:

march_hardware_interface
========================

Overview
--------

Broadcasters
------------

march_motor_controller_state_broadcaster
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Broadcasts the state of the motors.


**Published Topics**

- :code:`/march/motor_controller_states` (`march_shared_msgs/msg/MotorControllerStates <link>`_)
  
Broadcasts the state of the motors controllers to ROS.

march_pdb_state_broadcaster
^^^^^^^^^^^^^^^^^^^^^^^^^^^
Broadcasts the state of the power distribution board.

**Published Topics**

- :code:`/march/pdb_data` (`march_shared_msgs/msg/PowerDistributionBoardData <link>`_)`

Interfaces
-----------

march_hardware
^^^^^^^^^^^^^^
The march_hardware package contains the hardware information for the exoskeleton. It includes objects and classes which are used to interact with the hardware of the exoskeleton.

march_hardware_builder
^^^^^^^^^^^^^^^^^^^^^^
This package contains a library that facilitates the creation of a MarchRobot from YAML configuration 
files. The YAML files specify certain properties of the hardware used in the robot. The core logic of this 
implementation is encapsulated in the HardwareBuilder class, which can be found in 
`march_hardware_builder/src/hardware_builder.cpp.`. This package is largely independent from ROS logic,
only using it for logging purposes.

march_system_interface
^^^^^^^^^^^^^^^^^^^^^^
The exo system interface ties the hardware components to the controllers. It runs in several phases, starting with loading all the state and command interfaces. After these are configured, it is able to start, and continuously both read and write to and from the hardware (depending on if it is using a state/command interface).

Furthermore, it couples the command and state interfaces to the JointInfo struct, which contains all the information deemed relevant - think of position, velocities, limits, etc. - for each respective joint.

Once the export_state_interfaces()function is called, the high level can start communicating with low level hardware.

Tutorials
---------
