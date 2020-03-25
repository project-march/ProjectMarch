.. _march-hardware-builder-label:

march_hardware_builder
======================

Overview
--------
This package contains a library which can be included to create a ``MarchRobot`` from YAML config files
and a robot description in URDF. Whereas the YAML describes some properties of the hardware used in the robot,
the hardware builder uses the URDF for joint limits. The joints in the YAML and the URDF must match.
The implementation is rather straightforward and nearly all of the logic can be found in the
:hardware-interface:`HardwareBuilder class <march_hardware_builder/src/hardware_builder.cpp>`.

ROS API
-------
The hardware builder is written without depending on ROS to ensure that it can remain functional even when ROS will no longer be used.
The package does depend on ROS for logging, but that can be easily changed if needed.


Tutorials
---------

Using the package
^^^^^^^^^^^^^^^^^

Add a dependency on this package to your project and ``#include <march_hardware_builder/hardware_builder.h>`` to start using the :hardware-interface:`HardwareBuilder class <march_hardware_builder/include/march_hardware_builder/hardware_builder.h>`.

There are four main ways to interact with the hardware builder:

* Instantiate with an AllowedRobot:

  .. code::

    HardwareBuilder hardwareBuilder = HardwareBuilder(AllowedRobot::march4);
    marchRobot = hardwareBuilder.createMarchRobot();

  A list of currently allowed robots can be seen in :hardware-interface:`include/AllowedRobot.h <march_hardware_builder/include/march_hardware_builder/allowed_robot.h>`

* Instantiate with a path to a yaml file:

  .. code::

    HardwareBuilder hardwareBuilder = HardwareBuilder("path/to/robot/march4.yaml");
    marchRobot = hardwareBuilder.createMarchRobot();

  An example yaml file can be seen in :hardware-interface:`robots/march4.yaml <march_hardware_builder/robots/march4.yaml>`

* Instantiate empty. This function is mostly used for testing wrongly configured robots and is not needed in a release.

  .. note:: This means you are responsible for passing a YAML::Node to the .createMarchRobot() function!

  .. code::

    YAML::Node robotConfig = YAML::LoadFile("path/to/robot.yaml");
    HardwareBuilder hardwareBuilder = HardwareBuilder();
    marchRobot = hardwareBuilder.createMarchRobot(robotConfig);

* It is also possible to create parts of a MarchRobot (IMotionCubes, Encoders, etc...) in a similar way.

  .. code::

    YAML::Node encoderConfig = YAML::LoadFile("path/to/encoder.yaml");
    HardwareBuilder hardwareBuilder = HardwareBuilder();
    marchRobot = hardwareBuilder.createEncoder(encoderConfig);
