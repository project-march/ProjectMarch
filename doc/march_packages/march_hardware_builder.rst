march_hardware_builder
======================

Overview
--------
This package contains a library which can be included to to create a MarchRobot from .yaml config files.
The implementation is rather straightforward and nearly all of the logic can be found in the :hardware-interface:`HardwareBuilder class <march_hardware_builder/src/HardwareBuilder.cpp>`.

ROS API
-------
The hardware builder is written without depending on ROS to ensure that it can remain functional even when ROS will no longer be used.
The package does depend on ROS for logging, but that can be easily changed if needed.


Tutorials
---------

Using the package
^^^^^^^^^^^^^^^^^

Add a dependency on this package to your project and ``#include <march_hardware_builder/HardwareBuilder.h>`` to start using the :hardware-interface:`HardwareBuilder class <march_hardware_builder/include/march_hardware_builder/HardwareBuilder.h>`.

There are four main ways to interact with the hardware builder:

* Instantiate with an AllowedRobot:

  .. code::

    HardwareBuilder hardwareBuilder = HardwareBuilder(AllowedRobot::march4);
    marchRobot = hardwareBuilder.createMarchRobot();

  A list of currently allowed robots can be seen in :hardware-interface:`include/AllowedRobot.h <march_hardware_builder/include/march_hardware_builder/AllowedRobot.h>`

* Instantiate with a path to a yaml file:

  .. code::

    HardwareBuilder hardwareBuilder = HardwareBuilder("path/to/robot/march4.yaml");
    marchRobot = hardwareBuilder.createMarchRobot();

  An example yaml file can be seen in :hardware-interface:`src/robots/march4.yaml <march_hardware_builder/src/robots/march4.yaml>`

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
