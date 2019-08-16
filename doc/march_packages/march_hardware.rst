.. _march-hardware-label:

march_hardware
==============

Overview
--------

The march_hardware package a C++ package to interact with the physical |m4| exoskeleton.
Its goal is to provide an api for actuating the exoskeleton without understanding of the underlying hardware.
We use `SOEM <https://github.com/OpenEtherCATsociety/SOEM>`_ to communicate with the hardware and send commands.

Class Structure
---------------
.. tip::
  The :ref:`march-hardware-builder-label` can help with the class structure of the hardware package,
  especially when looking at an :hardware-interface:`example yaml <march_hardware_builder/src/robots/march4.yaml>`.

The highest level class is the MarchRobot, it contains a list of Joints which can be accessed.
Each Joint is identified by a name and optionally has an IMotionCube and TemperatureGES.
This is enough knowledge to start using

Example usage
-------------
Assuming the MarchRobot is already created with the correct joints, here are some commands you
.. code::

  MarchRobot robot;
  robot.startEthercat();
  Joint rightKnee = robot.getJoint("right_knee");
  rightKnee.prepareActuation();
  rightKnee.actuateRad(0.2);
  rightKnee.getRad();
  rightKnee.getTemperature();
  robot.stopEthercat();


ROS API
-------
The hardware package is written without depending on ROS to ensure that it can remain functional even when ROS will no longer be used.
The package does depend on ROS for logging, but that can be easily changed if needed.