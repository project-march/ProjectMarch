.. _march-hardware-label:

march_hardware
==============

Overview
--------

The march_hardware package is a C++ library to interact with the physical
|march|. Its goal is to provide an api for actuating the exoskeleton without
knowledge of the underlying hardware.

Class Structure
---------------
.. tip::
  The :ref:`march-hardware-builder-label` can help with the class structure of
  the hardware package, especially when looking at an
  :hardware-interface:`example yaml <march_hardware_builder/robots/march4.yaml>`.

The highest level class is the MarchRobot, it contains a list of Joints, which
can be accessed. Each Joint is identified by a name and optionally has an
IMotionCube and TemperatureGES. This is enough knowledge to start interacting
with the robot.

Example usage
-------------
Here is an example of the commands you can use on a MarchRobot object when it is
created with the correct joints.

.. code-block:: cpp

  MarchRobot robot;  // This step should use the hardware builder, but uses this to simplify the example
  robot.startEthercat();
  Joint& right_knee = robot.getJoint("right_knee");
  right_knee.prepareActuation();
  right_knee.actuateRad(0.2);
  double angle = right_knee.getAngleRadAbsolute();
  float temperature = right_knee.getTemperature();
  robot.stopEthercat();

EtherCAT
--------
EtherCAT is the fieldbus protocol used to communicate with the local hardware
('slaves') in the exoskeleton. `SOEM <https://github.com/OpenEtherCATsociety/SOEM>`_
(short for Simple Open EtherCAT Master) is a C library which provides most
EtherCAT functionalities. As said, the hardware package is mainly a class
structure on top of SOEM functioning as an api for controlling the exoskeleton.
The interfaces can be roughly divided into three parts:

* EthercatMaster
* PDO messages
* SDO messages

EthercatMaster
**************
The EthercatMaster class contains the main EtherCAT functionality and the main
interface with SOEM. EtherCAT can be started and stopped via the EthercatMaster.
When EtherCAT is started, a parallel thread is started that continuously sends
and receives the latest EtherCAT PDO data and monitors if all slaves are still
present and operational. When EtherCAT is stopped, this parallel thread stops
running.

The EthercatMaster class expects several arguments in its constructor: the
network interface name (ifname) of your device, the number of EtherCAT slaves in
your robot, and the cycle time of the EtherCAT thread in milliseconds. An
example is provided below.
    
.. code-block:: cpp

  EthercatMaster ethercat("enp2s0", 14, 2); // 14 slaves, 2 ms cycle time

.. tip::
  Find out the ifname of your device by running :code:`ifconfig` in a terminal.

PDO messages
************
Process Data Objects (PDOs) are the cyclic and continuous type of messages of
EtherCAT, and the main form of exchanging data with the slaves. Reading and
writing EtherCAT PDO messages is possible through the other classes in the
march_hardware package (e.g. Joint, IMotionCube, Encoder, PowerDistributionBoard).
For example, the `Encoder class <march_hardware/include/march_hardware/encoder/encoder.h>`
``getAngleIU()`` function reads and returns the latest encoder value that was
received by the EthercatMaster. Functions such as ``getAngleIU()`` all call the
same generic interface for reading from and writing to slaves. The generic
interface is defined in `pdo_interface.cpp <march_hardware/include/march_hardware/ethercat/pdo_interface.h>`.

.. note::
  When reading or writing PDO messages over EtherCAT via SOEM, you need three
  things:

  * The index of the slave you want to read from or write to.
  * The amount of bytes you want to read or write.
  * The byte offset of the data you want to read or write for a particular slave.

  The byte offsets need to correspond with how the slave is configured. For GES
  and PDB slaves, this is stored in the EEPROM of the slave. For the IMotionCube,
  this is configurable during startup through a process called PDO mapping.

SDO messages
************
Service Data Objects (SDOs) are non-cyclic EtherCAT messages. They are used for
sending one-time messages, for example when initializing an IMotionCube.
The generic SDO interface can be found in `sdo_interface.h <march_hardware/include/march_hardware/ethercat/sdo_interface.h>`.

Power Distribution Board
------------------------
The PowerDistributionBoard class contains all functionality for communicating
with the Power Distribution Board over EtherCAT. For example, high voltage nets
can be turned on and off via this class, and the currents that the Power
Distribution Board measures can be read. The PowerDistributionBoard class
contains a HighVoltage and a LowVoltage class which contain methods for
controlling the high voltage and low voltage nets.

.. code-block:: cpp

  PowerDistributionBoard pdb;
  float current = pdb.getPowerDistributionBoardCurrent();
  pdb.getHighVoltage().setNetOnOff(true, 2); // Turn on net 2

.. note::
  The PowerDistributionBoard constructor requires EtherCAT byte offsets as
  arguments. These need to be specified in the robot description yaml files of
  the hardware_builder package.

.. note::
  The functionality of the PowerDistributionBoard is highly dependent on the
  software running on the LPC1768 of the Power Distribution Board. See the
  `ethercat-slaves repository <src/pdb>` for the LPC1768 code.

Exceptions
----------
Because safety is very important, the march_hardware package will throw an
exception whenever it encounters something that should not happen. The
march_hardware package implements custom exceptions in the
`error module <march_hardware/include/march_hardware/error>`
with error types for different situations. See :ref:`error-codes` for all types
of errors and how to possibly fix them. When such an exception occurs, the high
voltage is turned off and the exoskeleton will stop moving.

ROS API
-------
The hardware package is written without depending on ROS to ensure that it can
remain functional even when ROS will no longer be used. The package does depend
on ROS for logging, but that can be easily changed if needed.
