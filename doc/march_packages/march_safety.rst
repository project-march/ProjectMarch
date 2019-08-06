.. _march-safety-label:

march_safety
============

Overview
--------
March safety is responsible for monitoring the system during run-time. When unexpected behavior is perceived, this
will be logged. Depending on the severity the system can also be stopped or terminated.


Implementation
^^^^^^^^^^^^^^
March safety consists of a list of **SafetyType**-objects. Such an object monitors a part of the system. If a certain condition is
violated, the object will call the **SafetyHandler** to invoke an error. The **SafetyHandler** is responsible for sending error messages and sounds.
All **SafetyType**-objects:

* **TemperatureSafety**, monitors the joint temperatures. Depending on which threshold is exceeded an warning, non-fatal or fatal is thrown.
* **InputDeviceSafety**, checks if the input device is still connected. Otherwise a non-fatal is thrown.
* **TrajectorySafety**, checks whether the trajectory controller is within its trajectory constraints. Once the constraints have been passed a position hold command is sent, completed and then the trajectory controller is stopped.

Error severity
--------------

Warning
^^^^^^^
This event is a deviation from the normal behavior. However, no extra steps needs to be taken to deal with the consequences.
The system will continue running. The event will only be logged.

Non-Fatal
^^^^^^^^^
This event is a deviation from the normal behavior. Which has consequences the system should deal with.
An stop message is send indistinguishable from a stop message from an input device.

Fatal
^^^^^
This event is a unrecoverable deviation from the normal behavior. There is no (safe) way to make the system behave properly again.
Therefore, the system should terminate.

ROS API
-------

Nodes
^^^^^

*march_safety* - This node reads parameters from the parameter server and runs all **SafetyType**-objects.

Published Topics
^^^^^^^^^^^^^^^^

*/march/error* (:march-iv:`march_shared_resources/Error <march_shared_resources/msg/Error.msg>`)
  Publish error for other packages to make it able to respond to errors. This error message is also logged.

*/march/input_device/instruction* (:march-iv:`march_shared_resources/GaitInstruction <march_shared_resources/msg/GaitInstruction.msg>`)
  Send instructions to the state machine. This topic is only used to send an stop instruction.

Tutorials
---------

Add new safety rule to existing SafetyType
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


Create new SafetyType
^^^^^^^^^^^^^^^^^^^^^

