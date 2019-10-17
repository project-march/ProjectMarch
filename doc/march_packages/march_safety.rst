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
* **TrajectorySafety**, checks whether the trajectory controller is within its trajectory constraints. Once the constraints
  have been passed a position hold command is sent, completed and then the trajectory controller is stopped.

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

*/march/error* (:march:`march_shared_resources/Error <march_shared_resources/msg/Error.msg>`)
  Publish error for other packages to make it able to respond to errors. This error message is also logged.

*/march/input_device/instruction* (:march:`march_shared_resources/GaitInstruction <march_shared_resources/msg/GaitInstruction.msg>`)
  Send instructions to the state machine. This topic is only used to send an stop instruction.

Tutorials
---------

.. note:: Always add tests when adding code to the *march_safety* package. Bugs in this package can have dangerous consequences for the system.


Add new safety rule to existing SafetyType
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
This is pretty straightforward to do. This tutorial will mostly give remarks and tips:

* Add your new rule to the most suited **SafetyType**, if no *SafetyTypes* are suited create a new one (see :ref:`march-safety-add-new-rule-label`).
* When using values that are probably going to change in the future place them as parameter in the **march_safety.launch** file.
  Definitely do this with values that are probably going to be changed by other team members, by placing the value in the
  launch file you make it much easier for them.
* It's possible that the new rule is automatically called, for example because it's a callback. However, when this is not the case
  call the new rule in the **update** method of the **SafetyType**. This method is executed every cycle of the **SafetyNode**.

.. _march-safety-add-new-rule-label:

Create new SafetyType
^^^^^^^^^^^^^^^^^^^^^
For this example we will create a safety type which checks the temperature.

* Set temperature threshold in the **march_safety.launch** file, this way the threshold is easy to adjust.
* Create a class **TemperatureSafety** which extends **SafetyType**
* In the constructor of this class you probably want to:

    * Pass on a reference to the *NodeHandle* and *SafetyHandle*.
    * Obtain the threshold parameter form the parameter service.

    .. code::

        n->getParam(ros::this_node::getName() + "/temperature_threshold_non_fatal", non_fatal_temperature_threshold);

    * Subscribe to the temperature topic.

    .. code::

        ros::Subscriber subscriber_temperature = n.subscribe<sensor_msgs::Temperature>("/march/temperature", 1000, temperatureCallback);

* Create a callback method for the temperature subscriber.

    * In this callback you want to compare the received value with the threshold

    .. code::

      if (msg->temperature > non_fatal_temperature_threshold)
      {
        // Temperature exceeds threshold
      }

    * When the threshold is exceeded you probably want to call the non-fatal method form the *SafetyHandle*. This is example code:

    .. code::

        safety_handler->publishNonFatal(error_message);

* You have to overwrite the **update** method from the **SafetyType**. However, in this example we are not using the update method.
  This method is used when you want to execute some code every **SafetyNode** cycle. For example if you want to check if a certain node
  is still alive this would be de perfect place to call this code. For this example we will overwrite this method, but keep it empty:

  .. code::

    void update() override
    {
    }

* Finally you need to add the **TemperatureSafety** to the **safety_list** in the **SafetyNode.cpp**:

.. code::

    safety_list.push_back(std::unique_ptr<SafetyType>(new TemperatureSafety(&n, &safetyHandler)));

