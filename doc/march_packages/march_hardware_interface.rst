.. _march-hardware-interface-label:

march_hardware_interface
========================
The march_hardware_interface package uses the march_hardware (see
:ref:`march-hardware-label`) library to create an interface between ros_control
and the exoskeleton hardware. It has one runnable node which functions as the
control loop. The loop consists of the following steps:

#. Reading data from the robot
#. Updating the connected controllers using the read data
#. Writing the output of the controllers to the robot

The class that is responsible for all this is the ``MarchHardwareInterface``.
It implements the ``hardware_interface::RobotHW`` class from `ros_control`_.
It implements three important functions from that base class: ``init``, ``read``
and ``write``. Furthermore, the class owns an instance of the ``MarchRobot``
class, which it uses for reading from and writing to the robot. The reading
and writing must happen in real time to ensure it runs deterministically and,
therefore, run at high frequencies. To guarantee this, real time publishers are
used from `realtime_tools`_ and messages from ROS are published and received in
a ``ros::AsyncSpinner``. Furthermore, all dynamically allocated resources, such
as ``std::vector``, reserve their memory in the ``init`` function to guarantee
no allocations take place during real time.

.. _ros_control: https://wiki.ros.org/ros_control
.. _realtime_tools: https://wiki.ros.org/realtime_tools

Controllers
^^^^^^^^^^^
Currently, the hardware interface can use two types of controllers: position
and effort controllers. Effort control is currently the default. The
march_hardware_interface package is also responsible for launching the
controllers and providing their configurations. These configurations are stored
in the :hardware-interface:`config <march_hardware_interface/config>` directory.
These contain settings for the PID controllers and the joint trajectory action
server. See the `Controllers`_ section on the ros_control wiki for more info on
what type of controllers exists and how to configure them.

.. _controllers: https://wiki.ros.org/ros_control#Controllers

Safety Limits
^^^^^^^^^^^^^
Besides controlling the motors, the hardware interface also enforces safety
limits. For this it uses the ``joint_limits_interface`` from ros_control. The
robot has so called *soft limits*, which provide a buffer for the actual
mechanical limits of the joints. The soft limit values are loaded from the URDF.
The job of the limits interface is to limit the output of the controllers as to
not exceed the joint limits defined in the URDF. Enforcing the limits occurs at
the start of the ``write`` function, before writing the actuate commands to the
robot.

.. seealso::
  See the `Joint Limits`_ section on the ros_control wiki for more info on
  the joint limits. Also see `Joint Safety Limits Explained`_ for more info on
  how the limits work.

.. _joint limits: https://wiki.ros.org/ros_control#Joint_Limits
.. _Joint Safety Limits Explained: https://wiki.ros.org/pr2_controller_manager/safety_limits

Custom Controller Interfaces
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The march_hardware_interface package also includes two custom controller
interfaces, specifically designed for the March exoskeleton. These include the
``MarchPdbStateInterface`` and the ``MarchTemperatureSensorInterface``. the
``MarchPdbStateInterface`` is an interface for the ``MarchPdbStateController``
, which is responsible for publishing the state of the power distribution board.
The ``MarchTemperatureSensorInterface`` is an interface for the
``MarchTemperatureSensorController``, which is responsible for publishing the
state of the temperature sensors in the joints.


ROS API
-------
Since the hardware interface uses the controller_manager from ros_control, it
has many topics that enable the user to send trajectory commands and read the
state of loaded joints.

Nodes
^^^^^
march_hardware_interface_node
  Builds an instance of ``MarchRobot`` from a given robot name and starts the
  control loop. The node must be run with an argument of which robot to build
  and use. For example, to run it without roslaunch/rosrun (This will only work
  if you have a ``roscore`` running):

  .. code::

    ./march_hardware_interface_node march4

  The :hardware-interface:`hardware.launch <march_hardware_interface/launch/hardware.launch>`
  file has a launch argument, which it passes to the node.

Published Topics
^^^^^^^^^^^^^^^^
*/march/imc_states* (:march:`march_shared_resources/ImcState <march_shared_resources/msg/ImcState.msg>`)
  Publishes statuses of all iMotionCubes every control loop.

*/march/controller/after_limit_joint_command* (:march:`march_shared_resources/AfterLimitJointCommand <march_shared_resources/msg/AfterLimitJointCommand.msg>`)
  Publishes joint commands after they have been limited by the joint safety
  limit controller.
