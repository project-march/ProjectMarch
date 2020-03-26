.. _march-imu-manager-label:

march_imu_manager
=================

Overview
--------
The ``march_imu_manager`` interfaces between ROS and the `XSense MTw Awinda IMU's <https://www.xsens.com/products/mtw-awinda>`_.
The IMU's connect wireless to an USB dongle, which should be inserted in the computer the node is running on.
The package contains and uses the Xsens library belonging to the IMU to read the data from the dongle. The data is then published in a
`sensors_msgs/imu message <http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Imu.html>`_ on the ``/march/imu`` rostopic.

Launch
^^^^^^
The node is not automatically launched when launching from `march_launch` at the moment. It has no launch file, but can be launched with:

.. code::

  rosrun march_imu_manager march_imu_manager_node


Interaction
^^^^^^^^^^^
The IMU position is defined as a link in the
`URDF <https://wiki.ros.org/urdf>`_, see :ref:`march-description-label`. Linking the IMU to an existing frame can be done by setting ``imu/header/fame_id``.
Linking an IMU to a frame allows for nice integrations with ROS. For instance, the data coming from the IMU is used in the march_data_collector package to update the tf2 (:ref:`robot-model-label`) data.
Thereby updating the orientation of the joints with respect to world, which is used for other calculations.

Placement
^^^^^^^^^
Currently there is only support for one IMU position, namely in the backpack. This is also defined in the URDF with the ``imu_link``.
The IMU should be placed in a special spot next to the battery. The IMU should be orientated with the LED towards the pilot.

Debugging
---------

In this section IMU specific problems are mentioned, it is assumed everything with ROS works perfectly.

* No wireless master found:
    The package is unable to locate the USB dongle.

    - Is the USB dongle inserted in the computer on which the node runs?

    - Are you in a group (often `dialout` or `uucp`) that can read the USB port?

        This can be checked by running the following.

        .. code::

            $ ls -l /dev/ttyUSB0
            $ groups

        You can set yourself to the right group by running the following. Replace 'dialout' with your own USB group.

        .. code::

            $ sudo usermod -G dialout -a $USER
            $ newgrp dialout

* The package found a wireless master, but does not connect with an IMU:
    The IMU is off or out of reach. Turn the IMU on by pressing the button until the LED blinks.