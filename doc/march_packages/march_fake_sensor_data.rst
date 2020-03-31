.. _march-fake-sensor-data:

march_fake_sensor_data
======================
The march_fake_sensor_data publishes fake sensor data to simulate real sensors when running a simulation of the exoskeleton.

See `this tutorial <http://wiki.ros.org/dynamic_reconfigure/Tutorials/HowToWriteYourFirstCfgFile>`_
to implement dynamically reconfigurable parameters.

Implemented sensors
-------------------

Temperature sensors
^^^^^^^^^^^^^^^^^^^
A dynamically reconfigurable temperature publisher is implemented in fake_temperature_data.cpp.
The fake temperature sensors are generated automatically from the joint names from the ``/march/joint_names`` parameter
on the parameter server. The min and max for random temperatures can be configured in
:simulation:`march_fake_sensor_data.launch <march_fake_sensor_data/launch/march_fake_sensor_data.launch>`.
The min and max will apply to all fake temperature sensors.

How to use
""""""""""
Run the fake temperature publishers

.. code::

  roslaunch march_fake_sensor_data march_fake_sensor_data.launch

You can change the parameters live using:

.. code::

  rosrun rqt_reconfigure rqt_reconfigure

Furthermore, the temperature data can be visualized using ``rqt_plot`` and
listening to the ``temperature`` or ``variance`` value on the ``/march/temperature/<joint_name>`` topics.
