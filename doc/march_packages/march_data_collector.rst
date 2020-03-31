.. _march-data-collector-label:

march_data_collector
====================

Overview
--------
The `march_data_collector` package subscribes to the topics on which data is published to process the data. Basically three things are happening to the data:

* Based on the position, the acceleration and jerk (through second and third order backward Euler) are calculated for future analysis.

* The IMU is used to adapt the tf frames and based on this the center of mass (CoM) and the capture point (CP) are calculated.

* The data is send to an `Event Stream Processing` Engine.

Furthermore the  package also publishes incoming data from the pressure soles and sends data to the software that came with the pressure soles.

Calculating `CoM` and `CP`
--------------------------
The data from the IMU (:ref:`march-imu-manager-label`) is used to change the tf frames (see :ref:`robot-model-label`).
such that the tf frames are orientated with respect to the world correctly.
The tf frames together with the masses of the links from the :ref:`URDF <robot-model-label>` can be used to calculate the center of mass.
This is then published as a marker on ``/march/com_marker``.
The calculation is based on an implementation from `Hamburg Bit-Bots <https://github.com/bit-bots>`_.
The marker can also be visualized in :ref:`rviz <robot-model-label>`. The tf frames and CoM can be combined to calculate the capture point.
This is the point where one should place their foot in order to return to a stable standing position.
This calculation is done for both feet and published as a marker on ``/march/cp_marker_ankle_plate_{left|right}``
The center of mass and capture point are used for (research on) balancing the exoskeleton.

Event Stream Processing
-----------------------
At March we use the `Event Stream Processing` engine from `SAS <https://www.sas.com/nl_nl/home.html>`_.
Here we will just describe the integration in the `march_data_collector`, but more information on `ESP` and how we use it can be found at :ref:`event stream processing <event-stream-processing-label>`.
The engine allows for real-time data analysis and processing. With the installation also comes a module to inject events, which is used in this package.
The :march:`esp_adapter.py <march_data_collector/src/march_data_collector/esp_adapter.py>` functions as the interface between our software and ESP. This ROS node subscribes to
ROS topics containing interesting data. It converts the ROS messages into csv strings and injects the data into the correct window of the ESP engine.
Injecting data can only happen into a source window.


Adding a publisher into `ESP`
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
1.
    Add a source window to the :march:`model <march_data_collector/esp_models/march.xml>`.

2.
    Create a callback function that takes a ROS message and calls the ``send_to_esp`` function with the msgs as csv
    string in :march:`esp_adapter.py <march_data_collector/src/march_data_collector/esp_adapter.py>`.

3.
    In the same file add a call to ``configure_source`` to the ``__init__``.

Pressure Soles
--------------
The `march_data_collector` interfaces between the Moticon pressure soles and the exoskeleton software.
It  sends data to and receives data from the desktop software that came with the pressure soles.
This data is send over `UDP <https://nl.wikipedia.org/wiki/User_Datagram_Protocol>`_.
More information on interacting with the Moticon software can be found `here <https://www.moticon.de/doc/science_desktop_software/record/udp/>`_.
The package assumes that the filter used in the Moticon software is ``force pressure cop``.
More information on using the pressure soles can be found at :ref:`using the pressure soles <using-the-pressure-soles-label>`.

ROS API
-------

Nodes
^^^^^
*march_data_collector_node* - Responsible for processing data.

Subscribed Topics
^^^^^^^^^^^^^^^^^
*/march/controller/trajectory/follow_joint_trajectory/feedback* (control_msgs/JointTrajectoryControllerState)
  Compute the acceleration and jerk.

*/march/imu* (sensor_msgs/Imu)
  Use IMU data to change the orientation of the tf frames and send to `ESP`.

*/march/pressure_soles* (march_shared_resources/PressureSole)
  Send pressure sole data to `ESP`.

*/march/imc_states* (march_shared_resources/ImcState)
  Send iMOTIONCUBE data to `ESP`.

*/march/gait/schedule/goal* (march_shared_resources/GaitActionGoal)
  Send gait data to `ESP`.

*/march/com_marker* (visualization_msgs/Marker)
  Send center of mass to `ESP`.

*/march/joint_values* (march_shared_resources/JointValues)
  Send position, velocity, acceleration and jerk of joints to `ESP`.

*/march/joint_states* (sensor_msgs/JointState)
  Send actual effort to `ESP`.

*/march/controller/after_limit_joint_command* (march_shared_resources/AfterLimitJointCommand)
  Send effort command to `ESP`.

*/march/temperature/\** (sensor_msgs/Temperature)
  Send temperature command to `ESP` for each joint.

Subscribes to the tf frames trough a `tf2 <http://wiki.ros.org/tf2>`_ buffer.

Published Topics
^^^^^^^^^^^^^^^^
*/march/com_marker* (visualization_msgs/Marker)
  A marker with the CoM point.

*/march/cp_marker_ankle_plate_left* (visualization_msgs/Marker)
  A marker with the capture point for the left foot.

*/march/cp_marker_ankle_plate_right* (visualization_msgs/Marker)
  A marker with the capture point for the right foot.

*/march/pressure_soles* (march_shared_resources/PressureSole)
  A marker with the capture point for the right foot.

*/march/pressure_soles* (march_shared_resources/JointValues)
  Send the data from the pressure soles.

Publishes into tf via a `tf2 <http://wiki.ros.org/tf2>`_ broadcaster.

Parameters
^^^^^^^^^^
*/march/march_data_collector/moticon_ip* (*string*, default: 192.168.8.105)
  IP-address the Moticon desktop software is running
*/march/march_data_collector/pressure_soles* (*bool*, default: false)
  Whether to connect with the pressure soles.

