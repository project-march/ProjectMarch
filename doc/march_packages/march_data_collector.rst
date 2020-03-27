.. _march-data-collector-label:

march_data_collector
====================

Overview
--------
The `march_data_collector` package subscribes to the topics on which data is published to process the date. Basically three things are happening to the data:

* Based on the position also the acceleration and jerk are calculated for future analysis.

* The IMU is used to adapt the tf frames and based on this the center of mass (CoM) and the capture point (CP) are calculated.

* The data is send to an `Event Stream Processing` Engine.

Furthermore the  package also publishes incoming data from the pressure soles and sends data to the software coming with the pressure soles.

Calculating `CoM` and `CP`
--------------------------
The data from the IMU (:ref:`march-imu-manager-label`) is used to change the tf frames (see :ref:`robot-model-label`).
Now the tf frames are orientated with respect to the world correctly.
The tf frames together with the masses of the links from the :ref:`URDF <robot-model-label>` can be used to calculate the center of mass.
This is then published as a marker on ``/march/com_marker``.
The calculation is based on an implementation from `Hamburg Bit-Bots <https://github.com/bit-bots>`_.
The marker can also be visualized in :ref:`rviz <robot-model-label>`. The tf frames and CoM together can be used to calculate the capture point.
This is the point in a walking gait where one should step in order to return to a stable standing position.
This calculation is done for both feet and published as a marker on ``/march/cp_marker_ankle_plate_{left|right}``
The center of mass and capture point are used for (research on) balancing the exoskeleton.

Event Stream Processing
-----------------------
An `Event Stream Processing` engine from `SAS <https://www.sas.com/nl_nl/home.html>`_ is used.
There is a lot of documentation on this engine at
`ESP documentation <https://documentation.sas.com/?cdcId=espcdc&cdcVersion=6.2&docsetId=espov&docsetTarget=home.htm&locale=nl>`_.
The Engine allows for real-time data analysis and processing.
An engine consists of multiple windows. The data comes in at several `source windows`.
The data is send to other windows with different functions, like joining different event streams, aggregating an
event stream, transforming an event stream or using the data in the events for an algorithm. The windows should form an acyclic graph together.
A group of windows is called a :march:`model <march_data_collector/esp_models/march.xml>`.
Other applications can  subscribe to a windows to use the data in the window. This can be done for data visualisation, csv output or streaming bag into our software.
The :march:`esp_adapter.py <march_data_collector/src/march_data_collector/esp_adapter.py>` subscribes to ROS msgs and transforms incoming messages to csv strings. The csv strings are send to the ESP engine.


Pressure Soles
--------------
At March we currently use the `Moticon <https://www.moticon.de/>`_ pressure soles. Live reading of the data from the pressure soles is very cumbersome. You need the following things:

* The pressure soles with charged batteries.
* An Android phone with the Moticon app installed.
* A Windows laptop with the Moticon desktop software installed.
* A computer with the March software installed.

All devices should be connected to the same WiFi for optimal functioning.

1. First you open the app on the phone and you pair the insoles with the phone.

.. note::

    The pressure soles are turned on by moving/shaking them, because the internal IMU is triggered.
    Shake the pressure soles when they do not automatically turn on.

2. Next you should set the record mode to live capture. Open the Moticon desktop app and go to the record section and settings. Set the following things:

    * UDP Input:
        - `Channel name:`           Joint_positions
        - `Number rof Channels:`    8
        - `Datatype:`               float32
        - `Port:`                   9999

    * UDP Output
        - `IP addess:`              \<exoskeleton_ip\>
        - `Port:`                   8888
        - `Filter:`                 force pressure cop

More info on this step can be found `here <https://www.moticon.de/doc/science_desktop_software/record/udp/>`_.

3. Lastly use should set the correct launch arguments. You should set pressure_soles to true and if you are not using the
standard SWitch laptop connected to the project March router, you should set the IP-address on which the Moticon desktop app is running.

ROS API
-------

Nodes
^^^^^
*march_data_collector_node* - Responsible for processing IMU data and calculating the jerk.

*com_calculator* - Responsible for calculating the center of mass.

*cp_calculator* - Responsible for calculating the capture points.

*esp_adapter* - Responsible for sending data to the ESP engine.

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
  Send position, velocity, acceleration and jerk of joints `ESP`.

*/march/joint_states* (sensor_msgs/JointState)
  Send actual effort to `ESP`.

*/march/controller/after_limit_joint_command'* (march_shared_resources/AfterLimitJointCommand)
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
  A marker with the capture point for the right foot.

Publishes tf frames via a `tf2 <http://wiki.ros.org/tf2>`_ broadcaster.

Parameters
^^^^^^^^^^
*/march/march_data_collector/moticon_ip* (*string*, default: 192.168.8.105)
  IP-address the Moticon desktop software is running
*/march/march_data_collector/pressure_soles* (*bool*, default: false)
  Whether to connect with the pressure soles.


Tutorials
---------

Adding a publisher into `ESP`
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
1. Add a source window to the :march:`model <march_data_collector/esp_models/march.xml>`.
2. Create a callback function that takes a ROS msgs and calls the  ``send_to_esp`` function with the msgs as csv string in :march:`esp_adapter.py <march_data_collector/srcmarch_data_collector/esp_adapter.py>`
3. In the same file add a call to ``configure_source`` to the ``__init__``.

Launching with `ESP`
^^^^^^^^^^^^^^^^^^^^

.. note::

    The ESP engine should be installed on the machine. You need a license for this.
    An engine is installed on the exoskeleton.

1. Launch an `ESP` server. On the exoskeleton the following terminal command is configured to start an `ESP` with the correct settings.

    .. code::

        esp_start

2. Do a normal launch (simulation, headless, normal) and set the launch argument ``esp`` to true. For instance:

    .. code::

        roslaunch march_launch march.launch esp:=true
