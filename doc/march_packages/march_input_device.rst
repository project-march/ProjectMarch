march_input_device
==================

Overview
--------
The march_input_device is the software running on the |m4| exoskeleton input device. The input device is used
to give input to the software running on the |m4| exoskeleton.


Communication
-------------
The communication between the input device and the |m4| exoskeleton is done with `rosserial <http://wiki.ros.org/rosserial>`_.
This can be run wired via usb cable or wireless via wifi connection.

Sending messages
^^^^^^^^^^^^^^^^
:march-iv_repo:`Gait Instruction Message <march_shared_resources/msg/GaitInstruction.msg>` This is a instruction to start the sent gait right now, if this is possible.

:march-iv_repo:`Stop Message <march_shared_resources/msg/GaitInstruction.msg>` This message can stop a repeating gait (such as walk). After receiving the stop message the exoskeleton will go to the standing pose.

.. todo:: (Tim) link this to the march_safety documentation.

`Stay Alive Message <http://docs.ros.org/kinetic/api/std_msgs/html/msg/Time.html>`_  Every loop a stay alive message is send. This way *march_safety* can detect when the input device loses connection.

Receiving messages
^^^^^^^^^^^^^^^^^^
`Gait Instruction Response Message <http://docs.ros.org/kinetic/api/std_msgs/html/msg/Bool.html>`_  This message indicate that the last gait instruction is handled. The value represent successful handled or rejected.

Tutorials
---------


How to run wired
^^^^^^^^^^^^^^^^

.. todo:: (Electro) Document how to change the input device to wired mode

Upload the code on the input device
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Skip this section if the correct code for running wired is already on the input device. More detailed information on how
to run and upload the code see the :input-device:`ReadMe <>`.

- Comment :code:`#define USE_WIRELESS` in the main.cpp.
- Upload to input device.

Start the software
~~~~~~~~~~~~~~~~~~
- Connect the input device via usb cable, this powers and launches the input device automatically.
- Type in a terminal:

.. code::

    roslaunch march_launch serial_connection.launch tcp:=false


How to run Wireless
^^^^^^^^^^^^^^^^^^^

.. todo:: (Electro) Document how to change the input device to wireless mode

.. note:: Make sure the pc that needs to receive the messages and the input device are on the same network.

Upload the code on the input device
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Skip this section if the correct code for running wired is already on the input device. More detailed information on how
to run and upload the code see the :input-device:`ReadMe <>`.

.. note:: If you want to change the network name and password that the input device uses: Change **ssid** and **password** in the *WirelessConnection.h*

- Type in terminal: :code:`hostname -I` Remember the output, this is the hostname. You have to use this address for the next instructions. From this point we call this value **your_hostname**.
- Uncomment :code:`#define USE_WIRELESS` in the *main.cpp*.
- Change :code:`IPAddress server(x, x, x, x)` to :code:`IPAddress server(your_hostname)` in the *WirelessConnection.h*
- Upload to input device.


Start the software
~~~~~~~~~~~~~~~~~~
- Type in every terminal you are going to use:

.. code::

    export ROS_MASTER_URI=http://<your_hostname>:11311/

- Type in a terminal:

.. code::

    roslaunch march_launch serial_connection.launch

- Make sure the batteries are charged and in the input device.
- Press on button, which is located on the electronics holder.

How to add a gait
^^^^^^^^^^^^^^^^^
.. todo:: (Karlijn) Document how to add new screens

- Add new screens.
- Add a new entry to the *stateToGaitMapping*. By adding a extra line in the constructor of the *StateMachine.cpp*:

.. code::

    stateToGaitMapping[State::<name_activated_state>] = "<gait_name>";

**<name_activated_state>** name of the activated state

**<gait_name>** name of the gait
