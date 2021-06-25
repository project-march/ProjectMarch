.. _march-input-device-label:

march_input_device
==================

Overview
--------
The march_input_device is the software running on the |march| input device. The input device is used
to give input to the software running on the |march|. See the
:input-device:`project-march/input-device <>` repository for more info.


ROS API
-------------
The communication between the input device and the |march| is done with `rosserial <http://wiki.ros.org/rosserial>`_.
This can be run wired via usb cable or wireless via wifi connection.

Published Topics
^^^^^^^^^^^^^^^^
*/march/input_device/instruction* (:march:`march_shared_resources/GaitInstruction <march_shared_resources/msg/GaitInstruction.msg>`)
  Sends instructions to execute

*/march/input_device/alive* (`std_msgs/Time <http://docs.ros.org/melodic/api/std_msgs/html/msg/Time.html>`_)
  Publish empty alive messages so :ref:`march-safety-label` does not throw an error.

Subscribed Topics
^^^^^^^^^^^^^^^^^
*/march/input_device/instruction_response* (:march:`march_shared_resources/GaitInstructionResponse <march_shared_resources/msg/GaitInstructionResponse.msg>`)
  Receives responses to instructions executed on */march/input_device/instruction*


Tutorials
---------

Set correct permissions on Linux
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The following error can occur when trying to access the ttyUSB port:

.. code::

  Cannot open /dev/ttyUSB0: Permission denied

To make sure your user is allowed to access the port, add your user to the required groups:

.. code::

  sudo usermod -a -G tty $USER
  sudo usermod -a -G dialout $USER

Now log out on your computer and log back in, the error should be resolved.

`Source <https://github.com/esp8266/source-code-examples/issues/26>`_

How to run wired
^^^^^^^^^^^^^^^^

Upload the code on the input device
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Skip this section if the correct code for running wired is already on the input device.
For more detailed information on how to run and upload the code see the :input-device:`README <>`.

.. code::

  pio run -t upload

Start the software
~~~~~~~~~~~~~~~~~~
- Make sure the jumper cap in the electronics holder is placed so the input device is powered via the USB cable rather than via batteries.
- Connect the input device via usb cable, this powers and launches the input device automatically.
- Type in a terminal:

.. code::

  roslaunch march_launch serial_connection.launch


How to run wireless
^^^^^^^^^^^^^^^^^^^

.. note:: Make sure the pc that needs to receive the messages and the input device are on the same network.

Upload the code on the input device
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Skip this section if the correct code for running wireless is already on the input device.
For more detailed information on how to run and upload the code see the :input-device:`README <>`.

- Uncomment ``#define USE_WIRELESS`` in the *main.cpp*.
- Type in terminal: ``hostname -I``, this is your IP address.
- Open :input-device:`wireless_connection.h <include/wireless_connection.h>`

  - at ``IPAddress server(x, x, x, x)`` fill the xs with the four numbers
    of your IP address from the previous step.
  - Change the value of ``ssid`` to the name of the WiFi network.
  - Change the value of ``password`` to the password of the WiFi network.

- Compile and upload to the input device.


Start the software
~~~~~~~~~~~~~~~~~~
- Type in every terminal you are going to use (replace ``<your_ip_address>`` with the output from ``hostname -I``):

  .. code-block:: bash

    export ROS_MASTER_URI=http://<your_ip_address>:11311/

- Type in a terminal:

  .. code::

    roslaunch march_launch serial_connection.launch wireless:=true

- Make sure the batteries are charged and in the input device.
- Make sure the jumper cap in the electronics holder is placed so the input device is powered by the batteries rather than via a USB cable.
- Press the on/off button to turn on the input device. The button is located on the electronics holder of the input device.


.. _how-to-add-a-gait-label:

How to add a gait
^^^^^^^^^^^^^^^^^

- Make new screens for the new gait, for normal, selected and activated.
- Make sure the new gait/new screens fit in the menu & create a selected & activated screen for the new gait.
- Put the screens on the SD card. Use the 4D Systems Workshop4 IDE software for this.
- Define the sector address of the images to be loaded on the screen in *sd_sector_addresses.h*.
  These addresses can be found via the 4D Systems Workshop4 IDE software. First
  load the desired images on the uSD card, then find the sector addresses of
  said images via the generated .Gc file. Example:

  .. code-block:: cpp

    // New gait
    #define NEW_GAIT SectorAddress { 0x0000, 0x0050 }
    #define NEW_GAIT_SELECTED SectorAddress { 0x0000, 0x0100 }
    #define NEW_GAIT_ACTIVATED SectorAddress { 0x0000, 0x0200 }

  The first value is the high part of the address and the second the low part.

- For this example, we will create a new gait screen next to the walk screen.
  Create a new state, i.e. gait, in the constructor of *state_machine.cpp*.

  .. code-block:: cpp

    State& new_gait = this->createGaitState(NEW_GAIT, NEW_GAIT_SELECTED, NEW_GAIT_ACTIVATED, "new_gait", nullptr);

  The ``createGaitState`` function automatically creates 3 new states for the
  normal, selected and activated screen with connections between them and adds
  the state to the state machine. The final argument is a pointer to a state the
  gait should go to once succeeded. In our case we want to return to the
  new_gait screen after it succeeded, so we pass ``nullptr``.

- Now connect the new state to the walk state

  .. code-block:: cpp

    new_gait.withRight(&walk);

  This creates a connection between the new_gait and walk screens. From new_gait
  we can move right to walk and from walk we can move left to new_gait. See the
  :input-device:`state.h <include/state.h>` header file for more methods to
  connect states.
