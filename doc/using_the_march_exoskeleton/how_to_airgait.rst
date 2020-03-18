.. _how-to-airgait-label:

How to airgait
==============
.. inclusion-introduction-start

This tutorial will help you to airgait with the |march|.

.. inclusion-introduction-end

Airgait on the exoskeleton directly
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The setup
---------
- Screen connected with a HDMI cable to the |march|.
- The |march| wireless keyboard and mouse.
- Exoskeleton hanging in the cage.
- Tools needed to power the |march|.


Start up
---------
#. **High voltage:** Turn the high voltage on by releasing the emergency button and enabling power.
#. **Start the exoskeleton pc:** Keep the start button pressed until the blue light is continuously on.
   The computer is now booting. The display, mouse and keyboard should automatically connect.
#. **Start the software:** Type in a sourced terminal :code:`roslaunch march_launch march.launch`.
#. **Select launch settings:** You can select launch settings by appending launch options to the launch.
   use :code:`roslaunch --ros-args march_launch march.launch` to see what options are available.
   See the :ref:`march-launch-label` page for more info on how to launch the exoskeleton.

Remote airgait
^^^^^^^^^^^^^^

.. note:: For this part you have to complete the :ref:`preparation-before-connecting-label`.


The setup
---------
- Your pc.
- Exoskeleton hanging in the cage.
- Tools needed to power the |march|.
- WiFi Router


Start up
---------
#. **High voltage:** Turn the high voltage on by releasing the emergency button and enabling power.
#. **Start the exoskeleton pc:** Keep the start button pressed until the blue light is continuously on.
   The computer is now booting. The computer will automatically connect with the WiFi router.
#. **Start your computer** Start your computer and connect with the WiFi router.
#. **SSH:** Start a terminal to connect with the exoskeleton pc. Connect with :code:`ssh march@march`.
#. **Start the software:** Type :code:`roslaunch march_launch march_headless.launch`.
#. **Input Device:** Type in a sourced terminal on your computer :code:`roslaunch march_rqt_input_device march_rqt_input_device.launch`
   to start the input device.


.. note:: For this part we assume that on the pc you are running everything is installed described in  :ref:`create-your-workspace-label`
