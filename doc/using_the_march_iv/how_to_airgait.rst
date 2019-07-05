How to airgait
==============
.. inclusion-introduction-start

This tutorial will help you to airgait with the |m4| exoskeleton.

.. inclusion-introduction-end

Airgait on the exoskeleton directly
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The Setup
---------
- Screen connected with a HDMI cable to the |m4| exoskeleton.
- The |m4| exoskeleton wireless keyboard and mouse.
- Exoskeleton hanging in the cage.
- Tools needed to power the |m4| exoskeleton.


Start up
---------
- **Low voltage:** Make sure the high voltage if off, by pressing the emergency button.
  Give the |m4| exoskeleton low voltage, by enabling power.
- **Start the exoskeleton pc:** Keep the start button pressed until the blue light is continuously on.
- The computer is now booting. The display, mouse and keyboard should automatically connect.
- **High voltage:** Turn the high voltage on, but releasing the emergency button.
- **Start the software:** Type in a sourced terminal :code:`roslaunch march_launch march_iv.launch`.
- **Select launch settings:** Select in the launch menu the settings you need.


Remote Airgait
^^^^^^^^^^^^^^

.. note:: For this part you have to complete the :ref:`preparation-before-connecting-label`.


The Setup
---------
- Your pc.
- Exoskeleton hanging in the cage.
- Tools needed to power the |m4| exoskeleton.
- Router (for now the switch laptop).


Start up
---------
- **Low voltage:** Make sure the high voltage if off, by pressing the emergency button.
  Give the |m4| exoskeleton low voltage, by enabling power.
- **Start the exoskeleton pc:** Keep the start button pressed until the blue light is continuously on.
- The computer is now booting. The computer will automatically connect with the **MARCH** hotspot.
- **Start your computer** Start your computer and connect with the **MARCH** hotspot from the router.
- **High voltage:** Turn the high voltage on, but releasing the emergency button.
- **SSH:** Start a terminal to connect with the exoskeleton pc. Connect with :code:`ssh march@march`.
- **Start the software:** Type in a sourced terminal :code:`roslaunch march_launch march_headless.launch`.
- **Input Device:** Type in a sourced terminal :code:`roslaunch march_rqt_input_device march_rqt_input_device.launch`.


.. note:: For this part we assume that on the pc you are running everything is installed described in  :ref:`create-your-workspace-label`
