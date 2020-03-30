High level overview
===================
.. inclusion-introduction-start

A high level overview of the components that make up the |march|.

.. inclusion-introduction-end

.. figure:: images/high_level_architecture.svg
   :align: center

   The March software stack

Simulation versus Exoskeleton
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
As the |march| is under construction most of the time, we have made use of
a simulation to ensure we can verify our higher level nodes work. As seen in
the above image, most of the hardware components have a simulated counterpart.

When launching, the user can choose whether to launch the simulation or the
hardware. See :ref:`march-launch-label` for more information on launching the
exoskeleton.

Input Device
^^^^^^^^^^^^
The pilot of the exoskeleton can give input on which gait they want to perform
through the input device embedded in one of the crutches, see
:ref:`march-input-device-label`. Furthermore, there exists a developer input
device in the form of an RQT plugin, see :ref:`march-rqt-input-device-label`.
Both of these ways of input can either be used in the simulation or the
hardware. They are not dependent on the configuration in which the exoskeleton
is run.
