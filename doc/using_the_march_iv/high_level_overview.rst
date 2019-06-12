High level overview
===================
.. inclusion-introduction-start

A high level overview of the components that make up the |m4| exoskeleton.

.. inclusion-introduction-end

.. figure:: images/software_stack.png
   :align: center

   The |m4| software stack

Simulation versus Exoskeleton
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
As our exoskeleton is under construction most of the time, we have made use of a simulation to ensure we can verify our higher level nodes work.
As seen in the above image, most of the hardware components have a simulated counterpart.

At launch time, the user is asked if they are trying to run on the exoskeleton or on the simulation.
Based on this selection, the required components are started.

Input Device
^^^^^^^^^^^^