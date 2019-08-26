.. _march-state-machine-label:

march_state_machine
===================

Overview
--------
The march_state_machine contains the general behavior of the |m4| exoskeleton. Simply put after launching the exoskeleton gait can be performed.
The state machine also knows which gait can be executed in every possible state. For example after sitting you can stand-up, but you cannot walk.

We are using an hierarchical state machine, which is a state machines whose states are them- selves state machines. We also use SMACH  this is a task-level
architecture for rapidly creating complex robot behavior(see :ref:`smach-label`).

Behavior
--------
There are 4 general states in the state machine: **LAUNCH**, **HEALTHY**, **SHUTDOWN**, **ERROR**. **LAUNCH** and **HEALTHY** are also state machines.
As mentioned above this is possible in a hierarchical state machine.

.. figure:: images/march_state_machine.png
   :align: center

LAUNCH
^^^^^^
Launch starts with a **WAIT ON SETTINGS** state. This state waits on user input to select what to start and what to skip. After the input is received the state machine
continues. Depending on the the user's input, the **HARDWARE SEQUENCE** or the **SIMULATION SEQUENCE** is started.
Read `SMACH sequences <http://wiki.ros.org/smach/Tutorials/Sequence%20container>`_ to understand sequences. Both sequences start the required or requested packages.
After this the last sequence starts all remaining packages that are required or requested. This is done in the **DEFAULT SEQUENCE**.


HEALTHY
^^^^^^^
When launched successful the **HEALTHY** state machine is active. This part contains all possible gaits. Besides gaits we also have *idle* states such as **STANDING**, **SITTING**, **UNKNOWN**.
**UNKNOWN** is the state in which the current position is not known. This can be on start-up, but also after an error is thrown and the |m4| exoskeleton did not fully execute the intended gait.
Transitions between *idle-states* to *gait-states* are present only when it's safe to execute this gait at that moment.

In this state machine the **SAFETY** state is always active. However, this is not the only active state.
Multiple active states the same time are called `concurrent states <http://wiki.ros.org/smach/Tutorials/Concurrent%20States>`_. The **SAFETY** state is responsible for
monitoring possible errors (see :ref:`march-safety-label`). This is done in a concurrent state, this way not every states in **HEALTHY** needs to have this responsibility.

ERROR
^^^^^
After an error is thrown (see :ref:`march-safety-label` when this happens) **ERROR** will become active instead of **HEALTHY**. After 5 seconds the state machine will try to recover.
This is done by going to the **UNKNOWN** state in **HEALTHY**.

SHUTDOWN
^^^^^^^^
**SHUTDOWN** is not implemented at this moment.


Tutorials
---------

Add a state to the state machine
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
See :ref:`add-gait-label` on how to add a gait state to the **HEALTY** state machine. Adding other non-gait states is similar.
