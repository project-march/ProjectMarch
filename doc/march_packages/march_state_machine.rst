.. _march-state-machine-label:

march_state_machine
===================

Overview
--------
The march_state_machine contains the general behavior of the |march|. Simply
put, after launching the exoskeleton the state machine decides which gaits
can be performed. The state machine also knows which gait can be executed in
every possible state. For example after sitting you can stand-up, but you
cannot walk.

We are using an hierarchical state machine, which is a state machine whose
states can be state machines as well. We also use SMACH, this is a task-level
architecture for rapidly creating complex robot behavior(see :ref:`smach-label`).

Behavior
--------
There are 4 general states in the state machine: **LAUNCH**, **HEALTHY**,
**SHUTDOWN**, **ERROR**, where **LAUNCH** and **HEALTHY** are also state
machines. As mentioned above this is possible in a hierarchical state machine.

.. figure:: images/march_state_machine.png
   :align: center

LAUNCH
^^^^^^
Launch starts with a **WAIT FOR GAIT SERVER** state. This state waits for the
gait selection server to be available, since that is where the state machine
sends requests for executing gaits.

HEALTHY
^^^^^^^
When launched successful the **HEALTHY** state machine is active. This part
contains all possible gaits. Besides gaits we also have *idle* states such as
**STANDING**, **SITTING** and **UNKNOWN**. **UNKNOWN** is the state in which
the current position is not known. This can be on start-up, but also after an
error is thrown and the |march| did not fully execute the intended gait.
Transitions between *idle-states* to *gait-states* are present only when it
is safe to execute this gait at that moment. For example, from the idle state
**SITTING** you can only stand-up.

In this state machine the **SAFETY** state is always active. However, this is
not the only active state. Multiple active states at the same time are called
`concurrent states <http://wiki.ros.org/smach/Tutorials/Concurrent%20States>`_.
The **SAFETY** state is responsible for monitoring possible errors
(see :ref:`march-safety-label`). This is done in a concurrent state and can
preempt any active state in the **HEALTHY** state machine when a fatal error
occurs.

ERROR
^^^^^
After an error is thrown (see :ref:`march-safety-label` when this happens),
**SAFETY** state preempts the **HEALTHY** state and **ERROR** will become
active. For now the **ERROR** state will automatically go to **SHUTDOWN**,
but could be used later to recover from errors.

SHUTDOWN
^^^^^^^^
**SHUTDOWN** shuts down ROS when it was called, without ROS being shutdown.
Furthermore, it tries to kill the Gazebo server sending a ``SIGTERM``, since
Gazebo has trouble shutting down by itself.


Tutorials
---------

Add a state to the state machine
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
See :ref:`add-gait-label` on how to add a gait state to the **HEALTY** state machine. Adding other non-gait states is similar.
