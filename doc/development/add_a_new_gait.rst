Add a new gait
==============
.. inclusion-introduction-start

Introduction
^^^^^^^^^^^^
This tutorial will teach you how to add a new gait to the codebase.
When you are done you will be able to perform the gait on the exoskeleton!

.. inclusion-introduction-end

Create new gait files
^^^^^^^^^^^^^^^^^^^^^
First create a new gait folder in the :ref:`march-gait-files-label` according to the :ref:`march-gait-files-structure-label`

Subgait files can be created with the gait generator (:ref:`using-the-gait-generator-label`) or manually.
New gaits should be added to `gait-files repository <https://github.com/project-march/gait-files>`_ with a pull request.

Extend the state machine
^^^^^^^^^^^^^^^^^^^^^^^^
Because a gait exists of multiple subgaits, we implement a gait as a state machine.

An example of a pull request adding states to the state machine can be found in `#64 <https://github.com/project-march/state-machine/pull/64>`_.

Create a new python script called ``<gait_name>_sm.py`` that looks like this:

.. code::

  #!/usr/bin/env python
  import smach

  from march_state_machine.states.GaitState import GaitState


  def create():
      sm_<gait_name> = smach.StateMachine(outcomes=['succeeded', 'preempted', 'failed'])
      with sm_<gait_name>:
          smach.StateMachine.add('<FIRST_SUBSTATE>', GaitState("<gait_name>", "<first_subgait_name>"),
                                 transitions={'succeeded': '<SECOND_SUBSTATE>', 'preempted': 'failed', 'aborted': 'failed'})
          smach.StateMachine.add('<SECOND_SUBSTATE>', GaitState("<gait_name>", "<second_subgait_name"),
                                 transitions={'succeeded': 'succeeded', 'preempted': 'preempted', 'aborted': 'failed'})
      return sm_<gait_name>

In :state-machine:`healthy_sm.py <march_state_machine/src/march_state_machine/healthy_sm.py>` add your new state machine as a state.

.. code::

  smach.StateMachine.add('GAIT <GAIT_NAME>', <gait_name>_sm.py.create(), transitions={'succeeded': 'STANDING', 'preempted': 'failed', 'failed': 'UNKNOWN'})

Make sure to add a transition from standing to your recently created state so the state machine can respond to commands from the input device:

.. code::

  'gait_<gait_name>': 'GAIT <GAIT_NAME>',

Add a button to the developer input device
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
To add a gait to the developer input device, follow :ref:`add-a-new-button-label`

Add the gait to the input device
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
To add a gait to the input device, follow :ref:`how-to-add-a-gait-label`

