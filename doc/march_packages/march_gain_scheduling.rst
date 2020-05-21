.. _march-march_gain_scheduling-label:

march_gain_scheduling
=====================

The march_gain_scheduling package is used to implement the gainscheduling control method. More about this control method
can be found on the `following page <https://en.wikipedia.org/wiki/Gain_scheduling>`_.
We implemented the method in such a way that it changes the PID gains/values when a different type of gait is selected.

Principle
---------
The Gainscheduling package uses the definition of gait_type as, for example, seen in `this gait file <https://github.com/project-march/gait-files/blob/develop/march_gait_files/training-v/sit/sit_down/MIV_final.subgait>`_.
to change all the PID values for the relevant joints.

The following gait_types are supported:

- `sit_like`
- `walk_like`
- `stairs_like`

These gait_types are set in the gait generator if required, otherwise the default walk_like is used.

The PID values are changed using a dynamic reconfigure client which alters the gains at ``/march/controller/trajectory/gains/ + joint_name``.
The node obtains the gait_type from the following topic: ``/march/gait/schedule/goal`` where ``GaitActionGoal`` type messages are published.

This node also offers the ability to change PID values linearly through the setting of a parameter during the launch of the node.

How to use
""""""""""

Gainscheduling is already set to run alongside the exoskeleton. You will most likely not need to launch this node manually.
If you do want to launch it manually run the following lines in a terminal:

.. code::

  roslaunch march_gain_scheduling march_gain_scheduling.launch

you can change the used configuration (exoskeleton, test_joint_rotational, test_joint_linear) using the ``configuration`` argument.
You can set or reset the use of the linearization using the ``linear`` argument and lastly you can set the slope with which the PID values are linearized using the ``slope`` argument.

.. note:: Always launch the gainscheduling node after starting the hardware interface. This prevents the node from searching for parameters and topics that are not yet alive.

How to tune
"""""""""""
Tuning using gainscheduling is rather easy. You can freely change the PID values in the `config folder <https://github.com/project-march/march/tree/develop/march_gain_scheduling/config>`_.
In this folder you'll find a number of ``.yaml`` files which correspond to different configurations of the exoskeleton and different test/run modes.

Please tune the exoskeleton in the following order:

- `try the tuning on a testjoint with a relevant load`
- `tune in airgait stand`
- `tune whilst groundgaiting`
- `tune with the pilot in the exo`

.. warning::

    Pay attention when tuning these values! Always double check if your values aren't absurdly large and pay attention to
    the amount these values change when a different gait_type is used. Too large changes can cause instability in the system.

.. warning::

    When using the ``linear`` argument (set to true by default) pay attention to the ``slope`` argument as well.
    large changes in gains need to be applied accordingly. Neither too slow (low slope value) for the gait to perform well, or too quick (high slope value)
    so that it won't introduce instability into the system.
