.. _march-gait-selection-label:

march_gait_selection
====================

Overview
--------
The march_gait_selection module is responsible for loading the specified gait
directory, generating a gait state machine and keeping track of exoskeleton
state. It uses the march_gait_files, which are parsed using the
march_shared_classes, to gather gait information in a standard format. Gait
class objects are stored as attributes in the ``GaitSelection`` object. The gait
selection node can use this ``GaitSelection`` object to extract the data
corresponding to the requested gait/subgait name.

Behavior
--------
The gait state machine accepts or declines a gait request originating from an
input device. If the gait is accepted by the state machine, the requested gait
will be loaded from the ``GaitSelection`` instance and performed. If executed
correctly, subgait trajectories will be sent over to the trajectory controller.
While the gait is being performed, the state machine will listen for more input
from the user about stopping or transitioning and inform the performing gait
when a request is made.

Gait selection services
^^^^^^^^^^^^^^^^^^^^^^^
The gait_selection_node also acts as a server for requesting subgait versions
and changing versions of subgaits.

Gait interface
^^^^^^^^^^^^^^
The package contains a class named ``GaitInterface``, which is the interface
that should be implemented for any executable gait. The gaits loaded from the
gait directory have their own implementation called ``SetpointsGait`` and the
home gaits have their implementation named ``HomeGait``. Any other gaits not
included in these classes should be implemented separately, for example, the
``BalanceGait``. The implemented gait can then be added to the ``GaitSelection``
and will then also be added to the gait state machine.

Gait state machine
^^^^^^^^^^^^^^^^^^
The state machine has two main phases: generating and running. First, when all
gaits have been loaded and added to the ``GaitSelection`` instance, the state
machine structure is generated. This is done by comparing all start and end
points of gaits and generating idle positions and home gaits. The transitions
are then stored as an adjacency list for constant time access of transitions.
When the generation phase finishes successfully, the state machine can be
started. The starting state is always ``UNKNOWN`` and the first action should be
to home to a known idle position. The state machine differentiates between to
main states: idle and gait. When it is idle it will listen for gait input and
when it is performing a gait it will keep updating the gait in a loop until it
decides to stop.

Finally, the state machine has the ability to add callbacks when different
actions happen in the state machine. This is useful for publishing gait and
state information or other actions that should trigger on certain transitions.

Gait state machine input
^^^^^^^^^^^^^^^^^^^^^^^^
The ``StateMachineInput`` class is used for communicating input to the
``GaitStateMachine``. Any implementation of this class should be able to give
input to the ``GaitStateMachine``, however, it is currently implemented to
listen for the input device topic.

Trajectory scheduler
^^^^^^^^^^^^^^^^^^^^
The ``TrajectoryScheduler`` is another dependency of the state machine. It gets
as input a trajectory from the state machine and communicates when it fails.

Transition subgait
^^^^^^^^^^^^^^^^^^
It is possible to transition between two matching named subgaits of two
different gaits (for example between the right swing of the walk small and walk
large). The transitioning of gaits is currently implemented in the
``SetpointsGait`` class, which responds to the transition requests. It uses the
underlying subgait graph for determining whether the transition is possible.

Balance walk
^^^^^^^^^^^^
The gait selection packages is currently also the package where we store dynamic gaits. One such dynamic gait is the
balance walk gait. The graph of the balance walk gait is defined in the :ref:`march-gait-files-label`.
Some of the subgaits in this gait are just static subgaits defined by a subgaitfile. However, some of the subgaits are dynamic.
In these subgaits we try to move towards a capture point. A capture point is a point on the ground such that if you would
place your foot there, you would come to a full stop. There are a lot of papers on the internet about this, most importantly
the one introducing the principle is this `paper <https://ieeexplore-ieee-org.tudelft.idm.oclc.org/document/4115602>`_
A capture point is calculated in the :ref:`march-data-collector-label` based on the inverted pendulum model and the movement of the center of mass.
We use the motion planning framework MoveIt to calculate a trajectory for the swing leg towards this point. For the stance leg, the regular
gait file is used to determine the endpoint and MoveIt is then used to calculate a trajectory towards this point.
The balance gait class then uses the `python moveIt commander Interface <http://docs.ros.org/jade/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html>`_ to plan. More information on MoveIt and
the integration can be found at :ref:`march-moveit-label`.

.. note:: Improved versions of the balance walk gait are continuously researched and developed.
    The above description is likely slightly outdated.

ROS API
-------

Nodes
^^^^^
*gait_selection_node* - Responsible for adding gait file information to the gait request.


Subscribed Topics
^^^^^^^^^^^^^^^^^
*/march/error* (:march:`march_shared_resources/msg/Error <march_shared_resources/msg/Error.msg>`)
  Listens for errors and shuts down when a fatal is thrown.

Published Topics
^^^^^^^^^^^^^^^^
*/march/gait_selection/current_state* (:march:`march_shared_resources/msg/CurrentState <march_shared_resources/msg/CurrentState.msg>`)
  Publishes the current state of the state machine
*/march/gait_selection/current_gait* (:march:`march_shared_resources/msg/CurrentGait <march_shared_resources/msg/CurrentGait.msg>`)
  Sends details about the current gait being performed

Services
^^^^^^^^
*/march/gait_selection/get_version_map* (`std_srvs/srv/Trigger <http://docs.ros.org/melodic/api/std_srvs/html/srv/Trigger.html>`_)
  Returns the current loaded gait version map.

*/march/gait_selection/set_gait_version* (:march:`march_shared_resources/srv/SetGaitVersion <march_shared_resources/srv/SetGaitVersion.srv>`)
  Sets a new gait version map in the gait_selection.

*/march/gait_selection/get_directory_structure* (`std_srvs/srv/Trigger <http://docs.ros.org/melodic/api/std_srvs/html/srv/Trigger.html>`_)
  Returns the directory structure of the gait_files repository.

*/march/gait_selection/update_default_versions* (`std_srvs/srv/Trigger <http://docs.ros.org/melodic/api/std_srvs/html/srv/Trigger.html>`_)
  Calls the update_default_versions function of the gait_selection.

*/march/gait_selection/contains_gait* (:march:`march_shared_resources/srv/ContainsGait <march_shared_resources/srv/ContainsGait.srv>`)
  Checks if gait is in parsed gaits in the gait selection module.

*/march/gait_selection/get_possible_gaits* (:march:`march_shared_resources/srv/PossibleGaits <march_shared_resources/srv/PossibleGaits.srv>`)
  Checks if gait is in parsed gaits in the gait selection module.

Parameters
^^^^^^^^^^
*march_gait_selection/gait_package* (*string*, default: ``march_gait_files``)
 The package where the gait files are located.

*march_gait_selection/gait_directory* (*string*, default: ``training-v``)
 The directory where the gait files are located, relative to the above package.

*march_gait_selection/update_rate* (*float*, default: ``120.0``)
 The update rate of the gait state machine in Hertz.

*march_gait_selection/sounds* (*bool*, default: ``false``)
 The update rate of the gait state machine in Hertz.
