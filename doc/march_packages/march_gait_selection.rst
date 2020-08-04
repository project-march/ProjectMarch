.. _march-gait-selection-label:

march_gait_selection
====================

Overview
--------
The march_gait_selection module provides the exoskeleton operating system with additional information about a gait in
order to perform the requested movement. It uses the march_gait_files, which are parsed using the march_shared_classes,
to gather this information in a standard format. These class objects are stored as attributes in the ``GaitSelection``
object. The gait selection node can use this ``GaitSelection`` object to extract the data corresponding to the requested
gait/subgait name.

Behavior
--------
The march_state_machine accepts or declines a gait request originating from the input device. If the gait is accepted
by the state machine, the request will be sent over to the march_gait_selection module using the subgait sequence
defined in the state machine. The gait selection module will extract the data from the parsed gaits, found in the
``GaitSelection``, in order to perform the movement. If executed correctly, a subgait trajectories will be sent over to
the march_gait_scheduler of the exoskeleton.

Transition_subgait
^^^^^^^^^^^^^^^^^^
It is possible to transition between two matching named subgaits of two different gaits (for example between the right
swing of the walk small and walk large). The gait_selection module will create a transition trajectory if the requested
gait format, found in :march:`GaitName <march_shared_resources/action/GaitName.action>`, consists
of an old and new gait name. This structure requires that both gait names have matching subgait names to use for the
transition. Using the two subgait trajectories a new transition trajectory is calculated and stored as a subgait object.
If this is calculated correctly, the new subgait trajectory is sent to the march_gait_scheduler.

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
*/march/gait/perform* (:march:`march_shared_resources/action/GaitName <march_shared_resources/action/GaitName.action>`)
  Receives requested subgait, with gait(s), originating from the march_state_machine.

Published Topics
^^^^^^^^^^^^^^^^
*/march/gait/schedule* (:march:`march_shared_resources/action/Gait <march_shared_resources/action/Gait.action>`)
  Sends subgait trajectories to the march_gait_scheduler

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

Parameters
^^^^^^^^^^
*march_gait_selection/gait_package* (*string*, default: ``march_gait_files``)
 The package where the gait files are located.

*march_gait_selection/gait_directory* (*string*, default: ``training-v``)
 The directory where the gait files are located, relative to the above package.
