.. _march-gait-selection-label:

march_gait_selection
====================

Overview
--------
The march_gait_selection module provides the exoskeleton operating system with gait-specific information to perform the
requested movement. It uses the march-gait-files, which are parsed using the march-shared-classes, to gather this
information in a standard format. These class objects are stored as attributes in the gait-selection object. The gait
selection node can use this gait-selection object to extract the data corresponding to the requested gait/subgait name.

Behavior
--------
The march-state-machine accepts or declines a gait request originating from the input device. If the gait is accepted
by the state machine the request will be send over to the march-gait-selection module using the subgait sequence defined
in the state machine. The gait selection module will extract the data from the parsed gaits, if possible, and if executed
correctly, will send the required data in a new format to the hardware-interface or simulation part of the exoskeleton
software.

Transition-subgait
^^^^^^^^^^^^^^^^^^
It is possible to transition between two matching named subgaits of two different gaits (for example between the right
swing of the walk small and walk large). The gait-selection module is able to accomplish this if the requested
gait/sub-gait format consists of an old and new gait name. This structure requires that both gait names have matching
subgait names to use for the transition. The transition algorithm calculates a new trajectory using the two subgait
trajectories. If this is calculated correctly the new trajectory is translated to a subgait file which is eventually
send to the hardware-interface or simulation part of the exoskeleton software.


ROS API
-------

Nodes
^^^^^
*gait_selection_node* - Responsible for doing the thing.

*perform_gait_action* - Responsible for doing the thing.


Subscribed Topics
^^^^^^^^^^^^^^^^^
*/march/gait/perform* (:march:`march_shared_resources/action/GaitName <march_shared_resources/action/GaitName.action>`)
  Receives requested subgait, with gait(s), originating from the march-state-machine.

Published Topics
^^^^^^^^^^^^^^^^
*/march/gait/schedule* (:march:`march_shared_resources/action/Gait <march_shared_resources/action/Gait.action>`)
  Send subgait data to the hardware interface of the exoskeleton software.

Services
^^^^^^^^
*/march/gait_selection/get_version_map* (`std_srvs/srv/Trigger <http://docs.ros.org/melodic/api/std_srvs/html/srv/Trigger.html>`_)
  Returns the name of the gait_version map of the gait_selection.

*/march/gait_selection/set_version_map* (:march:`march_shared_resources/srv/StringTrigger <march_shared_resources/srv/StringTrigger.srv>`)
  Sets a new gait version map in the gait_selection.

*/march/gait_selection/get_directory_structure* (`std_srvs/srv/Trigger <http://docs.ros.org/melodic/api/std_srvs/html/srv/Trigger.html>`_)
  Return the directory structure of the gait-files repository.

*/march/gait_selection/update_default_versions* (`std_srvs/srv/Trigger <http://docs.ros.org/melodic/api/std_srvs/html/srv/Trigger.html>`_)
  Calls the update_default_versions function of the gait_selection.


Parameters
^^^^^^^^^^
*march_gait_selection/gait_package* (*string*, default: ``march_gait_files``)
 The package where the gait files are located.

*march_gait_selection/gait_directory* (*string*, default: ``training-v``)
 The directory where the gait files are located, relative to the above package.
