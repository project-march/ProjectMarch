.. _march-gait-files-label:

march_gait_files
================

Introduction
------------
This is a static package that contains only information about the gaits.
Most gaits are created by the gait generator, but as they are simply text files they can also be edited manually.

Terminology
-----------

``Subgait folder``: A folder named after a physical movement. Contains multiple subgait files for different versions.

``Subgait file``: A text file containing a version of a subgait in the form of a
`JointTrajectory Message <http://docs.ros.org/melodic/api/trajectory_msgs/html/msg/JointTrajectory.html>`_.
The file is named <version>.subgait.

``Gait folder``: A folder containing a gait file and the subgait folders that constitute the gait.

``Gait file``: A text file defining the gait's transitions from subgait to subgait.
:gait-files:`Example <march_gait_files/airgait/walk/walk.gait>`

``<gait_directory>`` is a folder used to group gaits together, some gaits are ready for a training, while some are only for airgaiting.
:gait-files:`Example <march_gait_files/airgait>`

``default.yaml`` is required for each gait_directory. It specifies which versions of gaits are loaded by default when loading this gait_directory.
:gait-files:`Example <march_gait_files/airgait/default.yaml>`

Example:

====  ===========  =======
Gait  Subgait      Version
----  -----------  -------
walk  right_open   default
walk  right_open   slower
walk  left_swing   move_ankle_higher
sit   prepare_sit  now_a_bit_slower
====  ===========  =======

.. _march-gait-files-structure-label:

Structure
---------

.. code::

  march_gait_files
  ├─── <gait_directory>
  │    ├─── <gait_name>
  │    │    ├─── <subgait_name>
  │    │    │    └─── <version>.subgait
  |    │    └─── <gaint_name>.gait
  |    └─── default.yaml
  └─── airgait
       ├─── walk
       │    ├─── right_open
       │    │    ├─── normal.subgait
       │    │    └─── a_bit_slower.subgait
       │    ├─── left_swing
       │    │    ├─── normal.subgait
       │    │    └─── a_bit_faster.subgait
       │    └─── walk.gait
       └─── default.yaml
