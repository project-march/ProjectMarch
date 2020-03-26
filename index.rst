March Tutorials
===============

Welcome to the main documentation on the |march|.
Whether you are an experienced ROS developer or have recently started learning, these tutorials contain all knowledge required to work
with and develop the |march|.

Requirements
------------
In order to follow the tutorials, we expect you to have at least some knowledge of the following tools:

- `Ubuntu 18.04 <http://releases.ubuntu.com/18.04/>`_
- Linux command line
- `Git <https://git-scm.com/>`_

In case you are not comfortable with the above tools, please take some time to check the following references:

- `Git & GitHub Crash Course For Beginners <https://www.youtube.com/watch?v=SWYqp7iY_Tc>`_
- `Using The Terminal <https://help.ubuntu.com/community/UsingTheTerminal>`_


How to use the tutorials
------------------------
The tutorials are designed to provide hands-on experience with ROS and the |march|.
Here is an overview of the different types of tutorials.

Create your workspace
^^^^^^^^^^^^^^^^^^^^^
.. include:: doc/getting_started/create_your_workspace.rst
  :start-after: inclusion-introduction-start
  :end-before: inclusion-introduction-end


Useful ROS tools
^^^^^^^^^^^^^^^^
A selection of existing ROS tools to speed up development.

March Packages
^^^^^^^^^^^^^^
This section contains explanation and tutorials at the package level.
It is not needed to study all of them to completion, but it is advised to keep them as a reference whenever you want
start improving a specific package. This is especially useful when adding more content to a feature (e.g. gaits, launch settings).

Documentation
^^^^^^^^^^^^^
.. include:: doc/development/documentation.rst
  :start-after: inclusion-introduction-start
  :end-before: inclusion-introduction-end


Attribution
-----------
The structure of this documentation is heavily inspired by that of `MoveIt! <http://docs.ros.org/melodic/api/moveit_tutorials/html/index.html>`_


.. toctree::
   :caption: Getting started
   :maxdepth: 2
   :hidden:

   doc/getting_started/create_your_workspace

.. toctree::
   :caption: Useful ROS tools
   :maxdepth: 2
   :hidden:

   doc/useful_ros_tools/robot_model
   doc/useful_ros_tools/ros_control
   doc/useful_ros_tools/rqt
   doc/useful_ros_tools/ros_bag
   doc/useful_ros_tools/gazebo
   doc/useful_ros_tools/smach

.. toctree::
   :caption: Using the March Exoskeleton
   :maxdepth: 2
   :hidden:

   doc/using_the_march_exoskeleton/high_level_overview
   doc/using_the_march_exoskeleton/connecting_to_the_exoskeleton
   doc/using_the_march_exoskeleton/how_to_airgait
   doc/using_the_march_exoskeleton/how_to_view_live_data
   doc/using_the_march_exoskeleton/log_files
   doc/using_the_march_exoskeleton/using_the_gait_generator
   doc/using_the_march_exoskeleton/error_codes

.. toctree::
   :caption: Development
   :maxdepth: 3
   :hidden:

   doc/development/documentation
   doc/continuous_integration/continuous_integration
   doc/development/testing
   doc/development/style_guide
   doc/development/add_a_new_gait


.. toctree::
  :caption: March Packages
  :maxdepth: 4
  :hidden:

  doc/march_packages/march_description
  doc/march_packages/march_fake_sensor_data
  doc/march_packages/march_gait_files
  doc/march_packages/march_hardware
  doc/march_packages/march_hardware_builder
  doc/march_packages/march_hardware_interface
  doc/march_packages/march_input_device
  doc/march_packages/march_launch
  doc/march_packages/march_rqt_input_device
  doc/march_packages/march_rqt_software_check
  doc/march_packages/march_safety
  doc/march_packages/march_shared_classes
  doc/march_packages/march_shared_resources
  doc/march_packages/march_state_machine
  doc/march_packages/march_simulation
