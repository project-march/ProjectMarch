March Tutorials
===============

Welcome to the main documentation on the |m4| exoskeleton.
Whether you are an experienced ROS developer or have recently started learning, these tutorials contain all knowledge required to work
with and develop the |m4| exoskeleton.

Requirements
------------
In order to follow the tutorials, we expect you to have at least some knowledge of the following tools:

- `Ubuntu 16.04 <http://releases.ubuntu.com/16.04/>`_
- Linux command line
- `Git <https://git-scm.com/>`_

In case you are not comfortable with the above tools, please take some time to check the following references:

- `Git & GitHub Crash Course For Beginners <https://www.youtube.com/watch?v=SWYqp7iY_Tc>`_
- `Linux command line for beginners <https://tutorials.ubuntu.com/tutorial/command-line-for-beginners>`_


How to use the tutorials
------------------------
The tutorials are designed to provide hands-on experience with ROS and the |m4| exoskeleton.
Here is an overview of the different types of tutorials.

Create your workspace
^^^^^^^^^^^^^^^^^^^^^
.. include:: doc/getting_started/create_your_workspace.rst
  :start-after: inclusion-introduction-start
  :end-before: inclusion-introduction-end

Intro to ROS
^^^^^^^^^^^^
.. include:: doc/getting_started/intro_to_ros.rst
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
   doc/getting_started/intro_to_ros

.. toctree::
   :caption: Useful ROS tools
   :maxdepth: 2
   :hidden:

   doc/useful_ros_tools/robot_model
   doc/useful_ros_tools/ros_bag

.. toctree::
   :caption: Using the March IV
   :maxdepth: 2
   :hidden:

   doc/using_the_march_iv/high_level_overview
   doc/using_the_march_iv/connecting_to_the_exoskeleton
   doc/using_the_march_iv/how_to_airgait
   doc/using_the_march_iv/how_to_report_bugs
   doc/using_the_march_iv/log_files

.. toctree::
   :caption: Development
   :maxdepth: 3
   :hidden:

   doc/development/documentation
   doc/continuous_integration/continuous_integration


.. toctree::
  :caption: March Packages
  :maxdepth: 4
  :hidden:

  doc/march_packages/march_hardware_builder
  doc/march_packages/march_rqt_launch_menu
