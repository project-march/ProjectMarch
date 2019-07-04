.. _create-your-workspace-label:

Create your workspace
=====================
.. inclusion-introduction-start

This tutorial will help you set up a ROS workspace with all packages needed to run the |m4| exoskeleton.

.. inclusion-introduction-end

.. note:: If you already have ros installed and are familiar with ros installations, you can skip to the :ref:`automated-script-label` below.
  We strongly recommend to follow the manual guide at least once to familiarize yourself with the commands.

Install ROS and Catkin
^^^^^^^^^^^^^^^^^^^^^^
`Install ROS Kinetic <http://wiki.ros.org/kinetic/Installation/Ubuntu>`_.
It is easy to miss steps when going through the ROS installation tutorial. If you run into errors in the next few steps, a good place to start is to go back and make sure you have installed ROS correctly.

Once you have ROS installed, make sure you have the most up to date packages: ::

  rosdep update  # No sudo
  sudo apt-get update
  sudo apt-get dist-upgrade

Install `catkin <http://wiki.ros.org/catkin>`_ the ROS build system: ::

  sudo apt-get install ros-kinetic-catkin python-catkin-tools

Install catkin lint and documentation: ::

  pip install --user catkin_lint
  pip install --user catkin_tools_document


Create A Catkin Workspace
^^^^^^^^^^^^^^^^^^^^^^^^^
You will need to have a `catkin <http://wiki.ros.org/catkin>`_ workspace setup: ::

  mkdir -p ~/march_ws/src
  cd march_ws
  catkin init --workspace .
  catkin build

Download the march source code
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
We us `wstool <http://wiki.ros.org/wstool>`_ to easily maintain the multiple repositories in our workspace.
The process differs a bit, depending on if you have write access to our repositories or not.

With write access
-----------------
If you have write access, you can use our provided ``.rosinstall`` file to pull all repositories in our workspace

.. code::

  wstool init src https://raw.githubusercontent.com/project-march/tutorials/develop/doc/getting_started/.rosinstall
  wstool update -t src

Without write access
--------------------
If you do not have write access, you will need to create your own forks of our repositories.
Please check `this guide <https://guides.github.com/activities/forking/>`_ on how to work with forks if you haven't used them before.

Save the `.rosinstall file <https://raw.githubusercontent.com/project-march/tutorials/develop/doc/getting_started/.rosinstall>`_
locally and change the uri of the repositories you want to develop to the location of your forks. For example:

.. code::

  - git:
      local-name: simulation
      uri: https://github.com/<your-username>/march-simulation
      version: develop

Then call wstool with your edited ``.rosinstall`` file:

.. code::

  wstool init src ~/local/path/to/the/edited/.rosinstall
  wstool update -t src


Build your Catkin Workspace
^^^^^^^^^^^^^^^^^^^^^^^^^^^
The following will install from Debian any package dependencies not already in your workspace: ::

  rosdep install -y --from-paths src --ignore-src --rosdistro kinetic

The next command will configure your catkin workspace: ::

  catkin build

Source the catkin workspace: ::

  source ~/march_ws/devel/setup.bash

Optional: add the previous command to your ``.bashrc``: ::

   echo 'source ~/march_ws/devel/setup.bash' >> ~/.bashrc

.. note:: Sourcing the ``setup.bash`` automatically in your ``~/.bashrc`` is
   not required and often skipped by advanced users who use more than one
   catkin workspace at a time, but we recommend it for simplicity.

.. _automated-script-label:

Automated script
^^^^^^^^^^^^^^^^
You can run the following script or download it from :codedir:`here <getting_started/create_march_ws.sh>`.

.. literalinclude:: create_march_ws.sh
   :language: bash
