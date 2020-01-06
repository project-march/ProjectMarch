.. _create-your-workspace-label:

Create your workspace
=====================
.. inclusion-introduction-start

This tutorial will help you set up a ROS workspace with all packages needed to run the March exoskeleton.

.. inclusion-introduction-end

.. note:: If you already have ros installed and are familiar with ros installations, you can skip to the :ref:`automated-script-label` below.
  We strongly recommend to follow the manual guide at least once to familiarize yourself with the commands.

Install ROS and Colcon
^^^^^^^^^^^^^^^^^^^^^^
`Install ROS Melodic <https://wiki.ros.org/melodic/Installation/Ubuntu>`_.
It is easy to miss steps when going through the ROS installation tutorial. If you run into errors in the next few steps, a good place to start is to go back and make sure you have installed ROS correctly.

Once you have ROS installed, make sure you have the most up to date packages:

.. code::

  rosdep update  # No sudo
  sudo apt-get update
  sudo apt-get dist-upgrade

For building our packages we use colcon. Install `colcon <https://github.com/colcon>`_:

.. code::

  sudo apt-get install python3-colcon-common-extensions

To install some optional tools that are run by Travis Continuous Integration run:

.. code::

  pip install --user catkin_lint
  python2 -m pip install --user flake8 pep8-naming flake8-blind-except flake8-string-format flake8-builtins flake8-commas flake8-quotes flake8-print flake8-docstrings flake8-import-order
  sudo apt install clang-format


Create A Workspace
^^^^^^^^^^^^^^^^^^^^^^^^^
You will need to have a ROS workspace setup:

.. code::

  mkdir -p ~/march_ws/src
  cd march_ws


Download the march source code
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
We use `wstool <http://wiki.ros.org/wstool>`_ to easily maintain the multiple repositories in our workspace.
The process differs a bit, depending on if you have write access to our repositories or not.

With write access
-----------------
If you have write access, you can use our provided ``.rosinstall`` file to pull all repositories in our workspace

.. code::

  wstool init src https://raw.githubusercontent.com/project-march/tutorials/develop/doc/getting_started/.rosinstall
  wstool update -t src

The above rosinstall file uses https URLs to the git repositories. If you prefer ssh URLs use the following commands:

.. code::

  wstool init src https://raw.githubusercontent.com/project-march/tutorials/develop/doc/getting_started/ssh.rosinstall
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


.. _build-your-workspace-label:

Build your Workspace
^^^^^^^^^^^^^^^^^^^^^^^^^^^
The following will install from the Ubuntu repositories any package dependencies not already in your workspace:

.. code::

  rosdep install -y --from-paths src --ignore-src

The next command will build and install your workspace:

.. code::

  colcon build

In order for ROS to know where your files are installed, you have to provide a source. You do this with:
:code:`source ~/march_ws/install/setup.bash`.

This needs to be done for every new terminal you open, so it is advised to
`add this command to your <https://answers.ros.org/question/206876/how-often-do-i-need-to-source-setupbash/?answer=206976#post-id-206976>`_
:code:`~/.bashrc`, which is an Ubuntu script ran every time a new terminal is started. Restart your terminal after doing this.

.. note:: Sourcing the ``setup.bash`` automatically in your ``~/.bashrc`` is
   not required and often skipped by advanced users who use more than one
   catkin workspace at a time, but we recommend it for simplicity.


.. _automated-script-label:

Automated script
^^^^^^^^^^^^^^^^
You can run the following script or download it from :codedir:`here <getting_started/create_march_ws.sh>`.

.. literalinclude:: create_march_ws.sh
   :language: bash
