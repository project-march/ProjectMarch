
.. _create-your-workspace-label:

Create your workspace (Outdated)
================================
.. inclusion-introduction-start

This tutorial will help you set up a ROS workspace with all packages needed to run the |march|.

.. inclusion-introduction-end

.. note:: If you already have ros installed and are familiar with ros installations, you can skip to the :ref:`automated-script-label` below.
  We strongly recommend following the manual guide at least once to familiarize yourself with the commands.

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

To install some optional tools that are run by GitLab Continuous Integration run:

.. code::

  pip install --user catkin_lint
  python2 -m pip install --user flake8 pep8-naming flake8-blind-except flake8-string-format flake8-builtins flake8-commas flake8-quotes flake8-print flake8-docstrings flake8-import-order
  sudo apt install clang-format


Create A Workspace
^^^^^^^^^^^^^^^^^^^^^^^^^
You will need to have a ROS workspace setup:

.. code::

  mkdir -p ~/march_ws/
  cd march_ws


Download the march source code
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
To download the source code, you can either use ssh (recommended):

.. code::

 git clone git@gitlab.com:project-march/march.git

Or use https:

.. code::

 git clone https://gitlab.com/project-march/march.git

Build your Workspace
^^^^^^^^^^^^^^^^^^^^^^^^^^^
The following will install from the Ubuntu repositories any package dependencies not already in your workspace:

.. code::

  cd ~/march_ws/march
  rosdep install -y --from-paths src --ignore-src

The next command will build and install your workspace:

.. code::

  colcon build

In order for ROS to know where your files are installed, you have to provide a source. You do this with:
:code:`source ~/march_ws/march/install/setup.bash`.

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
