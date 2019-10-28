Testing
=======
This page describes the process of running and writing automated tests for
the march packages.


Prerequisites
-------------
In order to run the tests you must have already have a workspace setup.
See :ref:`create-your-workspace-label`.


Running the tests locally
-------------------------
Running all the tests within your workspace can be done by the following commands:

.. code::

  # colcon must be run from your workspace root
  cd ~/march_ws
  colcon build
  colcon test

The ``build`` step also builds the automated tests, so it is important to run
``build`` when you modify tests. This will run all the tests and will fail when
tests fail. Colcon also gives the possibility to review the test results and
give more information on any failing tests.

.. code::

  colcon test-result --all

This will output every test result of every package that includes tests.

When you are writing tests for a specific package you do not want to run all
the tests every time. Colcon also gives an option to only build tests for
specified packages.

.. code::

  colcon test --packages-select march_some_package march_some_other_package

Running roslint
^^^^^^^^^^^^^^^
The march packages that contain are also checked for code style with
`roslint <https://wiki.ros.org/roslint>`_. The ``roslint`` target should also
be run from the workspace root like so.

.. code::

  colcon build --cmake-target-skip-unavailable --cmake-target roslint

This command will run the ``roslint`` target on any packages that use
``roslint``. It will also output any linter warnings that you should fix,
otherwise the build will fail.

Running catkin_lint
^^^^^^^^^^^^^^^^^^^
Another tool that is used to lint packages is `catkin_lint <https://github.com/fkie/catkin_lint>`_.
``catkin_lint`` checks the package manifest and cmake configuration files for
any common errors. This command can be run anywhere in the workspace and will recurse into directories.
So in order to check all packages you should run this command in the root of the workspace.

.. code::

  catkin_lint -W2 --explain .

This will output any notes, warnings or errors with an explanation. This
command is normally run when you have created a new package or made changes to
a ``package.xml`` or ``CMakeLists.txt`` file.

