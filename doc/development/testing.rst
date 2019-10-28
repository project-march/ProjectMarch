Testing
=======
This process describes the process of running and writing automated tests for
the march packages.

Prerequisites
^^^^^^^^^^^^^
In order to run the tests you must have already have a workspace setup.
See :ref:`create-your-workspace-label`.

Running the tests locally
^^^^^^^^^^^^^^^^^^^^^^^^^
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


