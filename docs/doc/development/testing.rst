Testing
==================
This page describes the process of running and writing automated tests for
the march packages.


Prerequisites
-------------
In order to run the tests you must already have a workspace setup.
See :ref:`setup-your-workspace-label`.


Running the tests locally
-------------------------
Before running al test you first need to **build** and **source** your workspace this can be done with:

.. code::

  march_build_ros1 && sros1
  # or
  march_build_ros2 && sros2

Afterwards you can run the following command to run all tests in the current directory:

.. code::

  colcon test

The ``build`` step also builds the automated tests, so it is important to run
``build`` when you modify tests. The ``test`` step will run all the tests and
will fail when tests fail. Colcon also gives the possibility to review the test
results and give more information on any failing tests.

.. code::

  colcon test-result --all

This will output every test result of every package that includes tests.

When you are writing tests for a specific package you do not want to run all
the tests every time. Colcon also gives an option to only build tests for
specified packages.

.. code::

  colcon test --packages-select march_some_package march_some_other_package

General guidelines to writing tests
-----------------------------------
Here are some rules of thumb to follow when writing tests:

#. Write short understandable tests.
#. Separate setup code that is necessary for every test.
#. Do not depend on writing and reading from external files.
#. Do not depend on timing in your tests.
#. Use one ``assert`` per test (i.e. only test one function)
#. If possible, every method should have at least one unit test
#. Make a test for every edge case!
#. Usually, your test code is about as long as your normal code.

Next to these rules there exist some conventions in structuring tests:

#. Put your tests in a separate directory next to ``src/`` named ``test/``.
#. The same as for all filenames, name the test files using snake_case.
#. Put rostest ``.test`` launch files inside ``test/launch/``.
#. Put rostest source files inside ``test/rostest/``.
#. If writing unittests and rostests for Python, put the unit tests inside ``test/unittests/``.
#. Put necessary resources, e.g. files, for tests in a separate directory inside ``test/``.

See the March packages for examples on how to write and structure tests.

Writing your own tests
----------------------
For testing ROS packages we make a distinction between two different kinds of tests.

1. Unit tests (library level)
2. Integration tests (node level)

Also, see the `ROS wiki on testing <https://wiki.ros.org/Quality/Tutorials/UnitTesting>`_.
The unit tests are different for C++ and python projects. C++ projects use the
`gtest <https://github.com/google/googletest>`_ framework, whereas Python projects use
`unittest <https://pythontesting.net/framework/unittest/unittest-introduction/>`_.
For node level tests, `rostest <https://wiki.ros.org/rostest>`_ is used.
Rostest uses launch files to launch the nodes under test and the actual tests.
See the following tutorials on writing tests using ROS:

* `Writing C++ tests with gtest <https://wiki.ros.org/gtest>`_
* `Writing Python tests with unittest <https://wiki.ros.org/unittest>`_
* `Writing rostest files <https://wiki.ros.org/rostest/Writing>`_
