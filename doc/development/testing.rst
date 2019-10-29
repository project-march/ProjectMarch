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

Writing your own tests
----------------------
For testing ROS packages we make a distinction between two different kinds of tests.

1. Unit tests (library level)
2. Integration tests (node level)

Also see the `ROS wiki on testing <https://wiki.ros.org/Quality/Tutorials/UnitTesting>`_.
The unit tests are different for c++ and python projects. C++ projects use the
`gtest <https://github.com/google/googletest>`_ framework, whereas Python projects use
`unittest <http://pythontesting.net/framework/unittest/unittest-introduction/>`_.
For node level tests, `rostest <https://wiki.ros.org/rostest>`_ is used.
Rostest uses launch files to launch the nodes under test and the actual tests.

The next sections will describe the process of writing tests. For this
tutorial we will use the :codedir:`ros_test_tutorial package <development/ros_test_tutorial>`.
This package already includes some c++ and python code to write tests for.

Writing c++ unit tests
^^^^^^^^^^^^^^^^^^^^^^
Tests are always located in the ``test/`` directory of a package. To create a
test we create a new cpp file in the ``test`` directory called ``AddTest.cpp``
with the following contents.

.. code::

    #include <gtest/gtest.h>

    #include "ros_test_tutorial/Add.h"

    TEST(AddTest, addTwoAndOne)
    {
        Add add;
        ASSERT_EQ(add.add(2, 1), 3);
    }

    int main(int argc, char* argv[])
    {
        testing::InitGoogleTest(&argc, argv);
        return RUN_ALL_TESTS();
    }

Feel free to experiment with writing tests. The actual test is written using
the ``TEST`` macro.  The first argument is the test suite, the same as the
filename in this case.  The second argument is name of the actual test. So this
would be another example of a test.

.. code::

    TEST(AddTest, addNegative)
    {
        Add add;
        ASSERT_EQ(add.add(0, -1), -1);
    }

In order to build and run the unit tests we must add ``rosunit`` as test
dependency to our package. So normally we would add the following to the
``package.xml``. However, this has already been done in highlevel
``package.xml``.

.. code::

  <test_depend>rosunit</test_depend>

Next we must tell cmake which tests it has to build. So add the following to the ``CMakeLists.txt``.

.. code::

    if(CATKIN_ENABLE_TESTING)
        catkin_add_gtest(add_test test/AddTest.cpp)
        target_link_libraries(add_test ${PROJECT_NAME} ${catkin_LIBRARIES})
    endif()

Here we tell ``cmake`` to build the test and use our library. Now when
``colcon test`` is run from the workspace root you should see tests passing.

Writing c++ node tests
^^^^^^^^^^^^^^^^^^^^^^


Writing python unit tests
^^^^^^^^^^^^^^^^^^^^^^^^^
Tests for python are also always located in the ``test/`` directory. We will
first start by writing our test. So create the ``MultiplyTest.py`` file to your
``test/`` directory and fill it with the following contents.

.. code::
    #!/usr/bin/env python

    import unittest
    import rosunit

    from ros_test_tutorial import multiply

    class MultiplyTest(unittest.TestCase):

        def test_multiply_one_and_one():
            self.assertEqual(multiply(1, 1), 1)

    if __name__ == '__main__':
        rosunit.unitrun(PKG, 'test_multiply', MultiplyTest)

The actual tests are written as functions inside the ``MultiplyTest`` class.
See if you can add more tests (and make them fail). See the `python unittest
documentation <https://docs.python.org/3/library/unittest.html>`_ for more
information on writing tests.

Next we must tell ``cmake`` to build and how to run the tests. We do this by
adding ``catkin_add_nosetests()`` inside our ``CATKIN_ENABLE_TESTING``.

.. code::

    if(CATKIN_ENABLE_TESTING)
        ...
        catkin_add_nosetests(test/MultiplyTest.py)
        ...
    endif()

The tests can also be run with ``colcon test``.
Also see the `ros wiki on writing unit tests for python <https://wiki.ros.org/unittest#Code-level_Python_Unit_Tests>`_.

Writing python node tests
^^^^^^^^^^^^^^^^^^^^^^^^^

