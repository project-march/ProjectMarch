Continuous Integration
======================
.. inclusion-introduction-start

This tutorial will describe the current Continuous Integration (CI) setup and
provide a tutorial on how to setup a new repository with CI.

.. inclusion-introduction-end

Prerequisites
^^^^^^^^^^^^^
Before reading this tutorial it is useful to read the documentation on `Travis CI <https://docs.travis-ci.com/>`_.

Setup
^^^^^
The march repositories all use `open source Travis CI <https://travis-ci.com/project-march>`_
for builds, which run on commit pushes. There are no extra builds configured
for pull requests since branches have to be up to date with the target branch
in order to merge. You can download a template :codedir:`.travis.yml <continuous_integration/template.travis.yml>`
for new repositories.

Build Notifications
-------------------
Travis builds can send notifications when builds succeed or fail. By default
Travis sends emails about builds. The march repositories disable the email
notifications and enable secure slack integration. This is done by the
following setting in ``.travis.yml``.

.. literalinclude:: template.travis.yml
    :language: yaml
    :lines: 9-12

Where ``<secure string>`` is replaced by an encrypted connection string from
slack. See `the Travis docs on slack integration <https://docs.travis-ci.com/user/notifications/#configuring-slack-notifications>`_.

Configuring Industrial CI
-------------------------
The ``.travis.yml`` scripts use the master branch of
`ros-industrial/industrial_ci
<https://github.com/ros-industrial/industrial_ci/tree/master>`_ tools to run
various tests. Industrial CI makes it easy to set things up and it is a
structured way of running CI. ROS industrial CI is configured using environment
variables.

.. literalinclude:: template.travis.yml
    :language: yaml
    :lines: 17-28

The ``global`` variables are used for every Travis build, whereas the
``matrix`` variables are used for different builds. So the template runs two
builds: one on Kinetic and one on Melodic. ``BUILDER`` configures to use
``colcon`` as build tool, since that is not the default for ROS1 packages.
``CATKIN_LINT`` configures to run ``catkin_lint`` and fail on warnings and
give error descriptions.

Industrial CI works with `different workspaces
<https://github.com/ros-industrial/industrial_ci/blob/master/doc/index.rst#workspace-management>`_.
The template uses the ``UPSTREAM_WORKSPACE``. This workspace contains packages
necessary to build the current package. So it downloads the repositories
defined in ``dependencies.rosinstall`` file and builds the current package
against those. This type of workspace is used for example by
:gait-generation:`gait-generation <.travis.yml>` and :simulation:`simulation <.travis.yml>`.
The other type of workspace that can be used is the ``DOWNSTREAM_WORKSPACE``.
The downstream workspace contains packages that depend on the current package.
It builds and runs tests on downstream packages to check if there are any
breaking changes. This is currently only used in :march-iv:`march-iv <.travis.yml>`.

Finally there is the ``AFTER_SCRIPT`` option. This runs the given script after
all the default tests. Currently it is set in the template to run roslint on
the target packages. The build can fail if the after script fails.

Running Industrial CI
---------------------
Industrial CI is run by cloning the ``industrial_ci`` master branch and running
the ``travis.sh`` script. The master branch currently supports ``colcon`` as a
build tool.

.. literalinclude:: template.travis.yml
    :language: yaml
    :lines: 30-34

Complete Travis Configuration
-----------------------------

.. literalinclude:: template.travis.yml
    :linenos:
    :language: yaml


Setting up Travis CI on Repository
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
These steps assume you already have a public Project March repository with ROS
packages and you are a member of the Project March organization on GitHub.

1. Go to `Project March Travis CI settings
   <https://travis-ci.com/organizations/project-march/repositories>`_
2. Enable Travis for your repository
3. Travis will only start building once a ``.travis.yml`` file is present in
   the master branch, so add the template to the root of your repository.
4. If the packages in your repository need dependencies or have dependencies
   on other march repositories add those to a ``.rosinstall`` file and add
   configuration for upstream and/or downstream.
5. `Create a secure slack connection string <https://docs.travis-ci.com/user/notifications/#configuring-slack-notifications>`_ and replace the value.
6. Push it to the master branch and wait for your build.
7. Good luck!
