Adding dependencies
===================
.. inclusion-introduction-start

This page will explain the steps that have to be followed when you want to add a new dependency to the
code base of Project MARCH.

.. inclusion-introduction-end

Before you start
^^^^^^^^^^^^^^^^
Before you include a new dependency, it is important to look at the license of the dependency. Some
of the libraries or dependencies you might want to include, have certain restrictions that are not
compatible with the license of Project MARCH. Right now, this is quite a complex story because the
code base does not yet have an open source license. Therefore, it is recommended to contact the
software department or the maintainers of the Project MARCH repositories on GitLab.

Different types
^^^^^^^^^^^^^^^
The different dependencies are split in three different categories:

* ROS specific libraries
* Non-ROS libraries
* Generated files that are expensive to generate

Each category has their own list of steps that have to be followed if you want to add a dependency.

ROS specific libraries
----------------------
These libraries are specifically designed to work with ROS. These packages are listed in the
`ROS distro list`_. However, not all packages are 100% compatible with our version of ROS
and therefore adding ROS specific libraries requires some care.
The following steps should be followed, in order, until a step matches the situation:

1) In case it is a library that is available via rosdep for the latest version, use rosdep.
2) In case it is a library that *not* is available via rosdep for the latest version AND you
   do *not* want to change the source code, add it to the code base as a `submodule`_.
3) In case you want to change the source code, fork the library to the `Libraries and forks group`_
   on GitLab and add the fork location as a `submodule`_ to the code base.

Non-ROS libraries
-----------------
These libraries are not necessarily designed to work with ROS. Libraries such as scipy, numpy
or Boost fall in this category. The following steps should be followed, in order, until a step matches
the situation:

1) In case it is a library that is available via pip, add it to ``requirements.txt``.
   This allows the package to be installed with ``pip install -r requirements.txt``.
2) In case it is only available as source and you *don't* want to make any changes to the library,
   add it to the code base as a `submodule`_.
3) In case you want to change the source code, fork the library to the `Libraries and forks group`_
   on GitLab and add the fork location as a `submodule`_ to the code base.


.. Note::
    Avoid binary files in your fork. In case the fork contains binaries, add these binaries to the repository
    with `Git LFS`_. Check if CMake fetches these Git LFS files as well!

Generated files that are expensive to generate
----------------------------------------------
Sometimes, it is necessary to add a dependency in the form of generated files that are expensive to
generate. An example of this are the files that are generated for model predictive control.

These files should be in their own repository in the  `Libraries and forks group`_ on GitLab
and the repository can be included with CMake with the FetchContent_Declare command.

.. _ROS distro list: https://github.com/ros/rosdistro/
.. _Libraries and forks group: https://gitlab.com/project-march/libraries
.. _Git LFS: https://git-lfs.github.com/
.. _submodule: https://git-scm.com/book/en/v2/Git-Tools-Submodules
