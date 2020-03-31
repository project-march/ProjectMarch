.. _style-guide:

Style Guide
===========
All code in the March repositories should adhere to a certain set of style guides.
Since most of the March packages are ROS packages, these should all adhere to the
ROS style guide for `C++ <https://wiki.ros.org/CppStyleGuide>`_ and `Python <https://wiki.ros.org/PyStyleGuide>`_.

Running style checkers
----------------------
In order to make checking for style guides easier we use tools to automatically check the code.
We use `clang-format <https://clang.llvm.org/docs/ClangFormat.html>`_ for C++ and
`flake8 <https://flake8.pycqa.org/en/latest/>`_ for Python. These are configured in ``.clang-format``
and ``.flake8`` files in the root of a repository. See the :march:`march repo <>` for examples of these files.
The ``.clang-format`` files are based on the one from https://github.com/davetcoleman/roscpp_code_format.

clang-format
^^^^^^^^^^^^
clang-format is a tool for checking and fixing code style. You can install it using:

.. code::

    sudo apt install clang-format

clang-format must be run from a repository root, where a ``.clang-format`` is located, e.g. ``~/march_ws/src/march``.
clang-format runs on specific files given to it by parameters and in order to fix all found style violations you
can use the ``-i`` option. So, to fix a given cpp file you can run:

.. code::

    clang-format --style=file -i src/some_file.cpp

This will edit the file in-place. That should not be a problem since we use git.
However, this still fixes one file. In order to fix all C++ header and source files you could use:

.. code::

    find . -name '*.h' -or -name '*.cpp' | xargs clang-format -i -style=file

This will recursively search for header and source files and format them using clang-format.
See the official `clang-format documentation <https://clang.llvm.org/docs/ClangFormat.html>`_
for more ways of running clang-format.

flake8
^^^^^^
What makes flake8 so useful is that it is able to install plugins, which add checks.
Flake8 checks by default for `PEP 8 <https://www.python.org/dev/peps/pep-0008>`_ style guide.
At March, we also use more plugins. To install flake8 with the plugins run:

.. code::

    pip2 install --user flake8 pep8-naming flake8-blind-except flake8-string-format flake8-builtins flake8-commas flake8-quotes flake8-print flake8-docstrings flake8-import-order flake8-colors

If you are wondering what a plugin checks for you can search for them on `PyPI <https://pypi.org>`_.

.. caution::

    It is important to install flake8 for python 2 using ``pip2``, since we use python 2 (for now).
