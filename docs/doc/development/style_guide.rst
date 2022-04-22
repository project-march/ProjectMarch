.. _style-guide:

Style Guide
===========
All code in the March repositories should adhere to a certain set of style guides.

In our code we adhere to the following style guides:

    C++: https://wiki.ros.org/CppStyleGuide
        * https://github.com/PickNikRobotics/roscpp_code_format
        * clang-format: https://clang.llvm.org/docs/ClangFormat.html
    Python: https://wiki.ros.org/PyStyleGuide
        * PEP8: https://www.python.org/dev/peps/pep-0008/
        * Google Style Python Docstrings: https://www.sphinx-doc.org/en/master/usage/extensions/example_google.html

It is not important to read these specifications, the most important aspects will be summarized below.
In order to check whether your code is formatted according to the rules defined in the guides above,
you have to perform static analysis locally. See the next chapter for more information about static analysis,
and how to run it.

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

clang-tidy
^^^^^^^^^^
clang-tidy is a tool for performing static analysis on your code. It will warn about potential bugs or memory leaks.
You can install it using:

.. code::

    sudo apt-get install -y clang-tidy

clang-tidy needs your code to be built and to contain a :code:`compile_commands.json` file.
This happens automatically if you have built using the docker images.
If you have not built using docker, make sure you pass :code:`-DCMAKE_EXPORT_COMPILE_COMMANDS=ON` to your build command.
Important note: Before running the static analysis, make sure your build is up to date.

Copy and paste the following aliases in your :code:`~/.march_bash_aliases` or :code:`~/.bashrc` file.

.. code::

    # static analysis shortcuts
    alias march_static_analysis_ros1='echo "Running analysis, this can take 77 seconds" && find ~/march/ros1/src -name "*.hpp" -or -name "*.h" -or -name "*.cpp" -or -name "*.c" | grep -v "src/libraries" | grep -v "xsens" | xargs -L1 -P4 -I{} -- clang-tidy -p ~/march/ros1/build {} 2> /dev/null; true && echo -e "\n----done---"'
    alias march_static_analysis_ros1_here='echo "Running analysis, on files in folder $(pwd)" && find . -name "*.hpp" -or -name "*.h" -or -name "*.cpp" -or -name "*.c" | grep -v "src/libraries" | grep -v "xsens" | xargs -L1 -P4 -I{} -- clang-tidy -p ~/march/ros1/build {} 2> /dev/null; true && echo -e "\n----done---"'

    alias march_static_analysis_ros2='echo "Running analysis, this can take 77 seconds" && find ~/march/ros2/src -name "*.hpp" -or -name "*.h" -or -name "*.cpp" -or -name "*.c" | grep -v "src/libraries" | grep -v "xsens" | xargs -L1 -P4 -I{} -- clang-tidy -p ~/march/ros2/build {} 2> /dev/null; true && echo -e "\n----done---"'
    alias march_static_analysis_ros2_here='echo "Running analysis, on files in folder $(pwd)" && find . -name "*.hpp" -or -name "*.h" -or -name "*.cpp" -or -name "*.c" | grep -v "src/libraries" | grep -v "xsens" | xargs -L1 -P4 -I{} -- clang-tidy -p ~/march/ros2/build {} 2> /dev/null; true && echo -e "\n----done---"'

Now you can run the analysis with a couple of very simple commands.
Keep in mind that, when using the '..._here' commands, you choose to use the ros1 command for ros1 code and the ros2 command for ros2 code. The clang-tidy code checks for some violations, but can't thoroughly check your code if can't find your build folder.

.. code::

    march_static_analysis_ros1  # To run static analysis for the entire ros1/src folder.
    march_static_analysis_ros1_here  # To run static analysis on the current directory (must be located in ros1).

    march_static_analysis_ros2 # To run static analysis for the entire ros2/src folder.
    march_static_analysis_ros2_here # To run static analysis on the current directory (must be located in ros2).

.. note::

    Make sure your build is up to date and contains a :code:`compile_commands.json` file.

flake8
^^^^^^
What makes flake8 so useful is that it is able to install plugins, which add checks.
Flake8 checks by default for `PEP 8 <https://www.python.org/dev/peps/pep-0008>`_ style guide.
At March we also use more plugins. To ensure everybody uses the same plugins we run it with docker images.
Docker is a product that ensures that code runs the same on everyone's device by having all needed files and dependencies in
a closed off container. See `this 100 sec explanation <https://www.youtube.com/watch?v=Gjnup-PuquQ>`_ for more information.

flake8 setup
~~~~~~~~~~~~
First make sure that you have docker installed, if not you can do that with the following code,
or check out `this link <https://docs.docker.com/engine/install/ubuntu/>`_:

.. code-block:: bash

    # To check if you have docker installed run:
    docker --version
    # If it outputs something like "Docker version 20.10.7, build 20.10.7-0ubuntu5~20.04.2", then you have docker installed.

    # To install docker, use the following code or check out `https://docs.docker.com/engine/install/ubuntu/` for alternative options.
    curl -fsSL https://get.docker.com -o get-docker.sh  # Download docker installer script.
    sudo sh get-docker.sh  # Install docker by running installer script.
    rm get-docker.sh  # Removes the installer script.

    # Optional commands for easier docker use:
    sudo groupadd docker
    usermod -aG docker $USER  # To remove the need for 'sudo' in front of every docker command.
    sudo gpasswd -a $USER docker # To remove the need for 'sudo' in front of every docker command.
    newgrp docker  # To activate the previous command, if you still need sudo restart your computer

.. note::

    If you don't do the "Optional commands for easier use" you will get error messages if you run docker commands
    without prepending 'sudo'.


Copy and paste the following aliases in your :code:`~/.march_bash_aliases` or :code:`~/.bashrc` file.

..
    The 'dev' in the code block below might be changed to 'main' to keep the flake8 more consistent but slower to adapt.

.. code-block:: bash

    # Flake8 shortcuts (python code style checker)
    alias march_flake8_update='FLAKE8_GIT="registry.gitlab.com/project-march/march/flake8:dev" && \
    docker pull $FLAKE8_GIT && docker tag $FLAKE8_GIT march/flake8 && docker rmi $FLAKE8_GIT'
    alias march_flake8='docker run --rm -v ~/march:/home/march:ro march/flake8'
    alias march_flake8_here='docker run --rm -v `pwd`:`pwd`:ro march/flake8 `pwd`'

    # Black shortcuts (python code formatter)
    alias march_py_auto_format='docker run --rm -v ~/march:/home/march --entrypoint black march/flake8 ros1/src ros2/src utility_scripts/'
    alias march_py_auto_format_check='docker run --rm -v ~/march:/home/march:ro --entrypoint black march/flake8 \
    --check --diff --color ros1/src ros2/src utility_scripts/'
    alias march_py_auto_format_here="docker run --rm -v `pwd`:`pwd` --entrypoint black march/flake8 `pwd` -l 120 --extend-exclude '^/.*/libraries/'"
    alias march_py_auto_format_check_here="docker run --rm -v `pwd`:`pwd`:ro --entrypoint black march/flake8 -l 120 --extend-exclude '^/.*/libraries/' --check --diff --color `pwd`"

Update your flake8 docker image. You can redo this step if it doesn't produce the same output as gitlab,
or if someone from the Project MARCH software department announces to you that the docker image should be updated.

.. code-block:: bash

    # If you added the alias:
    march_flake8_update
    # Or, if you want to do it manually:
    FLAKE8_GIT="registry.gitlab.com/project-march/march/flake8:main" && \
    docker pull $FLAKE8_GIT && docker tag $FLAKE8_GIT march/flake8 && docker rmi $FLAKE8_GIT

Running flake8
~~~~~~~~~~~~~~

If you have everything set up you can very easily run it with the following commands:

.. code-block:: bash

    # To run flake8 on your whole march folder:
    march_flake8

    # To run flake8 in you current directory:
    march_flake8_here

    # To run flake8 without the aliases:
    docker run --rm -v [local_src]:[dest_in_docker]:[ro for readonly] -w [work_dir_in_docker] [image name (e.g. march/flake8)] [flake 8 arguments]

If there are any violations after running the march_flake8 alias where it says "black would make changes",
run the following commands:

.. code-block:: bash

    # Auto-format your python code (with black):
    march_py_auto_format # To auto-format all code in the march directory.
    march_py_auto_format_here # To auto-format your code according to black in your current directory.
    march_py_auto_format_check # To see what should be changed according to black in your ~/march folder.
    march_py_auto_format_check_here # To see what should be changed according to black in your current directory.

Naming Conventions
------------------
This section will explain the different naming schemes and which scheme we use for which type of object.

Naming schemes
^^^^^^^^^^^^^^
The MARCH code uses the following five naming schemes:

* **PascalCase**: The name starts with a capital letter, and has a capital letter for each new word, with no underscores.
* **camelCase**: Like PascalCase, but with a lower-case first letter.
* **snake_case**: The name uses only lower-case letters, with words separated by underscores.
* **UPPER_CASE**: All capital letters, with words separated by underscores.
* **kebab-case**: The name uses only lower-case letters, with words separated by lines.

A prefix is a common word placed before the rest of the name. For example: the prefix for ROS Packages is 'march'.
If you want to create a package called 'state_machine', the package should be named 'march_state_machine'.

General naming guidelines
^^^^^^^^^^^^^^^^^^^^^^^^^
* Avoid abbreviations: prefer getIMotionCubes() over getIMCs()
* Be descriptive
    * The name of a function should make clear what action it performs. Prefer isAlive() over alive()
    * The name of a variable or class should make clear what is represents. Prefer is_alive over alive

Naming conventions
^^^^^^^^^^^^^^^^^^
.. list-table:: Naming conventions
    :header-rows: 1

    * - Type
      - Case
      - Prefix
      - Postfix
      - Example
    * - Repositories
      - kebab-case
      -
      -
      - gait-generation
    * - ROS Packages
      - snake_case
      - march
      -
      - march_state_machine
    * - Nodes
      - snake_case
      -
      -
      - march_hardware_interface
    * - Topics / Services
      - PascalCase
      -
      -
      - GaitInstruction.msg
    * - Files
      - snake_case
      -
      -
      - march_hardware_interface_node.cpp
    * - Classes
      - PascalCase
      -
      -
      - HardwareBuilder
    * - Variables
      - snake_case
      -
      -
      - cycle_time
    * - Class fields (C++)
      - snake_case
      -
      - _
      - \net_number_
    * - Class fields (Python)
      - snake_case
      -
      -
      - field_name
    * - Private fields (Python only)
      - snake_case
      - _
      -
      - _private_something
    * - Methods / functions (C++)
      - camelCase
      -
      -
      - createMarchRobot()
    * - Methods / functions (Python)
      - snake_case
      -
      -
      - do_something()
    * - Constants
      - UPPER_CASE
      -
      -
      - MAXIMUM_TORQUE
    * - Namespaces
      - snake_case
      -
      -
      - march
