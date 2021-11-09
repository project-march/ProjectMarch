.. _style-guide:

Style Guide (Outdated)
======================
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
At March, we also use more plugins. To ensure everybody uses the same plugins we run it with docker images.

flake8 setup
~~~~~~~~~~~~
First make sure that you have docker installed, if not you can do that with the following code:

.. code-block:: bash

    curl -fsSL https://get.docker.com -o get-docker.sh  # Download docker installer script.
    sudo sh get-docker.sh  # Install docker by running installer script.

    # Optional commands for easier use:
    rm get-docker.sh  # Removes the installer script.
    usermod -aG docker $USER  # To remove the need for 'sudo' in front of every docker command.
    newgrp docker  # To activate the previous command, if you still need sudo restart your computer

Copy and paste the following aliases in your :code:`~/.march_bash_aliases` or :code:`~/.bashrc` file.

.. code-block:: bash

    # Flake8 shortcuts (python code style checker)
    alias march_flake8_update='FLAKE8_GIT="registry.gitlab.com/project-march/march/flake8:main" && \
    docker pull $FLAKE8_GIT && docker tag $FLAKE8_GIT march/flake8 && docker rmi $FLAKE8_GIT'
    alias march_flake8='docker run -v ~/march:/home/march:ro march/flake8'
    alias march_flake8_here='docker run -v `pwd`:`pwd`:ro -w /. -v ~/march/.flake8:/.flake8:ro march/flake8 `pwd`'

    # Black shortcuts (python code formatter)
    alias march_black='docker run -v ~/march:/home/march --entrypoint black march/flake8 ros1/src ros2/src utility_scripts/'
    alias march_black_check='docker run -v ~/march:/home/march:ro --entrypoint black march/flake8 \
    --check --diff --color ros1/src ros2/src utility_scripts/'
    alias march_black_here='docker run -v `pwd`:`pwd` --entrypoint black march/flake8 `pwd`'
    alias march_black_check_here='docker run -v `pwd`:`pwd`:ro --entrypoint black march/flake8 --check --diff --color `pwd`'

Update your flake8 docker image. You can redo do this step if it doesn't produce the same output as gitlab,
or if someone from software sends a slack message.

.. code-block:: bash

    # If you added the alias:
    march_flake8_update
    # Or, if you want to do it manually:
    FLAKE8_GIT="registry.gitlab.com/project-march/march/flake8:main" && \
    docker pull $FLAKE8_GIT && docker tag $FLAKE8_GIT march/flake8 && docker rmi $FLAKE8_GIT

Running flake8
~~~~~~~~~~~~~~

If you have everything setup you can very easily run it with the following commands:

.. code-block:: bash

    # To run flake8 on your whole march folder:
    march_flake8

    # To run flake8 in you current directory:
    march_flake8_here

    # To run flake8 without the aliases:
    docker run -v [local_src]:[dest_in_docker]:[ro for readonly] -w [work_dir_in_docker] [image name (e.g. march/flake8)] [flake 8 arguments]

If there is an violations anywhere in the march_flake8 where it says "black would make changes" run the following commands:

.. code-block:: bash

    march_black # To auto-format all code in the march directory.
    march_black_here # To auto-format you code according to black in you current directory.
    march_black_check # To see what should be changes according to black in you ~/march folder.
    march_black_check_here # To see what should be changes according to black in you current directory.
