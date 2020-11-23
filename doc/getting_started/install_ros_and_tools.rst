
.. install_ros_and_march-label:

Install ROS and March
=====================
.. inclusion-introduction-start

This tutorial will help you install ROS1 Melodic, ROS2 Foxy, the MARCH repository, the required buildtools and dependencies.

.. inclusion-introduction-end

Download the MARCH source code
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
First, make sure both git and git lfs are installed:

.. code:: bash

    sudo apt install git
    curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash

The next step is to download the source code, you can either use ssh (recommended):

.. code:: bash

 cd ~/
 git lfs install
 git clone git@gitlab.com:project-march/march.git

Or use https:

.. code:: bash

 cd ~/
 git lfs install
 git clone https://gitlab.com/project-march/march.git

Install ROS1 Melodic
^^^^^^^^^^^^^^^^^^^^
This tutorial is mostly copied from `Install ROS1 Melodic <https://wiki.ros.org/melodic/Installation/Ubuntu>`_.

1. First, install some tools:

..  code:: bash

    sudo apt install lsb-release gnupg2 python3-colcon-common-extensions


2. The next step is to setup your sources.list:

.. code:: bash

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'


3. Then, setup your keys:

.. code:: bash

    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

4. If these steps went well, you should be able to install ROS Melodic:

.. code:: bash

    sudo apt update
    sudo apt install ros-melodic-desktop-full


5. Once you have ROS installed, make sure you have the most up to date packages:

.. code:: bash

  sudo apt install python-rosdep
  sudo rosdep init
  rosdep update  # No sudo
  sudo apt update
  sudo apt full-upgrade

6. Finally, the following will install any ROS1 Melodic package dependencies not already in your workspace:

.. code:: bash

  sudo apt update
  source /opt/ros/melodic/setup.bash
  cd ~/march/ros1/
  rosdep install -y --from-paths src --ignore-src

Install ROS2 Foxy
^^^^^^^^^^^^^^^^^
Installing ROS2 Foxy requires some more effort than installing ROS1 Melodic.
This tutorial is a slightly updated version of `Install ROS2 Foxy <https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Development-Setup/>`_.

1. The first step is to update the CMake version, as the default Ubuntu 18.04 doesn't comply with some ROS2 packages (This may take some time):

.. code:: bash

    cd ~/
    sudo apt update && sudo apt install -y build-essential libssl-dev wget
    wget https://github.com/Kitware/CMake/archive/v3.18.5.tar.gz && tar -zxvf v3.18.5.tar.gz && rm v3.18.5.tar.gz && cd CMake-3.18.5 && ./bootstrap && make && cd .. && mv CMake-3.18.5 .CMake-3.18.5
    cd .CMake-3.18.5 && sudo ln -s $(pwd) /usr/local/share/cmake-3.18 && sudo ln $(pwd)/bin/* /usr/local/bin

Verify CMake is correctly updated by running:

.. code:: bash

    cmake --version

The output should be exactly:

.. code::

    cmake version 3.18.5

    CMake suite maintained and supported by Kitware (kitware.com/cmake).

2. Then we have to make sure the right locale is set:

.. code:: bash

    locale  # check for UTF-8

    sudo apt update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

    locale  # verify settings

3. Now we can add the ROS2 apt repository:

.. code:: bash

    sudo apt update && sudo apt install curl gnupg2 lsb-release
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

4. And add the repository to the sources list

.. code:: bash

    sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

5. The next step is to install the necessary development and ROS tools:

.. code:: bash

    sudo apt update && sudo apt install -y \
      build-essential \
      cmake \
      git \
      libbullet-dev \
      python3-colcon-common-extensions \
      python3-flake8 \
      python3-pip \
      python3-pytest-cov \
      python3-setuptools \
      python3-vcstool \
      wget
    # install some pip packages needed for testing
    python3 -m pip install -U \
      argcomplete \
      flake8-blind-except \
      flake8-builtins \
      flake8-class-newline \
      flake8-comprehensions \
      flake8-deprecated \
      flake8-docstrings \
      flake8-import-order \
      flake8-quotes \
      pytest-repeat \
      pytest-rerunfailures \
      pytest
    # install Fast-RTPS dependencies
    sudo apt install --no-install-recommends -y \
      libasio-dev \
      libtinyxml2-dev
    # install Cyclone DDS dependencies
    sudo apt install --no-install-recommends -y \
      libcunit1-dev

6. Now we can create a ROS2 Foxy workspace and retrieve the code:

.. code:: bash

    mkdir -p ~/ros2_foxy/src
    cd ~/ros2_foxy
    wget https://raw.githubusercontent.com/ros2/ros2/foxy/ros2.repos
    vcs import src < ros2.repos

7. Install dependencies using rosdep:

.. code:: bash

    rosdep update
    rosdep install --from-paths src --ignore-src --rosdistro foxy -y --skip-keys "console_bridge fastcdr fastrtps rti-connext-dds-5.3.1 urdfdom_headers"

8. Some additional dependencies have to be manually added using the .repos file from the main march repository. Run the following:

.. code:: bash

    cd ~/ros2_foxy/
    wget https://gitlab.com/project-march/march/-/raw/main/ros2_dependencies.repos
    vcs import src < ros2_dependencies.repos

9. The final step is to build the ROS2 code. This may take a long time (> 1h):

.. code:: bash

    cd ~/ros2_foxy/
    # skip ros1_bridge package since that has to configured later
    colcon build --symlink-install --packages-skip ros1_bridge

Install Python dependencies
^^^^^^^^^^^^^^^^^^^^^^^^^^^
Some additional python dependencies have to be installed using pip:

.. code:: bash

  python -m pip install -r ~/march/requirements.txt

