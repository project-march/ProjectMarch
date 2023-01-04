
.. _install_ros_and_march-label:

Install ROS and March
=====================
.. inclusion-introduction-start

This tutorial will help you install ROS2 Foxy, the MARCH repository, the required buildtools and dependencies.
It is recommended to follow this tutorial on Ubuntu 20.04 (Focal), other operating systems have not been tested.

.. inclusion-introduction-end

Download the MARCH source code
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
First, make sure both git and git lfs are installed:

.. code:: bash

    sudo apt install git git-lfs

The next step is to download the source code, you can either use ssh (recommended):
If you do not have a ssh-key, you can read how to generate one `here <https://docs.gitlab.com/ee/user/ssh.html>`_

.. code:: bash

    cd ~/
    git clone git@gitlab.com:project-march/march.git --recurse-submodules
    cd march/
    git lfs install


Or use https:

.. code:: bash

    cd ~/
    git clone https://gitlab.com/project-march/march.git --recurse-submodules
    cd march/
    git lfs install

.. Note::
    It is not necessary to go through the following sections to run the code. You can run everything inside
    docker images. To see how to set that up see :ref:`install_docker-label`.

Install ROS2 Foxy
^^^^^^^^^^^^^^^^^
To install ROS2, follow: `Install ROS2 Foxy <https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html>`_.

Install dependencies using rosdep:

.. code:: bash

    source /opt/ros/foxy/local_setup.bash
    cd ~/march/ros2
    rosdep update
    rosdep install --from-paths src --ignore-src --rosdistro foxy -y

Install Python dependencies
^^^^^^^^^^^^^^^^^^^^^^^^^^^
Some additional python dependencies have to be installed using pip:

.. code:: bash

    python3 -m pip install -r ~/march/requirements.txt

Install RealSense dependencies
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
If you are planning to also use the Intel Realsense camera, you should also install the necessary packages for this:
https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages.

