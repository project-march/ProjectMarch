.. _install_docker-label:

Setting up docker
=================

We use docker to run the code in consistent isolated containers. For more information about docker see
`this short youtube video about docker <https://www.youtube.com/watch?v=Gjnup-PuquQ>`_,
or see their docs `here <https://docs.docker.com/get-started/>`_.

Download docker engine
^^^^^^^^^^^^^^^^^^^^^^
First thing we need is the docker engine itself. Check if you have it installed, if not follow the steps below.

If you are on mac or windows check out the `docker engine install page <https://docs.docker.com/engine/install/>`_
for an installation guide.

If you are on ubuntu follow the steps here below:

.. code-block:: bash

    # To check if you have docker installed run:
    docker --version
    # If it outputs something like "Docker version 20.10.7, build 20.10.7-0ubuntu5~20.04.2", then you have docker installed.

    # To install docker, use the following code or check out `https://docs.docker.com/engine/install/` for alternative options.
    curl -fsSL https://get.docker.com -o get-docker.sh  # Download docker installer script.
    sudo sh get-docker.sh  # Install docker by running installer script.
    rm get-docker.sh  # Removes the installer script.

    # Commands to remove the need for `sudo` before every docker command.
    sudo groupadd docker
    usermod -aG docker $USER  # To remove the need for 'sudo' in front of every docker command.
    sudo gpasswd -a $USER docker # To remove the need for 'sudo' in front of every docker command.
    newgrp docker  # To activate the previous command, if you still need sudo restart your computer

Download docker compose
^^^^^^^^^^^^^^^^^^^^^^^

If you are on mac or windows check out the `docker compose install page <https://docs.docker.com/compose/install/>`_
for an installation guide.

If you are on ubuntu check out the same link as for mac and windows, or follow the steps below:

.. code-block:: bash

    sudo curl -L "https://github.com/docker/compose/releases/download/1.29.2/docker-compose-$(uname -s)-$(uname -m)" \
        -o /usr/local/bin/docker-compose

    sudo chmod +x /usr/local/bin/docker-compose

    # For command-line completion:
    sudo curl \
        -L https://raw.githubusercontent.com/docker/compose/1.29.2/contrib/completion/bash/docker-compose \
        -o /etc/bash_completion.d/docker-compose

    # You need to reload your terminal, or do `source ~/.bashrc` to use docker-compose commands.

Download gpu support
^^^^^^^^^^^^^^^^^^^^
If you have a dedicated NVIDIA gpu in your computer you might need to follow the following steps,
if not you can skip this part. Before continuing make sure you have setup your video card correctly.
You can check this with the command :code:`prime-select query` it should return 'on-demand' or 'nvidia'.
If this does not work checkout
`use nvidia graphics card <https://www.linuxbabe.com/desktop-linux/switch-intel-nvidia-graphics-card-ubuntu>`_.

Next check out `docker resource constraint <https://docs.docker.com/config/containers/resource_constraints/#gpu>`_ or
follow the step below to setup docker Nvidia support:

.. code-block:: bash

    curl -s -L https://nvidia.github.io/nvidia-container-runtime/gpgkey | \
        sudo apt-key add -

    distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
    curl -s -L https://nvidia.github.io/nvidia-container-runtime/$distribution/nvidia-container-runtime.list | \
        sudo tee /etc/apt/sources.list.d/nvidia-container-runtime.list
    sudo apt-get update

    sudo apt-get install nvidia-container-runtime

    # Now restart docker daemon. This can be done by rebooting you computer, or calling 'service docker restart'
    # To check if it is correctly installed run the following line:
    docker run -it --rm --gpus all ubuntu nvidia-smi

    # This should show something like:
    #+-----------------------------------------------------------------------------+
    #| NVIDIA-SMI 384.130            	Driver Version: 384.130               	|
    #|-------------------------------+----------------------+----------------------+
    #| GPU  Name 	   Persistence-M| Bus-Id    	Disp.A | Volatile Uncorr. ECC |
    #| Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
    #|===============================+======================+======================|
    #|   0  GRID K520       	Off  | 00000000:00:03.0 Off |                  N/A |
    #| N/A   36C	P0    39W / 125W |  	0MiB /  4036MiB |      0%  	Default |
    #+-------------------------------+----------------------+----------------------+
    #+-----------------------------------------------------------------------------+
    #| Processes:                                                       GPU Memory |
    #|  GPU   	PID   Type   Process name                         	Usage  	|
    #|=============================================================================|
    #|  No running processes found                                                 |
    #+-----------------------------------------------------------------------------+

Afterward you need to uncomment the following code into your :code:`~/march/.docker_local/docker-compose.yaml`:

.. code-block:: bash

    # Uncomment the code shown below in the group called 'x-gui', it can be found bellow '- /tmp/.X11-unix:/tmp/.X11-unix:rw'.
    # This is (probably) on line 23 to line 27.
      deploy: # This is needed for gpu acceleration
        resources:
          reservations:
            devices:
              - capabilities: [ gpu ]


Log in to gitlab with docker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

    docker login registry.gitlab.com


Add aliases
^^^^^^^^^^^
To make everything easier to use for you we have provided some aliases.
Copy and paste the following code into your :code:`~/.bashrc` or :code:`~/.march_bash_aliases`.

.. code-block:: bash

    alias set_uid_gid='export M_UID=$(id -u $USER) && export M_GID=$(id -g $USER)'
    alias march_clean_containers='docker rm ros1 ros2 bridge'

    alias march_run='set_uid_gid && export ROS_DOCKER_START_TYPE=run && docker-compose -f ~/march/.docker_local/docker-compose.yaml up'
    alias march_run_bash='set_uid_gid && export ROS_DOCKER_START_TYPE=bash && docker-compose -f ~/march/.docker_local/docker-compose.yaml up'
    alias march_build='set_uid_gid && export ROS_DOCKER_START_TYPE=build && docker-compose -f ~/march/.docker_local/docker-compose.yaml up'

    alias march_docker_pull_ros1='ROS1_GIT="registry.gitlab.com/project-march/march/local:ros1" && cd ~/march/ && docker pull $ROS1_GIT && docker tag $ROS1_GIT ros1 && docker rmi $ROS1_GIT'
    alias march_docker_image_ros1='cd ~/march/ && docker build -f .docker_local/dockerfiles/noeticFull.Dockerfile -t ros1 .'

    alias march_docker_pull_ros2='ROS2_GIT="registry.gitlab.com/project-march/march/local:ros2" && cd ~/march/ && docker pull $ROS2_GIT && docker tag $ROS2_GIT ros2 && docker rmi $ROS2_GIT'
    alias march_docker_image_ros2='cd ~/march/ && docker build -f .docker_local/dockerfiles/foxyFull.Dockerfile -t ros2 .'

    alias march_docker_pull_bridge='BRIDGE_GIT="registry.gitlab.com/project-march/march/local:bridge" && cd ~/march/ && docker pull $BRIDGE_GIT && docker tag $BRIDGE_GIT bridge && docker rmi $BRIDGE_GIT'
    alias march_docker_image_bridge='cd ~/march/ && docker build -f .docker_local/dockerfiles/bridge.Dockerfile -t bridge .'

    alias march_docker_pull='march_docker_ros1_pull && march_docker_ros2_pull && march_docker_bridge_pull'
    alias march_docker_image='march_docker_ros1_build && march_docker_ros2_build && march_docker_bridge_build'

    # To build the ros code through docker images individually, it is a bit faster than march_build.
    alias march_docker_ros1_build='set_uid_gid && export ROS_DOCKER_START_TYPE=build && docker-compose -f ~/march/.docker_local/docker-compose.yaml up --no-deps ros1-service'
    alias march_docker_ros2_build='set_uid_gid && export ROS_DOCKER_START_TYPE=build && docker-compose -f ~/march/.docker_local/docker-compose.yaml up --no-deps ros2-service'
    alias march_docker_bridge_build='set_uid_gid && export ROS_DOCKER_START_TYPE=build && docker-compose -f ~/march/.docker_local/docker-compose.yaml up --no-deps bridge-service'

    # To run the ros code through docker images individually.
    alias march_docker_ros1_run='set_uid_gid && export ROS_DOCKER_START_TYPE=run && docker-compose -f ~/march/.docker_local/docker-compose.yaml up --no-deps ros1-service'
    alias march_docker_ros2_run='set_uid_gid && export ROS_DOCKER_START_TYPE=run && docker-compose -f ~/march/.docker_local/docker-compose.yaml up --no-deps ros2-service'
    alias march_docker_bridge_run='set_uid_gid && export ROS_DOCKER_START_TYPE=run && docker-compose -f ~/march/.docker_local/docker-compose.yaml up --no-deps bridge-service'

    # To start up the docker images individually.
    alias march_docker_ros1_bash='set_uid_gid && docker-compose -f ~/march/.docker_local/docker-compose.yaml run --no-deps ros1-service bash'
    alias march_docker_ros2_bash='set_uid_gid && docker-compose -f ~/march/.docker_local/docker-compose.yaml run --no-deps ros2-service bash'
    alias march_docker_bridge_bash='set_uid_gid && docker-compose -f ~/march/.docker_local/docker-compose.yaml run --no-deps bridge-service bash'


    alias march_ros1_bash='docker exec -it ros1 bash'
    alias march_ros2_bash='docker exec -it ros2 bash'
    alias march_bridge_bash='docker exec -it bridge bash'

    # Below is a script to automatically source the correct files on startup, inside the docker images.
    if [ -n "$ROS_DOCKER_TYPE" ];then
        if [ -f /opt/ros/noetic/local_setup.bash ];then
          source /opt/ros/noetic/local_setup.bash
        fi

        if [ -f /opt/ros/foxy/local_setup.bash ];then
          source /opt/ros/foxy/local_setup.bash
        fi

        if [ "$ROS_DOCKER_TYPE" == "ros1" ] && [ -f "$HOME"/march/ros1/install/local_setup.bash ];then
          source "$HOME"/march/ros1/install/local_setup.bash
        fi

        if [ "$ROS_DOCKER_TYPE" == "ros2" ] && [ -f "$HOME"/march/ros2/install/local_setup.bash ];then
          source "$HOME"/march/ros2/install/local_setup.bash
        fi

        if [ "$ROS_DOCKER_TYPE" == "bridge" ];then
          if [ -f "$HOME"/march/ros1/install/local_setup.bash ];then
            source "$HOME"/march/ros1/install/local_setup.bash
          fi

          if [ -f "$HOME"/march/ros2/install/local_setup.bash ];then
            source "$HOME"/march/ros2/install/local_setup.bash
          fi

          if [ -f "$HOME"/ros1_bridge/install/local_setup.bash ];then
            source "$HOME"/ros1_bridge/install/local_setup.bash
          fi
        fi
    fi

Downloading the docker images
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
You are now all ready to go. you only need to get the newest docker image from the gitlab repo.
You will needed to redo this step everytime there are new dependencies added to the ros code.
Luckily getting these new images can be done very easily with the following commands:

.. code-block:: bash

    # To pull all 3 images:
    march_docker_pull

    # Or to pull them all individually:
    march_docker_ros1_pull
    march_docker_ros2_pull
    march_docker_bridge_pull

    # You can also build them yourself, this is however not advised however.
    # You should only do this if you want to add anything to the ros dependencies
    march_docker_build
    march_docker_ros1_build
    march_docker_ros2_build
    march_docker_bridge_build

Running the march code
^^^^^^^^^^^^^^^^^^^^^^
Now that everything is downloaded you can run the ros code.

.. code-block:: bash

    march_run  # To run the entire march code. (ros1, ros2 and the bridge) (This also start up the 3 containers / "environments")
    march_build  # To run build for all of march code (ros1, ros2 and the bridge)
    march_run_bash  # To start up all 3 ros containers / "environments".

    # You can log into the ros environments with:
    march_ros1_bash
    march_ros2_bash
    march_bridge_bash

Giving arguments to march run
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
You can also add arguments to ros1 and ros2 startup. This is done by setting the environment variables
:code:`ROS_ARGS`, :code:`ROS1_ARGS` and :code:`ROS2_ARGS`. See the code block below on how to do this.
Note however, that because you set an environment variable these will persist within the terminal session.
This means that if you do :code:`march_run` again from the same window it will use the same startup arguments.
To unset this also see the code block below:

.. code-block:: bash

    # To add arguments to ros1 and ros2 startup, you need to set environment variable with:
    export ROS_ARGS='...'  # To set args for ros1 and ros2 (e.g. ground_gait:=true)
    export ROS1_ARGS='...'  # To set args for ros1 (e.g. gazebo_ui:=true)
    export ROS2_ARGS='...' # To set args for ros2

    # NOTE: These persist within in the terminal session, if you wish to unset them do:
    unset ROS_ARGS
    unset ROS1_ARGS
    unset ROS2_ARGS


