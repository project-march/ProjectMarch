.. _install_docker-label:

Setting up docker
=================

We use docker to run the code in consistent isolated containers. For more information about docker see
`this short youtube video about docker <https://www.youtube.com/watch?v=Gjnup-PuquQ>`_,
or see their docs `here <https://docs.docker.com/get-started/>`_.

Download docker engine
^^^^^^^^^^^^^^^^^^^^^^
The first thing we need is the docker engine itself. Check if you have it installed with :code:`docker --version`.
If this does not output something like "Docker version 20.10.11, build dea9396", then follow the steps below.

If you are on Mac or Windows, you can find an installation guide on the `docker engine install page <https://docs.docker.com/engine/install/>`_ website.

If you are on Ubuntu, follow the steps below:

.. code-block:: bash

    # To check if you have docker installed, run:
    docker --version
    # If it outputs something like "Docker version 20.10.7, build 20.10.7-0ubuntu5~20.04.2", then you have docker installed.

    # To install docker, use the following code or go to `https://docs.docker.com/engine/install/` for alternative options.
    curl -fsSL https://get.docker.com -o get-docker.sh  # Download docker installer script.
    sudo sh get-docker.sh  # Install docker by running installer script.
    rm get-docker.sh  # Removes the installer script.

    # Commands to remove the need for `sudo` before every docker command.
    sudo groupadd docker
    usermod -aG docker $USER  # To remove the need for 'sudo' in front of every docker command.
    sudo gpasswd -a $USER docker # To remove the need for 'sudo' in front of every docker command.
    newgrp docker  # To activate the previous command. If you still need 'sudo', restart your computer

Download docker compose
^^^^^^^^^^^^^^^^^^^^^^^
We use docker-compose to start multiple containers at the same time, and with the right settings.
See below on how to install docker-compose.

If you are on Mac or Windows, you can find an installation guide on the `docker compose install page <https://docs.docker.com/compose/install/>`_ website.

If you are on Ubuntu check out the same link as for Mac and Windows, or follow the steps below:

.. code-block:: bash

    sudo curl -L "https://github.com/docker/compose/releases/download/1.29.2/docker-compose-$(uname -s)-$(uname -m)" \
        -o /usr/local/bin/docker-compose

    sudo chmod +x /usr/local/bin/docker-compose

    # For command-line completion, this step can be checked by typing 'docker-com' and then pressing 'tab':
    sudo curl \
        -L https://raw.githubusercontent.com/docker/compose/1.29.2/contrib/completion/bash/docker-compose \
        -o /etc/bash_completion.d/docker-compose

    # You need to reload your terminal, or do `source ~/.bashrc` to use docker-compose commands.


Log in to GitLab with docker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

    docker login registry.gitlab.com -u <username> -p <token>

    # If you wish you can also input your password instead of token but this is insecure.
    # To find your GitLab username,
    #   1. Log in to your GitLab account in a browser
    #   2. Click on your name in the top right corner
    #   3. There you can find @<username>

    # To get a token you need to go to https://gitlab.com/-/profile/personal_access_tokens
    # and make a key with 'read_registry' access.


Add aliases
^^^^^^^^^^^
To make everything easier to use for you we, have provided some aliases.
Copy and paste the following code into your :code:`~/.bashrc` or :code:`~/.march_bash_aliases`.

You can change the :code:`$MARCH_DOCKER_CPU_CAP` to cap the ros cpu usage. If you have 12 CPU's you can put down any number
between 0.01 and 12. This specifies how much of the CPU's can be used with 12 being 100% in this case.
To check how many CPU's you have available, run :code:`nproc --all`.
You can change the :code:'$MARCH_DOCKER_MEMORY_CAP' to cap the memory usage. The minimum required is 4GB. You can check
the amount you have available by running :code:'vmstat -s | grep "total memory" | awk '{print $1$2}''. If you do not
have memory issues, you do not need to change the value.
Note: the docker works with the following units: k for kilobyte, m for megabyte, g for gigabyte and t for terabyte.

.. code-block:: bash

    export MARCH_COMPOSE_FILE="${HOME}/march/.docker_local/docker-compose.yaml"
    export MARCH_DOCKER_CPU_CAP=$(nproc --all) # you can adjust this from anywhere from 0.01 - <$(nproc --all)>
    export MARCH_DOCKER_MEMORY_CAP=$(vmstat -s | grep "total memory" | awk '{print $1$2}') # you can adjust this between
    4G and <$(vmstat -s | grep "total memory" | awk '{print $1$2}')>, which is the total amount of RAM in your computer

    alias set_uid_gid='export M_UID=$(id -u $USER) && export M_GID=$(id -g $USER)'
    alias march_clean_containers='docker rm ros1 ros2 bridge'

    alias march_run='set_uid_gid && export ROS_DOCKER_START_TYPE=run && docker-compose -f "${MARCH_COMPOSE_FILE}" up'
    alias march_run_bash='set_uid_gid && export ROS_DOCKER_START_TYPE=bash && docker-compose -f "${MARCH_COMPOSE_FILE}" up'
    alias march_build='set_uid_gid && export ROS_DOCKER_START_TYPE=build && docker-compose -f "${MARCH_COMPOSE_FILE}" up'

    alias march_docker_pull_ros2='ROS2_GIT="registry.gitlab.com/project-march/march/local:ros2" && cd ~/march/ && docker pull $ROS2_GIT && docker tag $ROS2_GIT ros2 && docker rmi $ROS2_GIT'
    alias march_docker_image_ros2='cd ~/march/ && docker build -f .docker_local/dockerfiles/foxyFull.Dockerfile -t ros2 .'

    # To build the ros code through docker images individually, it is a bit faster than march_build.
    alias march_docker_ros2_build='set_uid_gid && export ROS_DOCKER_START_TYPE=build && docker-compose -f "${MARCH_COMPOSE_FILE}" up --no-deps ros2-service'

    # To run the ros code through docker images individually.
   alias march_docker_ros2_run='set_uid_gid && export ROS_DOCKER_START_TYPE=run && docker-compose -f "${MARCH_COMPOSE_FILE}" up --no-deps ros2-service'

    # To start up the docker images individually.
    alias march_docker_ros2_bash='set_uid_gid && docker-compose -f "${MARCH_COMPOSE_FILE}" run --no-deps ros2-service bash'

    alias march_ros2_bash='docker exec -it ros2 bash'

    # Below is a script to automatically source the correct files on startup, inside the docker images.
    if [ -n "$ROS_DOCKER_TYPE" ];then
        if [ -f /opt/ros/foxy/local_setup.bash ];then
          source /opt/ros/foxy/local_setup.bash
        fi

        if [ "$ROS_DOCKER_TYPE" == "ros2" ] && [ -f "$HOME"/march/ros2/install/local_setup.bash ];then
          source "$HOME"/march/ros2/install/local_setup.bash
        fi
    fi


Download gpu support
^^^^^^^^^^^^^^^^^^^^
If you have a dedicated NVIDIA gpu in your computer, you might need to follow the following steps,
if not, you can skip this part. Before continuing, make sure you have setup your video card correctly.
You can check this with the command :code:`prime-select query`. It should return 'on-demand' or 'nvidia'.
If this does not work, go to this website to find out how to
`use nvidia graphics card <https://www.linuxbabe.com/desktop-linux/switch-intel-nvidia-graphics-card-ubuntu>`_.

To configure the gpu for usage in docker, go to the `docker resource constraint page <https://docs.docker.com/config/containers/resource_constraints/#gpu>`_ or
follow the step below to set up docker Nvidia support:

.. code-block:: bash

    curl -s -L https://nvidia.github.io/nvidia-container-runtime/gpgkey | \
        sudo apt-key add -

    distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
    curl -s -L https://nvidia.github.io/nvidia-container-runtime/$distribution/nvidia-container-runtime.list | \
        sudo tee /etc/apt/sources.list.d/nvidia-container-runtime.list
    sudo apt-get update

    sudo apt-get install nvidia-container-runtime

    # Now restart docker daemon. This can be done by rebooting you computer, or calling 'service docker restart'
    # To check if it is correctly installed, run the following line:
    docker run -it --rm --gpus all ubuntu nvidia-smi

    # This should show something like:
    #+-----------------------------------------------------------------------------+
    #| NVIDIA-SMI 384.130            	Driver Version: 384.130                    |
    #|-------------------------------+----------------------+----------------------+
    #| GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
    #| Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
    #|===============================+======================+======================|
    #|   0  GRID K520       	Off  | 00000000:00:03.0 Off |                  N/A |
    #| N/A   36C    P0    39W / 125W |      0MiB /  4036MiB |       0%     Default |
    #+-------------------------------+----------------------+----------------------+
    #+-----------------------------------------------------------------------------+
    #| Processes:                                                       GPU Memory |
    #|  GPU   	PID   Type   Process name                               Usage      |
    #|=============================================================================|
    #|  No running processes found                                                 |
    #+-----------------------------------------------------------------------------+

After this step, you need to change one of the previously added aliases.
Go to your :code:`~/.bashrc` or :code:`~/.march_bash_aliases`, and look for the :code:`export MARCH_COMPOSE_FILE=`.

.. code-block:: bash

    # Change:
    export MARCH_COMPOSE_FILE="${HOME}/march/.docker_local/docker-compose.yaml"
    # To:
    export MARCH_COMPOSE_FILE="${HOME}/march/.docker_local/docker-compose-gpu.yaml"


Downloading the docker images
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Now, you should be all ready to go. The last step is to get the newest docker image from the GitLab repository.
You will need to redo this step everytime there are new dependencies added to the ROS code.
Luckily getting these new images can be done very easily with the following commands:

.. code-block:: bash

    # To pull the ROS2 image:
    march_docker_pull_ros2

    # You can also build them yourself, but this is not advised.
    # You should only do this if you want to add anything to the ros dependencies
    march_docker_image
    march_docker_image_ros2

Running the march code
^^^^^^^^^^^^^^^^^^^^^^
Now that everything is downloaded, you can run the ros code.

.. code-block:: bash

    march_run  # To run the entire march code.
    march_build  # To run build for all of march code.
    march_run_bash  # To start up the ROS containers / environments

    # You can log into the ros environments with:
    march_ros2_bash

Giving arguments to march run
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
You can also add arguments to the ros2 startup. This is done by setting the environment variable
:code:`ROS2_ARGS`. The code block below shows how to do this.
Note however, that because you set an environment variable these will persist within the terminal session.
This means that if you do :code:`march_run` again from the same window it will use the same startup arguments.
To unset this also see the code block below:

.. code-block:: bash

    # To add arguments to ros2 startup, you need to set environment variable with:
    export ROS2_ARGS='...' # To set args for ros2

    # NOTE: These persist within in the terminal session, if you wish to unset them do:
    unset ROS2_ARGS


