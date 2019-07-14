
.. _connecting-to-the-exoskeleton-label:

Connecting to the exoskeleton
=============================
.. inclusion-introduction-start

This tutorial will help you setting up a connection with the exoskeleton.

.. inclusion-introduction-end

.. note:: Make sure your workspace is sourced and set-up correctly: :ref:`build-your-catkin-workspace-label`.

.. _preparation-before-connecting-label:

Preparation before connecting
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
This tutorial is written to establish a connection between your computer and the exoskeleton on the **same wifi**.
So, make sure you are on the same wifi. A public network such as eduroam will probably not work. Normally the |M4| exoskeleton
automatically connects with the hotspot: **MARCH**.

Running commands on the exoskeleton computer (SSH)
--------------------------------------------------
**Secure Shell (SSH)** is a network protocol which allows to run commands on the exoskeleton.
Connect with the exoskeleton by typing :code:`ssh march@march` in a terminal.
Now all commands in this terminal are executed on the |M4| exoskeleton computer.

ROS Master URI
--------------
To use ros commands you need to setup the right `ROS_MASTER_URI <http://wiki.ros.org/ROS/EnvironmentVariables>`_ in every terminal you use: ::

    export ROS_MASTER_URI=http://<exoskeleton ip>:11311/

**<exoskeleton ip>** can be obtained by running the command :code:`hostname -I` in a terminal on the exoskeleton computer.
If you're running remote/headless it's probably easier to **SSH** into the exoskeleton computer.

Instead of running this command in every terminal you open, you can do this automatically for every terminal on a computer by `adding this command to your <https://answers.ros.org/question/206876/how-often-do-i-need-to-source-setupbash/?answer=206976#post-id-206976>`_
:code:`~/.bashrc`

ROS IP
--------------
To use ros commands you also need to setup the right `ROS_IP <http://wiki.ros.org/ROS/EnvironmentVariables>`_ in every terminal you use: ::

    export ROS_MASTER_URI=<your ip>

**<your ip>** can be obtained by running the command :code:`hostname -I` in a terminal on your computer.

Instead of running this command in every terminal you open, you can do this automatically for every terminal on a computer by `adding this command to your <https://answers.ros.org/question/206876/how-often-do-i-need-to-source-setupbash/?answer=206976#post-id-206976>`_
:code:`~/.bashrc`

Add March network to hosts file
-------------------------------
You need to add the march network with ip to your hosts file:

- Type :code:`sudo nano /etc/hosts` in your terminal.
- add the line :code:`192.168.137.253       march` at the top of the file.
- Close and save the file with **ctrl + x**.

Test ROS on multiple devices
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
This is a very simple test to make sure you followed :ref:`preparation-before-connecting-label` correctly.

- Run :code:`roscore` in a terminal on the |M4| exoskeleton computer (you can always do this via SSH).
- Run :code:`rostopic list` in a terminal on your computer. This command should give the output **/rosout /rosout_agg**
- Terminate the :code:`rostopic list` command by pressing **ctrl + c**.
- Run :code:`rostopic echo /march/test` in a new terminal on the |M4| exoskeleton computer. This command gives a warning.
- Now publish a message with the command :code:`rostopic pub /march/test std_msgs/String "data: 'Hello World'"`
- The message is send and received successful if you received the message in the |M4| exoskeleton computer.

