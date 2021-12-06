.. _ros-cpp-clion:

Index ROS1 & ROS2 in Clion
==========================
To get code completion and static analysis for ros code to work you need to open Clion a bit differently than usual.

First thing you need is to make sure you have a :code:`compile_commands.json` file in the build folder of either ROS1
or ROS2. This should be the case if the code is built with docker. If this is not the case see
`jetbrains clion ros setup tutorial <https://www.jetbrains.com/help/clion/ros-setup-tutorial.html>`_.

If it is the case, continue with the following steps:

#. Open a new terminal and source ROS1 or ROS2 (type :code:`snoe` or :code:`sfox`).
#. Open clion (type :code:`clion`)
    * Tip: you can type :code:`clion > /dev/null 2> /dev/null & disown` to be able to close the terminal, and not output any information to the terminal.
#. Open ROS1 or ROS2 by opening the :code:`compile_commands.json` file, and click on 'Open as Project'.
    * ROS1 open: :code:`~/march/ros1/build/compile_commands.json`
    * ROS2 open: :code:`~/march/ros2/build/compile_commands.json`
#. After the project is opened change your repository root:
    #. Click on 'Tools' in the top of clion (in the row of 'File Edit View ...')
    #. Hover over 'Complication Database'
    #. Click on 'Change Project Root', and select ROS1 or ROS2.

Now Clion should have correctly index your ROS1 or ROS2. Goodluck and have fun "code knallen" :).

