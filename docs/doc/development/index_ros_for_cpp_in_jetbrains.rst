.. _ros-cpp-clion:

Index ROS1 & ROS2 in JetBrains (Clion & PyCharm)
================================================


Index ROS1 & ROS2 in Clion
^^^^^^^^^^^^^^^^^^^^^^^^^^
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

Index ROS1 & ROS2 in PyCharm
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
To get code completion and static analysis for ros code to work you need to open PyCharm a bit differently than usual.
Sadly to get it to work wel you need to open either ros1 or ros2 everytime.

First time setup steps:

#. Create a virtual environment with the bash script.
    #. :code:`cd march`
    #. :code:`bash create_venv.sh -n -clean`
        * The :code:`-n` erases the one if there is already there.
        * The :code:`-clean` redownloads the packages that are not installed by 'apt'
#. Build your ros source code :code:`march_build`.
#. Source the virtual environment and ros
    #. :code:`svenv` or :code:`source ~/march/.venv_march/bin/activate`
    #. source ros (:code:`snoe` or :code:`sfox`)
        * For ros1: :code:`snoe`
        * For ros2: :code:`sfox`
#. Open pycharm with :code:`pycharm-professional`. (Important to **do this in the same terminal as step 3**).
#. Select folder for ros1 or ros2.
#. Exclude most build files for duplicate code fragment errors
    #. Go to File | Settings | Project: docs | Project Structure
        #. Click on "File" in the top left of pycharm.
        #. Click on "Settings" in the dropdown menu of "File".
        #. Go to "Project: [name of project]" -> "project settings".
    #. Select "build", "log", "install" and click on "Excluded".
        Now on the left you will see these folders as excluded,
        and you will also see some files in the install folder listed as "source"

For everyday startup, start the pycharm up as in step 3 & 4.
Switch between ros1 and ros2 by closing pycharm, resourcing, and switch with the help of File | Open Recent | [name of project].

