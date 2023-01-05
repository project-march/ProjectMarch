.. _ros-cpp-clion:

Index ROS2 in JetBrains (Clion & PyCharm)
================================================
When your environment is properly indexed, you will be able to see the type and usage of a certain bit of text in the rest of the code. It makes finding dependencies a lot easier and less confusing. In Visual Studio Code, this is partly done by sourcing, but in CLion and PyCharm, it needs a bit of setting up. Use the following guides for that.

Index ROS2 in Clion
^^^^^^^^^^^^^^^^^^^^^^^^^^
To get code completion and static analysis for ROS code to work, you need to open Clion a bit differently than usual.

First thing you need is to make sure you have a :code:`compile_commands.json` file in the build folder of either ROS2.
This should be the case if the code is built with docker. If this is not the case, see
`jetbrains clion ros setup tutorial <https://www.jetbrains.com/help/clion/ros-setup-tutorial.html>`_.

If there is already such a file, continue with the following steps:

#. Open a new terminal and source ROS2 (type :code:`sfox`).
#. Open clion (type :code:`clion`)
    * Tip: you can type :code:`clion > /dev/null 2> /dev/null & disown` to be able to close the terminal, and not output any information to the terminal.
#. OpenROS2 by opening the :code:`compile_commands.json` file, and click on 'Open as Project'.   * ROS1 open: :code:`~/march/ros1/build/compile_commands.json`
    * ROS2 open: :code:`~/march/ros2/build/compile_commands.json`
#. After the project is opened change your repository root:
    #. Click on 'Tools' in the top of clion (in the row of 'File Edit View ...')
    #. Hover over 'Compilation Database'
    #. Click on 'Change Project Root', and select March.

Now Clion should have correctly indexed your ROS2. Good luck and have fun with "popping code".

Index ROS2 in PyCharm
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
To get code completion and static analysis for ROS code to work, you need to open PyCharm a bit differently than usual.

First-time setup steps:

#. Create a virtual environment with the bash script.
    #. :code:`cd march`
    #. :code:`bash create_venv.sh -n -clean`
        * The :code:`-n` erases the one if there is already there.
        * The :code:`-clean` redownloads the packages that are not installed by 'apt'
#. Build your ros source code :code:`march_build`.
#. Source the virtual environment and ROS
    #. :code:`svenv` or :code:`source ~/march/.venv_march/bin/activate`
    #. source ROS (:code:`sfox`)
        * For ROS2: :code:`sfox`
#. Open PyCharm with :code:`pycharm-professional`. (Important: **Do this in the same terminal as step 3**).
 * Tip: you can type :code:`pycharm-professional > /dev/null 2> /dev/null & disown` to be able to close the terminal, and not output any information to the terminal.
#. Select folder for ROS2.
#. Exclude parts of the built march code to prevent duplicate code fragment errors
    #. Go to File | Settings | Project: docs | Project Structure
        #. Click on "File" in the top left of pycharm.
        #. Click on "Settings" in the dropdown menu of "File".
        #. Go to "Project: [name of project]" -> "project settings".
    #. Select "build", "log", "install" and click on "Excluded".
        Now on the left you will see these folders as excluded,
        and you will also see some files in the install folder listed as "source"

For everyday startup, start the pycharm up as in step 3 & 4.

