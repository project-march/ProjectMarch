# Source ROS distribution
alias snoe='source /opt/ros/noetic/local_setup.bash'
alias sfox='source /opt/ros/foxy/local_setup.bash'
alias svenv='source ~/march/.venv_march/bin/activate'


# Source MARCH packages
alias sros1='source ~/march/ros1/install/local_setup.bash'
alias sros2='source ~/march/ros2/install/local_setup.bash'


# Navigate to MARCH directory
alias cm='cd ~/march/'
alias cm1='cd ~/march/ros1/'
alias cm2='cd ~/march/ros2/'


# Build and run ROS1
alias march_build_ros1='snoe && cm1 && colcon build --symlink-install'
alias march_run_ros1_sim='snoe && sros1 && roslaunch march_launch march_simulation.launch'
alias march_run_ros1_airgait='snoe && sros1 && roslaunch march_launch march.launch if_name:=enp2s0f0'
alias march_run_ros1_groundgait='march_run_ros1_airgait ground_gait:=true gain_tuning:=groundgait'
alias march_run_ros1_training='march_run_ros1_groundgait gain_tuning:=training'


# Build and run ROS2
alias march_build_ros2='sfox && cm2 && colcon build --symlink-install'
alias march_run_ros2_sim='sfox && sros2 && ros2 launch march_launch march_simulation.launch.py'
alias march_run_ros2_training='sfox && sros2 && ros2 launch march_launch march.launch.py rqt_input:=false'


# Build and run the bridge
alias march_build_bridge='snoe && sfox && sros1 && sros2 && cd ~/ros1_bridge && colcon build --packages-select ros1_bridge --cmake-force-configure --symlink-install && source install/local_setup.bash && ros2 run ros1_bridge dynamic_bridge --print-pairs'
alias march_run_bridge='snoe && sfox && sros1 && sros2 && cd ~/ros1_bridge && source install/local_setup.bash && export ROS_MASTER_URI=http://localhost:11311 && ros2 run ros1_bridge parameter_bridge'


# Shorter aliases
alias mb1='march_build_ros1'
alias mb2='march_build_ros2'
alias sim1='march_run_ros1_sim'
alias sim2='march_run_ros2_sim'
alias bridge='march_run_bridge'
alias b='march_run_bridge'


# Training aliases
export URDF6='robot:=march6_three_cameras'

alias cov1='march_run_ros1_sim point_finder:=true realsense_simulation:=true ground_gait:=true'
alias cov2='march_run_ros2_sim realsense_simulation:=true ground_gait:=true use_imu_data:=false'

alias ag1='march_run_ros1_airgait point_finder:=true ground_gait:=true use_imu_data:=true'
alias ag2='march_run_ros2_training use_imu_data:=true wireless_ipd:=true'

alias gg1='ag1 gain_tuning:=ground_gait'
alias gg2='ag2'


# Tools for during monitor sessions 
alias march_run_monitor='sfox && sros2 && ros2 launch march_monitor monitor.launch.py'
alias monitor="march_run_monitor"
alias multiplot="asrock_ros1 && rqt_multiplot"
alias recon1="snoe && sros1 && rosrun rqt_reconfigure rqt_reconfigure"
alias recon2="sfox && sros2 && ros2 run rqt_reconfigure rqt_reconfigure"
alias left="sfox && sros2 && ros2 topic echo /march/foot_position/left | grep -A 3 'displacement'"
alias right="sfox && sros2 && ros2 topic echo /march/foot_position/right | grep -A 3 'displacement'"


# Clean march builds
# script to ask for confirmation before cleaning ros
confirm() {
    echo -n "Do you want to run $*? [N/y] "
    read -N 1 REPLY
    echo
    if test "$REPLY" = "y" -o "$REPLY" = "Y"; then
        "$@"
    else
        echo "Cancelled by user"
    fi
}
alias march_clean_ros1='confirm rm -rf ~/march/ros1/build ~/march/ros1/log ~/march/ros1/install'
alias march_clean_ros2='confirm rm -rf ~/march/ros2/build ~/march/ros2/log ~/march/ros2/install'
alias march_clean_bridge='confirm rm -rf ~/ros1_bridge/build ~/ros1_bridge/log ~/ros1_bridge/install'
alias march_clean_all='march_clean_ros1 && march_clean_ros2 && march_clean_bridge'


# To give errors colors in ros2
export RCUTILS_COLORIZED_OUTPUT=1


# Install dependencies
alias install_dep_ros1='cm1 && snoe && rosdep install --from-paths src --ignore-src -y --rosdistro noetic'
alias install_dep_ros2='cm2 && sfox && rosdep install --from-paths src --ignore-src -y --rosdistro foxy'


# Format code
alias format_cpp='cm && python3 .scripts/run-clang-format.py -r ros1/src ros2/src --style=file -i'
alias format_py='cm && black .'

# Static analysis shortcuts (needs clang-tidy: `sudo apt-get install -y clang-tidy`)
alias march_static_analysis_ros1='echo "Running analysis, this can take 77 seconds" && find ~/march/ros1/src -name "*.hpp" -or -name "*.h" -or -name "*.cpp" -or -name "*.c" | grep -v "src/libraries" | grep -v "xsens" | xargs -L1 -P4 -I{} -- clang-tidy -p ~/march/ros1/build {} 2> /dev/null; true && echo -e "\n----done---"'
alias march_static_analysis_ros1_here='echo "Running analysis, on files in folder $(pwd)" && find . -name "*.hpp" -or -name "*.h" -or -name "*.cpp" -or -name "*.c" | grep -v "src/libraries" | grep -v "xsens" | xargs -L1 -P4 -I{} -- clang-tidy -p ~/march/ros1/build {} 2> /dev/null; true && echo -e "\n----done---"'

alias march_static_analysis_ros2='echo "Running analysis, this can take 77 seconds" && find ~/march/ros2/src -name "*.hpp" -or -name "*.h" -or -name "*.cpp" -or -name "*.c" | grep -v "src/libraries" | grep -v "xsens" | xargs -L1 -P4 -I{} -- clang-tidy -p ~/march/ros2/build {} 2> /dev/null; true && echo -e "\n----done---"'
alias march_static_analysis_ros2_here='echo "Running analysis, on files in folder $(pwd)" && find . -name "*.hpp" -or -name "*.h" -or -name "*.cpp" -or -name "*.c" | grep -v "src/libraries" | grep -v "xsens" | xargs -L1 -P4 -I{} -- clang-tidy -p ~/march/ros2/build {} 2> /dev/null; true && echo -e "\n----done---"'

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

# Start Clion & PyCharm with no consol output
alias pycharm_no_out='pycharm-professional > /dev/null 2> /dev/null & disown'
alias clion_no_out='clion > /dev/null 2> /dev/null & disown'


# Hardware aliases
alias odrive='cd ~/projects/odrivepython && python main.py'
alias odrive_temp='export_asrock_master_uri && snoe && sros1 && rostopic echo /march/motor_controller/states/temperature'
alias battery='snoe && sros1 && rostopic echo /march/pdb_data/battery_state -n 1'
alias pdb_echo='snoe && sros1 && rostopic echo /march/pdb_data'
alias pdb='snoe && sros1 && rostopic echo /march/pdb_data -n 1'


# Sourcing ROS distributions on the Asrock
alias export_asrock_master_uri='export ROS_MASTER_URI=http://192.168.1.177:11311/'
alias asrock_ros1="snoe && sros1 && export_asrock_master_uri"
alias asrock_ros2="sfox && sros2 && export_asrock_master_uri"

# Alias for telling a joke when Koen has to wait for a long time
alias joke='curl -H "Accept: text/plain" https://icanhazdadjoke.com/'
