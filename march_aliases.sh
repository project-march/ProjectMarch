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
alias march_run_ros2_training='sfox && sros2 && ros2 launch march_launch march.launch.py'


# Build and run the bridge
alias march_build_bridge='snoe && sfox && sros1 && sros2 && cd ~/ros1_bridge && colcon build --packages-select ros1_bridge --cmake-force-configure --symlink-install && source install/local_setup.bash && ros2 run ros1_bridge dynamic_bridge --print-pairs'
alias march_run_bridge='snoe && sfox && sros1 && sros2 && cd ~/ros1_bridge && source install/local_setup.bash && export ROS_MASTER_URI=http://localhost:11311 && ros2 run ros1_bridge parameter_bridge'


# Shorter aliases
alias mb1='march_build_ros1'
alias mb2='march_build_ros2'
alias bridge='march_run_bridge'
alias b='march_run_bridge'


# Training aliases
export URDF='robot_description:=march6_three_cameras'
alias sim1='march_run_ros1_sim $URDF point_finder:=true realsense_simulation:=true ground_gait:=true'
alias sim2='march_run_ros2_sim $URDF realsense_simulation:=true ground_gait:=true use_imu_data:=false'

alias ag1='march_run_ros1_airgait $URDF point_finder:=true ground_gait:=true use_imu_data:=true'
alias ag2='march_run_ros2_training $URDF use_imu_data:=true'

alias gg1='ag1 gain_tuning:=ground_gait'
alias gg2='ag2'


# Tools for during monitor sessions 
alias march_run_monitor='sfox && sros2 && ros2 launch march_monitor monitor.launch.py'
alias monitor="march_run_monitor"
alias multiplot="asrock_ros1 && rqt_multiplot"
alias recon="snoe && sros1 && rosrun rqt_reconfigure rqt_reconfigure"
alias recon2="sfox && sros2 && ros2 run rqt_reconfigure rqt_reconfigure"


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
alias export_asrock_master_uri='export ROS_MASTER_URI=http://192.168.1.253:11311/'
alias asrock_ros1="snoe && sros1 && export_asrock_master_uri"
alias asrock_ros2="sfox && sros2 && export_asrock_master_uri"
