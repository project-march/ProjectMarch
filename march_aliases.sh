# Source ROS distribution
alias sfox='source /opt/ros/foxy/local_setup.bash'
alias svenv='source ~/march/.venv_march/bin/activate'

# Source MARCH packages
alias sros2='source ~/march/ros2/install/local_setup.bash'


# Navigate to MARCH directory
alias cm='cd ~/march/'
alias cm2='cd ~/march/ros2/'

# Create xml needed for mujoco
alias parse_xml='cd ~/march/ros2/src/shared/march_description/urdf/ && python3 march8_v0_generate.py && python3 march7_v1_generate.py && cm2'

# Build and run ROS2
alias march_build_ros2='sfox && cm2 && colcon build --symlink-install'
alias march_run_ros2_sim='sfox && sros2 && ros2 launch march_launch march_simulation.launch.py'
alias march_run_ros2_training='sudo -v && sfox && sros2 && ros2 launch march_launch march.launch.py rqt_input:=false'

alias march_build_all='sfox && cm2 && pushd src/libraries/bluespace_ai_xsens_ros_mti_driver/lib/xspublic && make && popd && colcon build --symlink-install'
alias march_build='march_build_all --packages-skip control_msgs control_toolbox \
controller_interface controller_manager controller_manager_msgs diagnostic_updater diff_drive_controller effort_controllers \
force_torque_sensor_broadcaster forward_command_controller gazebo_ros2_control gazebo_ros2_control_demos gripper_controllers \
hardware_interface imu_sensor_broadcaster joint_state_broadcaster joint_state_controller joint_trajectory_controller \
position_controllers realsense_gazebo_plugin realtime_tools ros2_control ros2_control_test_assets ros2_controllers \
ros2bag rosbag2 rosbag2_compression rosbag2_compression_zstd rosbag2_cpp rosbag2_performance_benchmarking rosbag2_py \
rosbag2_storage rosbag2_storage_default_plugins rosbag2_test_common rosbag2_tests rosbag2_transport shared_queues_vendor \
ros2controlcli soem transmission_interface velocity_controllers'

# Shorter aliases
alias mb2='march_build_ros2'
alias mb='march_build'
alias mba='march_build_all'
alias gits='git status'

# NEW M9 ALIASES
alias plt='ros2 run plotjuggler plotjuggler -l src/march_launch/launch/joint_angles_plotjuggler.xml'
alias convert_db3_to_mcap='cm && sros2 && ./utility_scripts/convert_db3_to_mcap.sh'
alias tshark_record='cm2 && ../utility_scripts/tshark_recorder.sh'

alias izzy_launch='cm2 && sros2 && sfox && ros2 launch march_launch izzy.launch.py model_to_load_mujoco:="march9.xml" aie_force:="true"'

alias test_joints='cm2 && sros2 && sfox && ros2 launch march_launch test_joints.launch.py model_to_load_mujoco:="march9.xml"'

alias test_mpc='cm2 && sros2 && sfox && ros2 launch march_launch mpc.launch.py model_to_load_mujoco:="march8_v0_aie_v0.xml" rviz:=true'

alias change_gaits='cd ~/march && /bin/python3 /home/control/march/utility_scripts/change_gaits_gui.py'

# Alias to build one package, appended with specified package
alias mbp='mba --packages-select'

# Alias to build packages with dependencies
alias mbpa='mba --packages-up-to'

# Alias to run tests on a package
alias mbt='mbt'
mbt() {
  if [ $# -eq 0 ]; then
    echo "Please specify a package to test"
    return 1
  fi
  colcon build --packages-select "$@"
  colcon test --packages-select "$@" && colcon test-result --verbose
}

# Tools for during monitor sessions 
alias march_run_monitor='sfox && sros2 && ros2 launch march_monitor monitor.launch.py'
alias monitor="march_run_monitor"
alias multiplot="sfox && sros2 && ros2 launch march_plotjuggler_launcher plotjuggler.launch.py"
alias recon2="sfox && sros2 && ros2 run rqt_reconfigure rqt_reconfigure"
alias left="sfox && sros2 && ros2 topic echo /march/foot_position/left | grep -A 3 'displacement'"
alias right="sfox && sros2 && ros2 topic echo /march/foot_position/right | grep -A 3 'displacement'"

# run tetst setup
alias run_test_setup_rotational='sudo -v && sfox && sros2 &&  ros2 launch march_launch march.launch.py robot:=test_joint_rotational rviz:=false gait_directory:=test_joint_rotational_gaits layout:=test_joint realsense:=false control_yaml:='\''effort_control/test_joint_rotational_control.yaml'\'''
alias run_test_setup_linear='sudo -v && sfox && sros2 &&  ros2 launch march_launch march.launch.py robot:=test_joint_linear rviz:=false gait_directory:=test_joint_linear_gaits layout:=test_joint realsense:=false control_yaml:='\''effort_control/test_joint_linear_control.yaml'\'''

alias run_test_setup_pressure_sole='sudo -v && sfox && sros2 &&  ros2 launch march_launch march.launch.py robot:=test_pressure_sole rviz:=false realsense:=false control_yaml:='\''effort_control/pressure_sole_control.yaml'\'''

alias m8_test_rotational='sudo -v && sfox && sros2 &&  ros2 launch march_launch start_test_setup.launch.py test_rotational:=true'
alias m8_test_linear='sudo -v && sfox && sros2 &&  ros2 launch march_launch start_test_setup.launch.py test_rotational:=false'

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
alias march_clean_ros2='confirm rm -rf ~/march/ros2/build ~/march/ros2/log ~/march/ros2/install'
alias march_clean='find ~/march/ros2/build ~/march/ros2/log ~/march/ros2/install -maxdepth 1 -name "march*" -type d -print0 | xargs -0 rm -r --'


# To give errors colors in ros2
export RCUTILS_COLORIZED_OUTPUT=1


# Install dependencies
alias install_dep_ros2='cm2 && sfox && rosdep install --from-paths src --ignore-src -y --rosdistro foxy'


# Format code
# 'sudo apt install clang-format' should be run for format_cpp
alias format_cpp='cm && python3 .scripts/run-clang-format.py -r ros2/src --style=file -i'
alias format_py='cm &&flake8 .'
#alias format_py='cm && black .'
alias format='format_cpp && format_py'

# Static analysis shortcuts (needs clang-tidy: `sudo apt-get install -y clang-tidy`)
alias march_static_analysis_ros2='echo "Running analysis, this can take 88 seconds" && find src -name '*.hpp' -or -name '*.h' -or -name '*.cpp' -or -name "*.c" | grep -v "src/libraries" | grep -v "xsens" | xargs -L1 -P4 -I{} -- clang-tidy -p build {} 2> /dev/null && echo -e "\n----done---"'
alias march_static_analysis_ros2_here='echo "Running analysis, on files in folder $(pwd)" && find . -name "*.hpp" -or -name "*.h" -or -name "*.cpp" -or -name "*.c" | grep -v "src/libraries" | grep -v "xsens" | grep -v "cmake-build-debug" | xargs -L1 -P4 -I{} -- clang-tidy -p ~/march/ros2/build {} 2> /dev/null; true && echo -e "\n----done---"'
# Flake8 shortcuts (python code style checker)
alias march_flake8_update='FLAKE8_GIT="registry.gitlab.com/project-march/march/flake8:dev" && \
docker pull $FLAKE8_GIT && docker tag $FLAKE8_GIT march/flake8 && docker rmi $FLAKE8_GIT'
alias march_flake8='docker run --rm -v ~/march:/home/march:ro march/flake8'
alias march_flake8_here='docker run --rm -v `pwd`:`pwd`:ro march/flake8 `pwd`'

# Black shortcuts (python code formatter)
alias march_py_auto_format='docker run --rm -v ~/march:/home/march --entrypoint black march/flake8 ros2/src utility_scripts/'
alias march_py_auto_format_check='docker run --rm -v ~/march:/home/march:ro --entrypoint black march/flake8 \
--check --diff --color ros2/src utility_scripts/'
alias march_py_auto_format_here="docker run --rm -v `pwd`:`pwd` --entrypoint black march/flake8 `pwd` -l 120 --extend-exclude '^/.*/libraries/'"
alias march_py_auto_format_check_here="docker run --rm -v `pwd`:`pwd`:ro --entrypoint black march/flake8 -l 120 --extend-exclude '^/.*/libraries/' --check --diff --color `pwd`"

# Docs shortcuts
alias build_docs='cd ~/march/docs && bash ./build_locally.sh'
alias proof_docs='cd ~/march/docs && htmlproofer build --check-sri --enforce-https --only-4xx --ignore-files "build/search.html" --ignore-urls '#''

# Start Clion & PyCharm with no consol output
alias pycharm_no_out='pycharm-professional > /dev/null 2> /dev/null & disown'
alias clion_no_out='clion > /dev/null 2> /dev/null & disown'


# Hardware aliases
alias odrive='cd ~/projects/odrivepython && python main.py'

# Sourcing ROS distributions on the Asrock
alias export_asrock_master_uri='export ROS_MASTER_URI=http://192.168.1.177:11311/'
alias asrock_ros2="sfox && sros2 && export_asrock_master_uri"

FASTRTPS_DEFAULT_PROFILES_FILE=~/march/.fastrtps-profile.xml
export FASTRTPS_DEFAULT_PROFILES_FILE

# Alias for telling a joke when Koen has to wait for a long time
alias joke='curl -H "Accept: text/plain" https://icanhazdadjoke.com/ && echo'
#
## Aliases for performing IMU debugs
alias kernel_devboard="sudo /sbin/modprobe ftdi_sio && echo 2639 0300 | sudo tee /sys/bus/usb-serial/drivers/ftdi_sio/new_id"
alias dialout_grp="sudo usermod -G dialout -a $USER && newgrp dialout"

alias static_port_rules='cf'
cf() {
  sudo su -c "echo 'ACTION==\"add\", ATTRS{idVendor}==\"2639\", ATTRS{idProduct}==\"0300\", ATTRS{serial}==\"DBZ05NEA\", SYMLINK+=\"lowerIMU\"' > /etc/udev/rules.d/98-imu.rules"
  sudo su -c "echo 'ACTION==\"add\", ATTRS{idVendor}==\"2639\", ATTRS{idProduct}==\"0300\", ATTRS{serial}==\"DBZ03F66\", SYMLINK+=\"upperIMU\"' >> /etc/udev/rules.d/98-imu.rules"
  sudo udevadm control --reload
}
