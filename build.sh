#!/usr/bin/env bash

build_passed () {
    notify "Your local build has passed!"
}


build_failed () {
    notify "$1"
    exit 1
}

notify () {
    if [[ -x "$(command -v notify-send)" ]]; then
        notify-send -i $PWD/walking.png "$1"
    fi
}

# Enable test coverage
catkin config --cmake-args -DENABLE_COVERAGE_TESTING=ON -DCMAKE_BUILD_TYPE=Debug

# Build entire workspace
catkin build --summarize  --no-notify|| build_failed "Could not build workspace"
source devel/setup.bash

# Catkin lint
catkin lint -W2 --pkg march_hardware || build_failed "Catkin lint failed in march_hardware"
catkin lint -W2 --pkg march_hardware_builder || build_failed "Catkin lint failed in march_hardware_builder"
catkin lint -W2 --pkg march_hardware_interface || build_failed "Catkin lint failed in march_hardware_interface"
catkin lint -W2 --pkg march_temperature_sensor_controller || build_failed "Catkin lint failed in march_temperature_sensor_controller"
catkin lint -W2 --pkg march_hardware_state_controller || build_failed "Catkin lint failed in march_hardware_state_controller"

# Roslint
catkin build --no-deps --verbose march_hardware --no-notify --catkin-make-args roslint || build_failed "Roslint failed in march_hardware"
catkin build --no-deps --verbose march_hardware_builder --no-notify --catkin-make-args roslint || build_failed "Roslint failed in march_hardware_builder"
catkin build --no-deps --verbose march_hardware_interface --no-notify --catkin-make-args roslint || build_failed "Roslint failed in march_hardware_interface"
catkin build --no-deps --verbose march_temperature_sensor_controller --no-notify --catkin-make-args roslint || build_failed "Roslint failed in march_temperature_sensor_controller"
catkin build --no-deps --verbose march_hardware_state_controller --no-notify --catkin-make-args roslint || build_failed "Roslint failed in march_hardware_state_controller"

# Run all tests in the workspace, including roslaunch-checks if they exist
catkin build --summarize --catkin-make-args run_tests && catkin_test_results build/ --verbose || build_failed "Tests failed"
