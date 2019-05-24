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

# Build entire workspace
catkin build --summarize  --no-notify|| build_failed "Could not build workspace"
source devel/setup.bash

# Catkin lint
catkin lint -W2 --pkg march_rqt_gait_generator || build_failed "Catkin lint failed in march_rqt_gait_generator"


# Roslint
# march_description and march_launch do not need to be roslinted as they don't contain any code.
catkin build --no-deps --verbose march_rqt_gait_generator --no-notify --catkin-make-args roslint || build_failed "Roslint failed in march_rqt_gait_generator"


# Run all tests in the workspace, including roslaunch-checks if they exist
catkin build --summarize --catkin-make-args run_tests && catkin_test_results build/ --verbose || build_failed "Tests failed"
