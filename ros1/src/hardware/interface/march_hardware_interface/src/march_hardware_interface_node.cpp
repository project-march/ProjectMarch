// Copyright 2019 Project March.
#include "march_hardware_interface/march_hardware_interface.h"

#include <march_hardware/error/hardware_exception.h>
#include <march_hardware/march_robot.h>
#include <march_hardware_builder/hardware_builder.h>

#include <cstdlib>

#include <controller_manager/controller_manager.h>
#include <ros/ros.h>

std::unique_ptr<march::MarchRobot> build(
    AllowedRobot robot, bool remove_fixed_joints_from_ethercat_train);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "march_hardware_interface");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(/*thread_count=*/2);

    if (argc < 2) {
        ROS_FATAL("Missing robot argument\nusage: "
                  "march_hardware_interface_node ROBOT");
        return 1;
    }
    AllowedRobot selected_robot = AllowedRobot(argv[1]);
    ROS_INFO_STREAM("Selected robot: " << selected_robot);

    bool reset_imc = ros::param::param<bool>("~reset_imc", false);
    bool remove_fixed_joints_from_ethercat_train;
    if (ros::param::has("~remove_fixed_joints_from_ethercat_train")) {
        ros::param::get("~remove_fixed_joints_from_ethercat_train", remove_fixed_joints_from_ethercat_train)
    } else {
        ROS_ERROR("Required parameter remove_fixed_joints_from_ethercat_train"
                  " was not set.")
    }

    spinner.start();

    MarchHardwareInterface march(
        build(selected_robot, remove_fixed_joints_from_ethercat_train),
        reset_imc);

    try {
        bool success = march.init(nh, nh);
        if (!success) {
            std::exit(/*__status=*/1);
        }
    } catch (const std::exception& e) {
        ROS_FATAL("Hardware interface caught an exception during init");
        ROS_FATAL("%s", e.what());
        std::exit(/*__status=*/1);
    }

    controller_manager::ControllerManager controller_manager(&march, nh);
    ros::Time last_update_time = ros::Time::now();

    while (ros::ok()) {
        try {
            march.waitForPdo();

            const ros::Time now = ros::Time::now();
            ros::Duration elapsed_time = now - last_update_time;
            last_update_time = now;

            march.read(now, elapsed_time);
            march.validate();
            controller_manager.update(now, elapsed_time);
            march.write(now, elapsed_time);
        } catch (const std::exception& e) {
            ROS_FATAL("Hardware interface caught an exception during update");
            ROS_FATAL("%s", e.what());
            return 1;
        }
    }

    return 0;
}

std::unique_ptr<march::MarchRobot> build(
    AllowedRobot robot, bool remove_fixed_joints_from_ethercat_train)
{
    HardwareBuilder builder(robot, remove_fixed_joints_from_ethercat_train);
    try {
        return builder.createMarchRobot();
    } catch (const std::exception& e) {
        ROS_FATAL(
            "Hardware interface caught an exception during building hardware");
        ROS_FATAL("%s", e.what());
        std::exit(/*__status=*/1);
    }
}
