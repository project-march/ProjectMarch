#include <iostream>
#include "ros/ros.h"
#include "acado_mpc.h"


using namespace std;

const bool VERBOSE = 1; // Print debug information: 1, silent: 0.

int main(int argc, char** argv ) {

    // declare variables
    vector<string> joint_list;

    // Initialize the node with as third argument the name
    ros::init(argc, argv, "march_acado_mpc_node");

    // Create a node handle
     ros::NodeHandle nh;

    // Check if the ROS parameter /march/joint_names is available
    while (ros::ok() && !ros::param::has("/march/joint_names")) {
        ros::Duration(0.5).sleep();
        ROS_DEBUG("Waiting on /march/joint_names to be available");
    }

    if (!ros::ok()) {
        return 0;
    }

    // get joint names
    ros::param::get("/march/joint_names", joint_list);

    // run your controller function
    ModelPredictiveController mpc(joint_list);
    mpc.initSolver();

    vector<double> x0 {1.0,2.0};
    mpc.controller(x0);

    mpc.printDebug(VERBOSE);

    ros::spin();


    return 0;
}