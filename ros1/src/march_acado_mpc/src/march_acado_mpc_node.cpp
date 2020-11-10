#include <iostream>
#include "ros/ros.h"
#include "acado_mpc.h"

using namespace std;

int main(int argc, char** argv ) {

    // Initialize the node with as third argument the name
    ros::init(argc, argv, "march_acado_mpc_node");
//    ros::Time::init();
    // Create a node handle
     ros::NodeHandle nh;

    // Check if the ROS parameter /march/joint_names is available
    // http://wiki.ros.org/roscpp/Overview/Parameter%20Server
//    while (ros::ok() && !ros::param::has("/march/joint_names")) {
//        ros::Duration(0.5).sleep();
////        ROS_DEBUG("Waiting on /march/joint_names to be available");
//        cout << "Waiting on /march/joint_names to be available" << endl;
//    }

    if (!ros::ok()) {
        return 0;
    }

    // Run your controller function
    cout << "march_acado_mpc_node has been launched!" << endl;
    ModelPredictiveController mpc;

    ros::spin();


    return 0;
}