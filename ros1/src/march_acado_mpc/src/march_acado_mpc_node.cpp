#include <iostream>
#include "ros/ros.h"
#include "acado_mpc.h"


using namespace std;

const bool VERBOSE = 0; // Print debug information: 1, silent: 0.

int main(int argc, char** argv ) {

    // declare variables
    vector<string> joint_list;

    // Initialize the node with as third argument the name
    ros::init(argc, argv, "march_acado_mpc_node");

    // Create a node handle
     ros::NodeHandle nh;

    // Check if the ROS parameter /march/joint_names is available
//    while (ros::ok() && !ros::param::has("/march/joint_names")) {
//        ros::Duration(0.5).sleep();
//        ROS_DEBUG("Waiting on /march/joint_names to be available");
//    }

    if (!ros::ok()) {
        return 0;
    }

    // get joint names
    ros::param::get("/march/joint_names", joint_list);

    // run your controller function
    ModelPredictiveController mpc(joint_list);
    mpc.initSolver();

    vector<double> x0 {0.0,0.0};
//    real_t* x_next;
//    real_t* u;

//    for (int iter = 0; iter < 1000; iter++)
//    {
//        cout << x0[0] << ", " << x0[1] << endl;

    mpc.controller(x0);
//
//        x_next = acado_getVariablesX( );
//        u = acado_getVariablesU( );
//        x0[0] = x_next[2];
//        x0[1] = x_next[3];
//
////        cout << u[0] << endl;
//
//    }


    mpc.printDebug(VERBOSE);

    ros::spin();


    return 0;
}