.. _march_realsense_test_publisher-label:

march_realsense_test_publisher
==============================

The march_realsense_test_publisher publishes fake realsense pointclouds to simulate the RealSense camera when running a simulation of the exoskeleton.
This is a service (service_name = /camera/publish_test_cloud) so it will only be executed when called upon.

How to use
==========
Run ROS1 simulation with RealSense:

.. code::

  march_run_ros1_sim realsense:=true
  
Call upon the service with arguments using the following format:

.. code::

  rosservice call /camera/publish_test_cloud "selected_mode: 0"; "pointcloud_file_name: 'stairs1.ply'"; " save_camera_back: false"

Arguments
""""""""""
enum SelectedMode { next = 0, end = 1, custom = 2, save = 3 }
    Select "next" to cycle through the pointcloud files.
    Select "end" to stop publishing pointclouds.
    elect "custom" to publish a custom pointcloud specified in the next argument.
    Select "save" to save the current pointcloud as filename specified in the next argument.

string pointcloud_file_name: <file_from_config/datasets>
    Specify which file to publish or specify the name of the file to save to.

bool save_camera_back
    Whether to save the pointcloud from the back-RealSense instead of the front.

