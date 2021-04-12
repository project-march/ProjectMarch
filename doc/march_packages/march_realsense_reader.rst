.. _march-realsense_reader-label:

march_realsense_reader
======================

Overview
--------
The MARCH realsense reader package's goal is to obtain the dimensions of obstacles in a unknown environment. The MARCH VI
uses two `d435i <https://www.intelrealsense.com/depth-camera-d435i/>`_ camera's for this purpose.

ROS API
-------

Nodes
^^^^^
*realsense_reader_node* - Responsible for obtaining the pointlcouds from the camera's and processing them.

Subscribed Topics
^^^^^^^^^^^^^^^^^

*"/camera_front/depth/color/points"* sensor_msgs::PointCloud2
  The realsense front camera publishes on this topic.


Published Topics
^^^^^^^^^^^^^^^^
The package publisher several debug pointclouds and markers, these topics are purely for visualisation:

*/camera/preprocessed_cloud* (sensor_msgs::PointCloud2)
  The pointcloud outputted by the `preprocessor`. This cloud containts only the points in a xy-plane, and has a lower
  point density than the original cloud.

*/camera/region_cloud* (pcl::PointCloud<pcl::PointXYZRGB>)
  This topic contains a single region cloud, created by the `region_creator`. The points in the cloud are grouped into
  a 'region', indicated by a colour for every region.

*/camera/hull_marker_list* (visualization_msgs::Marker)
  Markers that visualise the boundary of regions. This boundary is created using a convex or concave hull. This hull is
  used to determine the possible foot locations.

*/camera/foot_locations_marker_array* (visualization_msgs::MarkerArray)
  Markers that indicate the points in the `preprocessed_cloud`, for which is it possible to move the foot of the
  exoskeleton. The most optimal of these locations is highlighted.

Services
^^^^^^^^
*/camera/process_pointcloud*
  Calls upon the `march_realsense_reader`. Outputs the `gait_parameters` from which a parametric gait can be constructed.

Parameters
^^^^^^^^^^
*/march/template/counter* (*int*, required)
  How many to count
*/march/template/countings* (*int[]*, default: [])
  List of countings


Tutorials
---------

Running the package in simulation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
It is recommended to run all commands in separate terminals. These tutorials use the convenient aliases from :ref:'_march_aliases-label`

.. code :: bash

    march_run_ros1 gazebo_ui:=true obstacle:=stairs ground_gait:=true realsense:=true
    march_run_ros2 ground_gait:=true
    march_run_bridge

This will start both RViz and Gazebo. We use Gazebo to model the stairs, and RViz to visualise the pointlcouds. In RViz
add a `pointcloud2`, and set the topic to `/camera/preprocessed_cloud` or `/camera/region_cloud`. Calling the service

.. code :: bash

  snoe && sros1 && rosservice call /camera/process_pointcloud "selected_gait: 0 frame_id_to_transform_to: 'foot_right'"

will result in a pointlcoud in RViz with regions indicated by colour.

Running with a camera
^^^^^^^^^^^^^^^^^^^^^
You need a camera for this example

FAQ
---

How do I x?
^^^^^^^^^^^
Please check the tutorials.

How do I z?
^^^^^^^^^^^
z is not available at the moment.