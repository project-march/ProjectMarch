# Realsense Reader
The MARCH realsense reader package's goal is to obtain the dimensions of obstacles in a unknown environment. The MARCH VI
uses two [d435i](https://www.intelrealsense.com/depth-camera-d435i/) camera's for this purpose. This package
subscribes the the realsense camera topics and processes one of pointclouds published there with a service callback.

The processing makes heavy use of the [Point Cloud Library (PCL)](https://pointclouds.org/) to do the processing.
Many of its classes are used even when not directly using methods from PCL.

The processing consists of 4 main phases:

* Preprocess the point cloud

* Find regions in the preprocessed cloud

* Create hulls from the found regions

* Find parameters with which gaits can be interpolated

The package publishes the results of all the steps for visualization in RVIZ, if the debug parameter is set to true.

These steps are embodied by the interfaces ``Preprocessor``, ``RegionCreator``, ``HullFinder`` and ``ParameterDeterminer``.
These have implementations which execute the logic, these classes can do this in different ways.
We currently use the implementations ``NormalsPreprocessor``, ``RegionGrower``, ``CHullFinder`` and ``HullParameterDeterminer``.
There are currently no other implementations available, but the code architecture does allow for it.  

## ROS API
### How to build
```bash
source /opt/ros/noetic/local_setup.bash
# In your ros1 directory
colcon build --packages-select march_realsense_reader
```  
.. hint:: This node launches automatically

To disable the camera, simply provide the launch argument
```bash
realsense:=false
```  
The D435i camera's need to be specified by serial number. So, when changin hardware, provide the new serial numbers using the launch arguments:
```bash
serial_no_camera_front:= # The serial number for the new camera in the right wing
serial_no_camera_back:=  # The serial number for the new camera in the right hip
```  

### Nodes
`realsense_reader_node` - Responsible for obtaining the pointlcouds from the camera's and processing them.

### Services

```bash
/camera/process_pointcloud
```
  Calls upon the `march_realsense_reader`. Outputs the `gait_parameters` from which a parametric gait can be constructed.
  requires the realsense category, which camera to use and the subgait name. The complete service can be found [here](https://gitlab.com/project-march/march/-/blob/main/ros2/src/shared/march_shared_msgs/srv/GetGaitParameters.srv)

### Subscribed topics
`"/camera_front/depth/color/points" sensor_msgs::PointCloud2`
  The realsense front camera publishes its pointcloud on this topic.

`"/camera_back/depth/color/points" sensor_msgs::PointCloud2`
  The realsense back camera publishes its pointcloud on this topic

`"/test_clouds" sensor_msgs::PointCloud2`  
  The `march_realsense_test_publisher` publishes its saved clouds to this topic.

![Pointcloud from front camera](images/raw_stairs.png "Pointcloud obtained from a stairs, using the front camera in simualition")

### Published topics
The package publisher several debug pointclouds and markers, if the debug flag is enabled. These topics are purely for visualisation:

.. hint:: The debug flag is enabled by default  

`/camera/preprocessed_cloud (sensor_msgs::PointCloud2)`  
  The pointcloud outputted by the `preprocessor`. This cloud contains only the points part of a locally roughly flat area, and has a lower
  point density than the original cloud and is transformed to the frame id specified by the /camera/process_pointcloud service.

`/camera/region_cloud (sensor_msgs::PointCloud2)`  
  This topic contains a single region cloud, created by the `region_creator`. The points in the cloud are grouped into
  a 'region', indicated by a colour for every region. Red points are not part of any region.

`/camera/hull_marker_list (visualization_msgs::Marker)`  
  Markers that visualise the boundary of regions. This boundary is created using a convex or concave hull. This hull is
  used to determine the possible foot locations.

`/camera/hull_area_cloud (sensor_msgs::PointCloud2`  
  A pointcloud that is purely for debugging. It represents the the plane constructed inside the hulls. The potential foot locations are
  elevated to this plane. This debug cloud is created by cropping a large grid on the ground to the hulls.

`/camera/foot_locations_marker_array (visualization_msgs::MarkerArray)`  
  Markers that visualize the steps of the parameter determiner. The optional locations are in blue, the possible locations are in green
  and the optimal location is highlighted in white. The locations lack support, the markers are purple.

## Functionality
### NormalsPreprocessor
  The idea behind this step is that a point cloud can be noisy, large, and filled with points which are not necessarily relevant.
Preprocessing aims to reduce the point cloud size and the useless areas.
We currently do not remove statistical outliers as this proved too computationally expensive where preprocessing aims to increase time efficiency.  
  
Preprocessing consists of the following steps:  

  1. Randomly remove points from the point cloud.  
     This is done because the pointcloud consist of almost 1 million points and that creates a heavy computational load. Reducing the size by leaving point in a grid of a certain size is possible too, but that is more computationally expensive.

  2. Filter points which are too far away, or too close from the origin, as points far away are not relevant to the upcoming steps of the exoskeleton. The camera can see parts of the exoskeleton, like the right hip, or the fixtures and legs. Removing points that are too close prevents these parts of the exoskeloton intefering with later steps.
  
  3. Transform the point cloud such that its origin matches the origin of the world frame. This is done to all clouds, so that dimensions can be compared in the same reference frame.

  4. Estimate the normal vectors of the remaining point cloud.

  5. Filter based on the normal vectors, we remove points which do not have a normal in an upwards direction (e.g. points on a wall, z-direction in world frame) as we are not capable of standing there.  

  ![Cloud after preprocessing](images/preprocessed.png "Pointcloud after preprocessing")

### RegionGrower

  The idea behind finding regions is that in the preprocessed cloud there are certain areas which belong together.
Regions where it is possible to place the foot, like a step of a stair. Region finding finds these regions which belong together. Wether a point belongs to a region depends on their closes neighbours, and their normal as computed in the preprocessor.

![Cloud with colours for every region](images/regions.png "Cloud with colours for every region")

We use the region growing algorithm to find regions in the cloud which is explained in detail in
[The pcl tutorial on region growing segmentation](https://pcl.readthedocs.io/projects/tutorials/en/latest/region_growing_segmentation.html)

### CHullFinder

In order to find a potential foot location in the point cloud, a test is required to verify if the foot (or hip base in case of sitting down), can be placed at some location. In the current implementation, a step of this is determing wether a potenial location is inside a region. To this end we need to make an outline or hull of the regions.

To make these hulls we need to do the following for each region we received from the region creator:

  1. Find a plane which fits the region. This is needed because the considered regions are areas and not volumes so two dimensional hulls are used. This requires the regions to be 2D, this is done by projecting a region to a fitting plane.

  2. Project all the points in the region to the plane.

  3. Transform the projected points into a hull. This hull can either be made convex or concave hull, hence the name CHullFinder.
     Implentation is done via PCL.

  4. Add the found hull, the parameters of the plane corresponding to the hull and a vector of the indices to vectors for future use.

![Hulls and planes](images/hulls_planes.png "The computed hulls and planes")

### ParameterDeterminer

With the hulls and (or bounded planes), a location can be found depending on the subgait and realsense category.

The specifics of this class differ per obstacle. Consider for example the stairs gait:
when finding a parameter for the stairs gait, the foot can be placed anywhere in a bounded 2D plane in space (anywhere between
low-undeep, low-deep, high-undeep, high-deep stair) and therefore also two parameters need to be found, one specifying the
depth of the step, and one specifying the height of the step. This contrasts with the ramp gait for which only the angle of the slope is required. The curb and bench only require height.

For the stairs method this is done as follows:

  1. The stairs gait is interpolated from a low deep, high deep, low undeep & high undeep gait.
     We can place the foot anywhere in the area between the end locations of those gaits.
     These boundary points are created in a custom reference frame. This is a frame where the z-axis is aligend with the world frame, but is origin and yaw is shared with foot frame. Afterwords these points are transformed to the world frame.

  2. For a number of foot locations on the ground test whether there is a potential foot location at some height. These points are created as follows:

  	   1. Create points in front of the foot to move, this is a frame where the z-axis is aligend with the world frame, but is origin and yaw is shared with foot frame.

  	   2. These points are transformed to the world frame.

  3. For a number of foot locations on the ground test whether there is a potential foot location at some height.This gives optional foot locations. This is done by, for each region:

       1. Elevating the points on the ground to the plane of the region.

       2. Checking if the elevated points are inside or outside the hull.

       3. The points inside the hull are optional foot locations.

  4. For all the optional foot locations find which one is valid (reachable by the gaits, and sufficient support) and closest to some
     ideal location (the lowest, smallest possible end location for example).

  5. Transform this into a parameter by finding at what percentage of the existing gaits end locations the foot location is located.

For the ramp gait this is done as follows:

  1. Create points in front of the foot to move.

  2. Elevate these points to plane, and check wether they are inside the hull.

  3. Compute the average slope:
  
  		1. Find the average normal of the points on the ramp
  		
  		2. Calculate the ramp from this (assume the x direction is to the front of the robot)

  4. Transform this into a parameter.
  
For the bench and curb this is done as follows:

  1. Create points in a grid behind the exoskeleton / in front of the foot to move
  
  2. Elevate the points to the hulls 
  
  3. Sort the points on height
  
  4. Get the medain height
  
  5. Transform this heightinto a parameter

![Markers indicating locations](images/params.png "Markers indicating the potential foot locations on the stairs")

## Tutorials
These tutorials use the convenient aliases.

### Running in simulation
It is recommended to run all commands in separate terminals, to prevent sourcing problems between ROS1 & ROS2.
```bash
march_run_ros1_sim ground_gait:=true gazebo_ui:=true obstacle:=stairs # T1
march_run_bridge # T2
march_run_ros2_sim ground_gait:=true #T3
```
This will start both RViz and Gazebo. We use Gazebo to model the stairs, and RViz to visualise the pointlcouds. In RViz
add a `pointcloud2`, and set the topic to `/camera/preprocessed_cloud` or `/camera/region_cloud`. Add more debug information from [Published topics](#published-topics). If you cannot see the robot, add a RobotModel to rviz, and set the frame to `world`.  
To process the pointcloud, two things can be done.

   1. Click one of the realsense stair gaits in the `rqt_input_device` that spawned with ROS2. For example, use `reaslsense_stairs_up_single_step`. This will call upon de [service](#services), use the parameters to interpolate between gaits, and execute the newly computed gait. Of course, the interpolation and execution is only done if the service call was succesfull (i.e. a valid parameter was found).

   2. Call upon the service directly, using the terminal:
   ```bash
   snoe && sros1 && rosservice call /camera/process_pointcloud "selected_gait: 0 camera_to_use:=0 subgait_name:='right_open'"
   ```
   This just computes a parameter, no gait is started.  

 Note the debug visualisation in RVIZ, and the debug output in the ROS1 terminal. The current available obstacles in the simulation are:
 * stairs
 * ramp_and_door
 * bench
 Their dimensions are adjustable, using the service described here for the [stairs and bench](https://gitlab.com/project-march/march/-/blob/main/ros2/src/shared/march_shared_msgs/srv/SetObstacleSizeBlockLike.srv), and for the [ramp](https://gitlab.com/project-march/march/-/blob/main/ros2/src/shared/march_shared_msgs/srv/SetObstacleSizeRampLike.srv).   


### Running with the real camera's in simulation
You will need both camera's for this tutorial. If you only have one, provide `use_camera_back:=false` or `use_camera_front:=false`. Note that the Inertial Measurement Unit (IMU) is located in the back camera. If you want to use the IMU in the front camera, different launch for ROS2 is required.
```bash
march_run_ros1_sim ground_gait:=true realsense_simulation:=false # T1
march_run_bridge # T2
march_run_ros2 ground_gait:=true #T3 imu_to_use:=front imu_topic:=/camera_front/imu/data
```
RViz will now visualise what the reals camera's see, additionally the exoskelton will rotate with the IMU. Using the camera's is the same as in [Running in simulation](#running-in-simulation). To choose camera's with the manual service call, use:
```bash
camera_to_use:=0 #front
camera_to_use:=1 #back
```
The `rqt_input_device` will choose the correct camera autmatically.

### Running the camera's with the MARCH VI exeskeleton
The default launch arguments are defaulted to startig the camera's. Launch by:
```bash
march_run_ros1_airgait # T1 airgait/groundgait/training
march_run_bridge # T2
march_run_ros2_training # T3
```
It can takes some time for the serial connections of the camera's to start. To visualise the packages debug output, start rviz:
```bash
snoe && sros1 && rosrun rviz rviz
```
Calling upon the service is still the same as in the simulation.
