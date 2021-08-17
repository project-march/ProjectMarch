# MPC Visualization
Package that subscribes to the `/march/mpc` topic in `ROS2`, and publishes to the local network via HTTP.  
Model predictive controllers (MPC) provide estimation of future states. Visualising these estimations via ROS is difficult,
since RQT does not support data in the future. This package provides a way to publish these estimates to the local network,
so that custom visualization tool can be used.

.. hint:: This node requires manual launching
  
## How to build
```bash
source /opt/ros/foxy/local_setup.bash
colcon build --packages-select march_mpc_visualization
```  

## How to run
First launch `ros1`, `ros2` and the `ros1_bridge`. Then in a new terminal:
```bash
# Sourcing the ROS2 installation, and the local build
source /opt/ros/foxy/local_setup.bash
source /ros2/install/local_setup.bash

ros2 launch march_mpc_visualization march_mpc_visualization.launch.py
```

## Publishing
The node starts publishing to two adresses:  

- http://0.0.0.0:5000/measurements  
  - Data in JSON format, containing the last measured position, velocity, input and its references, for all joints controlled by MPC.  
- http://0.0.0.0:5000/estimation 
  - Data in JSON format, containing the last estimation of position, velocity, input and its references, for all joints controlled by MPC.  

.. hint:: Change the host to the IP of the publishing device

## Testing
To check if the node is operating as expected, check the adresses mentioned in the section above.

If there is no data present, but the pages do exist, check if ROS2 contains the correct information:

```bash
# Sourcing the ROS2 installation, and the local build
source /opt/ros/foxy/local_setup.bash
source /ros2/install/local_setup.bash
ros2 topic echo /march/mpc
```  
If nothing is published, check the ros1 topic:
```bash
# Sourcing the ROS2 installation, and the local build
source /opt/ros/noetic/local_setup.bash
source /ros1/install/local_setup.bash
rostopic echo /march/mpc
```  
If ROS1 is publishing data, and ROS2 is not, rebuild the `ros1_bridge`.  
If the ros1 topic does not exist, MPC is not running. Ensure you are running the correct controller type.
