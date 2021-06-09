# Smartglasses bridge
Package that sends incoming messages to the `/march/input_device/current_state` topic
to the smartglasses via sockets.

## How to build
```bash
source /opt/ros/foxy/local_setup.bash
colcon build --packages-select march_smartglasses_bridge
```

## How to run
Before running the bridge, ensure that you have build the code and that
you have sourced both the general ROS installation and the march packages.
```bash
# With the default launch arguments
ros2 launch march_smartglasses_bridge smartglasses_bridge.launch.py

# With own launch arguments
ros2 launch march_smartglasses_bridge smartglasses_bridge.launch.py \
    hud_host:=127.0.0.1 \
    hud_port:=53003
```

### Launch arguments
The bridge can be configured with the following launch arguments:
* `hud_host`: The IP address of the smartglasses (Default = `localhost`)
* `hud_port`: The port on which the smartglasses listen for incoming 
messages (Default = `53003`)
