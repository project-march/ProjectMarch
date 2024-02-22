# March Mode Machine
**Author: Andrew Hutani, MIX**
## Overview
This package is the implementation of the Mode Machine that is used to control the mode of the whole exo. This packages also create the class `ExoMode` based on the `modes.json` file. It is a bit unconventional to have the class generated by a script, but this was done in order to centralized the possible modes in one place.

## ExoModeTransitions
This class holds all the possible transitions, and has three possible types. This is the case because the joint angle, cartesian and test joint implementations need different transitions. Note that if a mode is added, it also needs to be added to the transitions file, alongside its possible transitions.

## Node structure
### Server
- `'get_exo_mode_array'` ([GetExoModeArray.srv](https://gitlab.com/project-march/march/-/blob/dev/ros2/src/shared/march_shared_msgs/srv/GetExoModeArray.srv))

### Publishers
- `"current_mode"` ([ExoMode.msg](https://gitlab.com/project-march/march/-/blob/dev/ros2/src/shared/march_shared_msgs/msg/ExoMode.msg))

