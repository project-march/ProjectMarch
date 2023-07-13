# gait_state_machine

The gait_state_machine package is responsible for loading, selecting and running all gaits.

## General structure

The general structure of the package can be seen in the block diagram shown below.

![gait state machine block diagram](./gait_node_structure.png)

## GaitLoader

Instances of the gait classes are created in the GaitLoader. This class returns a dictionary containing the name of
the gait and the instance corresponding to the name. It also returns a dictionary containing predefined positions
(e.g. home_stand or home_sit) as EdgePositions and the name of the position. The GaitStateMachine is constructed with
the two dictionaries of gaits and positions. Therefore, which gaits are loaded cannot be changed after the state machine
is initialized. Some individual properties of the dynamic gaits can be changed with rqt_reconfigure.

## GaitStateMachine

If the GaitStateMachine is running, its `update()` method is called every timer period.  The `update()` method checks
three things:
1. If a shutdown of the state machine is requested. For example when an error occurs and all gait activity should be 
stopped.
2. If `force_unknown` is requested. `force_unknown` stops execution of the current gait and resets the position of the
exoskeleton to `UnknownEdgePosition`. This happens if the execution of a gait failed or if the force_unknown button is
pressed on the input device.
3. If the state machine is currently executing a gait. If this is not true, it will call `_process_idle_state()`. This 
method listens for a gait request from the input device. If a gait request is accepted, the `_executing_gait` flag will
be set to True. `update()` will then call `_process_gait_state()`. This method starts the requested gait and calls the 
`update()` method of the gait until the gait is finished.

### Early scheduling

With 'early scheduling' the start time of the scheduled gaits are set in the future. There is a small delay between
scheduling a gait and the actual execution. If the start time of the command is 'now' when it is being sent, that start
will be in the past when the command is executed. By scheduling each command with a start time that is in the future,
this issue can be avoided.

## DynamicSetpointGait

`DynamicSetpointGait` implements the dynamic walk gait. All other dynamic gaits (step and close, step, close)
inherit from this class. Its main function is communicating with the `GaitStateMachine`. For this, it
implements the `GaitInterface`. Most of the interaction with the state machine take place in the `start()` and 
`update()` methods. Both of these methods return a `GaitUpdate`. This dataclass contains an optional `TrajectoryCommand`
and the two booleans `is_new_subgait` and `is_finished`. At the early schedule duration, a GaitUpdate containing a new
TrajectoryCommand is sent to the state machine. At the point in time that the gait actually starts, a GaitUpdate with
`is_new_subgait` set to True is sent. If the gait is stopped and no new commands will be sent, a GaitUpdate with 
`is_finished` set to True is sent. 

## TrajectoryCommandHandler

`DynamicSetpointGait` uses the `TrajectoryCommandHandler` class to generate TrajectoryCommands. This class uses the
`CameraPointHandler` to get a foot position to step towards. Next, it tries to construct a trajectory with
`DynamicSubgait`. There are several safety checks to ensure that unsafe gaits are not executed. 
1. If the trajectory can not be constructed within the position and velocity limits of the joints, it
will iterate with increasing duration to try and create a feasible trajectory.
2. It checks if the consecutive step can also be made. This assumes that the consecutive step will be towards the
same point as the current step.

If one of these checks are not met, the TrajectoryCommandHandler will try to close the gait, such that the exoskeleton
ends in a comfortable position for the pilot. If the close gait is also not possible, the exoskeleton will do nothing.

## CameraPointHandler

`CameraPointHandler` handles all communication with the `GaitPreprocessor`. It gets a foot position to step towards
from the GaitPreprocessor. It checks the time at which the foot position was sent to ensure that positions that are 
too old are not used.

## DynamicSubgait

`DynamicSubgait` creates the `joint_trajectory_msg` for the TrajectoryCommand. It calls the 
`march_goniometric_ik_solver` to solve the middle and end positions, and then calls `DynamicJointTrajectory` to 
interpolate between the start, middle and end positions for each joint.  If enabled, it will add a setpoint for the 
swing leg ankle joint to create a push off motion. To prevent exceeding joint position soft limits, the ankle and knee 
of the stance leg middle point velocities are set to zero. 

## DynamicJointTrajectory

`DynamicJointTrajectory` interpolates between the setpoints given by DynamicSubgait. If a trajectory has a fixed
velocity in the middle setpoint(s), it uses a 
[Bernstein polynomials](https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.BPoly.html) for 
interpolation. If the middle point velocity is  not specified, it uses a 
[CubicSpline](https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.CubicSpline.html) with clamped 
boundary conditions of zero velocity at the start and end setpoint.

