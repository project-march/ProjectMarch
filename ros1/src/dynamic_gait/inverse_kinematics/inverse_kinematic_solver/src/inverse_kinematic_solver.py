#!/usr/bin/env python

import sys
import rospy
from march_shared_msgs.srv import SolveInverseKinematic, SolveInverseKinematicResponse
import moveit_msgs.srv
import moveit_msgs.msg
import numpy as np


def inverse_kinematic_solver(request):
    # Create ik_request and define planning group and frame_id:
    position_ik_request = moveit_msgs.msg.PositionIKRequest()
    position_ik_request.group_name = planning_group
    position_ik_request.ik_link_name = end_effector
    position_ik_request.robot_state.joint_state.header.frame_id = "base_link"
    position_ik_request.pose_stamped.header.frame_id = "base_link"

    # For now, the seed is just the neutral position:
    position_ik_request.robot_state.joint_state.name = [
        "left_ankle",
        "left_knee",
        "left_hip_fe",
        "left_hip_aa",
        "right_hip_aa",
        "right_hip_fe",
        "right_knee",
        "right_ankle",
    ]
    position_ik_request.robot_state.joint_state.position = [
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    ]

    # Set requested position:
    position_ik_request.pose_stamped.pose.position.x = (
        request.desired_location.x + x_offest
    )
    position_ik_request.pose_stamped.pose.position.y = (
        request.desired_location.y + y_offset
    )
    position_ik_request.pose_stamped.pose.position.z = (
        request.desired_location.z + z_offset
    )

    # Orientation is always the same for flat feet planning:
    position_ik_request.pose_stamped.pose.orientation.x = 0
    position_ik_request.pose_stamped.pose.orientation.y = 0.707
    position_ik_request.pose_stamped.pose.orientation.z = 0.707
    position_ik_request.pose_stamped.pose.orientation.w = 0

    # Make ankle constraint:
    ankle_constraint = moveit_msgs.msg.JointConstraint()
    ankle_constraint.joint_name = constraint_ankle
    ankle_constraint.position = np.deg2rad(5)
    ankle_constraint.tolerance_above = np.deg2rad(5)
    ankle_constraint.tolerance_below = np.deg2rad(5)
    ankle_constraint.weight = 1

    # Make knee constraint:
    knee_constraint = moveit_msgs.msg.JointConstraint()
    knee_constraint.joint_name = constraint_knee
    knee_constraint.position = np.deg2rad(0)
    knee_constraint.tolerance_above = np.deg2rad(5)
    knee_constraint.tolerance_below = np.deg2rad(5)
    knee_constraint.weight = 1

    # Add constraints:
    position_ik_request.constraints.joint_constraints = [
        ankle_constraint,
        knee_constraint,
    ]

    # Try to solve IK problem:
    solution_found = False
    attempts = 1
    while not solution_found and attempts < 100:
        response = compute_ik(position_ik_request)

        # If solution found:
        if response.error_code.val == 1:
            angles = np.rad2deg(response.solution.joint_state.position)
            hip_orientation = angles[0] - angles[1] + angles[2]

            # If hip is not straight, make it straight:
            if round(hip_orientation) != 0:
                left_hip_fe = response.solution.joint_state.position[2] - np.deg2rad(
                    hip_orientation
                )
                right_hip_fe = response.solution.joint_state.position[5] - np.deg2rad(
                    hip_orientation
                )
                new_position = []
                for i in range(len(response.solution.joint_state.position)):
                    if i == 2:
                        new_position.append(left_hip_fe)
                    elif i == 5:
                        new_position.append(right_hip_fe)
                    else:
                        new_position.append(response.solution.joint_state.position[i])
                response.solution.joint_state.position = new_position

            # Set solution found:
            solution_found = True

        # If no solution found:
        else:
            attempts += 1

    # Return result:
    solve_inverse_kinematics_response = SolveInverseKinematicResponse()
    if solution_found:
        solve_inverse_kinematics_response.success = "success"
    else:
        solve_inverse_kinematics_response.success = str(
            "Failed with error code " + str(response.error_code.val)
        )
    solve_inverse_kinematics_response.solution = response.solution.joint_state
    return solve_inverse_kinematics_response


def inverse_kinematic_solver_server():
    rospy.Service(
        "inverse_kinematic_solver", SolveInverseKinematic, inverse_kinematic_solver
    )
    rospy.spin()


if __name__ == "__main__":
    # Default offset:
    x_offset = 0.429
    y_offset = -0.258
    z_offset = 0.108

    # Start node and get argument:
    rospy.init_node("inverse_kinematic_solver")
    args = rospy.myargv(argv=sys.argv)
    if len(args) == 2:
        side = args[1]

        if side == "right":
            planning_group = "chain_lr"
            end_effector = "right_foot"
            constraint_ankle = "left_ankle"
            constraint_knee = "left_knee"
            rospy.loginfo("Starting inverse_kinematic_solver for right foot...")
            x_offest = x_offset
        elif side == "left":
            planning_group = "chain_rl"
            end_effector = "left_foot"
            constraint_ankle = "right_ankle"
            constraint_knee = "right_knee"
            rospy.loginfo("Starting inverse_kinematic_solver for left foot...")
            x_offest = -x_offset
        else:
            rospy.logerr(
                "Node inverse_kinematic_solver expects 'left' or 'right' as argument!"
            )

        # Define compute_ik service and start own server:
        compute_ik = rospy.ServiceProxy("compute_ik", moveit_msgs.srv.GetPositionIK())
        inverse_kinematic_solver_server()

    else:
        rospy.logerr("Node inverse_kinematic_solver expects 1 argument!")
