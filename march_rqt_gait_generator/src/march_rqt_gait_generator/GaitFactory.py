import rospy

from model.modifiable_subgait import ModifiableSubgait
from model.modifiable_joint_trajectory import ModifiableJointTrajectory
from march_shared_classes.gait.limits import Limits
from model.modifiable_setpoint import ModifiableSetpoint


def empty_gait(gait_generator, robot, duration):
    if robot is None:
        rospy.logerr("Cannot create gait without a loaded robot.")
    joint_list = []
    for i in range(0, len(robot.joints)):
        urdf_joint = robot.joints[i]
        if urdf_joint.type == "fixed":
            rospy.loginfo("Skipping fixed joint " + urdf_joint.name)
            continue

        if urdf_joint.limit is None:
            rospy.logwarn("Skipping joint " + urdf_joint.name + " because it has no limits.")
            continue

        default_setpoints = [
            ModifiableSetpoint(0.2, 0, 0),
            ModifiableSetpoint(3, 1.3, 0),
            ModifiableSetpoint(4, 1.3, 0),
            ModifiableSetpoint(duration, 0, 0)
        ]
        joint = ModifiableJointTrajectory(urdf_joint.name,
                                          Limits(urdf_joint.safety_controller.soft_lower_limit,
                                                 urdf_joint.safety_controller.soft_upper_limit,
                                                 urdf_joint.limit.velocity),
                                          default_setpoints,
                                          duration,
                                          gait_generator
                                          )
        joint_list.append(joint)
    return ModifiableSubgait(joint_list, duration)


def from_msg(gait_generator, robot, march_gait, gait_name, subgait_name, version):
    if robot is None:
        rospy.logerr("Cannot create gait without a loaded robot.")
        return None

    user_defined_setpoints = march_gait.setpoints

    if not user_defined_setpoints:
        rospy.logwarn("Subgait is missing setpoints, assuming all trajectory points are setpoints")
    if len(march_gait.trajectory.points) < 2:
        rospy.logwarn("Cannot load gait as it has only %s setpoints instead of the minimal 2",
                      str(len(march_gait.trajectory.points)))
        return None
    joint_trajectory = march_gait.trajectory

    # Check if all joints in this gait exist in the robot, joints in the robot but not in the gait are allowed.
    for joint_name in joint_trajectory.joint_names:
        if not joint_exists(robot, joint_name):
            rospy.logerr("Joint " + joint_name + " not found in robot description")
            return None

    joint_list = []
    duration = rospy.Duration(march_gait.duration.secs, march_gait.duration.nsecs).to_sec()
    for joint_name in joint_trajectory.joint_names:
        setpoints = []
        if user_defined_setpoints:
            for actual_setpoint in user_defined_setpoints:
                if joint_name in actual_setpoint.joint_names:
                    setpoints.append(get_setpoint_at_duration(
                        joint_trajectory, joint_name, actual_setpoint.time_from_start))
        else:
            joint_index = joint_trajectory.joint_names.index(joint_name)

            for point in joint_trajectory.points:
                time = rospy.Duration(point.time_from_start.secs, point.time_from_start.nsecs).to_sec()
                setpoints.append(ModifiableSetpoint(time, point.positions[joint_index],
                                                    point.velocities[joint_index]))

        rospy.loginfo("Joint " + joint_name + " has setpoints " + str(setpoints))
        urdf_joint = get_joint_from_urdf(robot, joint_name)

        limits = Limits(urdf_joint.safety_controller.soft_lower_limit,
                        urdf_joint.safety_controller.soft_upper_limit,
                        urdf_joint.limit.velocity)
        joint = ModifiableJointTrajectory(joint_name,
                                          limits,
                                          setpoints,
                                          duration,
                                          gait_generator
                                          )
        joint_list.append(joint)

    return ModifiableSubgait(joint_list, duration, march_gait.gait_type, gait_name, subgait_name,
                             version, march_gait.description)


def get_setpoint_at_duration(joint_trajectory, joint_name, duration):
    for point in joint_trajectory.points:
        if point.time_from_start == duration:
            index = joint_trajectory.joint_names.index(joint_name)
            time = rospy.Duration(point.time_from_start.secs, point.time_from_start.nsecs).to_sec()

            return ModifiableSetpoint(time, point.positions[index], point.velocities[index])
    return None


def joint_exists(robot, joint_name):
    return get_joint_from_urdf(robot, joint_name) is not None


def get_joint_from_urdf(robot, joint_name):
    for urdf_joint in robot.joints:
        if urdf_joint.name == joint_name:
            return urdf_joint
    return None
