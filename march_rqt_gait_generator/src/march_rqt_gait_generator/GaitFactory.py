import rospy


from model.Gait import Gait
from model.Joint import Joint
from model.Limits import Limits
from model.Setpoint import Setpoint


def empty_gait(robot, duration):
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
            Setpoint(0.2, 0, 0),
            Setpoint(3, 1.3, 0),
            Setpoint(4, 1.3, 0),
            Setpoint(duration, 0, 0)
        ]
        joint = Joint(urdf_joint.name,
                      Limits(urdf_joint.limit.lower, urdf_joint.limit.upper, urdf_joint.limit.velocity),
                      default_setpoints,
                      duration
                      )
        joint_list.append(joint)
    return Gait(joint_list, duration)


def from_msg(robot, march_gait, gait_name,  subgait_name, version):
    if robot is None:
        rospy.logerr("Cannot create gait without a loaded robot.")

    user_defined_setpoints = march_gait.setpoints
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
        for actual_setpoint in user_defined_setpoints:
            if joint_name in actual_setpoint.joint_names:
                setpoints.append(get_setpoint_at_duration(
                    joint_trajectory, joint_name, actual_setpoint.time_from_start))

        print "Joint " + joint_name + " has setpoints " + str(setpoints)
        urdf_joint = get_joint_from_urdf(robot, joint_name)

        limits = Limits(urdf_joint.limit.lower, urdf_joint.limit.upper, urdf_joint.limit.velocity)
        joint = Joint(joint_name,
                      limits,
                      setpoints,
                      duration
                      )
        joint_list.append(joint)

    return Gait(joint_list, duration, gait_name, subgait_name, version, march_gait.description)


def get_setpoint_at_duration(joint_trajectory, joint_name, duration):
    for point in joint_trajectory.points:
        if point.time_from_start == duration:
            index = joint_trajectory.joint_names.index(joint_name)
            time = rospy.Duration(point.time_from_start.secs, point.time_from_start.nsecs).to_sec()

            return Setpoint(time, point.positions[index], point.velocities[index])
    return None


def joint_exists(robot, joint_name):
    return get_joint_from_urdf(robot, joint_name) is not None


def get_joint_from_urdf(robot, joint_name):
    for urdf_joint in robot.joints:
        if urdf_joint.name == joint_name:
            return urdf_joint
    return None
