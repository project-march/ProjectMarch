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
                      Limits(urdf_joint.limit.upper, urdf_joint.limit.lower, urdf_joint.limit.velocity),
                      default_setpoints,
                      duration
                      )
        joint_list.append(joint)
    return Gait(joint_list, duration)


def from_file(robot, file):
    pass
