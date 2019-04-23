import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class Gait:

    def __init__(self, joints, duration, name="Dummy", version="First try",  description="Just a simple gait"):
        self.joints = joints
        self.name = name
        self.version = version
        self.description = description
        self.duration = duration
        self.current_time = 0

    def to_joint_trajectory(self):
        joint_trajectory = JointTrajectory()

        timestamps = self.get_unique_timestamps()

        for joint in self.joints:
            joint_trajectory.joint_names.append(joint.name)

        for timestamp in timestamps:
            joint_trajectory_point = JointTrajectoryPoint()
            joint_trajectory_point.time_from_start = rospy.Duration(timestamp)
            for joint in self.joints:
                interpolated_setpoint = joint.get_interpolated_setpoint(timestamp)

                if interpolated_setpoint.time != timestamp:
                    rospy.logerr("Time mismatch in joint " + joint.name + " at timestamp " + timestamp)
                joint_trajectory_point.positions.append(interpolated_setpoint.position)
                joint_trajectory_point.velocities.append(interpolated_setpoint.velocity)
            joint_trajectory.points.append(joint_trajectory_point)

        return joint_trajectory

    def get_unique_timestamps(self):
        timestamps = []
        for joint in self.joints:
            for setpoint in joint.setpoints:
                timestamps.append(setpoint.time)

        return sorted(set(timestamps))

    def get_joint(self, name):
        for i in range(0, len(self.joints)):
            if self.joints[i].name == name:
                return self.joints[i]
        rospy.logerr("Joint with name " + name + " does not exist in gait " + self.name + ".")
        return None

    # Setters to allow changing values in a callback
    def set_name(self, name):
        self.name = name

    def set_description(self, description):
        self.description = description

    def set_version(self, version):
        self.version = version

    def set_current_time(self, current_time):
        self.current_time = current_time
