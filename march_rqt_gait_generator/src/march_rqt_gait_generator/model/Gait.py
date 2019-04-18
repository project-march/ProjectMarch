import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class Gait:

    def __init__(self, joints, duration, name="Dummy gait", description="Just a simple gait"):
        self.joints = joints
        self.name = name
        self.description = description
        self.duration = duration
        self.current_time = 0

    def export_to_file(self):
        joint_trajectory = self.to_joint_trajectory()
        rospy.logwarn(joint_trajectory)

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

    def set_current_time(self, time):
        self.current_time = time
