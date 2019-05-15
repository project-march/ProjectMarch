import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from march_rqt_gait_generator.msg import ActualSetpoint


class Gait:

    def __init__(self, joints, duration, name="Dummy", version="First try", description="Just a simple gait"):
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

    def to_actual_setpoints(self):
        actual_setpoints = []
        timestamps = self.get_unique_timestamps()
        for timestamp in timestamps:
            actual_setpoint = ActualSetpoint()
            actual_setpoint.time_from_start = rospy.Duration.from_sec(timestamp)
            for joint in self.joints:
                for setpoint in joint.setpoints:
                    if setpoint.time == timestamp:
                        actual_setpoint.joint_names.append(joint.name)
            actual_setpoints.append(actual_setpoint)
        return actual_setpoints

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

    def has_setpoints_after_duration(self, duration):
        for joint in self.joints:
            for setpoint in joint.setpoints:
                if setpoint.time > duration:
                    return True
        return False

    # Setters to allow changing values in a callback
    def set_name(self, name):
        self.name = name

    def set_description(self, description):
        self.description = description

    def set_version(self, version):
        self.version = version

    def set_duration(self, duration, rescale=False):
        for joint in self.joints:
            # Loop in reverse to avoid out of bounds errors while deleting.
            for setpoint in reversed(joint.setpoints):
                if rescale:
                    setpoint.time = duration * setpoint.time / self.duration
                else:
                    if setpoint.time > duration:
                        joint.setpoints.remove(setpoint)
            joint.interpolated_setpoints = joint.interpolate_setpoints()

        self.duration = duration

    def set_current_time(self, current_time):
        self.current_time = current_time
