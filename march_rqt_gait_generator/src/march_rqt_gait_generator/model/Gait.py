import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from Joint import Joint

from march_shared_resources.msg import Setpoint
from march_rqt_gait_generator.UserInterfaceController import notify


class Gait:
    def __init__(self, joints, duration,
                 name="Walk", subgait="right_open", version="First try", description="Just a simple gait"):
        self.joints = joints
        self.name = name
        self.subgait = subgait
        self.version = version
        self.description = str(description)
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

    def to_setpoints(self):
        user_defined_setpoints = []
        timestamps = self.get_unique_timestamps()
        for timestamp in timestamps:
            user_defined_setpoint = Setpoint()
            user_defined_setpoint.time_from_start = rospy.Duration.from_sec(timestamp)
            for joint in self.joints:
                for setpoint in joint.setpoints:
                    if setpoint.time == timestamp:
                        user_defined_setpoint.joint_names.append(joint.name)
            user_defined_setpoints.append(user_defined_setpoint)
        return user_defined_setpoints

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

    def has_multiple_setpoints_before_duration(self, duration):
        for joint in self.joints:
            count = 0
            for setpoint in joint.setpoints:
                if setpoint.time <= duration:
                    count += 1
            if count < 2:
                return False
        return True

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
        self.description = str(description)

    def set_version(self, version):
        self.version = version

    def set_subgait(self, subgait):
        self.subgait = subgait

    def set_duration(self, duration, rescale=False):
        for joint in self.joints:
            # Loop in reverse to avoid out of bounds errors while deleting.
            for setpoint in reversed(joint.setpoints):
                if rescale:
                    setpoint.set_time(duration * setpoint.time / self.duration)
                else:
                    if setpoint.time > duration:
                        joint.setpoints.remove(setpoint)
            joint.interpolated_setpoints = joint.interpolate_setpoints()

            joint.duration = duration

        self.duration = duration

    def set_current_time(self, current_time):
        self.current_time = current_time

    def can_mirror(self, key_1, key_2):
        if not key_1 or not key_2:
            rospy.loginfo("Keys are invalid")
            return False

        # XNOR, only one key can and must exist in the subgait name
        if (key_1 in self.subgait) == (key_2 in self.subgait):
            rospy.loginfo("Multiple or no keys exist in subgait %s", self.subgait)
            return False

        # If a joint name has both keys, we wouldn't know how to replace them.
        for joint in self.joints:
            if key_1 in joint.name and key_2 in joint.name:
                rospy.loginfo("Both keys exist in joint %s", joint.name)
                return False
            if key_1 in joint.name:
                joint_1 = joint
                joint_2 = self.get_joint(joint.name.replace(key_1, key_2))
            elif key_2 in joint.name:
                joint_1 = self.get_joint(joint.name.replace(key_2, key_1))
                joint_2 = joint
            else:
                continue

            if joint_1 is None or joint_2 is None:
                rospy.logwarn("Joints %s and %s are not valid.", str(joint_1), str(joint_2))
                return False

            if joint_1.setpoints[0].position != joint_2.setpoints[-1].position \
                    or joint_1.setpoints[0].velocity != joint_2.setpoints[-1].velocity:
                rospy.loginfo("First setpoint of %s != last setpoint of %s", joint_1.name, joint_2.name)
                return False
            if joint_1.setpoints[-1].position != joint_2.setpoints[0].position \
                    or joint_1.setpoints[-1].velocity != joint_2.setpoints[0].velocity:
                rospy.loginfo("Last setpoint of %s != first setpoint of %s", joint_1.name, joint_2.name)
                return False

        return True

    def get_mirror(self, key_1, key_2):
        if not self.can_mirror(key_1, key_2):
            rospy.logwarn("Cannot mirror gait %s", self.name)
            return False

        if key_1 in self.subgait:
            mirrored_subgait_name = self.subgait.replace(key_1, key_2)
        elif key_2 in self.subgait:
            mirrored_subgait_name = self.subgait.replace(key_2, key_1)
        else:
            rospy.logerr("This case should have been caught by can_mirror()")
            return False

        mirrored_joints = []
        for joint in self.joints:
            if key_1 in joint.name:
                mirrored_name = str(joint.name.replace(key_1, key_2))
            elif key_2 in joint.name:
                mirrored_name = str(joint.name.replace(key_2, key_1))
            else:
                continue

            mirrored_joint = Joint(mirrored_name, joint.limits, joint.setpoints, joint.duration)
            mirrored_joints.append(mirrored_joint)

        return Gait(mirrored_joints, self.duration, self.name, mirrored_subgait_name, self.version, self.description)
