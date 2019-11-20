import rospy
from march_shared_classes.gait.subgait import Subgait
from modifiable_joint_trajectory import ModifiableJointTrajectory

from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from march_shared_resources.msg import Setpoint


class ModifiableSubgait(Subgait):
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

    def can_mirror(self, key_1, key_2):
        if not key_1 or not key_2:
            rospy.loginfo("Keys are invalid")
            return False

        # XNOR, only one key can and must exist in the subgait name
        if (key_1 in self.subgait_name) == (key_2 in self.subgait_name):
            rospy.loginfo("Multiple or no keys exist in subgait %s", self.subgait_name)
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
            rospy.logwarn("Cannot mirror gait %s", self.gait_name)
            return False

        if key_1 in self.subgait_name:
            mirrored_subgait_name = self.subgait_name.replace(key_1, key_2)
        elif key_2 in self.subgait_name:
            mirrored_subgait_name = self.subgait_name.replace(key_2, key_1)
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

            mirrored_joint = ModifiableJointTrajectory(mirrored_name, joint.limits, joint.setpoints, joint.duration)
            mirrored_joints.append(mirrored_joint)

        return ModifiableSubgait(mirrored_joints, self.duration, self.gait_type, self.gait_name, mirrored_subgait_name,
                                 self.version, self.description)
