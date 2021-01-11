import rospy

from march_shared_classes.gait.subgait import Subgait

from .modifiable_joint_trajectory import ModifiableJointTrajectory


class ModifiableSubgait(Subgait):

    joint_class = ModifiableJointTrajectory

    def has_multiple_setpoints_before_duration(self, duration):
        """Check if all setpoints are before a given duration."""
        return not any(joint.setpoints[1].time > duration for joint in self.joints)

    def has_setpoints_after_duration(self, duration):
        """Check if a joint has a setpoint which is larger then the given duration."""
        return any(joint.setpoints[-1].time > duration for joint in self.joints)

    def can_mirror(self, key_1, key_2):
        """Check if the given keys exist in the available joint names and subgait name and if a replacement is possible.

        :param key_1: The first key to check in the joint names and subgait name
        :param key_2: The second key key to check in the joint names and subgait name

        :return:
            True if the replacement is possible and no errors occurred
        """
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
                joint_2_name = joint.name.replace(key_1, key_2)

                if joint_2_name not in self.get_joint_names():
                    rospy.loginfo("joint %s does not exist", joint_2_name)
                    return False

                joint_2 = self.get_joint(joint_2_name)

            elif key_2 in joint.name:
                joint_2 = joint
                joint_1_name = joint.name.replace(key_2, key_1)

                if joint_1_name not in self.get_joint_names():
                    rospy.loginfo("joint %s does not exist", joint_1_name)
                    return False

                joint_1 = self.get_joint(joint_1_name)

            else:
                continue

            if not self.verify_mirrored_joint(joint_1, joint_2):
                return False

        return True

    def get_mirror(self, key_1, key_2):
        """Mirror the subgait by changing replacing key_1 to key_2 and visa versa in every subgait.

        :param key_1: prefix name which specifies a side within the subgait (for example: right)
        :param key_2: prefix name which specifies a side within the subgait (for example: left)

        :return:
            The mirrored subgait as subgait object
        """
        if not self.can_mirror(key_1, key_2):
            rospy.logwarn("Cannot mirror gait %s", self.gait_name)
            return False

        if key_1 in self.subgait_name:
            mirrored_subgait_name = str(self.subgait_name.replace(key_1, key_2))
        elif key_2 in self.subgait_name:
            mirrored_subgait_name = str(self.subgait_name.replace(key_2, key_1))
        else:
            rospy.logerr("This case should have been caught by can_mirror()")
            return False

        if key_1 in self.version:
            mirrored_version = str(self.version.replace(key_1, key_2))
        elif key_2 in self.version:
            mirrored_version = str(self.version.replace(key_2, key_1))
        else:
            mirrored_version = self.version

        mirrored_joints = []
        for joint in self.joints:
            if key_1 in joint.name:
                mirrored_name = str(joint.name.replace(key_1, key_2))
            elif key_2 in joint.name:
                mirrored_name = str(joint.name.replace(key_2, key_1))
            else:
                continue

            mirrored_joint = ModifiableJointTrajectory(
                mirrored_name, joint.limits, joint.setpoints, joint.duration
            )
            mirrored_joints.append(mirrored_joint)

        return ModifiableSubgait(
            mirrored_joints,
            self.duration,
            self.gait_type,
            self.gait_name,
            mirrored_subgait_name,
            mirrored_version,
            self.description,
        )

    @staticmethod
    def verify_mirrored_joint(joint_1, joint_2):
        """Verify if joint_1 and joint_2 are mirrored versions of each other."""
        if joint_1 is None or joint_2 is None:
            rospy.logwarn("Joints %s and %s are not valid.", str(joint_1), str(joint_2))
            return False

        if (
            joint_1.setpoints[0].position != joint_2.setpoints[-1].position
            or joint_1.setpoints[0].velocity != joint_2.setpoints[-1].velocity
        ):
            rospy.loginfo(
                "First setpoint of %s != last setpoint of %s. These subgaits will not be able to loop.",
                joint_1.name,
                joint_2.name,
            )

        if (
            joint_1.setpoints[-1].position != joint_2.setpoints[0].position
            or joint_1.setpoints[-1].velocity != joint_2.setpoints[0].velocity
        ):
            rospy.loginfo(
                "Last setpoint of %s != first setpoint of %s. These subgaits will not be able to loop.",
                joint_1.name,
                joint_2.name,
            )

        return True

    def scale_timestamps_subgait(self, new_duration, rescale=True):
        """Update the interpolated setpoints whenever a new duration is set to the subgait."""
        super(ModifiableSubgait, self).scale_timestamps_subgait(new_duration, rescale)

        for joint in self.joints:
            joint.interpolated_setpoints = joint.interpolate_setpoints()

    def set_gait_type(self, gait_type):
        """Set the subgait type as string."""
        self.gait_type = str(gait_type)

    def set_gait_name(self, gait_name):
        """Set the gait name as string."""
        self.gait_name = str(gait_name)

    def set_description(self, description):
        """Set the subgait description as string."""
        self.description = str(description)

    def set_version(self, version):
        """Set the subgait version as string."""
        self.version = str(version)

    def set_subgait_name(self, subgait_name):
        """Set the subgait name as string."""
        self.subgait_name = str(subgait_name)
