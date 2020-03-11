import rospy

from march_shared_classes.gait.limits import Limits
from march_shared_classes.gait.subgait import Subgait

from .modifiable_joint_trajectory import ModifiableJointTrajectory
from .modifiable_setpoint import ModifiableSetpoint


class ModifiableSubgait(Subgait):
    joint_class = ModifiableJointTrajectory

    @classmethod
    def empty_subgait(cls, gait_generator, robot, duration=8, gait_type='walk_like', gait_name='test_gait',
                      subgait_name='test_subgait', version='test_subgait_1', description='Just a simple gait'):
        if robot is None:
            rospy.logerr('Cannot create gait without a loaded robot.')
            return None
        joint_list = []
        for i in range(0, len(robot.joints)):
            urdf_joint = robot.joints[i]
            if urdf_joint.type == 'fixed':
                rospy.loginfo('Skipping fixed joint ' + urdf_joint.name)
                continue

            if urdf_joint.limit is None:
                rospy.logwarn('Skipping joint ' + urdf_joint.name + ' because it has no limits.')
                continue

            default_setpoints = [
                ModifiableSetpoint(0, 0, 0),
                ModifiableSetpoint(duration, 0, 0),
            ]
            limits = Limits(urdf_joint.safety_controller.soft_lower_limit,
                            urdf_joint.safety_controller.soft_upper_limit,
                            urdf_joint.limit.velocity,
                            urdf_joint.limit.effort,
                            urdf_joint.safety_controller.k_position,
                            urdf_joint.safety_controller.k_velocity)
            joint = ModifiableJointTrajectory(urdf_joint.name,
                                              limits,
                                              default_setpoints,
                                              duration,
                                              gait_generator,
                                              )
            joint_list.append(joint)
        return cls(joint_list, duration, gait_type, gait_name, subgait_name, version, description)

    def has_multiple_setpoints_before_duration(self, duration):
        for joint in self.joints:
            if joint.setpoints[1].time > duration:
                return False
        return True

    def has_setpoints_after_duration(self, duration):
        for joint in self.joints:
            if joint.setpoints[-1].time > duration:
                return True
        return False

    def can_mirror(self, key_1, key_2):
        if not key_1 or not key_2:
            rospy.loginfo('Keys are invalid')
            return False

        # XNOR, only one key can and must exist in the subgait name
        if (key_1 in self.subgait_name) == (key_2 in self.subgait_name):
            rospy.loginfo('Multiple or no keys exist in subgait %s', self.subgait_name)
            return False

        # If a joint name has both keys, we wouldn't know how to replace them.
        for joint in self.joints:
            if key_1 in joint.name and key_2 in joint.name:
                rospy.loginfo('Both keys exist in joint %s', joint.name)
                return False

            if key_1 in joint.name:
                joint_1 = joint
                joint_2_name = joint.name.replace(key_1, key_2)

                if joint_2_name not in self.get_joint_names():
                    rospy.loginfo('joint %s does not exist', joint_2_name)
                    return False
                joint_2 = self.get_joint(joint_2_name)

            elif key_2 in joint.name:
                joint_1_name = joint.name.replace(key_2, key_1)
                if joint_1_name not in self.get_joint_names():
                    rospy.loginfo('joint %s does not exist', joint_1_name)
                    return False
                joint_1 = self.get_joint(joint_1_name)

                joint_2 = joint
            else:
                continue

            if joint_1 is None or joint_2 is None:
                rospy.logwarn('Joints %s and %s are not valid.', str(joint_1), str(joint_2))
                return False

            if joint_1.setpoints[0].position != joint_2.setpoints[-1].position \
                    or joint_1.setpoints[0].velocity != joint_2.setpoints[-1].velocity:
                rospy.loginfo('First setpoint of %s != last setpoint of %s', joint_1.name, joint_2.name)
                return False
            if joint_1.setpoints[-1].position != joint_2.setpoints[0].position \
                    or joint_1.setpoints[-1].velocity != joint_2.setpoints[0].velocity:
                rospy.loginfo('Last setpoint of %s != first setpoint of %s', joint_1.name, joint_2.name)
                return False

        return True

    def get_mirror(self, key_1, key_2):
        if not self.can_mirror(key_1, key_2):
            rospy.logwarn('Cannot mirror gait %s', self.gait_name)
            return False

        if key_1 in self.subgait_name:
            mirrored_subgait_name = self.subgait_name.replace(key_1, key_2)
        elif key_2 in self.subgait_name:
            mirrored_subgait_name = self.subgait_name.replace(key_2, key_1)
        else:
            rospy.logerr('This case should have been caught by can_mirror()')
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

    def set_gait_type(self, gait_type):
        self.gait_type = str(gait_type)

    def set_gait_name(self, gait_name):
        self.gait_name = str(gait_name)

    def set_description(self, description):
        self.description = str(description)

    def set_version(self, version):
        self.version = str(version)

    def set_subgait_name(self, subgait_name):
        self.subgait_name = str(subgait_name)

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

            joint.duration = duration

        self.duration = duration
