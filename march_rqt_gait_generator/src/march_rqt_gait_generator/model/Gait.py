import rospy


class Gait:

    def __init__(self, joints, name="Dummy gait", description="Just a simple gait", duration=12):
        self.joints = joints
        self.name = name
        self.description = description
        self.duration = duration

    def get_joint(self, name):
        for i in range(0, len(self.joints)):
            if self.joints[i].name == name:
                return self.joints[i]
        rospy.logerr("Joint with name " + name + " does not exist in gait " + self.name + ".")
        return None
