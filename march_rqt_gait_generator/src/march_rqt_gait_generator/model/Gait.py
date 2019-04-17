import rospy


class Gait:

    def __init__(self, joints, duration, name="Dummy gait", description="Just a simple gait"):
        self.joints = joints
        self.name = name
        self.description = description
        self.duration = duration
        self.current_time = 0

    def get_joint(self, name):
        for i in range(0, len(self.joints)):
            if self.joints[i].name == name:
                return self.joints[i]
        rospy.logerr("Joint with name " + name + " does not exist in gait " + self.name + ".")
        return None

    def set_current_time(self, time):
        rospy.logerr("Time" + str(time))
        self.current_time = time
