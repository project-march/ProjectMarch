from sensor_msgs.msg import JointState
import rospy
import numpy as np
import matplotlib.pyplot as plt

from march_shared_classes.gait.dynamic_joint_trajectory import DynamicJointTrajectory
from march_shared_classes.gait.joint_trajectory import JointTrajectory
from trajectory_msgs import msg as trajectory_msg

publisher = rospy.Publisher(
    "/test/joint_trajectory_msg", trajectory_msg.JointTrajectory, queue_size=5
)

def listener():
    rospy.init_node("listener", anonymous=True)
    rospy.Subscriber("/test/joint_states", JointState, calculate_joint_trajectories)
    rospy.spin()

def calculate_joint_trajectories(JointState):
    dynamic_joint_trajectory = DynamicJointTrajectory()
    dynamic_joint_trajectory.set_middle_state()
    dynamic_joint_trajectory.get_desired_state()
    dynamic_joint_trajectory.get_current_state(JointState)
    position, velocity = dynamic_joint_trajectory.interpolate_setpoints()

    joint_trajectory_msg = dynamic_joint_trajectory.to_joint_trajectory_msg()
    # print(joint_trajectory_msg)
    publisher.publish(joint_trajectory_msg)

    print("Publishing joint_trajectory_msg")
    # time = np.linspace(0, 1, 50)
    # interpolated_position = position(time)
    # interpolated_velocity = velocity(time)

    # plt.figure()
    # plt.plot(time, interpolated_position)
    # plt.figure()
    # plt.plot(time, interpolated_velocity)
    # plt.show()


if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
