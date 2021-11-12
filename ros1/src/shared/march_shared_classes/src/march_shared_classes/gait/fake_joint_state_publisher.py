from sensor_msgs.msg import JointState
import rospy
import numpy as np

from march_shared_classes.gait.dynamic_joint_trajectory import DynamicJointTrajectory


def publish_fake_current_joint_states():
    publisher = rospy.Publisher("/test/joint_states", JointState, queue_size=5)
    rospy.init_node("publish_fake_current_joint_states", anonymous=True)
    rate = rospy.Rate(0.25)
    while not rospy.is_shutdown():
        current_position = np.deg2rad([0.0, 8.0, 18.0, 2.0, 2.0, -9.5, 8.0, 0.0])
        current_velocity = np.deg2rad([0.0, 0.0, -20.2, 0.0, 0.0, 0.0, 0.0, 0.0])

        # joint_trajectory_msg = trajectory_msg.JointTrajectory()
        joint_state_msg = JointState()
        joint_state_msg.name = [
            "left_ankle",
            "left_hip_aa",
            "left_hip_fe",
            "left_knee",
            "right_ankle",
            "right_hip_aa",
            "right_hip_fe",
            "right_knee",
        ]
        joint_state_msg.position = current_position
        joint_state_msg.velocity = current_velocity

        publisher.publish(joint_state_msg)
        rate.sleep()


if __name__ == "__main__":
    try:
        publish_fake_current_joint_states()
    except rospy.ROSInterruptException:
        pass