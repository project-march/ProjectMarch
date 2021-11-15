from sensor_msgs.msg import JointState
import rospy
from contextlib import suppress

from march_shared_classes.gait.dynamic_joint_trajectory import DynamicJointTrajectory
from trajectory_msgs import msg as trajectory_msg

publisher = rospy.Publisher(
    "/test/joint_trajectory_msg", trajectory_msg.JointTrajectory, queue_size=5
)


def listener():
    rospy.init_node("listener", anonymous=True)
    rospy.Subscriber("/test/joint_states", JointState, calculate_joint_trajectories)
    rospy.spin()


def calculate_joint_trajectories(joint_state):
    dynamic_joint_trajectory = DynamicJointTrajectory()
    dynamic_joint_trajectory.set_middle_state()
    dynamic_joint_trajectory.get_desired_state()
    dynamic_joint_trajectory.get_current_state(joint_state)
    position, velocity = dynamic_joint_trajectory.interpolate_setpoints()

    joint_trajectory_msg = dynamic_joint_trajectory.to_joint_trajectory_msg()
    publisher.publish(joint_trajectory_msg)
    print("Publishing joint_trajectory_msg")


if __name__ == "__main__":
    with suppress(rospy.ROSInterruptException):
        listener()
