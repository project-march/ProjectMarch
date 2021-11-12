from re import S
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from march_gait_selection.state_machine.trajectory_scheduler import TrajectoryCommand
from march_utility.utilities.duration import Duration

# from march_utility.gait.joint_trajectory import JointTrajectory
# from march_gait_selection.state_machine.trajectory_scheduler import TrajectoryScheduler
from trajectory_msgs import msg as trajectory_msg
from march_shared_msgs.msg import (
    FollowJointTrajectoryGoal,
    FollowJointTrajectoryActionGoal,
)
from std_msgs.msg import Header
from actionlib_msgs.msg import GoalID

class ToTrajectoryCommandNode(Node):
    def __init__(self):
        print("initializing ToTrajectoryCommandNode")
        super().__init__("to_trajectory_command_node")
        self.subscription = self.create_subscription(
            msg_type=trajectory_msg.JointTrajectory,
            topic="/test/joint_trajectory_msg",
            callback=self.to_trajectory_command,
            qos_profile=5,
        )
        self.subscription

        self.publisher_ = self.create_publisher(
            msg_type=FollowJointTrajectoryActionGoal,
            topic="/march/controller/trajectory/follow_joint_trajectory/goal",
            qos_profile=5,
        )

    def to_trajectory_command(self, msg):
        """Generate a new trajectory command."""
        # From joint_trajectory_msg to TrajectoryCommand
        trajectory = msg

        time_from_start = trajectory.points[-1].time_from_start
        self._current_subgait_duration = Duration.from_msg(time_from_start)
        # _start_time = self.get_clock().now().to_msg()
        self._start_time = Time(seconds=30, nanoseconds=0)
        # self._start_time["goal"]
        # self._end_time = 1
        print(self._start_time, '\n', self._current_subgait_duration)
        self.get_logger().info("Creating TrajectoryCommand")
        command = TrajectoryCommand(
            trajectory,
            self._current_subgait_duration,
            "dynamic_joint_trajectory",
            self._start_time,
        )

        # From TrajectoryCommand to FollowJointTrajectoryGoal.msg
        self.get_logger().info("Publishing FollowJointTrajectoryActionGoal")
        stamp = self._start_time.to_msg()
        command.trajectory.header.stamp = stamp
        goal = FollowJointTrajectoryGoal(trajectory=command.trajectory)
        self.publisher_.publish(
            FollowJointTrajectoryActionGoal(
                header=Header(stamp=stamp),
                goal_id=GoalID(stamp=stamp, id=str(command)),
                goal=goal,
            )
        )

def main(args=None):
    rclpy.init(args=args)
    to_trajectory_command_node = ToTrajectoryCommandNode()
    rclpy.spin(to_trajectory_command_node)

    to_trajectory_command_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
