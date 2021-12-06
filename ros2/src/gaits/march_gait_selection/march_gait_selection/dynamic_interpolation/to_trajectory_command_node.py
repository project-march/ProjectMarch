import rclpy
from rclpy.node import Node
from rclpy.time import Time

from march_gait_selection.state_machine.trajectory_scheduler import TrajectoryCommand
from march_utility.utilities.duration import Duration

from rosgraph_msgs.msg import Clock
from march_shared_msgs.msg import FollowJointTrajectoryGoal
from march_shared_msgs.msg import FollowJointTrajectoryActionGoal
from march_shared_msgs.msg import CurrentGait
from control_msgs.msg import JointTrajectoryControllerState
from std_msgs.msg import Header
from actionlib_msgs.msg import GoalID

from dynamic_subgait import DynamicSubgait

import time


class ToTrajectoryCommandNode(Node):
    def __init__(self):
        super().__init__("to_trajectory_command_node")
        self.ros_one_time = None
        self.current_state_msg = None
        self.subgait_duration = 1.5
        self.mid_point_frac = 0.33
        self.id = "right"

        self.get_ros_one_time = self.create_subscription(
            msg_type=Clock,
            topic="/clock",
            callback=self.get_sim_time,
            qos_profile=5,
        )

        self.read_current_state = self.create_subscription(
            msg_type=JointTrajectoryControllerState,
            topic="march/controller/trajectory/state",
            callback=self.current_state_callback,
            qos_profile=5,
        )

        self.publisher_ = self.create_publisher(
            msg_type=FollowJointTrajectoryActionGoal,
            topic="/march/controller/trajectory/follow_joint_trajectory/goal",
            qos_profile=5,
        )

        self.publish_current_gait = self.create_publisher(
            msg_type=CurrentGait,
            topic="/march/gait_selection/current_gait",
            qos_profile=5,
        )

        self.timer = self.create_timer(3, self.publish_trajectory_command)

    def get_sim_time(self, msg):
        clock = msg.clock
        self.ros_one_time = Time(
            seconds=clock.sec,
            nanoseconds=clock.nanosec,
        )

    def current_state_callback(self, msg: JointTrajectoryControllerState):
        self.current_state_msg = msg

    def to_trajectory_command(self):
        """Generate a new trajectory command."""
        start_time = time.time()

        desired_ankle_x = 28
        desired_ankle_y = 3
        # Skip desired velocity for now
        trajectory = DynamicSubgait(
            self.subgait_duration,
            self.mid_point_frac,
            self.current_state_msg,
            self.id,
            desired_ankle_x,
            position_y=desired_ankle_y,
        ).to_joint_trajectory_msg()

        # Send trajectory command to the topic listened to by the simulation
        time_from_start = trajectory.points[-1].time_from_start
        self._current_subgait_duration = Duration.from_msg(time_from_start)

        command = TrajectoryCommand(
            trajectory,
            self._current_subgait_duration,
            "dynamic_joint_trajectory",
            self.ros_one_time,
        )

        stamp = self.ros_one_time.to_msg()
        command.trajectory.header.stamp = stamp
        goal = FollowJointTrajectoryGoal(trajectory=command.trajectory)
        self.publisher_.publish(
            FollowJointTrajectoryActionGoal(
                header=Header(stamp=stamp),
                goal_id=GoalID(stamp=stamp, id=str(command)),
                goal=goal,
            )
        )

        # Publish gait type. This is needed for the CoM plugin to work
        # and thus to be able to groundgait
        current_gait_msg = CurrentGait(
            header=Header(stamp=stamp),
            gait="walk",
            subgait=str(self.id + "_swing"),
            version="dynamic_walk_v1",
            duration=Duration(self.subgait_duration).to_msg(),
            gait_type="walk_like",
        )

        self.publish_current_gait.publish(current_gait_msg)

        if self.id == "right":
            self.id = "left"
        elif self.id == "left":
            self.id = "right"

        print(f"--- {(time.time() - start_time)} seconds ---")

    def publish_trajectory_command(self):
        self.to_trajectory_command()


def main(args=None):
    rclpy.init(args=args)
    to_trajectory_command_node = ToTrajectoryCommandNode()
    rclpy.spin(to_trajectory_command_node)

    to_trajectory_command_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
