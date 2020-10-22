import rclpy
from rcl_interfaces.srv import GetParameters
from urdf_parser_py import urdf

from .gait_selection import GaitSelection
from march_gait_selection.state_machine.gait_state_machine import GaitStateMachine
from march_gait_selection.state_machine.state_machine_input import StateMachineInput
from march_gait_selection.state_machine.trajectory_scheduler import TrajectoryScheduler




def main():
    rclpy.init()
    gait_selection = GaitSelection()

    # Before we can load the gaits and completely spin the gait selection, we need to get the robot description
    robot_description_client = gait_selection.create_client(srv_type=GetParameters,
                       srv_name='/robot_state_publisher/get_parameters')
    robot_future = robot_description_client.call_async(request=GetParameters.Request(names=['robot_description']))

    gait_selection.get_logger().info("spinning until future complete")
    rclpy.spin_until_future_complete(gait_selection, robot_future)
    gait_selection.get_logger().info("Done spinning")

    # Now we can set the initial robot and load the gaits
    gait_selection._robot = urdf.Robot.from_xml_string(robot_future.result().values[0].string_value)
    gait_selection.load_gaits()

    # balance_gait = BalanceGait.create_balance_subgait(gait_selection['balance_walk'])
    # if balance_gait is not None:
    #     gait_selection.add_gait(balance_gait)
    scheduler = TrajectoryScheduler(gait_selection) #'/march/controller/trajectory/follow_joint_trajectory')
    gait_state_machine = GaitStateMachine(gait_selection, scheduler)
    # rospy.loginfo('Gait state machine successfully generated')
    # rospy.core.add_preshutdown_hook(lambda reason: gait_state_machine.request_shutdown())

    gait_state_machine.run()
    rclpy.spin(gait_selection)
