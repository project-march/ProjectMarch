import cProfile
import signal
import sys

import rclpy
from rclpy.executors import MultiThreadedExecutor

from .gait_selection import GaitSelection
from march_gait_selection.state_machine.gait_state_machine import GaitStateMachine
from march_gait_selection.state_machine.trajectory_scheduler import TrajectoryScheduler


def sys_exit(*_):
    sys.exit(0)


def main():
    """ Starts up the gait selection node with the state machine and scheduler. """
    with cProfile.Profile() as pr:
        rclpy.init()

        gait_selection = GaitSelection()
        scheduler = TrajectoryScheduler(gait_selection)
        gait_state_machine = GaitStateMachine(gait_selection, scheduler)
        gait_state_machine.run()
        executor = MultiThreadedExecutor()

        signal.signal(signal.SIGTERM, sys_exit)

        try:
            rclpy.spin(gait_selection, executor)
        except KeyboardInterrupt:
            pass

        rclpy.shutdown()
    pr.dump_stats("march_gait_selection.prof")
