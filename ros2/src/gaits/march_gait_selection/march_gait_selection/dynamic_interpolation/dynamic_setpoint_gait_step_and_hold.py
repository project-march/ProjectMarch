"""Author: Marten Haitjema, MVII."""

from queue import Queue
from typing import Optional
from rclpy.node import Node

from march_gait_selection.dynamic_interpolation.dynamic_setpoint_gait_step import DynamicSetpointGaitStep
from march_gait_selection.state_machine.gait_update import GaitUpdate
from march_gait_selection.state_machine.trajectory_scheduler import TrajectoryCommand
from march_utility.utilities.logger import Logger

from march_shared_msgs.msg import FootPosition, GaitInstruction
from geometry_msgs.msg import Point

from march_utility.utilities.utility_functions import get_position_from_yaml

HOLD_FOOT_POSITION = FootPosition(duration=1.3, processed_point=Point(x=0.0, y=0.07, z=0.45))


class DynamicSetpointGaitStepAndHold(DynamicSetpointGaitStep):
    """Class for stepping to a hold position firstly, and to the desired position secondly."""

    def __init__(self, gait_selection_node: Node):
        self.foot_location = FootPosition()
        super().__init__(gait_selection_node)
        self.logger = Logger(gait_selection_node, __class__.__name__)
        self.gait_name = "dynamic_step_and_hold"
        self.first_half = True

    def _reset(self) -> None:
        """Reset all attributes of the gait."""
        self._should_stop = False
        self._end = False

        self._start_time_next_command = None
        self._current_time = None

        self._next_command = None

        self._start_is_delayed = True
        self._scheduled_early = False

        if self.foot_location == HOLD_FOOT_POSITION:
            self.first_half = False

    def _update_state_machine(self) -> GaitUpdate:
        """Update the state machine that the single single step has finished. Also switches the subgait_id.

        Returns:
            GaitUpdate: a GaitUpdate for the state machine
        """
        if not self._trajectory_failed and not self.first_half:
            self.first_half = True
            if self.subgait_id == "right_swing":
                self.subgait_id = "left_swing"
            elif self.subgait_id == "left_swing":
                self.subgait_id = "right_swing"

        if self._end:
            self.subgait_id = "right_swing"
            self._fill_queue()

        return GaitUpdate.finished()

    def _get_trajectory_command(self, start=False, stop=False) -> Optional[TrajectoryCommand]:
        """Return a TrajectoryCommand based on current subgait_id, _use_position_queue and _first_half.

        Args:
            start (Optional[bool]): whether it is a start gait or not, default False
            stop (Optional[bool]): whether it is a stop gait or not, default False
        Returns:
            TrajectoryCommand: command with the current subgait and start time
        """
        if stop:
            self.logger.info("Stopping dynamic gait.")
        else:
            if self.first_half:
                self.foot_location = HOLD_FOOT_POSITION
                self.logger.warn("Stepping to hold position.")
            else:
                if self._use_position_queue:
                    if not self.position_queue.empty():
                        self.foot_location = self._get_foot_location_from_queue()
                    else:
                        stop = True
                        self._end = True
                else:
                    try:
                        self.foot_location = self._get_foot_location(self.subgait_id)
                    except AttributeError:
                        self.logger.info("No FootLocation found. Connect the camera or use simulated points.")
                        self._end = True
                        return None
                    stop = self._check_msg_time(self.foot_location)

                self.logger.warn(
                    f"Stepping to location ({self.foot_location.processed_point.x}, "
                    f"{self.foot_location.processed_point.y}, {self.foot_location.processed_point.z})"
                )

        if start and stop:
            return None

        self.logger.info("Trying to get trajectory.")

        return self._get_first_feasible_trajectory(start, stop)

    def _callback_force_unknown(self, msg: GaitInstruction) -> None:
        """Resets the subgait_id, _trajectory_failed and position_queue after a force unknown.

        Args:
            msg (GaitInstruction): the GaitInstruction message that may contain a force unknown
        """
        if msg.type == GaitInstruction.UNKNOWN:
            # TODO: Refactor such that _reset method can be used
            self.start_position_actuating_joints = self.gait_selection.get_named_position("stand")
            self.start_position_all_joints = get_position_from_yaml("stand")
            self.subgait_id = "right_swing"
            self._trajectory_failed = False
            self.first_half = True
            self.foot_location = FootPosition()
            self.position_queue = Queue()
            self._fill_queue()
