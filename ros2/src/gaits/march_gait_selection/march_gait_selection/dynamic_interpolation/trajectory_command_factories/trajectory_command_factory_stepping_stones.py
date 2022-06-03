"""Author: Marten Haitjema, MVII."""

from typing import Dict, Optional
from copy import deepcopy

from march_gait_selection.state_machine.trajectory_scheduler import TrajectoryCommand
from march_gait_selection.dynamic_interpolation.trajectory_command_factories.trajectory_command_factory_fixed_sizes import (
    TrajectoryCommandFactoryFixedSizes,
)


class TrajectoryCommandFactorySteppingStones(TrajectoryCommandFactoryFixedSizes):
    """TrajectoryCommandFactory for the stepping stones gait."""

    def __init__(self, gait, points_handler):
        super().__init__(gait, points_handler)

    def get_trajectory_command(
        self, subgait_id: str, start_position_all_joints: Dict[str, float], start=False, stop=False
    ) -> Optional[TrajectoryCommand]:
        """Return a TrajectoryCommand based on current subgait_id, or based on a predetermined foot location.

         Args:
            subgait_id (str): whether it is a right_swing or left_swing
            start_position_all_joints (Dict[str, float]): start joint angles of all joints
            start (bool): whether it is a start gait, default False
            stop (bool): whether it is a stop gait, default False

        Returns:
            TrajectoryCommand: command with the current subgait and start time
        """
        self.subgait_id = subgait_id
        self.start_position_all_joints = start_position_all_joints

        if stop:
            self._logger.info("Stopping dynamic gait.")
        else:
            if self._gait.use_predetermined_foot_location:
                self.foot_location = deepcopy(self._predetermined_foot_location)
                self._gait.use_predetermined_foot_location = False
            else:
                try:
                    self.foot_location = self._points_handler.get_foot_location(self.subgait_id)
                    stop = self._points_handler.is_foot_location_too_old(self.foot_location)
                    if stop:
                        return None
                except AttributeError:
                    self._logger.info("No FootLocation found. Connect the camera or use simulated points.")
                    self._end = True
                    return None

            if not stop:
                self._points_handler.publish_chosen_foot_position(self.subgait_id, self.foot_location)
                self._logger.info(
                    f"Stepping to location ({self.foot_location.processed_point.x}, "
                    f"{self.foot_location.processed_point.y}, {self.foot_location.processed_point.z})"
                )

        return self._get_first_feasible_trajectory(start, stop)
