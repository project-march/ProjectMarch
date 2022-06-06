"""Author: Marten Haitjema, MVII."""

from march_gait_selection.dynamic_interpolation.trajectory_command_factories.trajectory_command_factory_queue import (
    TrajectoryCommandFactoryQueue,
)
from march_shared_msgs.msg import FootPosition
from march_utility.exceptions.gait_exceptions import WrongStartPositionError
from march_utility.utilities.node_utils import DEFAULT_HISTORY_DEPTH
from std_msgs.msg import String
from geometry_msgs.msg import Point

PREDETERMINED_FOOT_LOCATIONS = {
    "small_narrow": FootPosition(duration=1.5, processed_point=Point(x=0.55, y=0.03, z=0.44699999999999995)),
    "small_wide": FootPosition(duration=1.5, processed_point=Point(x=0.65, y=0.03, z=0.44699999999999995)),
    "large_narrow": FootPosition(duration=2.0, processed_point=Point(x=0.75, y=0.03, z=0.44699999999999995)),
    "large_wide": FootPosition(duration=2.0, processed_point=Point(x=0.85, y=0.03, z=0.44699999999999995)),
}


class TrajectoryCommandFactoryFixedSizes(TrajectoryCommandFactoryQueue):
    """TrajectoryCommandFactory class with the ability to use predetermined step sizes and change the start side."""

    def __init__(self, gait, points_handler):
        super().__init__(gait, points_handler)
        self._logger = gait.node.get_logger().get_child(__class__.__name__)

        self._gait.node.create_subscription(
            String,
            "/march/step_and_hold/start_side",
            self._set_start_subgait_id,
            DEFAULT_HISTORY_DEPTH,
        )
        self._gait.node.create_subscription(
            String,
            "/march/step_and_hold/step_size",
            self._predetermined_foot_location_callback,
            DEFAULT_HISTORY_DEPTH,
        )

    def _set_start_subgait_id(self, start_side: String) -> None:
        """Sets the subgait_id to the given start side, if and only if exo is in homestand."""
        try:
            if self.start_position_all_joints == self._gait.home_stand_position_all_joints:
                self.subgait_id = start_side.data
                if self.subgait_id == "left_swing":
                    self._gait.start_from_left_side = True
                else:
                    self._gait.start_from_left_side = False
                self._logger.info(f"Starting subgait set to {self.subgait_id}")
            else:
                raise WrongStartPositionError(self._gait.home_stand_position_all_joints, self.start_position_all_joints)
        except WrongStartPositionError as e:
            self._logger.warn(f"Can only change start side in home stand position. {e.msg}")

    def _predetermined_foot_location_callback(self, msg: String) -> None:
        self._gait.use_predetermined_foot_location = True
        self._predetermined_foot_location = PREDETERMINED_FOOT_LOCATIONS[msg.data]
        self._logger.info(
            f"Stepping to stone {msg.data} with a step size of {self._predetermined_foot_location.processed_point.x}"
        )
