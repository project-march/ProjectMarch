"""Author: MVI."""

from __future__ import annotations

from threading import Event
from typing import Optional, List, Dict, TYPE_CHECKING, Union

from march_gait_selection.state_machine.gait_update import GaitUpdate
from march_gait_selection.gaits.setpoints_gait import SetpointsGait

if TYPE_CHECKING:
    from march_gait_selection.gait_selection import GaitSelection

from march_shared_msgs.msg import GaitParameters
from march_shared_msgs.srv import GetGaitParameters
from march_utility.gait.edge_position import (
    StaticEdgePosition,
    EdgePosition,
    DynamicEdgePosition,
)
from march_utility.gait.subgait import Subgait
from march_utility.gait.subgait_graph import SubgaitGraph
from march_utility.utilities.duration import Duration
from march_utility.utilities.logger import Logger
from march_utility.utilities.dimensions import (
    InterpolationDimensions,
    amount_of_subgaits,
    amount_of_parameters,
)
from march_utility.exceptions.gait_exceptions import (
    UnknownDimensionsError,
    WrongRealSenseConfigurationError,
    NonValidGaitContentError,
)
from rclpy import Future
from rclpy.time import Time
from rclpy.client import Client
from urdf_parser_py import urdf


class RealsenseGait(SetpointsGait):
    """The RealsenseGait class is used for creating gaits based on the parameters given by the realsense reader.

    From these parameters the subgaits to interpolate are interpolated after a realsense call during the start
    of the gait. It is based on the setpoints gait, and it uses the interpolation over 1 or 2 dimensions with 2 or 4
    subgaits respectively.

    Args:
        gait_name (str): Name of the gait
        subgaits (dict): Mapping of names to subgait instances
        graph (SubgaitGraph): Mapping of subgait names to transitions
        gait_selection (GaitSelection): the gait selection node
        realsense_category (str): ??? TODO: Add docs
        camera_to_use (str): which camera to use
        subgaits_to_interpolate (dict): dict containing subgaits
        dimensions (InterpolationDimensions): dimensions of interpolation
        process_service (Client): The service from which to get the gait parameters
        starting_position (EdgePosition): starting position of the gait
        final_position (EdgePosition): final position of the gait
        parameters (List[float]): parameters
        dependent_on (List[str]): ??? TODO: Add docs
        responsible_for (List[str]): ??? TODO: Add docs

    Attributes:
        logger (Logger): used to log to the terminal
        parameters (List[float): ??? TODO: Add docs
        dimensions (InterpolationDimensions): ??? TODO: Add docs
        realsense_category (str): ??? TODO: Add docs
        camera_to_use (str): which camera to use
        subgaits_to_interpolate (dict): subgaits that are used for interpolation, amount should be equal to the
            dimensions of interpolation
        realsense_service_event (Event): ??? TODO: Add docs
        realsense_service_result (???): ??? TODO: Add docs
        _gait_selection (GaitSelection): the gait selection node
        _get_gait_parameters_service (Client): The service from which to get the gait parameters
        _dependent_on (List[str]): ??? TODO: Add docs
        _responsible_for (List[str]): ??? TODO: Add docs

    """

    SERVICE_TIMEOUT = Duration(seconds=2.0)
    INITIAL_START_DELAY_TIME = Duration(seconds=10.0)
    CAMERA_NAME_MAP = {
        "front": GetGaitParameters.Request.CAMERA_FRONT,
        "back": GetGaitParameters.Request.CAMERA_BACK,
    }
    REALSENSE_CATEGORY_MAP = {
        "stairs_up": GetGaitParameters.Request.STAIRS_UP,
        "stairs_down": GetGaitParameters.Request.STAIRS_DOWN,
        "ramp_up": GetGaitParameters.Request.RAMP_UP,
        "ramp_down": GetGaitParameters.Request.RAMP_DOWN,
        "sit": GetGaitParameters.Request.SIT,
        "curb_up": GetGaitParameters.Request.CURB_UP,
        "curb_down": GetGaitParameters.Request.CURB_DOWN,
    }

    def __init__(
        self,
        gait_name: str,
        subgaits: dict,
        graph: SubgaitGraph,
        gait_selection: GaitSelection,
        realsense_category: str,
        camera_to_use: str,
        subgaits_to_interpolate: dict,
        dimensions: InterpolationDimensions,
        process_service: Client,
        starting_position: EdgePosition,
        final_position: EdgePosition,
        parameters: List[float],
        dependent_on: List[str],
        responsible_for: List[str],
    ):
        super(RealsenseGait, self).__init__(gait_name, subgaits, graph)
        self._gait_selection = gait_selection
        self.logger = Logger(self._gait_selection, __class__.__name__)
        self.parameters = parameters
        self.dimensions = dimensions
        self.realsense_category = self.realsense_category_from_string(realsense_category)
        self.camera_to_use = self.camera_msg_from_string(camera_to_use)
        self.subgaits_to_interpolate = subgaits_to_interpolate
        # Set up service and event for asynchronous
        self._get_gait_parameters_service = process_service
        self.realsense_service_event = Event()
        self.realsense_service_result = None
        self._starting_position = starting_position
        self._final_position = final_position
        self._dependent_on = dependent_on
        self._responsible_for = responsible_for

    @property
    def dependent_on(self):
        """Return of what it is dependent TODO: what does this mean?"""
        return self._dependent_on

    @property
    def responsible_for(self):
        """Return for what it is responsible TODO: what does this mean?"""
        return self._responsible_for

    @property
    def subsequent_subgaits_can_be_scheduled_early(self) -> bool:
        """Whether a subgait can be scheduled early.

        This is not possible for the realsense gait, since this will later have a service call to
        determine the next subgait.
        """
        return True

    @property
    def first_subgait_can_be_scheduled_early(self) -> bool:
        """Whether the first subgait can be started with a delay, this is possible for the realsense gait."""
        return True

    @property
    def starting_position(self) -> EdgePosition:
        """Returns the starting position of the subgait."""
        return self._starting_position

    @property
    def final_position(self) -> EdgePosition:
        """Returns the final position of the subgait."""
        return self._final_position

    @classmethod
    def from_yaml(
        cls,
        gait_selection: GaitSelection,
        robot: urdf.Robot,
        gait_name: str,
        gait_config: dict,
        gait_graph: dict,
        gait_directory: str,
        process_service: Client,
    ) -> RealsenseGait:
        """Construct a realsense gait from the gait_config from the realsense_gaits.yaml.

        Args:
            gait_selection (GaitSelection): The GaitSelection node that will be used for making the service calls to the
                realsense reader.
            robot (urdf.Robot): The urdf robot that can be used to verify the limits of the subgaits
            gait_name (str): The name of the gait
            gait_config (dict): the yaml node with the needed configurations
            gait_graph (dict): the graph from the .gait file with subgait transitions
            gait_directory (str): the gait_directory that is being used
            process_service (Client): The service from which to get the gait parameters
        Returns:
            RealsenseGait: the constructed realsense gait
        Raises:
            WrongRealSenseConfigurationError

        """
        graph = SubgaitGraph(gait_graph)
        subgaits_to_interpolate = {}
        try:
            dimensions = InterpolationDimensions.from_integer(gait_config["dimensions"])
            dependent_on = gait_config.get("dependent_on", [])
            responsible_for = gait_config.get("responsible_for", [])

            parameters = [0.0 for _ in range(amount_of_parameters(dimensions))]

            realsense_category = gait_config["realsense_category"]
            camera_to_use = gait_config["camera_to_use"]
            subgait_version_map = gait_config["subgaits"]
            # Create subgaits to interpolate with
            for subgait_name in subgait_version_map:
                subgaits_to_interpolate[subgait_name] = [
                    Subgait.from_name_and_version(robot, gait_directory, gait_name, subgait_name, version)
                    for version in subgait_version_map[subgait_name]
                ]
                if len(subgaits_to_interpolate[subgait_name]) != amount_of_subgaits(dimensions):
                    raise WrongRealSenseConfigurationError(
                        f"The amount of subgaits in the realsense version map "
                        f"({len(subgaits_to_interpolate[subgait_name])}) doesn't match "
                        f"the amount of dimensions for subgait {subgait_name}"
                    )

            subgaits = {}
            for subgait_name in subgait_version_map:
                if subgait_name not in ("start", "end"):
                    subgaits[subgait_name] = Subgait.interpolate_n_subgaits(
                        dimensions=dimensions,
                        subgaits=subgaits_to_interpolate[subgait_name],
                        parameters=parameters,
                        use_foot_position=True,
                    )

            starting_position = cls.parse_edge_position(
                gait_config["starting_position"],
                subgaits[graph.start_subgaits()[0]].starting_position,
            )
            final_position = cls.parse_edge_position(
                gait_config["final_position"],
                subgaits[graph.end_subgaits()[0]].final_position,
            )

        except KeyError as e:
            raise WrongRealSenseConfigurationError(
                f"There was a missing key to create realsense gait in gait {gait_name}: {e}"
            )
        except ValueError as e:
            raise WrongRealSenseConfigurationError(
                f"There was a wrong value in the config for the realsense gait {gait_name}: {e}"
            )
        return cls(
            gait_name=gait_name,
            subgaits=subgaits,
            graph=graph,
            gait_selection=gait_selection,
            realsense_category=realsense_category,
            camera_to_use=camera_to_use,
            subgaits_to_interpolate=subgaits_to_interpolate,
            dimensions=dimensions,
            process_service=process_service,
            starting_position=starting_position,
            final_position=final_position,
            parameters=parameters,
            dependent_on=dependent_on,
            responsible_for=responsible_for,
        )

    @classmethod
    def parse_edge_position(
        cls, config_value: str, position_values: Dict[str, float]
    ) -> Union[StaticEdgePosition, DynamicEdgePosition]:
        """Parse the edge position based on the string in the realsense_gaits.yaml.

        Args:
            config_value (str): the value in yaml file
            position_values (Dict[str, float]): the actual joint positions at the edge of the gait
        Returns
            StaticEdgePosition: if config_value is static
            DynamicEdgePosition: if config_value is dynamic
        Raises:
            WrongRealSenseConfigurationError: raised when config_value is not valid
        """
        if config_value == "static":
            return StaticEdgePosition(position_values)
        elif config_value == "dynamic":
            return DynamicEdgePosition(position_values)
        else:
            raise WrongRealSenseConfigurationError(
                "The edge position did not have a "
                "valid value, should be static or "
                f"dynamic, but was `{config_value}`"
            )

    @classmethod
    def realsense_category_from_string(cls, gait_name: str) -> int:
        """Construct the realsense gait from the string in the realsense_gaits.yaml.

        Args:
            gait_name (str): the string from the config
        Returns:
            int: The integer to send to the realsense reader to define the category.
        """
        if gait_name not in cls.REALSENSE_CATEGORY_MAP:
            raise WrongRealSenseConfigurationError(
                f"Gait name {gait_name} from the config is not known as a possible "
                f"realsense reader gait configuration"
            )
        return cls.REALSENSE_CATEGORY_MAP[gait_name]

    @classmethod
    def camera_msg_from_string(cls, camera_name: str) -> int:
        """Construct the camera name msg from the string in the realsense_gaits.yaml.

        Args:
            camera_name (str): The string from the config.

        Returns:
            int: The integer to send to the realsense reader to define the camera.
        """
        if camera_name not in cls.CAMERA_NAME_MAP:
            raise WrongRealSenseConfigurationError(
                f"The camera configuration {camera_name} from the realsense_gaits.yaml"
                f"is not one of the known camera names: {cls.CAMERA_NAME_MAP.keys()}"
            )
        return cls.CAMERA_NAME_MAP[camera_name]

    DEFAULT_FIRST_SUBGAIT_DELAY_START_RS_DURATION = Duration(0)

    def start(
        self,
        current_time: Time,
        first_subgait_delay: Optional[Duration] = DEFAULT_FIRST_SUBGAIT_DELAY_START_RS_DURATION,
    ) -> GaitUpdate:
        """This function is called to start the realsense gait.

        It does the following:
        1) Make a service call to march_realsense_reader.
        2) Update all subgaits to interpolated subgaits with the given parameters
        (this will later become only some subgaits when the update function is
        also used).
        3) Update the gait parameters to prepare for start
        4) Return the first subgait, if correct parameters were found.

        Args:
            current_time (Time): current time
            first_subgait_delay (:obj: Duration, optional): delay with which the first subgait will be
                scheduled. Defaults to zero
        Returns:
            GaitUpdate: A gait update that tells the state machine what to do. Empty means
        that that state machine should not start a gait.
        """
        self._reset()
        # Delay start until parameterization is done
        self._start_is_delayed = True
        # Start time will be set later, but to prevent updates during the service
        # calls to think the gait start time has passed, set start time in the future.
        self._start_time = current_time + self.INITIAL_START_DELAY_TIME
        self._current_time = current_time
        # If a gait is dependent on some other gait its subgaits are already
        # interpolated from parameters so we can skip the realsense call
        if not self._dependent_on:
            realsense_update_successful = self.get_realsense_update()
            if not realsense_update_successful:
                return GaitUpdate.empty()

        self._current_subgait = self.subgaits[self.graph.start_subgaits()[0]]
        self._next_subgait = self._current_subgait
        if first_subgait_delay is None:
            first_subgait_delay = self.DEFAULT_FIRST_SUBGAIT_DELAY_START_RS_DURATION
        self._start_time = self._gait_selection.get_clock().now() + first_subgait_delay
        self._end_time = self._start_time + self._current_subgait.duration
        return GaitUpdate.should_schedule_early(self._command_from_current_subgait())

    def get_realsense_update(self) -> bool:
        """Makes a realsense service call and handles the result.

        Returns:
            bool: Whether the call was successful
        """
        service_call_succesful = self.make_realsense_service_call()
        if not service_call_succesful:
            self.logger.warn("No service response received within timeout")
            return False

        gait_parameters_response = self.realsense_service_result
        if gait_parameters_response is None or not gait_parameters_response.success:
            self.logger.warn(f"No gait parameters were found, gait will not be started, {gait_parameters_response}")
            return False

        return self.update_gaits_from_realsense_call(gait_parameters_response.gait_parameters)

    def update_gaits_from_realsense_call(self, gait_parameters: GaitParameters) -> bool:
        """Update the gait parameters based on the message of the current gaits and its responsibilities.

        Args:
            gait_parameters (GaitParameters): The parameters to update to
        Returns:
            bool: True if successfully updated
        """
        success = True
        self.set_parameters(gait_parameters)
        success &= self.interpolate_subgaits_from_parameters()
        if self._responsible_for and success:
            for gait_name in self._responsible_for:
                gait = self._gait_selection.gaits[gait_name]
                # Make a recursive call to also handle the dependencies of the
                # dependent gait
                if isinstance(gait, RealsenseGait):
                    gait.update_gaits_from_realsense_call(gait_parameters)
        return success

    def make_realsense_service_call(self) -> bool:
        """Make a call to the realsense service, if it is available and returns the response.

        Returns:
            bool: Whether the call was successful
        """
        if self._current_subgait is not None:
            subgait_name = self._current_subgait.subgait_name
        else:
            # Assume that the gait is starting and use the first subgait name
            subgait_name = self.graph.start_subgaits()[0]

        request = GetGaitParameters.Request(
            realsense_category=self.realsense_category,
            camera_to_use=self.camera_to_use,
            subgait_name=subgait_name,
        )
        self.realsense_service_event.clear()
        if self._get_gait_parameters_service.wait_for_service(timeout_sec=self.SERVICE_TIMEOUT.seconds):
            gait_parameters_response_future = self._get_gait_parameters_service.call_async(request)
            gait_parameters_response_future.add_done_callback(self._realsense_response_cb)
        else:
            self.logger.error(
                f"The service took longer than {self.SERVICE_TIMEOUT} to become "
                f"available, is the realsense reader running?"
            )
            return False

        return self.realsense_service_event.wait(timeout=self.SERVICE_TIMEOUT.seconds)

    def _realsense_response_cb(self, future: Future) -> None:
        """Set capture point result when the capture point service returns.

        Args:
            future (Future): ??? TODO: Add docs
        """
        self.realsense_service_result = future.result()
        self.realsense_service_event.set()

    def interpolate_subgaits_from_parameters(self) -> bool:
        """Change all subgaits to one interpolated from the current parameters.

        Returns:
            bool: True if successfully interpolated
        Raises:
            NonValidGaitContentError: if subgaits cannot be set
        """
        self.logger.info(f"Interpolating gait {self.gait_name} with parameters: {self.parameters}")

        new_subgaits = {}
        for subgait_name in self.subgaits.keys():
            new_subgaits[subgait_name] = Subgait.interpolate_n_subgaits(
                dimensions=self.dimensions,
                subgaits=self.subgaits_to_interpolate[subgait_name],
                parameters=self.parameters,
                use_foot_position=True,
            )

        try:
            self.set_subgaits(new_subgaits, self._gait_selection)
        except NonValidGaitContentError:
            return False

        return True

    def set_parameters(self, gait_parameters: GaitParameters) -> None:
        """Set the gait parameters based on the message.

        Args:
            gait_parameters (GaitParameters): The parameters to set
        """
        if self.dimensions == InterpolationDimensions.ONE_DIM:
            self.parameters = [gait_parameters.first_parameter]
        elif self.dimensions == InterpolationDimensions.TWO_DIM:
            self.parameters = [
                gait_parameters.first_parameter,
                gait_parameters.second_parameter,
            ]
        else:
            raise UnknownDimensionsError(self.dimensions)

    def set_edge_positions(self, starting_position: EdgePosition, final_position: EdgePosition) -> None:
        """Set the new edge positions.

        Overrides from the setpoints gait, which does not store the starting or final position.

        Args:
            starting_position (EdgePosition): starting position of the gait
            final_position (EdgePosition): final position of the gait
        """
        self._starting_position = starting_position
        self._final_position = final_position
