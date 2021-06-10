from threading import Event
from typing import Optional, List

from march_gait_selection.state_machine.gait_update import GaitUpdate
from march_gait_selection.state_machine.setpoints_gait import SetpointsGait
from march_shared_msgs.msg import GaitParameters
from march_shared_msgs.srv import GetGaitParameters
from march_utility.gait.subgait import Subgait
from march_utility.gait.subgait_graph import SubgaitGraph
from march_utility.utilities.duration import Duration
from march_utility.utilities.dimensions import (
    InterpolationDimensions,
    amount_of_subgaits,
    amount_of_parameters,
)
from march_utility.exceptions.gait_exceptions import (
    UnknownDimensionsError,
    WrongRealSenseConfigurationError,
)
from rclpy import Future
from rclpy.node import Node
from rclpy.time import Time
from rclpy.client import Client
from urdf_parser_py import urdf



class RealSenseGait(SetpointsGait):
    """
    The RealSenseGait class is used for creating gaits based on the parameters given
    by the realsense reader. It is based on the setpoints gait, and it uses the
    interpolation over 1 or 2 dimensions with 2 or 4 subgaits respectively.
    """

    SERVICE_TIMEOUT = Duration(seconds=2.0)
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
    }
    SUBGAIT_NAME_TO_REALSENSE_FRAME_ID_MAP = {
        "right_open": "foot_right",
        "left_open": "foot_left",
        "right_swing": "foot_right",
        "left_swing": "foot_left",
        "right_close": "foot_right",
        "left_close": "foot_left",
        "sit_down": "foot_right",
    }

    def __init__(
        self,
        gait_name: str,
        subgaits: dict,
        graph: SubgaitGraph,
        node: Node,
        realsense_category: str,
        camera_to_use: str,
        subgaits_to_interpolate: dict,
        dimensions: InterpolationDimensions,
        parameters: List[float],
        process_service: Client,
    ):
        super(RealSenseGait, self).__init__(gait_name, subgaits, graph)
        self._node = node
        self.parameters = parameters
        self.dimensions = dimensions
        self.realsense_category = self.realsense_category_from_string(
            realsense_category
        )
        self.camera_to_use = self.camera_msg_from_string(camera_to_use)
        self.subgaits_to_interpolate = subgaits_to_interpolate
        # Set up service and event for asynchronous
        self._get_gait_parameters_service = process_service
        self.realsense_service_event = Event()
        self.realsense_service_result = None

    @property
    def can_be_scheduled_early(self) -> bool:
        return False

    @classmethod
    def from_yaml(
        cls,
        node: Node,
        robot: urdf.Robot,
        gait_name: str,
        gait_config: dict,
        gait_graph: dict,
        gait_directory: str,
        process_service: Client,
    ):
        """
        Construct a realsense gait from the gait_config from the realsense_gaits.yaml.

        :param node: The node that will be used for making the service calls to the
        realsense reader.
        :param robot: The urdf robot that can be used to verify the limits of the
        subgaits.
        :param gait_name: The name of the gait.
        :param gait_config: The yaml node with the needed configurations.
        :param gait_graph: The graph from the .gait file with the subgait transitions.
        :param gait_directory: The gait_directory that is being used.
        :return: The constructed RealSenseGait
        """
        graph = SubgaitGraph(gait_graph)
        subgaits_to_interpolate = {}
        try:
            dimensions = InterpolationDimensions.from_integer(gait_config["dimensions"])
            parameters = [float(param) for param in gait_config["default_parameters"]]
            if len(parameters) != amount_of_parameters(dimensions):
                raise WrongRealSenseConfigurationError(
                    f"The amount of parameters in the config file ({len(parameters)}), "
                    f"doesn't match the dimensions"
                )
            realsense_category = gait_config["realsense_category"]
            camera_to_use = gait_config["camera_to_use"]
            subgait_version_map = gait_config["subgaits"]
            # Create subgaits to interpolate with
            for subgait_name in subgait_version_map:
                subgaits_to_interpolate[subgait_name] = [
                    Subgait.from_name_and_version(
                        robot, gait_directory, gait_name, subgait_name, version
                    )
                    for version in subgait_version_map[subgait_name]
                ]
                if len(subgaits_to_interpolate[subgait_name]) != amount_of_subgaits(
                    dimensions
                ):
                    raise WrongRealSenseConfigurationError(
                        f"The amount of subgaits in the realsense version map "
                        f"({len(subgaits_to_interpolate[subgait_name])}) doesn't match "
                        f"the amount of dimensions for subgait {subgait_name}"
                    )

            subgaits = dict()
            for subgait_name in subgait_version_map:
                if subgait_name not in ("start", "end"):
                    subgaits[subgait_name] = Subgait.interpolate_n_subgaits(
                        dimensions=dimensions,
                        subgaits=subgaits_to_interpolate[subgait_name],
                        parameters=parameters,
                        use_foot_position=True,
                    )

        except KeyError as e:
            raise WrongRealSenseConfigurationError(
                f"There was a missing key to create realsense gait in gait {gait_name}:"
                f" {e}"
            )
        except ValueError as e:
            raise WrongRealSenseConfigurationError(
                f"There was a wrong value in the config for the realsense gait"
                f" {gait_name}: {e}"
            )
        return cls(
            gait_name,
            subgaits,
            graph,
            node,
            realsense_category,
            camera_to_use,
            subgaits_to_interpolate,
            dimensions,
            parameters,
            process_service,
        )

    @classmethod
    def realsense_category_from_string(cls, gait_name: str) -> int:
        """
        Construct the realsense gait from the string in the realsense_gaits.yaml.

        :param gait_name: The string from the config.
        :return: The integer to send to the realsense reader to define the category.
        """
        if gait_name not in cls.REALSENSE_CATEGORY_MAP:
            raise WrongRealSenseConfigurationError(
                f"Gait name {gait_name} from the config is not known as a possible "
                f"realsense reader gait configuration"
            )
        return cls.REALSENSE_CATEGORY_MAP[gait_name]

    @classmethod
    def camera_msg_from_string(cls, camera_name: str) -> int:
        """
        Construct the camera name msg from the string in the realsense_gaits.yaml.

        :param camera_name: The string from the config.
        :return: The integer to send to the realsense reader to define the camera.
        """
        if camera_name not in cls.CAMERA_NAME_MAP:
            raise WrongRealSenseConfigurationError(
                f"The camera configuration {camera_name} from the realsense_gaits.yaml"
                f"is not one of the known camera names: {cls.CAMERA_NAME_MAP.keys()}"
            )
        return cls.CAMERA_NAME_MAP[camera_name]

    def start(
        self, current_time: Time, first_subgait_delay: Optional[Duration] = Duration(0)
    ) -> GaitUpdate:
        """
        This function is called to start the realsense gait, it does the following.
        1) Make a service call to march_realsense_reader.
        2) Update all subgaits to interpolated subgaits with the given parameters
        (this will later become only some of the subgaits when the update function is
         also used).
        3) Update the gait parameters to prepare for start
        4) Return the first subgait, if correct parameters were found.

        :return: A gait update that tells the state machine what to do. Empty means
        that that state machine should not start a gait.
        """
        self._reset()
        self._current_time = current_time
        self._current_subgait = self.subgaits[self.graph.start_subgaits()[0]]
        self._next_subgait = self._current_subgait

        # Currently, we hardcode foot_right in start, since this is almost
        # always a right_open
        frame_id_to_transform_to = self.get_frame_id_to_transfrom_to()
        if frame_id_to_transform_to is None:
            self._node.get_logger().warn(
                "No frame id to transform to was found. gait will not be started"
            )
            return GaitUpdate.empty()

        service_call_succesful = self.make_realsense_service_call(
            frame_id_to_transform_to
        )
        if not service_call_succesful:
            self._node.get_logger().warn("No service response received within timeout")
            return GaitUpdate.empty()

        gait_parameters_response = self.realsense_service_result
        if gait_parameters_response is None or not gait_parameters_response.success:
            self._node.get_logger().warn(
                "No gait parameters were found, gait will not be started"
            )
            return GaitUpdate.empty()

        self.update_parameters(gait_parameters_response.gait_parameters)
        self.interpolate_subgaits_from_parameters()

        # Delay first subgait if duration is greater than zero
        if first_subgait_delay is not None and first_subgait_delay > Duration(0):
            self._start_is_delayed = True
            self._update_time_stamps(self._current_subgait, first_subgait_delay)
            return GaitUpdate.should_schedule_early(
                self._command_from_current_subgait()
            )
        else:
            self._start_is_delayed = False
            self._update_time_stamps(self._current_subgait)
            return GaitUpdate.should_schedule(self._command_from_current_subgait())

    def get_frame_id_to_transfrom_to(self):
        try:
            self._node.get_logger().warn(
                f"frame id = "
                f"{self.SUBGAIT_NAME_TO_REALSENSE_FRAME_ID_MAP[self._current_subgait.subgait_name]}"
            )
            return self.SUBGAIT_NAME_TO_REALSENSE_FRAME_ID_MAP[
                self._current_subgait.subgait_name
            ]
        except KeyError as e:
            self._node.get_logger().warn(
                f"The current subgait name {self._current_subgait.subgait_name} "
                f"has no known associated frame id."
            )
            return None

    def make_realsense_service_call(self, frame_id_to_transform_to: str) -> bool:
        """
        Make a call to the realsense service, if it is available
        and returns the response.

        :param frame_id_to_transform_to: The frame that should be given to the reader.
        :return: Whether the call was successful
        """
        request = GetGaitParameters.Request(
            realsense_category=self.realsense_category,
            camera_to_use=self.camera_to_use,
            frame_id_to_transform_to=frame_id_to_transform_to,
            subgait_name=self._current_subgait.subgait_name
        )
        self.realsense_service_event.clear()
        if self._get_gait_parameters_service.wait_for_service(
            timeout_sec=self.SERVICE_TIMEOUT.seconds
        ):
            gait_parameters_response_future = (
                self._get_gait_parameters_service.call_async(request)
            )
            gait_parameters_response_future.add_done_callback(
                self._realsense_response_cb
            )
        else:
            self._node.get_logger().error(
                f"The service took longer than {self.SERVICE_TIMEOUT} to become "
                f"available, is the realsense reader running?"
            )
            return False

        event_wait = self.realsense_service_event.wait(timeout=self.SERVICE_TIMEOUT.seconds)
        self._node.get_logger().warn(f"event_wait variable is {event_wait}")
        return event_wait

    def _realsense_response_cb(self, future: Future):
        """Set capture point result when the capture point service returns."""
        if future.done():
            self._node.get_logger().warn("The future is done in the realsense responde cb")
        else:
            self._node.get_logger().warn("The future is NOT done in the realsense responde cb")
        self.realsense_service_result = future.result()
        self._node.get_logger().warn(f"The result is {self.realsense_service_result}")
        self.realsense_service_event.set()

    def interpolate_subgaits_from_parameters(self) -> None:
        """Change all subgaits to one interpolated from the current parameters."""
        new_subgaits = {}
        for subgait_name in self.subgaits.keys():
            self._node.get_logger().debug(
                f"Interpolating with parameters={self.parameters}"
            )
            new_subgaits[subgait_name] = Subgait.interpolate_n_subgaits(
                dimensions=self.dimensions,
                subgaits=self.subgaits_to_interpolate[subgait_name],
                parameters=self.parameters,
                use_foot_position=True,
            )
        self.set_subgaits(new_subgaits)

    def update_parameters(self, gait_parameters: GaitParameters) -> None:
        """
        Update the gait parameters based on the message.

        :param gait_parameters: The parameters to update to.
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
