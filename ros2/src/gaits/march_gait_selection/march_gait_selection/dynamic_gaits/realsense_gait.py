from threading import Event
from typing import Optional, List

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
from urdf_parser_py import urdf

from march_gait_selection.state_machine.gait_update import GaitUpdate


class RealSenseGait(SetpointsGait):
    """
    The RealSenseGait class is used for creating gaits based on the parameters given
    by the realsense reader. It is based on the setpoints gait, and it uses the
    interpolation over 1/2 dimensions with 2/4 subgaits.
    """

    SERVICE_TIMEOUT = Duration(seconds=2.0)
    CAMERA_NAME_MAP = {
        "front": GetGaitParameters.Request.CAMERA_FRONT,
        "back": GetGaitParameters.Request.CAMERA_BACK,
    }
    SELECTED_REALSENSE_GAIT_MAP = {"stairs_up": GetGaitParameters.Request.STAIRS_UP}

    def __init__(
        self,
        gait_name: str,
        subgaits: dict,
        graph: SubgaitGraph,
        node: Node,
        selected_gait: str,
        camera_to_use: str,
        subgaits_to_interpolate: dict,
        dimensions: InterpolationDimensions,
        parameters: List[float],
    ):
        super(RealSenseGait, self).__init__(gait_name, subgaits, graph)
        self._node = Node(gait_name)
        self._get_gait_parameters_service = node.create_client(
            srv_type=GetGaitParameters, srv_name="/camera/process_pointcloud"
        )
        self.parameters = parameters
        self.dimensions = dimensions
        self.selected_gait = self.selected_realsense_gait_msg_from_string(selected_gait)
        self.camera_to_use = self.camera_msg_from_string(camera_to_use)
        self.subgaits_to_interpolate = subgaits_to_interpolate
        self.realsense_service_event = Event()
        self.realsense_service_result = None

    @classmethod
    def from_yaml(
        cls,
        node: Node,
        robot: urdf.Robot,
        gait_name: str,
        gait_config: dict,
        gait_graph: dict,
        gait_directory: str,
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
            selected_gait = gait_config["gait_type"]
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
        return cls(
            gait_name,
            subgaits,
            graph,
            node,
            selected_gait,
            camera_to_use,
            subgaits_to_interpolate,
            dimensions,
            parameters,
        )

    @classmethod
    def selected_realsense_gait_msg_from_string(cls, gait_name: str) -> int:
        """
        Construct the realsense gait from the string in the realsense_gaits.yaml.

        :param gait_name: The string from the config.
        :return: The integer to send to the realsense reader to define the selected
        gait.
        """
        if gait_name not in cls.SELECTED_REALSENSE_GAIT_MAP:
            raise WrongRealSenseConfigurationError(
                f"Gait name {gait_name} from the config is not known as a possible "
                f"realsense reader gait configuration"
            )
        return cls.SELECTED_REALSENSE_GAIT_MAP[gait_name]

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
        3) Update the gait parameters to prepare for stari
        4) Return the first subgait, if correct parameters were found.

        :return: A gait update that tells the state machine what to do.
        """
        # Currently, we hardcode foot_right in start, since this is almost
        # always a right_open
        self._node.get_logger().info("start start")
        gait_parameters_response = self.make_realsense_service_call(
            frame_id_to_transform_to="foot_right"
        )
        self._node.get_logger().info("received response")
        if gait_parameters_response is None or not gait_parameters_response.success:
            self._node.get_logger().warn(
                "No gait parameters were found, gait will not be started"
            )
            return GaitUpdate.empty()

        self.update_parameters(gait_parameters_response.gait_parameters)
        self.interpolate_subgaits_from_parameters()
        self._node.get_logger().info("interpolated")

        self._reset()
        self._current_time = current_time
        self._current_subgait = self.subgaits[self.graph.start_subgaits()[0]]
        self._next_subgait = self._current_subgait
        self._node.get_logger().info("setted params")

        # Delay first subgait if duration is greater than zero
        if first_subgait_delay is not None and first_subgait_delay > Duration(0):
            self._node.get_logger().info("delay schedule")
            self._start_is_delayed = True
            self._update_time_stamps(self._current_subgait, first_subgait_delay)
            return GaitUpdate.should_schedule_early(
                self._command_from_current_subgait()
            )
        else:
            self._node.get_logger().info("not delayed schedule")
            self._start_is_delayed = False
            self._update_time_stamps(self._current_subgait)
            return GaitUpdate.should_schedule(self._command_from_current_subgait())

    def make_realsense_service_call(
        self, frame_id_to_transform_to: str
    ) -> Optional[GetGaitParameters.Response]:
        """
        Make a call to the realsense service, if it is available
        and returns the response.

        :param frame_id_to_transform_to: The frame that should be given to the reader.
        :return: The response from the service, None if it was not available.
        """
        self._node.get_logger().info("make call start")
        request = GetGaitParameters.Request(
            selected_gait=self.selected_gait,
            camera_to_use=self.camera_to_use,
            frame_id_to_transform_to=frame_id_to_transform_to,
        )

        if self._get_gait_parameters_service.wait_for_service(
            timeout_sec=self.SERVICE_TIMEOUT.seconds
        ):
            self._node.get_logger().info("service is available")
            gait_parameters_response_future = \
                self._get_gait_parameters_service.call_async(request)
            self._node.get_logger().info("async call done")
        else:
            self._node.get_logger().error(
                f"The service took longer than {self.SERVICE_TIMEOUT} to become "
                f"available, is the realsense reader running?"
            )
            return None

        gait_parameters_response_future.add_done_callback(self.realsense_response_cb)
        self._node.get_logger().info("added done callback")
        wait_res =  self.realsense_service_event.wait(
            timeout=self.SERVICE_TIMEOUT.seconds)
        self._node.get_logger().info(f"{wait_res}")
        if wait_res:
            return self.realsense_service_result
        else:
            return None

    def realsense_response_cb(self, future: Future):
        """Set capture point result when the capture point service returns."""
        self.realsense_service_result = future.result()
        self.realsense_service_event.set()

    def interpolate_subgaits_from_parameters(self) -> None:
        """ Change all subgaits to one interpolated from the current parameters."""
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
