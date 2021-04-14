from typing import Optional

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
    SubgaitInterpolationError,
    WrongRealSenseConfigurationError,
)
from rclpy.node import Node
from urdf_parser_py import urdf


class RealSenseGait(SetpointsGait):
    SERVICE_TIMEOUT = 2.0  # secs
    CAMERA_NAME_MAP = {
        "front": GetGaitParameters.Request.CAMERA_FRONT,
        "back": GetGaitParameters.Request.CAMERA_BACK,
    }
    REALSENSE_GAIT_MAP = {"stairs_up": GetGaitParameters.Request.STAIRS_UP}

    def __init__(
        self,
        gait_name,
        subgaits,
        graph,
        node,
        selected_gait: str,
        camera_to_use: str,
        subgaits_to_interpolate,
        dimensions,
        parameters,
    ):
        super(RealSenseGait, self).__init__(gait_name, subgaits, graph)
        self._node = node
        self._get_gait_parameters_service = node.create_client(
            srv_type=GetGaitParameters, srv_name="/camera/process_pointcloud"
        )
        self.parameters = parameters
        self.dimensions = dimensions
        self.selected_gait = self.realsense_gait_msg_from_string(selected_gait)
        self.camera_to_use = self.camera_msg_from_string(camera_to_use)
        self.subgaits_to_interpolate = subgaits_to_interpolate

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
    def realsense_gait_msg_from_string(cls, gait_name: str):
        return cls.REALSENSE_GAIT_MAP[gait_name]

    @classmethod
    def camera_msg_from_string(cls, camera_name: str):
        return cls.CAMERA_NAME_MAP[camera_name]

    def start(self):
        gait_parameters_response = self.make_realsense_service_call()
        if gait_parameters_response is None or not gait_parameters_response.success:
            self._node.get_logger().warn(
                "No gait parameters were found, gait will not be started"
            )
            return None

        self.update_parameters(gait_parameters_response.gait_parameters)
        self.update_subgait_versions()

        self._current_subgait = self.subgaits[self.graph.start_subgaits()[0]]
        self._should_stop = False
        self._transition_to_subgait = None
        self._is_transitioning = False
        self._time_since_start = Duration(0)

        return self._current_subgait.to_joint_trajectory_msg()

    def make_realsense_service_call(self) -> Optional[GetGaitParameters.Response]:
        request = GetGaitParameters.Request(
            selected_gait=self.selected_gait,
            camera_to_use=self.camera_to_use,
            frame_id_to_transform_to="foot_right",
        )

        if self._get_gait_parameters_service.wait_for_service(
            timeout_sec=self.SERVICE_TIMEOUT
        ):
            gait_parameters_response = self._get_gait_parameters_service.call(request)
        else:
            self._node.get_logger().error(
                f"The service took longer than {self.SERVICE_TIMEOUT} to become "
                f"available, is the realsense reader running?"
            )
            return None

        return gait_parameters_response

    def update_subgait_versions(self):
        new_subgaits = {}
        for subgait_name in self.subgaits.keys():
            new_subgaits[subgait_name] = Subgait.interpolate_n_subgaits(
                dimensions=self.dimensions,
                subgaits=self.subgaits_to_interpolate[subgait_name],
                parameters=self.parameters,
                use_foot_position=True,
            )
        self.set_subgaits(new_subgaits)

    def update_parameters(self, gait_parameters: GaitParameters):
        if self.dimensions == InterpolationDimensions.ONE_DIM:
            self.parameters = [gait_parameters.first_parameter]
        elif self.dimensions == InterpolationDimensions.TWO_DIM:
            self.parameters = [
                gait_parameters.first_parameter,
                gait_parameters.second_parameter,
            ]
        else:
            raise UnknownDimensionsError(self.dimensions)
        self._node.get_logger().debug(
            f"Updated parameters of {self.gait_name} to" f" {self.parameters}"
        )
