from march_gait_selection.state_machine.setpoints_gait import SetpointsGait
from march_shared_msgs.msg import GaitParameters
from march_shared_msgs.srv import GetGaitParameters
from march_utility.gait.subgait import Subgait
from march_utility.gait.subgait_graph import SubgaitGraph
from march_utility.utilities.duration import Duration
from rclpy.node import Node
from urdf_parser_py import urdf

class RealSenseGait(SetpointsGait):
    def __init__(
        self,
        gait_name,
        subgaits,
        graph,
        node,
        selected_gait: str,
        camera_to_use,
        robot,
        gait_directory,
    ):
        super(RealSenseGait, self).__init__(gait_name, subgaits, graph)
        self._node = node
        self._get_gait_parameters_service = node.create_client(
            srv_type=GetGaitParameters, srv_name="/camera/process_pointcloud"
        )
        self.gait_directory = gait_directory
        self.selected_gait = self.realsense_gait_msg_from_string(selected_gait)
        self.camera_to_use = self.camera_msg_from_string(camera_to_use)
        self.robot = robot

    def realsense_gait_msg_from_string(self, gait_name: str):
        mapping = {"stairs_up": GetGaitParameters.Request.STAIRS_UP}
        return mapping[gait_name]

    def camera_msg_from_string(self, camera_name: str):
        mapping = {"front": GetGaitParameters.Request.CAMERA_FRONT,
                   "back": GetGaitParameters.Request.CAMERA_BACK}
        return mapping[camera_name]

    def start(self):
        self._node.get_logger().info("Starting the realsense gait")
        request = GetGaitParameters.Request(
            selected_gait=self.selected_gait, camera_to_use=self.camera_to_use,
            frame_id_to_transform_to="foot_right"
        )

        self._node.get_logger().info("Make the service call for the realsense gait")
        if self._get_gait_parameters_service.wait_for_service(timeout_sec=3.0):
            gait_parameters_response = self._get_gait_parameters_service.call(request)
        else:
            self._node.get_logger().info("Timeout")
            return None

        self._node.get_logger().info(f"Received response: {gait_parameters_response.success}")
        if not gait_parameters_response.success:
            self._node.get_logger().logwarn(
                "No gait parameters were found, gait will not be started"
            )
            return None

        self.update_subgait_versions_from_parameters(
            gait_parameters_response.gait_parameters
        )
        self._node.get_logger().info(f"Done updating subgait versions")

        self._current_subgait = self.subgaits[self.graph.start_subgaits()[0]]
        self._node.get_logger().info(f"Done updating current subgait")
        self._should_stop = False
        self._transition_to_subgait = None
        self._is_transitioning = False
        self._time_since_start = Duration(0)

        return self._current_subgait.to_joint_trajectory_msg()

    def update_subgait_versions_from_parameters(self, gait_parameters: GaitParameters):
        raise NotImplementedError(
            "Any RealSense gait implementation should specify "
            "an update function to update all subgaits"
        )


class RealSense2DGait(RealSenseGait):
    def __init__(
        self,
        gait_name: str,
        subgaits,
        graph,
        node: Node,
        selected_gait: str,
        camera_to_use: str,
        robot: urdf.Robot,
        subgait_versions: dict,
        gait_directory: str,
    ):
        super(RealSense2DGait, self).__init__(
            gait_name,
            subgaits,
            graph,
            node,
            selected_gait,
            camera_to_use,
            robot,
            gait_directory,
        )
        self.first_parameter = 0.0  # For stairs, this is depth
        self.second_parameter = 0.0  # For stairs, this is height
        self.subgait_versions = subgait_versions
        self.dependents = ["realsense_stairs"]

    def update_subgait_versions_from_parameters(self, gait_parameters: GaitParameters):
        self.first_parameter = gait_parameters.first_parameter
        self.second_parameter = gait_parameters.second_parameter

        new_subgaits = {}
        for subgait_name in self.subgaits.keys():
            version_path_list = self.subgait_versions[subgait_name]
            parameter_list = [self.first_parameter, self.second_parameter]
            new_subgaits[subgait_name] = \
                Subgait.from_four_names_and_versions_interpolated(
                self.robot,
                version_path_list=version_path_list,
                parameter_list=parameter_list,
                gait_directory=self.gait_directory,
                gait_name=self.gait_name,
                subgait_name=subgait_name,
                use_foot_position=True,
                node=self._node,
            )
        self.set_subgaits(new_subgaits)

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
        try:
            selected_gait = gait_config["obstacle"]
            camera_to_use = gait_config["camera_to_use"]
            subgait_version_map = gait_config["subgaits"]
            graph = SubgaitGraph(gait_graph)
            subgaits = dict(
                [
                    (
                        name,
                        Subgait.from_four_names_and_versions_interpolated(
                            robot,
                            version_path_list=subgait_version_map[name],
                            parameter_list=[1, 1],
                            gait_directory=gait_directory,
                            gait_name=gait_name,
                            subgait_name=name,
                            use_foot_position=True,
                            node=node
                        ),
                    )
                    for name in subgait_version_map
                    if name not in ("start", "end")
                ]
            )
            subgait_versions = gait_config["subgaits"]
        except KeyError as e:
            node.get_logger().error(
                f"Not all information to create realsense gait "
                f"{gait_name} was available"
            )
        return cls(
            gait_name,
            subgaits,
            graph,
            node,
            selected_gait,
            camera_to_use,
            robot,
            subgait_versions,
            gait_directory,
        )


class RealSense1DGait(RealSenseGait):
    def __init__(
        self,
        gait_name: str,
        subgaits,
        graph,
        node: Node,
        obstacle,
        camera_to_use,
        robot,
        subgait_versions: dict,
        gait_directory: str,
    ):
        super(RealSense1DGait, self).__init__(
            gait_name,
            subgaits,
            graph,
            node,
            obstacle,
            camera_to_use,
            robot,
            gait_directory,
        )
        self.parameter = 0.0  # For ramp for example, this is steepness of the ramp
        self.subgait_versions = subgait_versions

    def update_subgait_versions_from_parameters(self, gait_parameters: GaitParameters):
        self.first_parameter = gait_parameters.first_parameter
        self.second_parameter = gait_parameters.second_parameter

        for subgait_name in self.subgaits.keys():
            version_path_list = self.subgait_versions[subgait_name]
            self.subgaits[
                subgait_name
            ] = Subgait.from_two_names_and_versions_interpolated(
                self.robot,
                first_file_name=version_path_list[0],
                second_file_name=version_path_list[1],
                parameter=self.parameter,
                gait_directory=self.gait_directory,
                gait_name=self.gait_name,
                subgait_name=subgait_name,
                use_foot_position=True,
            )

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
        try:
            selected_gait = gait_config["obstacle"]
            camera_to_use = gait_config["camera_to_use"]
            subgait_version_map = gait_config["subgaits"]
            graph = SubgaitGraph(gait_graph)
            subgaits = dict(
                [
                    (
                        name,
                        Subgait.from_two_names_and_versions_interpolated(
                            robot,
                            first_file_name=subgait_version_map[name][0],
                            second_file_name=subgait_version_map[name][1],
                            parameter=0.5,
                            gait_directory=gait_directory,
                            gait_name=gait_name,
                            subgait_name=name,
                            use_foot_position=True,
                        ),
                    )
                    for name in subgait_version_map
                    if name not in ("start", "end")
                ]
            )
            subgait_versions = gait_config["subgaits"]
        except KeyError as e:
            node.get_logger().error(
                f"Not all information to create realsense gait "
                f"{gait_name} was available"
            )
        return cls(
            gait_name,
            subgaits,
            graph,
            node,
            selected_gait,
            camera_to_use,
            robot,
            subgait_versions,
            gait_directory,
        )
