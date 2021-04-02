from march_gait_selection.state_machine.setpoints_gait import SetpointsGait
from march_shared_msgs.msg import GaitParameters
from march_shared_msgs.srv import GetGaitParametersRequest, GetGaitParameters
from march_utility.gait.gait import Gait
from march_utility.gait.subgait import Subgait
from march_utility.gait.subgait_graph import SubgaitGraph
from march_utility.utilities.duration import Duration
from rclpy import Node

from ros2.src.shared.march_utility.march_utility.utilities.camera import CameraSide


class RealSenseGait(SetpointsGait):

    def __init__(self, gait_name, subgaits, graph, node, obstacle, camera_to_use,
                 robot):
        super(RealSenseGait, self).__init__(gait_name, subgaits, graph)
        self._node = node
        self._get_gait_parameters_service = node.create_client(
            srv_type=GetGaitParameters,
            srv_name="/camera/process_pointcloud"
        )
        self.obstacle = obstacle
        self.camera_to_use = camera_to_use
        self.robot = robot

    @classmethod
    def from_yaml(cls, node: Node, robot: urdf.Robot, gait_name: str, gait_config:
    dict):
        try:
            obstacle = gait_config["obstacle"]
            camera_to_use = CameraSide.from_string(gait_config["camera_to_use"])
            subgait_version_map = gait_config["subgaits"]
            graph = SubgaitGraph(subgait_version_map)
            subgaits = dict(
                [
                    (
                        name,
                        Subgait.from_four_files_interpolated(
                            robot,
                            version_path_list=subgait_version_map[name],
                            parameter_list=[0.5, 0.5]
                        )
                    )
                    for name in subgait_version_map
                    if name not in ("start", "end")
                ]
    )

    def start(self):
        request = GetGaitParametersRequest(obstacle=self.obstacle,
                                           camera_to_use=self.camera_to_use)

        gait_parameters_response = self._get_gait_parameters_service.call(request)
        if not gait_parameters_response.success:
            self._node.get_logger().logwarn("No gait parameters were found, gait will not be started")
            return None

        self.update_subgait_versions_from_parameters(
            gait_parameters_response.gait_parameters
        )

        self._current_subgait = self.subgaits[self.graph.start_subgaits()[0]]
        self._should_stop = False
        self._transition_to_subgait = None
        self._is_transitioning = False
        self._time_since_start = Duration(0)
        return self._current_subgait.to_joint_trajectory_msg()



    def update_subgait_versions_from_parameters(self, gait_parameters: GaitParameters):
        raise NotImplementedError("Any RealSense gait implementation should specify "
                                  "an update function to update all subgaits")


class RealSense2DGait(RealSenseGait):

    def __init__(self, gait_name: str, subgaits, graph, node: Node, obstacle,
                 camera_to_use, robot, subgait_versions: dict):
        super(RealSense2DGait, self).__init__(gait_name, subgaits, graph, node,
                                              obstacle, camera_to_use, robot)
        self.first_parameter = 0.0 # For stairs, this is depth
        self.second_parameter = 0.0 # For stairs, this is height
        self.subgait_versions = subgait_versions
        self.robot = robot

    def update_subgait_versions_from_parameters(self, gait_parameters: GaitParameters):
        self.first_parameter = gait_parameters.first_parameter
        self.second_parameter = gait_parameters.second_parameter

        for subgait_name in self.subgaits.keys():
            version_path_list = self.subgait_versions[subgait_name]
            parameter_list = [self.first_parameter, self.second_parameter]
            self.subgaits[subgait_name] = Subgait.from_four_files_interpolated(
                self.robot,
                version_path_list=version_path_list,
                parameter_list=parameter_list
            )


class RealSense1DGait(RealSenseGait):

    def __init__(self, gait_name: str, subgaits, graph, node: Node, obstacle,
                 camera_to_use, robot, subgait_versions: dict):
        super(RealSense1DGait, self).__init__(gait_name, subgaits, graph, node,
                                              obstacle, camera_to_use, robot)
        self.parameter = 0.0 # For ramp for example, this is steepness of the ramp
        self.subgait_versions = subgait_versions
        self.robot = robot

    def update_subgait_versions_from_parameters(self, gait_parameters: GaitParameters):
        self.first_parameter = gait_parameters.first_parameter
        self.second_parameter = gait_parameters.second_parameter

        for subgait_name in self.subgaits.keys():
            version_path_list = self.subgait_versions[subgait_name]
            self.subgaits[subgait_name] = Subgait.from_two_files_interpolated(
                self.robot,
                first_file_name=version_path_list[0],
                second_file_name=version_path_list[1],
                first_parameter=self.parameter
            )