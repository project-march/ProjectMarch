from march_gait_selection.state_machine.setpoints_gait import SetpointsGait
from march_shared_msgs.msg import GaitParameters
from march_shared_msgs.srv import GetGaitParametersRequest, GetGaitParameters
from march_utility.gait.subgait import Subgait
from march_utility.utilities.duration import Duration
from rclpy import Node


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
        self.height_parameter = 0.0
        self.depth_parameter = 0.0
        try:
            self.low_deep_subgait_versions = subgait_versions["low_deep"]
            self.high_deep_subgait_versions = subgait_versions["high_deep"]
            self.low_undeep_subgait_versions = subgait_versions["low_undeep"]
            self.high_undeep_subgait_versions = subgait_versions["high_undeep"]
        except KeyError:
            node.get_logger().logerror(f"2D realsense gait {gait_name} was being "
                                       "created without all correct subgait versions "
                                       "in the yaml.")

    def update_subgait_versions_from_parameters(self, gait_parameters: GaitParameters):
        for subgait_name in self.subgaits.keys():
            version_path_list = [self.low_deep_subgait_versions[subgait_name],
                                 self.low_undeep_subgait_versions[]]
            self.subgaits[subgait_name] = Subgait.from_four_files_interpolated(robot, version_path_list=)
            self._node.get_logger().loginfo(subgait)