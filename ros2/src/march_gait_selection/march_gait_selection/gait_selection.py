import os
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import yaml
from march_shared_classes.exceptions.gait_exceptions import GaitError, GaitNameNotFound
from march_shared_classes.exceptions.general_exceptions import FileNotFoundError, PackageNotFoundError
from march_shared_classes.gait.subgait import Subgait
from .state_machine.setpoints_gait import SetpointsGait
from .state_machine.state_machine_input import StateMachineInput
from .state_machine.trajectory_scheduler import TrajectoryScheduler

NODE_NAME = 'gait_selection'
DEFAULT_GAIT_FILES_PACKAGE = 'march_gait_files'
DEFAULT_GAIT_DIRECTORY = 'minimal'
DEFAULT_UPDATE_RATE = 120.0

class GaitSelection(Node):
    """Base class for the gait selection module."""

    def __init__(self):

        package = self.get_parameter_or('gait_package', alternative_value=DEFAULT_GAIT_FILES_PACKAGE)
        directory = self.get_parameter_or('gait_directory', alternative_value=DEFAULT_GAIT_DIRECTORY)
        update_rate = self.get_parameter_or('update_rate', alternative_value=DEFAULT_UPDATE_RATE)
        robot = urdf.Robot.from_parameter_server('/robot_description')

        super(GaitSelection, self).__init__(NODE_NAME)
        package_path = self.get_ros_package_path(package)
        self._gait_directory = os.path.join(package_path, directory)
        if not os.path.isdir(self._gait_directory):
            self.get_logger().err('Gait directory does not exist: {0}'.format(directory))
            raise FileNotFoundError(file_path=self._gait_directory)

        self._default_yaml = os.path.join(self._gait_directory, 'default.yaml')
        if not os.path.isfile(self._default_yaml):
            raise FileNotFoundError(file_path=self._default_yaml)

        self._robot = robot

        self._gait_version_map, self._positions = self._load_configuration()
        self._loaded_gaits = self._load_gaits()

    @staticmethod
    def get_ros_package_path(package):
        """Returns the path of where the given (ros) package is located."""
        return get_package_share_directory(package)

    @property
    def robot(self):
        return self._robot

    @property
    def gait_version_map(self):
        """Returns the mapping from gaits and subgaits to versions."""
        return self._gait_version_map

    @property
    def positions(self):
        """Returns the named idle positions."""
        return self._positions

    def create_services(self):
        self.create_service(Trigger, '/march/gait_selection/get_version_map',
                                      lambda msg: [True, str(gait_selection.gait_version_map)])

        self.create_service(SetGaitVersion, '/march/gait_selection/set_gait_version',
                                      lambda msg: set_gait_versions(msg, gait_selection))

        self.create_service(Trigger, '/march/gait_selection/get_directory_structure',
                                      lambda msg: [True, str(gait_selection.scan_directory())])

        self.create_service(Trigger, '/march/gait_selection/update_default_versions',
                                      lambda msg: gait_selection.update_default_versions())

        self.create_service(ContainsGait, '/march/gait_selection/contains_gait',
                                      lambda msg: contains_gait(msg, gait_selection))


    def set_gait_versions(self, gait_name, version_map):
        """Sets the subgait versions of given gait.

        :param str gait_name: Name of the gait to change versions
        :param dict version_map: Mapping subgait names to versions
        """
        if gait_name not in self._loaded_gaits:
            raise GaitNameNotFound(gait_name)

        # Only update versions that are different
        version_map = dict([(name, version) for name, version in version_map.items() if
                            version != self._gait_version_map[gait_name][name]])
        self._loaded_gaits[gait_name].set_subgait_versions(self._robot, self._gait_directory, version_map)
        self._gait_version_map[gait_name].update(version_map)

    def scan_directory(self):
        """Scans the gait_directory recursively and create a dictionary of all subgait files.

        :returns:
            dictionary of the maps and files within the directory
        """
        gaits = {}
        for gait in os.listdir(self._gait_directory):
            gait_path = os.path.join(self._gait_directory, gait)

            if os.path.isdir(gait_path):
                subgaits = {}

                for subgait in os.listdir(gait_path):
                    subgait_path = os.path.join(gait_path, subgait)

                    if os.path.isdir(subgait_path):
                        versions = sorted([v.replace('.subgait', '') for v in os.listdir(os.path.join(subgait_path)) if
                                           v.endswith('.subgait')])
                        subgaits[subgait] = versions

                gaits[gait] = subgaits
        return gaits

    def update_default_versions(self):
        """Updates the default.yaml file with the current loaded gait versions."""
        new_default_dict = {'gaits': self._gait_version_map, 'positions': self._positions}

        try:
            with open(self._default_yaml, 'w') as default_yaml_content:
                yaml_content = yaml.dump(new_default_dict, default_flow_style=False)
                default_yaml_content.write(yaml_content)
            return [True, 'New default values were written to: {pn}'.format(pn=self._default_yaml)]

        except IOError:
            return [False, 'Error occurred when writing to file path: {pn}'.format(pn=self._default_yaml)]

    def add_gait(self, gait):
        """Adds a gait to the loaded gaits if it does not already exist.

        The to be added gait should implement `GaitInterface`.
        """
        if gait.name in self._loaded_gaits:
            self.get_logger().warn('Gait `{gait}` already exists in gait selection'.format(gait=gait.name))
        else:
            self._loaded_gaits[gait.name] = gait

    def _load_gaits(self):
        """Loads the gaits in the specified gait directory.

        :returns dict: A dictionary mapping gait name to gait instance
        """
        gaits = {}

        for gait in self._gait_version_map:
            gaits[gait] = SetpointsGait.from_file(gait, self._gait_directory, self._robot, self._gait_version_map)

        return gaits

    def _load_configuration(self):
        """Loads and verifies the gaits configuration."""
        with open(self._default_yaml, 'r') as default_yaml_file:
            default_config = yaml.load(default_yaml_file, Loader=yaml.SafeLoader)

        version_map = default_config['gaits']

        if not isinstance(version_map, dict):
            raise TypeError('Gait version map should be of type; dictionary')

        if not self._validate_version_map(version_map):
            raise GaitError(msg='Gait version map: {gm}, is not valid'.format(gm=version_map))

        return version_map, default_config['positions']

    def _validate_version_map(self, version_map):
        """Validates if the current versions exist.

        :param dict version_map: Version map to verify
        :returns bool: True when all versions exist, False otherwise
        """
        for gait_name in version_map:
            gait_path = os.path.join(self._gait_directory, gait_name)
            if not os.path.isfile(os.path.join(gait_path, gait_name + '.gait')):
                self.get_logger().warn('gait {gn} does not exist'.format(gn=gait_name))
                return False

            for subgait_name in version_map[gait_name]:
                version = version_map[gait_name][subgait_name]
                if not Subgait.validate_version(gait_path, subgait_name, version):
                    self.get_logger().warn('{0}, {1} does not exist'.format(subgait_name, version))
                    return False
        return True

    def __getitem__(self, name):
        """Returns a gait from the loaded gaits."""
        return self._loaded_gaits.get(name)

    def __iter__(self):
        """Returns an iterator over all loaded gaits."""
        return iter(self._loaded_gaits.values())



def set_gait_versions(msg, gait_selection):
    """Sets a new gait version to the gait selection instance.

    :type msg: march_shared_resources.srv.SetGaitVersionRequest
    :type gait_selection: GaitSelection
    :rtype march_shared_resources.srv.SetGaitVersionResponse
    """
    if len(msg.subgaits) != len(msg.versions):
        return [False, '`subgaits` and `versions` array are not of equal length']

    version_map = dict(zip(msg.subgaits, msg.versions))
    try:
        gait_selection.set_gait_versions(msg.gait, version_map)
        return [True, '']
    except Exception as e:
        return [False, str(e)]


def contains_gait(request, gait_selection):
    """
    Checks whether a gait and subgait are loaded.

    :type request: ContainsGaitRequest
    :param request: service request
    :param gait_selection: current loaded gaits
    :return: True when the gait and subgait are loaded
    """
    gait = gait_selection[request.gait]
    if gait is None:
        return ContainsGait.Response(False)
    for subgait in request.subgaits:
        if gait[subgait] is None:
            return ContainsGait.Response(False)

    return ContainsGait.Response(True)



def main():
    gait_selection = GaitSelection()

    # gait_package = rospy.get_param('~gait_package', DEFAULT_GAIT_FILES_PACKAGE)
    # gait_directory = rospy.get_param('~gait_directory', DEFAULT_GAIT_DIRECTORY)
    # update_rate = rospy.get_param('~update_rate', DEFAULT_UPDATE_RATE)
    # robot = urdf.Robot.from_parameter_server('/robot_description')

    # gait_selection = GaitSelection(gait_package, gait_directory, robot)
    # rospy.loginfo('Gait selection initialized with package {0} of directory {1}'.format(gait_package, gait_directory))

    # balance_gait = BalanceGait.create_balance_subgait(gait_selection['balance_walk'])
    # if balance_gait is not None:
    #     gait_selection.add_gait(balance_gait)

    scheduler = TrajectoryScheduler('/march/controller/trajectory/follow_joint_trajectory')

    state_input = StateMachineInput(gait_selection)
    gait_state_machine = GaitStateMachine(gait_selection, scheduler, state_input, update_rate)
    # rospy.loginfo('Gait state machine successfully generated')

    rospy.core.add_preshutdown_hook(lambda reason: gait_state_machine.request_shutdown())

    gait_selection.create_services()
    gait_state_machine.create_services()
    gait_state_machine.create_publishers
    create_publishers(gait_state_machine)
    create_sounds(gait_state_machine)

    gait_state_machine.run()
