import os
import rclpy
from march_shared_msgs.srv import SetGaitVersion, ContainsGait
from rcl_interfaces.srv import GetParameters
from rclpy.exceptions import ParameterNotDeclaredException
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import yaml
from march_shared_classes.exceptions.gait_exceptions import GaitError, GaitNameNotFound
from march_shared_classes.gait.subgait import Subgait
from std_msgs.msg import String
from std_srvs.srv import Trigger
from urdf_parser_py import urdf
from .state_machine.setpoints_gait import SetpointsGait

NODE_NAME = 'gait_selection'


class GaitSelection(Node):
    """Base class for the gait selection module."""

    def __init__(self, gait_package=None, directory=None, robot=None):
        super().__init__(NODE_NAME, automatically_declare_parameters_from_overrides=True)
        try:
            if gait_package is None:
                gait_package = self.get_parameter('gait_package')\
                    .get_parameter_value().string_value
            if directory is None:
                directory = self.get_parameter('gait_directory')\
                    .get_parameter_value().string_value

        except ParameterNotDeclaredException:
            self.get_logger().error(
                'Gait selection node started without required parameters '
                'gait_package and gait_directory')

        package_path = get_package_share_directory(gait_package)
        self._gait_directory = os.path.join(package_path, directory)
        self._default_yaml = os.path.join(self._gait_directory, 'default.yaml')

        if not os.path.isdir(self._gait_directory):
            self.get_logger().error(f'Gait directory does not exist: {directory}')
        if not os.path.isfile(self._default_yaml):
            self.get_logger().error(
                f'Gait default yaml file does not exist: {directory}/default.yaml')

        self._gait_version_map, self._positions = self._load_configuration()
        self._robot = self._initial_robot_description() if robot is None else robot
        # Subsribe to the robot description channel to be kept up to date of
        # changes in the robot description
        self.robot_description_sub = self.create_subscription(
            msg_type=String, topic='/robot_description',
            callback=self._update_robot_description_cb,
            qos_profile=10)

        self._create_services()
        self._loaded_gaits = self._load_gaits()
        self.get_logger().info('Successfully initialized gait selection node.')

    def _initial_robot_description(self):
        """
        Initialize the robot description by getting it from the robot state
        publisher.
        """
        robot_description_client = self.create_client(
            srv_type=GetParameters, srv_name='/robot_state_publisher/get_parameters')
        while not robot_description_client.wait_for_service(timeout_sec=2):
            self.get_logger().warn("Robot description is not being published, waiting..")

        robot_future = robot_description_client.call_async(
            request=GetParameters.Request(names=['robot_description']))
        rclpy.spin_until_future_complete(self, robot_future)

        return urdf.Robot.from_xml_string(robot_future.result().values[0].string_value)

    def _create_services(self):
        self.create_service(srv_type=Trigger,
                            srv_name='/march/gait_selection/get_version_map',
                            callback=lambda msg: [True, str(self.gait_version_map)])

        self.create_service(srv_type=SetGaitVersion,
                            srv_name='/march/gait_selection/set_gait_version',
                            callback=self.set_gait_versions_cb)

        self.create_service(srv_type=Trigger,
                            srv_name='/march/gait_selection/get_directory_structure',
                            callback=lambda msg: [True, str(self.scan_directory())])

        self.create_service(srv_type=Trigger,
                            srv_name='/march/gait_selection/update_default_versions',
                            callback=lambda msg: self.update_default_versions())

        self.create_service(srv_type=ContainsGait,
                            srv_name='/march/gait_selection/contains_gait',
                            callback=self.contains_gait_cb)

    @property
    def robot(self):
        """ Return the robot obtained from the robot state publisher."""
        return self._robot

    @property
    def gait_version_map(self):
        """Returns the mapping from gaits and subgaits to versions."""
        return self._gait_version_map

    @property
    def positions(self):
        """Returns the named idle positions."""
        return self._positions

    def _update_robot_description_cb(self, msg):
        """
        Callback that is used to update the robot description when
        robot_state_publisher sends out an update.
        """
        self._robot = urdf.Robot.from_xml_string(msg.data)

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
        self._loaded_gaits[gait_name].set_subgait_versions(
            self._robot, self._gait_directory, version_map)
        self._gait_version_map[gait_name].update(version_map)

    def set_gait_versions_cb(self, msg):
        """Sets a new gait version to the gait selection instance.

        :type msg: march_shared_resources.srv.SetGaitVersionRequest
        :rtype march_shared_resources.srv.SetGaitVersionResponse
        """
        if len(msg.subgaits) != len(msg.versions):
            return [False, '`subgaits` and `versions` array are not of equal length']

        version_map = dict(zip(msg.subgaits, msg.versions))
        try:
            self.set_gait_versions(msg.gait, version_map)
            return [True, '']
        except Exception as e:
            return [False, str(e)]

    def contains_gait_cb(self, request):
        """
        Checks whether a gait and subgait are loaded.

        :type request: ContainsGaitRequest
        :param request: service request
        :return: True when the gait and subgait are loaded
        """
        gait = self._loaded_gaits.get(request.gait)
        if gait is None:
            return ContainsGait.Response(contains=False)
        for subgait in request.subgaits:
            if gait[subgait] is None:
                return ContainsGait.Response(contains=False)
        return ContainsGait.Response(contains=True)

    def scan_directory(self):
        """Scans the gait_directory recursively and create a dictionary of all
        subgait files.

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
                        versions = sorted([v.replace('.subgait', '')
                                           for v in os.listdir(os.path.join(subgait_path))
                                           if v.endswith('.subgait')])
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

            return [True, 'New default values were written to: {pn}'
                          .format(pn=self._default_yaml)]

        except IOError:
            return [False, 'Error occurred when writing to file path: {pn}'
                           .format(pn=self._default_yaml)]

    def add_gait(self, gait):
        """Adds a gait to the loaded gaits if it does not already exist.

        The to be added gait should implement `GaitInterface`.
        """
        if gait.name in self._loaded_gaits:
            self.get_logger().warn('Gait `{gait}` already exists in gait selection'
                                   .format(gait=gait.name))
        else:
            self._loaded_gaits[gait.name] = gait

    def _load_gaits(self):
        """Loads the gaits in the specified gait directory.

        :returns dict: A dictionary mapping gait name to gait instance
        """
        gaits = {}

        for gait in self._gait_version_map:
            gaits[gait] = SetpointsGait.from_file(
                gait, self._gait_directory, self._robot, self._gait_version_map)

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
                    self.get_logger().warn('{0}, {1} does not exist'
                                           .format(subgait_name, version))
                    return False
        return True

    def __getitem__(self, name):
        """Returns a gait from the loaded gaits."""
        return self._loaded_gaits.get(name)

    def __iter__(self):
        """Returns an iterator over all loaded gaits."""
        return iter(self._loaded_gaits.values())
