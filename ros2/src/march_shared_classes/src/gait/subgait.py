import os
import re
from typing import List

from rclpy.duration import Duration
from trajectory_msgs import msg as trajectory_msg
import yaml
from urdf_parser_py import urdf

from march_shared_classes.exceptions.gait_exceptions import NonValidGaitContent, SubgaitInterpolationError, GaitError

from .joint_trajectory import JointTrajectory
from .limits import Limits
from .setpoint import Setpoint

PARAMETRIC_GAITS_PREFIX = '_pg_'


class Subgait(object):
    """Base class for usage of the defined subgaits."""

    joint_class = JointTrajectory

    def __init__(self, joints: List[JointTrajectory], duration: Duration, gait_type: str = 'walk_like',
                 gait_name: str = 'Walk', subgait_name: str = 'right_open', version: str = 'First try',
                 description: str = 'Just a simple gait'):

        self.joints = joints
        self.gait_type = gait_type
        self.gait_name = gait_name

        self.subgait_name = subgait_name
        self.version = version
        self.description = str(description)
        self.duration = duration

    # region Create subgait
    @classmethod
    def from_file(cls, robot: urdf.Robot, file_name: str, *args):
        """Extract sub gait data of the given yaml.

        :param robot:
            The robot corresponding to the given subgait file
        :param file_name:
            The .yaml file name of the subgait

        :returns
            A populated Subgait object
        """
        if file_name is None or not os.path.isfile(file_name):
            raise FileNotFoundError(file_path=file_name)
        try:
            gait_name = file_name.split('/')[-3]
            subgait_name = file_name.split('/')[-2]
            version = file_name.split('/')[-1].replace('.subgait', '')

            with open(file_name, 'r') as yaml_file:
                subgait_dict = yaml.load(yaml_file, Loader=yaml.SafeLoader)

        except Exception as e:
            return None

        return cls.from_dict(robot, subgait_dict, gait_name, subgait_name, version, *args)

    @classmethod
    def from_name_and_version(cls, robot: urdf.Robot, gait_dir: str, gait_name: str, subgait_name: str, version: str,
                              *args):
        """Load subgait based from file(s) based on name and version.

        :param robot: The robot corresponding to the given subgait file
        :param gait_dir: The directory with all the gaits
        :param gait_name: The name of the corresponding gait
        :param subgait_name: The name of the subgait to load
        :param version: The version to use, this can be parametric
        :param args:
        :return: A populated Subgait object.
        """
        if version.startswith(PARAMETRIC_GAITS_PREFIX):
            base_version, other_version, parameter = Subgait.unpack_parametric_version(version)
            base_path = os.path.join(gait_dir, gait_name, subgait_name, base_version + '.subgait')
            other_path = os.path.join(gait_dir, gait_name, subgait_name, other_version + '.subgait')
            return cls.from_files_interpolated(robot, base_path, other_path, parameter)
        else:
            subgait_path = os.path.join(gait_dir, gait_name, subgait_name, version + '.subgait')
            return cls.from_file(robot, subgait_path, *args)

    @classmethod
    def from_files_interpolated(cls, robot: urdf.Robot, file_name_base: str, file_name_other: str, parameter: float,
                                *args):
        """Extract two subgaits from files and interpolate.

        :param robot:
            The robot corresponding to the given subgait file
        :param file_name_base:
            The .yaml file name of the base subgait
        :param file_name_other:
        :param parameter:
            The parameter to use for interpolation. Should be 0 <= parameter <= 1

        :return:
            A populated Subgait object
        """
        base_subgait = cls.from_file(robot, file_name_base, *args)
        other_subgait = cls.from_file(robot, file_name_other, *args)
        return cls.interpolate_subgaits(base_subgait, other_subgait, parameter)

    @classmethod
    def from_dict(cls, robot: urdf.Robot, subgait_dict: dict, gait_name: str, subgait_name: str, version: str, *args):
        """List parameters from the yaml file in organized lists.

        :param robot:
            The robot corresponding to the given sub-gait file
        :param subgait_dict:
            The dictionary extracted from the yaml file
        :param gait_name:
            The name of the parent gait
        :param subgait_name:
            The name of the child (sub)gait
        :param version:
            The version of the yaml file

        :returns
            A populated Subgait object
        """
        if robot is None:
            raise GaitError('Cannot create gait without a loaded robot.')

        duration = Duration(seconds=subgait_dict['duration']['secs'],
                            nanoseconds=subgait_dict['duration']['nsecs']).nanoseconds

        joint_list =[]
        for name, points in sorted(subgait_dict['joints'].items(), key=lambda item: item[0]):
            urdf_joint = cls.joint_class.get_joint_from_urdf(robot, name)
            if urdf_joint is None or urdf_joint.type == 'fixed':
                continue
            limits = Limits.from_urdf_joint(urdf_joint)
            joint_list.append(cls.joint_class.from_setpoints(name, limits, points, duration, *args))
        subgait_type = subgait_dict['gait_type'] if subgait_dict.get('gait_type') else ''
        subgait_description = subgait_dict['description'] if subgait_dict.get('description') else ''

        return cls(joint_list, duration, subgait_type, gait_name, subgait_name, version, subgait_description)

    # endregion

    # region Create messages
    def to_joint_trajectory_msg(self):
        """Create trajectory msg for the publisher.

        :returns
            a ROS msg for the joint trajectory
        """
        joint_trajectory_msg = trajectory_msg.JointTrajectory()

        joint_trajectory_msg.joint_names = [joint.name for joint in self.joints]

        timestamps = self.get_unique_timestamps()
        for timestamp in timestamps:
            joint_trajectory_point = trajectory_msg.JointTrajectoryPoint()
            joint_trajectory_point.time_from_start = Duration(seconds=timestamp)

            for joint in self.joints:
                interpolated_setpoint = joint.get_interpolated_setpoint(timestamp)

                joint_trajectory_point.positions.append(interpolated_setpoint.position)
                joint_trajectory_point.velocities.append(interpolated_setpoint.velocity)

            joint_trajectory_msg.points.append(joint_trajectory_point)

        return joint_trajectory_msg

    # endregion

    # region Validate subgait
    def validate_subgait_transition(self, next_subgait):
        """Validate the trajectory transition of this gait to a given gait.

        :param next_subgait:
            The subgait subsequently to this gait (not the previous one!)

        :returns:
            True if trajectory transition correct else False
        """
        from_subgait_joint_names = set(self.get_joint_names())
        to_subgait_joint_names = set(next_subgait.get_joint_names())

        if from_subgait_joint_names != to_subgait_joint_names:
            raise NonValidGaitContent(msg='Gait {gait}, structure of joints does not match between '
                                          'subgait {fn} and subgait {tn}'.format(gait=self.gait_name,
                                                                                 fn=self.subgait_name,
                                                                                 tn=next_subgait.subgait_name))

        for joint_name in to_subgait_joint_names:
            from_joint = self.get_joint(joint_name)
            to_joint = next_subgait.get_joint(joint_name)
            if not from_joint.validate_joint_transition(to_joint):
                return False

        return True

    # endregion

    # region Manipulate subgait
    def scale_timestamps_subgait(self, new_duration, rescale=True):
        """Scale or cut off all the setpoint to match the duration in both subgaits.

        :param new_duration: the new duration to scale the setpoints with
        :param rescale: set to true if all points should be rescaled, alternative is cut off after new duration
        """
        new_duration = round(new_duration, Setpoint.digits)

        for joint in self.joints:
            joint.set_duration(new_duration, rescale)
        self.duration = new_duration

    def create_interpolated_setpoints(self, timestamps):
        """Equalize the setpoints of the subgait match the given timestamps.

        :param timestamps: the new timestamps to use when creating the setpoints
        """
        timestamps = sorted(set(timestamps + self.get_unique_timestamps()))

        for joint in self.joints:
            new_joint_setpoints = []
            for timestamp in timestamps:
                if timestamp > self.duration:
                    raise IndexError('Gait {gait}, subgait {subgait} could not extrapolate timestamp outside max '
                                     'duration'.format(gait=self.gait_name, subgait=self.subgait_name))

                new_joint_setpoints.append(joint.get_interpolated_setpoint(timestamp))

            joint.setpoints = new_joint_setpoints

    @classmethod
    def interpolate_subgaits(cls, base_subgait, other_subgait, parameter):
        """Linearly interpolate two subgaits with the parameter to get a new subgait.

        :param base_subgait:
            base subgait, return value if parameter is equal to zero
        :param other_subgait:
            other subgait, return value if parameter is equal to one
        :param parameter:
            The parameter to use for interpolation. Should be 0 <= parameter <= 1

        :return:
            The interpolated subgait
        """
        if parameter == 1:
            return other_subgait
        if parameter == 0:
            return base_subgait
        if not (0 < parameter < 1):
            raise ValueError('Parameter for interpolation should be in the interval [0, 1], but is {0}'
                             .format(parameter))

        if sorted(base_subgait.get_joint_names()) != sorted(other_subgait.get_joint_names()):
            raise SubgaitInterpolationError('The subgaits to interpolate do not have the same joints, base'
                                            ' subgait has {0}, while other subgait has {1}'.
                                            format(sorted(base_subgait.get_joint_names()),
                                                   sorted(other_subgait.get_joint_names())))
        joints = []
        try:
            for base_joint in base_subgait.joints:
                other_joint = other_subgait.get_joint(base_joint.name)
                if other_joint is None:
                    raise SubgaitInterpolationError('Could not find a matching joint for base joint with name {0}.'.
                                                    format(base_joint.name))
                joints.append(cls.joint_class.interpolate_joint_trajectories(base_joint, other_joint, parameter))
        except SubgaitInterpolationError as e:
            raise e

        description = 'Interpolation between base version {0}, and other version {1} with parameter{2}'.format(
            base_subgait.version, other_subgait.version, parameter)

        duration = base_subgait.duration * parameter + (1 - parameter) * other_subgait.duration
        gait_type = base_subgait.gait_type if parameter <= 0.5 else other_subgait.gait_type
        version = '{0}{1}_({2})_({3})'.format(PARAMETRIC_GAITS_PREFIX, parameter, base_subgait.version,
                                              other_subgait.version)
        return Subgait(joints, duration, gait_type, base_subgait.gait_name, base_subgait.subgait_name, version,
                       description)
    # endregion

    # region Get functions
    def get_unique_timestamps(self):
        """The timestamp that is unique to a setpoint."""
        timestamps = []
        for joint in self.joints:
            for setpoint in joint.setpoints:
                timestamps.append(setpoint.time)

        return sorted(set(timestamps))

    def get_joint(self, name):
        """Get joint object with given name or index."""
        return next(joint for joint in self.joints if joint.name == name)

    def get_joint_names(self):
        """Get the names of all the joints existing in the joint list."""
        return [joint.name for joint in self.joints]

    @property
    def starting_position(self):
        """Returns a dictionary of joint positions at the start of this subgait."""
        return {joint.name: joint.setpoints[0].position for joint in self.joints}

    @property
    def final_position(self):
        """Returns a dictionary of joint positions at the end of this subgait."""
        return {joint.name: joint.setpoints[-1].position for joint in self.joints}

    # endregion

    def to_yaml(self):
        """Returns a YAML string representation of the subgait."""
        duration = Duration(seconds=self.duration)
        output = {
            'description': self.description,
            'duration': {
                'nsecs': duration.nanoseconds,
                'secs': duration.nanoseconds,
            },
            'gait_type': self.gait_type,
            'joints': dict([(joint.name, [{
                'position': setpoint.position,
                'time_from_start': {
                    'nsecs': Duration(seconds=setpoint.time).nanoseconds,
                    'secs': int(setpoint.time),
                },
                'velocity': setpoint.velocity}
                for setpoint in joint.setpoints]) for joint in self.joints]),
            'name': self.subgait_name,
            'version': self.version,
        }
        return yaml.dump(output)

    # region Class methods
    def __getitem__(self, index):
        return self.joints[index]

    def __len__(self):
        return len(self.joints)
    # endregion

    @staticmethod
    def validate_version(gait_path, subgait_name, version):
        """Check whether a gait exists for the gait."""
        if version.startswith(PARAMETRIC_GAITS_PREFIX):
            base_version, other_version, _ = Subgait.unpack_parametric_version(version)
            base_subgait_path = os.path.join(gait_path, subgait_name, base_version + '.subgait')
            other_subgait_path = os.path.join(gait_path, subgait_name, other_version + '.subgait')
            for subgait_path in [base_subgait_path, other_subgait_path]:
                if not os.path.isfile(subgait_path):
                    return False
        else:
            subgait_path = os.path.join(gait_path, subgait_name, version + '.subgait')
            if not os.path.isfile(subgait_path):
                return False
        return True

    @staticmethod
    def unpack_parametric_version(version):
        """Unpack a version to base version, other version and parameter."""
        parameter_str = re.search(r'^{0}(\d+\.\d+)_'.format(PARAMETRIC_GAITS_PREFIX), version).group(1)
        parameter = float(parameter_str)
        versions = re.findall(r'\([^\)]*\)', version)
        base_version = versions[0][1:-1]
        other_version = versions[1][1:-1]
        return base_version, other_version, parameter
