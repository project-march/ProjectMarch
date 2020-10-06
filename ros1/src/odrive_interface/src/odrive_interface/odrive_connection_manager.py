import json
import os
import threading

from fibre import Logger, protocol, remote_object, usbbulk_transport
import rospkg
import rospy

_JSON_FILE_CACHE_PREFIX = 'config/odrive'
_CONFIG_KEY = 'config'
_CRC_KEY = 'crc16'


class OdriveConnectionManager:
    def __init__(self, serial_numbers=None, fibre_logger=Logger(verbose=False)):
        self.serial_numbers = serial_numbers
        self.odrives = {}

        self._channel = None
        self._fibre_logger = fibre_logger

        self._package_path = self.get_ros_package_path('odrive_interface')
        self._json_config_path = os.path.join(self._package_path, _JSON_FILE_CACHE_PREFIX)
        self._json_file_prefix = 'odrive-cache-{sn}.json'

        self.find_odrive()

    @staticmethod
    def get_ros_package_path(package):
        """Get the path of where the given (ros) package is located."""
        return rospkg.RosPack().get_path(package)

    def setup_odrive(self, config, crc):
        """Set up fibre remote object using given configuration."""
        json_data = {'name': 'fibre_node', 'members': config}
        self._channel._interface_definition_crc = crc

        obj = remote_object.RemoteObject(json_data, None, self._channel, self._fibre_logger)
        obj.__dict__['_json_data'] = json_data['members']
        obj.__dict__['_json_crc'] = crc

        return obj

    def read_config_from_device(self):
        """Read fibre connection configuration from odrive device directly, then cache it in a JSON file."""
        json_bytes = self._channel.remote_endpoint_read_buffer(0)

        try:
            json_string = json_bytes.decode('ascii')
        except UnicodeDecodeError as e:
            rospy.logwarn('Loaded JSON bytes not valid ascii: %s.', str(e))
            return None

        rospy.logdebug('Calculating crc for JSON string.')
        crc = protocol.calc_crc16(protocol.PROTOCOL_VERSION, json_bytes)

        try:
            config = json.loads(json_string)
        except json.JSONDecodeError as e:
            rospy.logwarn('Loaded JSON string from odrive invalid: %s.', str(e))
            return None

        odrive = self.setup_odrive(config=config, crc=crc)

        # logger.info('Caching JSON data in file.')
        _hex_serial_number = '{sn:12X}'.format(sn=odrive.serial_number)
        _new_json_file_path = os.path.join(self._json_config_path, self._json_file_prefix.format(_hex_serial_number))

        with open(_new_json_file_path, 'w') as f:
            json.dump({_CONFIG_KEY: config, _CRC_KEY: crc}, f)

        self.odrives[_hex_serial_number] = odrive

    def find_odrive(self):
        """Find odrive device and set up fibre connection."""
        cancellation_token = threading.Event()

        def callback(channel):
            self._channel = channel
            cancellation_token.set()

        if self.serial_numbers is None:
            self.read_config_from_device()

        else:
            for serial_number in self.serial_numbers:
                usbbulk_transport.discover_channels(path=None, callback=callback, logger=self._fibre_logger,
                                                    serial_number=serial_number, cancellation_token=cancellation_token,
                                                    channel_termination_token=None)

                _json_cache_file_path = os.path.join(self._json_config_path,
                                                     self._json_file_prefix.format(sn=serial_number))

                try:
                    with open(_json_cache_file_path, 'r') as _json_cache_file:
                        data = json.load(_json_cache_file)
                        config = data[_CONFIG_KEY]
                        crc = data[_CRC_KEY]

                        odrive = self.setup_odrive(config=config, crc=crc)
                        self.odrives[serial_number] = odrive

                except Exception as e:
                    rospy.logerr('Could not set the odrive configuration. error: (%s)', str(e))
                    break

    def __getitem__(self, item):
        """Get odrive with specified serial number."""
        return next(odrive for serial_number, odrive in self.odrives.items() if serial_number == item)
