

import rospy

from .odrive_connection_manager import OdriveConnectionManager


ODRIVE_CONNECTION_MANAGER = OdriveConnectionManager(serial_numbers=['2084387E304E'])

TYPE_ERROR_MSG = 'Given type {gt} for {nm} is insufficient, requested type is {rt}'


class OdriveMotor(object):
    def __init__(self, odrive, odrive_axis):
        """Base class to communicate with a specific motor connected to a specified Odrive.

        :param odrive: an odrive object (example: odrv0)
        :param odrive_axis: specify the axis of the odrive which represent the connected motor
        """
        self._odrive = odrive
        self._motor = odrive_axis

    @classmethod
    def create_odrive(cls, serial_number, axis_nr):
        """Create an Odrive object to communicate with a specific motor.

        :param serial_number: Serial number reference of the odrive in hex
        :param axis_nr: The axis number of the motor ['axis0', 'axis1']
        """
        odrive = ODRIVE_CONNECTION_MANAGER[serial_number]

        if odrive is None:
            rospy.logerr('Odrive with serial number: {sn}, could not be found'.format(sn=serial_number))
            return None

        axis_reference = getattr(odrive, axis_nr, None)

        if axis_reference is None:
            rospy.logerr('Odrive {sn} does not have axis: {ax}'.format(sn=serial_number, ax=axis_nr))
            return None

        return cls(odrive, axis_reference)

    # Odrive variables
    @property
    def voltage(self):
        """Return the voltage on the odrive."""
        return self._odrive.vbus_voltage

    @property
    def serial_number(self):
        """Return the serial number linked to this odrive."""
        return '{sn:12X}'.format(sn=self._odrive.serial_number)

    @property
    def uptime(self):
        return self._odrive.uptime

    # Motor variables
    @property
    def error(self):
        """Return the error state of the motor."""
        return self._motor.error

    @property
    def current_state(self):
        """Return the current state of the motor."""
        return self._motor.current_state

    @property
    def watchdog_timer(self):
        """Return the currently set watchdog timer."""
        return self._motor.config.watchdog_timeout

    @watchdog_timer.setter
    def watchdog_timer(self, watchdog_timeout):
        """Set the watchdog timer using the watchdog property.

        :param watchdog_timeout: the watchdog time as float
        """
        if not isinstance(watchdog_timeout, float):
            raise TypeError(TYPE_ERROR_MSG.format(gt=type(watchdog_timeout), nm='watchdog_timeout', rt='float'))
        self._motor.config.watchdog_timeout = watchdog_timeout

    # Control variables
    @property
    def control_mode(self):
        """Return the control mode of the motor."""
        return self._motor.config.control_mode

    @control_mode.setter
    def control_mode(self, control_mode):
        """Set the control mode of the motor using the control mode property.

        :param control_mode: The control mode as int8
        """
        if not isinstance(control_mode, int):
            raise TypeError(TYPE_ERROR_MSG.format(gt=type(control_mode), nm='control_mode', rt='int8'))
        self._motor.config.control_mode = control_mode

    @property
    def current_limit(self):
        """Return the current limit of the motor."""
        return self._motor.motor.config.current_lim

    @current_limit.setter
    def current_limit(self, current_limit):
        """Set the current limit using the current limit property.

        :param current_limit: The current limit as float
        """
        if not isinstance(current_limit, float):
            raise TypeError(TYPE_ERROR_MSG.format(gt=type(current_limit), nm='current_limit', rt='float'))
        self._motor.motor.config.current_lim = current_limit
