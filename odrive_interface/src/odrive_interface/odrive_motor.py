

import rospy

from .odrive_connection_manager import OdriveConnectionManager


odrive_connection_manager = OdriveConnectionManager(serial_numbers=['2084387E304E'])


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
        odrive = odrive_connection_manager[serial_number]

        if odrive is None:
            rospy.logerr('Odrive with serial number: {sn}, could not be found'.format(sn=serial_number))
            return None

        axis_reference = getattr(odrive, axis_nr, None)

        if axis_reference is None:
            rospy.logerr('Odrive {sn} does not have axis: {ax}'.format(sn=serial_number, ax=axis_nr))
            return None

        return cls(odrive, axis_reference)

    @property
    def voltage(self):
        """Get the voltage on the odrive."""
        return self._odrive.vbus_voltage
