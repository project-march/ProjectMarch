"""Author: Marco Bak MVIII."""

import math
from mujoco import mjtSensor
from geometry_msgs.msg import Vector3, Quaternion, Point
from sensor_msgs.msg import Imu


class SensorDataExtraction:
    """A class used to extract sensor data correctly from the Mujoco model and data.

    This sensor data will be send to the Hardware interface to be used for sending gaits.
    Also the sensors should receive incoming commands from the March gait follower
    """

    def __init__(self, sensordata, sensor_type, sensor_adr):
        """A class that extracts and rewrites the sensor data from mujoco.

        Args:
            sensordata (array): array with the sensor data from the simulation
            sensor_type (array): array with the sensors_types of the sensors in the simulation
            sensor_adr (array): array with the sensor_adr of the sensor in the sensordata array
            model (Mujoco struct): refers to the simulated body in Mujoco
        """
        self.sensordata = sensordata
        self.sensor_type = sensor_type
        self.sensor_adr = sensor_adr

    def get_joint_pos(self):
        """This class extracts the data from the position sensors on the joints of the model.

        The data is retrieved from the sensordata array of the simulation.
        To make sure that the data is correctly retrieved, the address of the pos sensors have to be retrieved from the
        sensor_adr array.
        """
        joint_pos = []
        joint_pos_sensor_adr = []
        for i, sensor_type in enumerate(self.sensor_type):
            if sensor_type == mjtSensor.mjSENS_JOINTPOS:
                joint_pos_sensor_adr.append(self.sensor_adr[i])
        for adr in joint_pos_sensor_adr:
            joint_pos.append(self.sensordata[adr])
        return joint_pos

    def get_joint_vel(self):
        """This class extracts the data from the velocity sensors on the joints of the model.

        The data is retrieved from the sensordata array of the simulation.
        To make sure that the data is correctly retrieved, the address of the vel sensors have to be retrieved from the
        sensor_adr array.
        """
        joint_vel = []
        joint_vel_sensor_adr = []
        for i, sensor_type in enumerate(self.sensor_type):
            if sensor_type == mjtSensor.mjSENS_JOINTVEL:
                joint_vel_sensor_adr.append(self.sensor_adr[i])
        for adr in joint_vel_sensor_adr:
            joint_vel.append(self.sensordata[adr])
        return joint_vel

    def get_joint_acc(self):
        """This class extracts the data from the torque sensors on the joints of the model.

        The data is retrieved from the sensordata array of the simulation.
        To make sure that the data is correctly retrieved, the address of the torque sensors should be retrieved
        from the sensor_adr array.
        Since the torque sensors give a 3d x y z output, this should be generalized using the following formula:
        sqrt(x^ + y^2 + z^2)
        """
        joint_acc = []
        joint_acc_sensor_adr = []
        for i, sensor_type in enumerate(self.sensor_type):
            if sensor_type == mjtSensor.mjSENS_TORQUE:
                joint_acc_sensor_adr.append(self.sensor_adr[i])
        for adr in joint_acc_sensor_adr:
            torque_x = self.sensordata[adr]
            torque_y = self.sensordata[adr + 1]
            torque_z = self.sensordata[adr + 2]
            torque_res = math.sqrt(torque_x**2 + torque_y**2 + torque_z**2)
            joint_acc.append(torque_res)
        return joint_acc



    def get_imu_data(self):
        """This class extracts the data from the imus of the model.

        In Mujoco there is no imu sensor, so gyro, accelerometer and magneto is used to simulate the IMU.
        The data is retrieved from the sensordata array of the simulation.
        Since the sensors give a 3d x y z output, this should be generalized using the following formula:
        sqrt(x^ + y^2 + z^2)
        """
        gyro_adr = []
        accelero_adr = []
        quat_adr = []
        pos_adr = []
        lin_vel_adr = []
        for i, sensor_type in enumerate(self.sensor_type):
            if sensor_type == mjtSensor.mjSENS_ACCELEROMETER:
                accelero_adr.append(self.sensor_adr[i])
            elif sensor_type == mjtSensor.mjSENS_GYRO:
                gyro_adr.append(self.sensor_adr[i])
            elif sensor_type == mjtSensor.mjSENS_FRAMEQUAT:
                quat_adr.append(self.sensor_adr[i])
            elif sensor_type == mjtSensor.mjSENS_FRAMEPOS:
                pos_adr.append(self.sensor_adr[i])
            elif sensor_type == mjtSensor.mjSENS_FRAMELINVEL:
                lin_vel_adr.append(self.sensor_adr[i])
        backpack_imu = Imu()
        torso_imu = Imu()
        backpack_position = Point()

        for i in range(len(gyro_adr)):
            gyro = Vector3()
            accel = Vector3()
            quat = Quaternion()
            gyro.x = self.sensordata[gyro_adr[i]]
            gyro.y = self.sensordata[gyro_adr[i] + 1]
            gyro.z = self.sensordata[gyro_adr[i] + 2]
            accel.x = self.sensordata[accelero_adr[i]]
            accel.y = self.sensordata[accelero_adr[i] + 1]
            accel.z = self.sensordata[accelero_adr[i] + 2]
            quat.w = self.sensordata[quat_adr[i]]
            quat.x = self.sensordata[quat_adr[i] + 1]
            quat.y = self.sensordata[quat_adr[i] + 2]
            quat.z = self.sensordata[quat_adr[i] + 3]

            if i == 0:
                backpack_imu.angular_velocity = gyro
                backpack_imu.linear_acceleration = accel
                backpack_imu.orientation = quat
            else:
                torso_imu.angular_velocity = gyro
                torso_imu.linear_acceleration = accel
                torso_imu.orientation = quat

        backpack_position = Point()
        backpack_position.x = self.sensordata[pos_adr[0]]
        backpack_position.y = self.sensordata[pos_adr[0] + 1]
        backpack_position.z = self.sensordata[pos_adr[0] + 2]

        backpack_velocity = Vector3()
        backpack_velocity.x = self.sensordata[lin_vel_adr[0]]
        backpack_velocity.y = self.sensordata[lin_vel_adr[0] + 1]
        backpack_velocity.z = self.sensordata[lin_vel_adr[0] + 2]

        return backpack_imu, torso_imu, backpack_position, backpack_velocity
