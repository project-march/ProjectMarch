"""Author: Marco Bak MVIII."""

import math
import numpy as np
from mujoco import mjtSensor
from geometry_msgs.msg import Vector3, Quaternion, Point
from sensor_msgs.msg import Imu


class SensorDataExtraction:
    """A class used to extract sensor data correctly from the Mujoco model and data.

    This sensor data will be send to the Hardware interface to be used for sending gaits.
    Also the sensors should receive incoming commands from the March gait follower
    """

    def __init__(self, data, model, sensordata, sensor_type, sensor_adr):
        """A class that extracts and rewrites the sensor data from mujoco.

        Args:
            sensordata (array): array with the sensor data from the simulation
            sensor_type (array): array with the sensors_types of the sensors in the simulation
            sensor_adr (array): array with the sensor_adr of the sensor in the sensordata array
            model (Mujoco struct): refers to the simulated body in Mujoco
        """
        self.data = data
        self.model = model
        self.sensordata = sensordata
        self.sensor_type = sensor_type
        self.sensor_adr = sensor_adr

        self.joint_names = []
        for i in range(model.njnt):
            if "safety_catch" in model.joint(i).name:
                continue
            self.joint_names.append(model.joint(i).name)
        self.joint_names.sort()

        # Store the address of the joint sensors
        self.sensor_joint_pos_adr = []
        self.sensor_joint_vel_adr = []
        self.sensor_joint_torque_adr = []

        for i, sensor_type in enumerate(self.sensor_type):
            if sensor_type == mjtSensor.mjSENS_JOINTPOS:
                self.sensor_joint_pos_adr.append(sensor_adr[i])
            elif sensor_type == mjtSensor.mjSENS_JOINTVEL:
                self.sensor_joint_vel_adr.append(sensor_adr[i])
            elif sensor_type == mjtSensor.mjSENS_TORQUE:
                self.sensor_joint_torque_adr.append(sensor_adr[i])

        # Store the address of the imu sensors
        self.sensor_imu_acc_adr = []
        self.sensor_imu_gyro_adr = []
        self.sensor_imu_quat_adr = []
        self.sensor_imu_lin_pos_adr = []
        self.sensor_imu_lin_vel_adr = []

        for i, sensor_type in enumerate(self.sensor_type):
            if sensor_type == mjtSensor.mjSENS_ACCELEROMETER:
                self.sensor_imu_acc_adr.append(sensor_adr[i])
            elif sensor_type == mjtSensor.mjSENS_GYRO:
                self.sensor_imu_gyro_adr.append(sensor_adr[i])
            elif sensor_type == mjtSensor.mjSENS_FRAMEQUAT:
                self.sensor_imu_quat_adr.append(sensor_adr[i])
            elif sensor_type == mjtSensor.mjSENS_FRAMEPOS:
                self.sensor_imu_lin_pos_adr.append(sensor_adr[i])
            elif sensor_type == mjtSensor.mjSENS_FRAMELINVEL:
                self.sensor_imu_lin_vel_adr.append(sensor_adr[i])

        # Obtain the joint names, and store in alphabetical order
        self.joint_names = []
        for i in range(model.nq):
            name = ""
            j = model.name_jntadr[i]
            while model.names[j] != 0:
                name += chr(model.names[j])
                j += 1
            # Skip safety catch joints
            if "safety_catch" in name:
                continue
            self.joint_names.append(name)
        self.joint_names.sort()

    def get_joint_pos(self):
        """This class extracts the data from the position sensors on the joints of the model.

        The data is retrieved from the sensordata array of the simulation.
        To make sure that the data is correctly retrieved, the address of the pos sensors have to be retrieved from the
        sensor_adr array.
        """
        return [self.sensordata[adr] for adr in self.sensor_joint_pos_adr], self.joint_names

    def get_joint_vel(self):
        """This class extracts the data from the velocity sensors on the joints of the model.

        The data is retrieved from the sensordata array of the simulation.
        To make sure that the data is correctly retrieved, the address of the vel sensors have to be retrieved from the
        sensor_adr array.
        """
        joint_vel = []
        for joint_name in self.joint_names:
            try:
                joint_vel.append(self.data.sensor(joint_name + "_vel_output").data[0].copy())
            except:
                joint_vel.append(0.0)
        return joint_vel

    def get_joint_acc(self):
        """This class extracts the data from the torque sensors on the joints of the model.

        The data is retrieved from the sensordata array of the simulation.
        To make sure that the data is correctly retrieved, the address of the torque sensors should be retrieved
        from the sensor_adr array.
        Since the torque sensors give a 3d x y z output, this should be generalized using the following formula:
        sqrt(x^2 + y^2 + z^2)
        """
        joint_acc = []
        for joint_name in self.joint_names:
            try:
                joint_acc.append(np.linalg.norm(self.data.sensor(joint_name + "_tor_output").data.copy()))
            except:
                joint_acc.append(0.0)
        return joint_acc

    def get_imu_data(self):
        """This class extracts the data from the imus of the model.

        In Mujoco there is no imu sensor, so gyro, accelerometer and magneto is used to simulate the IMU.
        The data is retrieved from the sensordata array of the simulation.
        Since the sensors give a 3d x y z output, this should be generalized using the following formula:
        sqrt(x^ + y^2 + z^2)
        """    
        backpack_imu = Imu()
        torso_imu = Imu()
        backpack_position = Point()

        for i in range(len(self.sensor_imu_acc_adr)):
            gyro = Vector3()
            accel = Vector3()
            quat = Quaternion()
            gyro.x = self.sensordata[self.sensor_imu_gyro_adr[i]]
            gyro.y = self.sensordata[self.sensor_imu_gyro_adr[i]+1]
            gyro.z = self.sensordata[self.sensor_imu_gyro_adr[i]+2]
            accel.x = self.sensordata[self.sensor_imu_acc_adr[i]]
            accel.y = self.sensordata[self.sensor_imu_acc_adr[i]+1]
            accel.z = self.sensordata[self.sensor_imu_acc_adr[i]+2]
            quat.w = self.sensordata[self.sensor_imu_quat_adr[i]]
            quat.x = self.sensordata[self.sensor_imu_quat_adr[i]+1]
            quat.y = self.sensordata[self.sensor_imu_quat_adr[i]+2]
            quat.z = self.sensordata[self.sensor_imu_quat_adr[i]+3]

            if i == 0:
                backpack_imu.angular_velocity = gyro
                backpack_imu.linear_acceleration = accel
                backpack_imu.orientation = quat
            else:
                torso_imu.angular_velocity = gyro
                torso_imu.linear_acceleration = accel
                torso_imu.orientation = quat

        backpack_position = Point()
        backpack_position.x = self.sensordata[self.sensor_imu_lin_pos_adr[0]]
        backpack_position.y = self.sensordata[self.sensor_imu_lin_pos_adr[0]+1]
        backpack_position.z = self.sensordata[self.sensor_imu_lin_pos_adr[0]+2]

        backpack_velocity = Vector3()
        backpack_velocity.x = self.sensordata[self.sensor_imu_lin_vel_adr[0]]
        backpack_velocity.y = self.sensordata[self.sensor_imu_lin_vel_adr[0]+1]
        backpack_velocity.z = self.sensordata[self.sensor_imu_lin_vel_adr[0]+2]

        return backpack_imu, torso_imu, backpack_position, backpack_velocity
