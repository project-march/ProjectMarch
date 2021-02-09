import errno
from math import pi
import socket

import rclpy
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import TransformStamped
import numpy
from rcl_interfaces.srv import GetParameters
from rclpy import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler, quaternion_multiply
import tf2_ros
from urdf_parser_py import urdf
from urdf_parser_py.urdf import URDF
from visualization_msgs.msg import Marker

from march_shared_msgs.msg import JointValues, PressureSole


from .com_calculator import CoMCalculator
from .cp_calculator import CPCalculator


class DataCollector(Node):
    def __init__(self, node_name: str):
        # key is the swing foot and item is the static foot
        super().__init__(node_name)
        feet = {"foot_left": "foot_right", "foot_right": "foot_left"}
        robot = self._initial_robot_description()
        self.differentiation_order = 2
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer, self)
        self.feet = feet

        self._com_calculator = CoMCalculator(self, robot, self.tf_buffer)
        self._cp_calculators = [
            CPCalculator(self, self.tf_buffer, swing_foot, static_foot)
            for swing_foot, static_foot in feet.items()
        ]

        self.position_memory = []
        self.time_memory = []
        self.joint_values = JointValues()

        self.joint_values_publisher = self.create_publisher(
            topic="/march/joint_values", msg_type=JointValues,
            qos_profile=10
        )

        self._imu_broadcaster = tf2_ros.TransformBroadcaster(self)
        self._com_marker_publisher = self.create_publisher(
            topic="/march/com_marker", msg_type=Marker, qos_profile=10
        )

        self._trajectory_state_subscriber = self.create_subscription(
            topic="/march/controller/trajectory/state",
            msg_type=JointTrajectoryControllerState,
            callback=self.trajectory_state_callback,
            qos_profile=10
        )

        self._imu_subscriber = self.create_subscription(
            topic="/march/imu",
            msg_type=Imu,
            callback=self.imu_callback,
            qos_profile=10
        )

        self.pressure_soles_on = self.get_parameter(
            "~pressure_soles").get_parameter_value().bool_value

        self.transform_imu = TransformStamped()

        self.transform_imu.header.frame_id = "world"
        self.transform_imu.child_frame_id = "imu_link"
        self.transform_imu.transform.translation.x = 0.0
        self.transform_imu.transform.translation.y = 0.0

        if self.pressure_soles_on:
            self.get_logger().debug("Connecting to pressure soles")
            self.output_host = self.get_parameter("~moticon_ip").get_parameter_value().string_value
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.connect(("8.8.8.8", 53))
            self.input_host = sock.getsockname()[0]
            sock.close()
            self.output_port = 8888
            self.output_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

            self.input_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                self.input_sock.bind((self.input_host, 9999))
            except socket.error:
                self.get_logger().warn(
                    "Cannot connect to host, is the adress correct? \nrunning without pressure soles"
                )
                self.pressure_soles_on = False
                self.close_sockets()

            self._pressure_sole_publisher = self.create_publisher(
                topic="/march/pressure_soles", msg_type=PressureSole, qos_profile=1
            )
        else:
            self.get_logger().debug("running without pressure soles")


    def _initial_robot_description(self):
        """
        Initialize the robot description by getting it from the robot state
        publisher.
        """
        robot_description_client = self.create_client(
            srv_type=GetParameters,
            srv_name="/march/robot_state_publisher/get_parameters",
        )
        while not robot_description_client.wait_for_service(timeout_sec=2):
            self.get_logger().warn(
                "Robot description is not being published, waiting.."
            )

        robot_future = robot_description_client.call_async(
            request=GetParameters.Request(names=["robot_description"])
        )
        rclpy.spin_until_future_complete(self, robot_future)

        return urdf.Robot.from_xml_string(robot_future.result().values[0].string_value)


    def trajectory_state_callback(self, data):
        com = self._com_calculator.calculate_com()
        for cp_calculator in self._cp_calculators:
            cp_calculator.center_of_mass = com
        self._com_marker_publisher.publish(com)
        if self.pressure_soles_on:
            self.send_udp(data.actual.positions)

        self.position_memory.append(data.actual.positions)
        self.time_memory.append(
            data.header.stamp.secs + data.header.stamp.nsecs * 10 ** (-9)
        )
        if len(self.position_memory) > self.differentiation_order + 1:
            self.position_memory.pop(0)
            self.time_memory.pop(0)
        if len(self.position_memory) > self.differentiation_order:
            velocity = numpy.gradient(
                self.position_memory,
                self.time_memory,
                edge_order=self.differentiation_order,
                axis=0,
            )
            acceleration = numpy.gradient(
                velocity,
                self.time_memory,
                edge_order=self.differentiation_order,
                axis=0,
            )
            jerk = numpy.gradient(
                acceleration,
                self.time_memory,
                edge_order=self.differentiation_order,
                axis=0,
            )

            self.joint_values.controller_output = data
            self.joint_values.velocities = velocity[-1]
            self.joint_values.accelerations = acceleration[-1]
            self.joint_values.jerks = jerk[-1]

            self.joint_values_publisher.publish(self.joint_values)

    def imu_callback(self, data):
        if data.header.frame_id == "imu_link":

            z_diff = float("-inf")
            try:
                old_z = self.tf_buffer.lookup_transform(
                    "world", "imu_link", self.get_clock().now()
                ).transform.translation.z
                for foot in self.feet:
                    trans = self.tf_buffer.lookup_transform("world", foot,
                                                            self.get_clock().now())
                    z_diff = max(z_diff, old_z - trans.transform.translation.z)

                self.transform_imu.header.stamp = self.get_clock().now()
                self.transform_imu.transform.translation.z = z_diff

                imu_rotation = quaternion_multiply(
                    [
                        -data.orientation.x,
                        -data.orientation.y,
                        data.orientation.z,
                        data.orientation.w,
                    ],
                    quaternion_from_euler(0, -0.5 * pi, 0),
                )
                self.transform_imu.transform.rotation.x = imu_rotation[0]
                self.transform_imu.transform.rotation.y = imu_rotation[1]
                self.transform_imu.transform.rotation.z = imu_rotation[2]
                self.transform_imu.transform.rotation.w = imu_rotation[3]

                self._imu_broadcaster.sendTransform(self.transform_imu)
            except tf2_ros.TransformException as e:
                self.get_logger().debug(
                    f"Cannot calculate imu transform, because tf frames are not "
                    f"available, {e}"
                )

    def send_udp(self, data):
        message = " ".join([str(180 * val / pi) for val in data])
        self.output_sock.sendto(
            message.encode("utf-8"), (self.output_host, self.output_port)
        )

    def receive_udp(self):
        try:
            data, addr = self.input_sock.recvfrom(1024)
            datachannels = data.split()
            values = [float(x) for x in datachannels]
            pressure_sole_msg = PressureSole()
            pressure_sole_msg.header.stamp = self.get_clock().now()
            pressure_sole_msg.pressure_soles_time = Time(seconds=values[0])
            pressure_sole_msg.cop_left = values[1:3]
            pressure_sole_msg.pressure_left = values[3:19]
            pressure_sole_msg.total_force_left = values[19]
            pressure_sole_msg.cop_right = values[20:22]
            pressure_sole_msg.pressure_right = values[22:38]
            pressure_sole_msg.total_force_right = values[38]
            self._pressure_sole_publisher.publish(pressure_sole_msg)
        except socket.timeout:
            self.get_logger().info(
                "Has not received pressure sole data in a while, are they on?"
            )
        except socket.error as error:
            if error.errno == errno.EINTR:
                self.close_sockets()
            else:
                raise
        return

    def close_sockets(self):
        self.output_sock.close()
        self.input_sock.close()