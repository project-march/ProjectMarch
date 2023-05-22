#include "torque_converter/torque_converter_node.hpp"
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

TorqueConverterNode::TorqueConverterNode()
    : Node("torque_converter")
    , m_torque_converter()
{
    declare_parameter("robot_description", std::string(""));
    auto robot_description = this->get_parameter("robot_description").as_string();
    m_torque_converter.load_urdf_model(robot_description);

     // create subscribers
    m_joint_state_subscriber = this->create_subscription<sensor_msgs::msg::JointState>(
        "/measured_joint_states", 10, std::bind(&TorqueConverterNode::joint_state_subscriber_callback, this, _1));

    m_trajectory_subscriber = this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
        "joint_trajectory_controller/state", 10, std::bind(&TorqueConverterNode::trajectory_subscriber_callback, this, _1));
   
    // create publisher
    m_torque_trajectory_publisher = this->create_publisher<trajectory_msgs::msg::JointTrajectoryPoint>(
        "torque_trajectory", 10); // m_torque_trajectory_publisher->publish(trajectory);

}

void TorqueConverterNode::joint_state_subscriber_callback(sensor_msgs::msg::JointState::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "mq size joint callback is %i", m_torque_converter.m_q.size());
    m_torque_converter.set_joint_config(msg);
    m_torque_converter.set_inertia_matrix();
    m_torque_converter.set_coriolis_matrix();
    m_torque_converter.set_gravity_matrix();
}

void TorqueConverterNode::trajectory_subscriber_callback(control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg)
{
    m_torque_converter.set_desired_pos(msg->desired.positions);
    m_torque_converter.set_vel_des(msg->desired.positions);
    m_torque_converter.set_desired_acc(msg->desired.accelerations);

    Eigen::VectorXd torque_des;
    torque_des = m_torque_converter.convert_to_torque();
    std::stringstream ss;
    ss << "torque_des = " << torque_des.transpose();

    RCLCPP_INFO(this->get_logger(),"torque des = %s", ss.str());

    trajectory_msgs::msg::JointTrajectoryPoint torque_point;
    torque_point.effort
        = std::vector<double>(torque_des.data(),torque_des.data() + torque_des.size());
    m_torque_trajectory_publisher->publish(torque_point);

}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TorqueConverterNode>());
    rclcpp::shutdown();
    return 0;
}
