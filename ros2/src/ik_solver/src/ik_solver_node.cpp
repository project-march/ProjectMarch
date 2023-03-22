#include "ik_solver/ik_solver_node.hpp"

IkSolverNode::IkSolverNode()
: Node("ik_solver")
, m_ik_solver()
{
    declare_parameter("robot_description", std::string(""));
    auto robot_description = this->get_parameter("robot_description").as_string();
    RCLCPP_INFO(this->get_logger(), "urdf string: ", robot_description);
    m_ik_solver.load_urdf_model(robot_description);
    RCLCPP_INFO(this->get_logger(), "URDF loaded successfully!");
    m_ik_solver.set_joint_configuration();
    int test = m_ik_solver.set_jacobian();

    RCLCPP_INFO(this->get_logger(), "Result is %i", test);
    RCLCPP_INFO(this->get_logger(), "Joint amount is %i", m_ik_solver.get_model_joints());
    auto jacobian = m_ik_solver.get_model_jacobian();
    std::stringstream ss;
    ss << jacobian.format(Eigen::IOFormat(4, 0, ", ", "\n", "", ""));
    RCLCPP_INFO(this->get_logger(), "Jacobian size: [%ix%i]", jacobian.rows(), jacobian.cols());
    RCLCPP_INFO(this->get_logger(), "Jacobian is :\n" + ss.str());
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IkSolverNode>());
    rclcpp::shutdown();
    return 0;
}
