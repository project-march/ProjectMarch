#include "ik_solver/ik_solver_node.hpp"

IkSolverNode::IkSolverNode()
    : Node("ik_solver")
    , m_ik_solver()
{
    declare_parameter("robot_description", std::string(""));
    auto robot_description = this->get_parameter("robot_description").as_string();
    m_ik_solver.load_urdf_model(robot_description);

    // pinocchio::Model test_model = m_ik_solver.get_model();
    // for(pinocchio::FrameIndex i=0; i<static_cast<pinocchio::FrameIndex>(test_model.nframes); i++){
    //     RCLCPP_INFO(this->get_logger(), test_model.frames[i].name);
    // }

    m_ik_solver.initialize_solver();
    m_ik_solver.set_joint_configuration();
    m_ik_solver.set_current_state();
    m_ik_solver.set_jacobian();
    auto jacobian = m_ik_solver.get_model_jacobian();
    std::stringstream ss;
    ss << jacobian.format(Eigen::IOFormat(4, 0, ", ", "\n", "", ""));
    RCLCPP_INFO(this->get_logger(), "Jacobian size: [%ix%i]", jacobian.rows(), jacobian.cols());
    RCLCPP_INFO(this->get_logger(), "Jacobian is :\n" + ss.str());

    Eigen::VectorXd test_sol = m_ik_solver.solve_for_velocity(m_ik_solver.get_state(), m_ik_solver.get_state());
    std::stringstream ss2;
    ss2 << test_sol.format(Eigen::IOFormat(8, 0, ", ", "\n", "", ""));
    RCLCPP_INFO(this->get_logger(), "solution is :\n" + ss2.str());
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IkSolverNode>());
    rclcpp::shutdown();
    return 0;
}
