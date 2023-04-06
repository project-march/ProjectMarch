#include "ik_solver/ik_solver_node.hpp"
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

IkSolverNode::IkSolverNode()
    : Node("ik_solver")
    , m_ik_solver()
{
    m_trajectory_subscriber = this->create_subscription<march_shared_msgs::msg::IkSolverCommand>(
        "/ik_solver_input", 10, std::bind(&IkSolverNode::trajectory_subscriber_callback, this, _1));

    m_joint_state_subscriber = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_state", 10, std::bind(&IkSolverNode::joint_state_subscriber_callback, this, _1));

    m_foot_subscriber = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/est_foot_position", 10, std::bind(&IkSolverNode::foot_subscriber_callback, this, _1));

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

void IkSolverNode::trajectory_subscriber_callback(march_shared_msgs::msg::IkSolverCommand::SharedPtr msg)
{
}

void IkSolverNode::joint_state_subscriber_callback(sensor_msgs::msg::JointState::SharedPtr msg)
{
}

void IkSolverNode::foot_subscriber_callback(geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    set_foot_placement(msg);
}

void IkSolverNode::set_foot_placement(geometry_msgs::msg::PointStamped::SharedPtr setter)
{
    m_latest_placed_foot = setter;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IkSolverNode>());
    rclcpp::shutdown();
    return 0;
}
