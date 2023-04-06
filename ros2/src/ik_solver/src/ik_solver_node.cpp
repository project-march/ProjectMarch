#include "ik_solver/ik_solver_node.hpp"
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

IkSolverNode::IkSolverNode()
    : Node("ik_solver")
    , m_ik_solver()
    , m_trajectory_index(0)
    , m_right_foot_on_ground(true)
    , m_desired_state()
{
    m_trajectory_subscriber = this->create_subscription<march_shared_msgs::msg::IkSolverCommand>(
        "/ik_solver_input", 10, std::bind(&IkSolverNode::trajectory_subscriber_callback, this, _1));

    m_joint_state_subscriber = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_state", 10, std::bind(&IkSolverNode::joint_state_subscriber_callback, this, _1));

    m_foot_subscriber = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/est_foot_position", 10, std::bind(&IkSolverNode::foot_subscriber_callback, this, _1));

    m_solving_timer = this->create_wall_timer(8ms, std::bind(&IkSolverNode::timer_callback, this));

    // Initialiing the IK solver
    declare_parameter("robot_description", std::string(""));
    auto robot_description = this->get_parameter("robot_description").as_string();
    m_ik_solver.load_urdf_model(robot_description);
    m_ik_solver.initialize_solver();

    // // pinocchio::Model test_model = m_ik_solver.get_model();
    // // for(pinocchio::FrameIndex i=0; i<static_cast<pinocchio::FrameIndex>(test_model.nframes); i++){
    // //     RCLCPP_INFO(this->get_logger(), test_model.frames[i].name);
    // // }
    // auto jacobian = m_ik_solver.get_model_jacobian();
    // std::stringstream ss;
    // ss << jacobian.format(Eigen::IOFormat(4, 0, ", ", "\n", "", ""));
    // RCLCPP_INFO(this->get_logger(), "Jacobian size: [%ix%i]", jacobian.rows(), jacobian.cols());
    // RCLCPP_INFO(this->get_logger(), "Jacobian is :\n" + ss.str());

    // Eigen::VectorXd test_sol = m_ik_solver.solve_for_velocity(m_ik_solver.get_state(), m_ik_solver.get_state());
    // std::stringstream ss2;
    // ss2 << test_sol.format(Eigen::IOFormat(8, 0, ", ", "\n", "", ""));
    // RCLCPP_INFO(this->get_logger(), "solution is :\n" + ss2.str());
}

void IkSolverNode::trajectory_subscriber_callback(march_shared_msgs::msg::IkSolverCommand::SharedPtr msg)
{
    // Reset the timer
    m_solving_timer->reset();
    m_trajectory_index = 0;
    // Update desired state
    m_trajectory_container = msg;
}

void IkSolverNode::joint_state_subscriber_callback(sensor_msgs::msg::JointState::SharedPtr msg)
{
    m_ik_solver.set_joint_configuration(msg);
    m_ik_solver.set_current_state();
    m_ik_solver.set_jacobian();
}

void IkSolverNode::foot_subscriber_callback(geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    set_foot_placement(msg);
}

void IkSolverNode::set_foot_placement(geometry_msgs::msg::PointStamped::SharedPtr setter)
{
    m_latest_placed_foot = setter;
}

void IkSolverNode::timer_callback()
{
    // Construct the state
    m_trajectory_index++;

    if (!(m_latest_placed_foot) || !(m_trajectory_container)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for input");
    } else {
        m_desired_state.right_foot_pose << m_latest_placed_foot->point.x, m_latest_placed_foot->point.y,
            m_latest_placed_foot->point.z, 0.0, 0.0, 0.0;
        m_desired_state.left_foot_pose << m_trajectory_container->swing_trajectory[m_trajectory_index].x,
            m_trajectory_container->swing_trajectory[m_trajectory_index].y,
            m_trajectory_container->swing_trajectory[m_trajectory_index].z, 0.0, 0.0, 0.0;
        m_desired_state.com_pos << m_trajectory_container->com_trajectory[m_trajectory_index].x,
            m_trajectory_container->com_trajectory[m_trajectory_index].y,
            m_trajectory_container->com_trajectory[m_trajectory_index].z;
        // Get solution
        Eigen::VectorXd solution = m_ik_solver.solve_for_velocity(m_ik_solver.get_state(), m_desired_state);
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IkSolverNode>());
    rclcpp::shutdown();
    return 0;
}
