#include "ik_solver/ik_solver_node.hpp"
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

IkSolverNode::IkSolverNode()
    : Node("ik_solver")
    , m_ik_solver()
    , m_desired_state()
    , m_right_foot_on_ground(true)
    , m_swing_trajectory_index(0)
    , m_com_trajectory_index(0)
    , m_stance_foot(0)
{
    m_com_trajectory_subscriber = this->create_subscription<march_shared_msgs::msg::IkSolverCommand>(
        "/ik_solver_com_input", 10, std::bind(&IkSolverNode::com_trajectory_subscriber_callback, this, _1));

    m_swing_trajectory_subscriber = this->create_subscription<march_shared_msgs::msg::IkSolverCommand>(
        "/ik_solver_swing_input", 10, std::bind(&IkSolverNode::swing_trajectory_subscriber_callback, this, _1));

    m_joint_state_subscriber = this->create_subscription<sensor_msgs::msg::JointState>(
        "/measured_joint_states", 10, std::bind(&IkSolverNode::joint_state_subscriber_callback, this, _1));

    m_foot_subscriber = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/est_foot_position", 10, std::bind(&IkSolverNode::foot_subscriber_callback, this, _1));

    m_stance_foot_subscriber = this->create_subscription<std_msgs::msg::Int32>(
        "/current_stance_foot", 10, std::bind(&IkSolverNode::stance_foot_callback, this, _1));

    m_joint_trajectory_publisher = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "joint_trajectory_controller/joint_trajectory", 10);

    // Initializing the IK solver
    declare_parameter("robot_description", std::string(""));
    auto robot_description = this->get_parameter("robot_description").as_string();
    m_ik_solver.load_urdf_model(robot_description);
    m_ik_solver.initialize_solver();

    // Initializing the timestep
    declare_parameter("timestep", 1000);
    m_timestep = this->get_parameter("timestep").as_int();

    m_solving_timer = this->create_wall_timer(
        std::chrono::milliseconds(m_timestep), std::bind(&IkSolverNode::timer_callback, this));

    pinocchio::Model test_model = m_ik_solver.get_model();
    for (pinocchio::FrameIndex i = 0; i < static_cast<pinocchio::FrameIndex>(test_model.nframes); i++) {
        RCLCPP_INFO(this->get_logger(), test_model.frames[i].name);
    }
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

void IkSolverNode::com_trajectory_subscriber_callback(march_shared_msgs::msg::IkSolverCommand::SharedPtr msg)
{
    // Reset the timer
    // m_solving_timer->reset();
    m_com_trajectory_index = 0;
    // Update desired state
    m_com_trajectory_container = msg;
}

void IkSolverNode::swing_trajectory_subscriber_callback(march_shared_msgs::msg::IkSolverCommand::SharedPtr msg)
{
    // Reset the timer
    // m_solving_timer->reset();
    m_swing_trajectory_index = 0;
    // Update desired state
    m_swing_trajectory_container = msg;
}

void IkSolverNode::joint_state_subscriber_callback(sensor_msgs::msg::JointState::SharedPtr msg)
{
    m_ik_solver.set_joint_configuration(msg);
    m_ik_solver.set_current_state();
    m_ik_solver.set_jacobian();
}

void IkSolverNode::foot_subscriber_callback(geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    set_foot_placement(msg);
}

void IkSolverNode::set_foot_placement(geometry_msgs::msg::PoseArray::SharedPtr setter)
{
    m_latest_foot_positions = setter;
}

void IkSolverNode::stance_foot_callback(std_msgs::msg::Int32::SharedPtr msg)
{
    m_stance_foot = msg->data;
}

void IkSolverNode::timer_callback()
{
    // Construct the state
    m_com_trajectory_index++;
    m_swing_trajectory_index++;

    if (!(m_latest_foot_positions) || !(m_com_trajectory_container) || !(m_swing_trajectory_container)
        || (m_stance_foot == 0)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for input");
    } else {
        // IN THE POSE ARRAY, INDEX 0 IS RIGHT AND INDEX 1 IS LEFT
        if (m_stance_foot == 1) {
            m_desired_state.right_foot_pose << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            m_desired_state.right_foot_vel << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

            m_desired_state.left_foot_pose
                << m_swing_trajectory_container->velocity[m_swing_trajectory_index].x * (m_timestep * 1e-3),
                m_swing_trajectory_container->velocity[m_swing_trajectory_index].y * (m_timestep * 1e-3),
                m_swing_trajectory_container->velocity[m_swing_trajectory_index].z * (m_timestep * 1e-3), 0.0, 0.0, 0.0;

            m_desired_state.left_foot_vel << m_swing_trajectory_container->velocity[m_swing_trajectory_index].x,
                m_swing_trajectory_container->velocity[m_swing_trajectory_index].y,
                m_swing_trajectory_container->velocity[m_swing_trajectory_index].z, 0.0, 0.0, 0.0;
        }
        if (m_stance_foot == -1) {
            m_desired_state.left_foot_pose << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

            m_desired_state.left_foot_vel << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

            m_desired_state.right_foot_pose
                << m_swing_trajectory_container->velocity[m_swing_trajectory_index].x * (m_timestep * 1e-3),
                m_swing_trajectory_container->velocity[m_swing_trajectory_index].y * (m_timestep * 1e-3),
                m_swing_trajectory_container->velocity[m_swing_trajectory_index].z * (m_timestep * 1e-3), 0.0, 0.0, 0.0;

            m_desired_state.right_foot_vel << m_swing_trajectory_container->velocity[m_swing_trajectory_index].x,
                m_swing_trajectory_container->velocity[m_swing_trajectory_index].y,
                m_swing_trajectory_container->velocity[m_swing_trajectory_index].z, 0.0, 0.0, 0.0;
        }

        m_desired_state.com_pos << m_com_trajectory_container->velocity[m_com_trajectory_index].x * (m_timestep * 1e-6),
            m_com_trajectory_container->velocity[m_com_trajectory_index].y * (m_timestep * 1e-6),
            m_com_trajectory_container->velocity[m_com_trajectory_index].z * (m_timestep * 1e-6);

        m_desired_state.com_vel << m_com_trajectory_container->velocity[m_com_trajectory_index].x,
            m_com_trajectory_container->velocity[m_com_trajectory_index].y,
            m_com_trajectory_container->velocity[m_com_trajectory_index].z;
        // Get solution
        Eigen::VectorXd solution_velocity
            = m_ik_solver.solve_for_velocity(m_ik_solver.get_state(), m_desired_state, m_stance_foot);

        Eigen::VectorXd solution_position
            = m_ik_solver.velocity_to_pos(solution_velocity, static_cast<double>(m_timestep) / 1000.0);

        trajectory_msgs::msg::JointTrajectory trajectory = trajectory_msgs::msg::JointTrajectory();
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions
            = std::vector<double>(solution_position.data(), solution_position.data() + solution_position.size());
        point.velocities
            = std::vector<double>(solution_velocity.data(), solution_velocity.data() + solution_velocity.size());
        trajectory.points.push_back(point);
        trajectory.header.stamp = this->get_clock()->now();
        m_joint_trajectory_publisher->publish(trajectory);
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IkSolverNode>());
    rclcpp::shutdown();
    return 0;
}
