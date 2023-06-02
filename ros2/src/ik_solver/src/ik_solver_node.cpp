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

    m_reset_subscriber = this->create_subscription<std_msgs::msg::Int32>(
        "/trajectory_reset_gate", 10, std::bind(&IkSolverNode::reset_subscriber_callback, this, _1));

    // Initializing the IK solver
    declare_parameter("robot_description", std::string(""));
    auto robot_description = this->get_parameter("robot_description").as_string();
    m_ik_solver.load_urdf_model(robot_description);
    m_ik_solver.initialize_solver();

    // Initializing the timestep
    declare_parameter("timestep", 8);
    m_timestep = this->get_parameter("timestep").as_int();

    m_solving_timer = this->create_wall_timer(
        std::chrono::milliseconds(m_timestep), std::bind(&IkSolverNode::timer_callback, this));

    // pinocchio::Model test_model = m_ik_solver.get_model();
    // for (int i = 0; i < test_model.names.size(); i++) {
    //     RCLCPP_INFO(this->get_logger(), test_model.names[i]);
    // }

    // for (pinocchio::FrameIndex i = 0; i < static_cast<pinocchio::FrameIndex>(test_model.nframes); i++) {
    //     RCLCPP_INFO(this->get_logger(), test_model.frames[i].name);
    // }

    // WARNING THIS IS TEMPORARY!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    m_stance_foot = 1;
    // auto jacobian = m_ik_solver.get_model_jacobian();
    // std::stringstream ss;
    // ss << jacobian.format(Eigen::IOFormat(4, 0, ", ", "\n", "", ""));
    // // RCLCPP_INFO(this->get_logger(), "Jacobian size: [%ix%i]", jacobian.rows(), jacobian.cols());
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
    // RCLCPP_INFO(this->get_logger(), "obtained com trajectory");
}

void IkSolverNode::swing_trajectory_subscriber_callback(march_shared_msgs::msg::IkSolverCommand::SharedPtr msg)
{
    // Reset the timer
    // m_solving_timer->reset();
    m_swing_trajectory_index = 0;
    // Update desired state
    m_swing_trajectory_container = msg;
    // RCLCPP_INFO(this->get_logger(), "obtained swing trajectory");
}

void IkSolverNode::joint_state_subscriber_callback(sensor_msgs::msg::JointState::SharedPtr msg)
{
    m_joint_names = msg->name;
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

void IkSolverNode::reset_subscriber_callback(std_msgs::msg::Int32::SharedPtr msg)
{
    // RESET -1-> RESET TRAJECTORIES
    // RESET  1-> Send 0 Trajectories
    if (msg->data == -1) {
        m_swing_trajectory_container = nullptr;
        m_com_trajectory_container = nullptr;
    }

    if (msg->data == 1) {
        geometry_msgs::msg::Point point_container;
        point_container.x = 0.0;
        point_container.y = 0.0;
        point_container.z = 0.0;

        m_com_trajectory_container = std::make_shared<march_shared_msgs::msg::IkSolverCommand>();
        m_com_trajectory_container->velocity.push_back(point_container);
        m_com_trajectory_container->velocity.push_back(point_container);
        m_com_trajectory_index = 0;

        m_swing_trajectory_container = std::make_shared<march_shared_msgs::msg::IkSolverCommand>();
        m_swing_trajectory_container->velocity.push_back(point_container);
        m_swing_trajectory_container->velocity.push_back(point_container);
        m_swing_trajectory_index = 0;
    }
}

void IkSolverNode::timer_callback()
{
    // Construct the state

    if (!(m_latest_foot_positions) || !(m_com_trajectory_container) || !(m_swing_trajectory_container)
        || (m_stance_foot == 0)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for input");
        return;
    } else {
        // IN THE POSE ARRAY, INDEX 1 IS RIGHT AND INDEX -1 IS LEFT
        if (m_stance_foot == 1) {
            // RCLCPP_INFO(this->get_logger(), "Stance foot is right");
            m_desired_state.right_foot_pose << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            m_desired_state.right_foot_vel << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

            m_desired_state.left_foot_pose << m_swing_trajectory_container->velocity[m_swing_trajectory_index].x,
                m_swing_trajectory_container->velocity[m_swing_trajectory_index].y,
                m_swing_trajectory_container->velocity[m_swing_trajectory_index].z, 0.0, 0.0, 0.0;

            // m_desired_state.left_foot_vel << m_swing_trajectory_container->velocity[m_swing_trajectory_index].x,
            // m_swing_trajectory_container->velocity[m_swing_trajectory_index].y,
            // m_swing_trajectory_container->velocity[m_swing_trajectory_index].z, 0.0, 0.0, 0.0;

            // m_desired_state.left_foot_pose << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            // m_desired_state.left_foot_vel << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }
        if (m_stance_foot == -1) {
            // RCLCPP_INFO(this->get_logger(), "Stance foot is left");
            m_desired_state.left_foot_pose << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

            m_desired_state.left_foot_vel << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

            m_desired_state.right_foot_pose << m_swing_trajectory_container->velocity[m_swing_trajectory_index].x,
                m_swing_trajectory_container->velocity[m_swing_trajectory_index].y,
                m_swing_trajectory_container->velocity[m_swing_trajectory_index].z, 0.0, 0.0, 0.0;

            // m_desired_state.right_foot_vel << m_swing_trajectory_container->velocity[m_swing_trajectory_index].x,
            //     m_swing_trajectory_container->velocity[m_swing_trajectory_index].y,
            //     m_swing_trajectory_container->velocity[m_swing_trajectory_index].z, 0.0, 0.0, 0.0;

            // m_desired_state.right_foot_pose << 0.0, 0.0, 0.0, 0.0, 0.0, 0;
            // m_desired_state.right_foot_vel << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }
        // RCLCPP_INFO(this->get_logger(), "Initialized stance foot");

        m_desired_state.com_pos << m_com_trajectory_container->velocity[m_com_trajectory_index].x,
            m_com_trajectory_container->velocity[m_com_trajectory_index].y,
            m_com_trajectory_container->velocity[m_com_trajectory_index].z, 0.0, 0.0, 0.0;
        // m_desired_state.com_pos << 0.005, 0.0, 0.0, 0.0, 0.0, 0.0;

        // RCLCPP_INFO(this->get_logger(), "Set desired com_pos");

        m_desired_state.com_vel << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        // Get solution

        // RCLCPP_INFO(this->get_logger(), "Set desired com_vel");
        Eigen::VectorXd solution_velocity = m_ik_solver.solve_for_velocity(
            m_ik_solver.get_state(), m_desired_state, static_cast<double>(m_timestep) / 1000.0, m_stance_foot);

        // RCLCPP_INFO(this->get_logger(), "Solved for velocity");
        std::stringstream ss;
        // ss << solution_velocity.format(Eigen::IOFormat(6, 0, ", ", "\n", "", ""));
        // RCLCPP_INFO(rclcpp::get_logger(""), "Solution is :\n" + ss.str() + "\n");
        // ss.clear();
        // ss.str("");
        Eigen::VectorXd solution_position
            = m_ik_solver.velocity_to_pos(solution_velocity, static_cast<double>(m_timestep) / 1000.0);
        ss << solution_position.format(Eigen::IOFormat(6, 0, ", ", "\n", "", ""));
        RCLCPP_INFO(rclcpp::get_logger(""), "Solution is :\n" + ss.str() + "\n");
        ss.clear();
        ss.str("");
        publish_joint_states(
            std::vector<double>(solution_position.data(), solution_position.data() + solution_position.size()));

        // RCLCPP_INFO(this->get_logger(), "Solved for Position");
        if (m_swing_trajectory_index > m_swing_trajectory_container->velocity.size()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Reached end of swing trajectory");
        } else {
            m_swing_trajectory_index++;
        }

        if (m_com_trajectory_index > m_com_trajectory_container->velocity.size()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Reached end of swing trajectory");
        } else {
            m_com_trajectory_index++;
        }

        // RCLCPP_INFO(this->get_logger(), "Published trajectory");
    }
}

void IkSolverNode::publish_joint_states(std::vector<double> joint_positions)
{
    trajectory_msgs::msg::JointTrajectory trajectory = trajectory_msgs::msg::JointTrajectory();
    trajectory_msgs::msg::JointTrajectoryPoint point_prev;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    trajectory.joint_names.push_back("left_ankle");
    trajectory.joint_names.push_back("left_hip_aa");
    trajectory.joint_names.push_back("left_hip_fe");
    trajectory.joint_names.push_back("left_knee");
    trajectory.joint_names.push_back("right_ankle");
    trajectory.joint_names.push_back("right_hip_aa");
    trajectory.joint_names.push_back("right_hip_fe");
    trajectory.joint_names.push_back("right_knee");

    pinocchio::JointIndex index = 0;

    pinocchio::Model pinocchio_model = m_ik_solver.get_model();
    std::vector<double> previous_joint_positions = m_ik_solver.get_joint_pos();

    for (auto& i : trajectory.joint_names) {
        index = pinocchio_model.getJointId(i);
        // RCLCPP_INFO(this->get_logger(), "Joint id of %s is %i", i.c_str(), index);
        point.positions.push_back(joint_positions[pinocchio_model.joints[index].idx_q()]);
        // RCLCPP_INFO(this->get_logger(), "publishing position %f, which is equal to %f",
        // joint_positions[pinocchio_model.joints[index].idx_q()], msg.position.end());
    }

    point_prev = point_prev = point_prev_saved;
    point_prev.time_from_start.sec = 0.0;
    point_prev.time_from_start.nanosec = 0.0;
    point.time_from_start.sec = 0;
    point.time_from_start.nanosec = 1 * m_timestep * 1e6;
    trajectory.points.push_back(point_prev);
    trajectory.points.push_back(point);
    trajectory.header.stamp = this->get_clock()->now();
    point_prev_saved = point;
    // RCLCPP_INFO(this->get_logger(), "size is %i", msg.position.size());

    m_joint_trajectory_publisher->publish(trajectory);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IkSolverNode>());
    rclcpp::shutdown();
    return 0;
}
