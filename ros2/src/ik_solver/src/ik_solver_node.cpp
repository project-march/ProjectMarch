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
    , m_stance_foot(1)
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

    m_ik_visualizer = this->create_publisher<visualization_msgs::msg::Marker>("ik_visualizations", 10);

    // Initializing the IK solver
    declare_parameter("robot_description", std::string(""));
    auto robot_description = this->get_parameter("robot_description").as_string();
    m_ik_solver.load_urdf_model(robot_description);
    m_ik_solver.initialize_solver();

    publish_ik_visualizations();

    // Initializing the timestep
    declare_parameter("timestep", 50);
    m_timestep = this->get_parameter("timestep").as_int();

    m_solving_timer = this->create_wall_timer(
        std::chrono::milliseconds(m_timestep), std::bind(&IkSolverNode::timer_callback, this));
}

/**
 * This callback updates the com_trajectory that the exo needs to follow, and for which the IK solver calculates the
 * trajectory for. When the reset flag is -1, the gait selection node is used so the callback should not update
 * anything.
 * @param msg the message received from the zmp-mpc with the com trajectory.
 */
void IkSolverNode::com_trajectory_subscriber_callback(march_shared_msgs::msg::IkSolverCommand::SharedPtr msg)
{
    if (this->m_reset != -1) {
        // Reset the timer
        m_com_trajectory_index = 0;
        // Update desired state
        m_com_trajectory_container = msg;
    }
}

/**
 * This callback is activated when a new swing-leg trajectory is received.
 *
 * When the reset flag is -1, the gait selection node is used so the callback should not update anything.
 * @param msg the message received from the sing-leg trajectory generator.
 */
void IkSolverNode::swing_trajectory_subscriber_callback(march_shared_msgs::msg::IkSolverCommand::SharedPtr msg)
{
    if (this->m_reset != -1) {
        // Reset the timer
        m_swing_trajectory_index = 0;
        // Update desired state
        m_swing_trajectory_container = msg;
    }
}

/**
 * This callback receives the updates of the joint states from the state estimator.
 *
 * When the reset flag is -1, the gait selection node is used so the callback should not update anything.
 * @param msg
 */
void IkSolverNode::joint_state_subscriber_callback(sensor_msgs::msg::JointState::SharedPtr msg)
{
    if (this->m_reset != -1) {
        // m_joint_names = msg->name;
        // m_ik_solver.set_joint_configuration(msg);
        // m_ik_solver.set_current_state();
        // m_ik_solver.set_jacobian();
    }
}

/**
 * Callback that updates the current foot positions of the ex.
 *
 * When the reset flag is -1, the gait selection node is used so the callback should not update anything.
 * @param msg
 */
void IkSolverNode::foot_subscriber_callback(geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    if (this->m_reset != -1) {
        set_foot_placement(msg);
    }
}

/**
 * set the foot positions to the last received foot positions.
 * @param setter the foot positions to which the state should be set.
 */
void IkSolverNode::set_foot_placement(geometry_msgs::msg::PoseArray::SharedPtr setter)
{
    m_latest_foot_positions = setter;
}

/**
 * This callback updates the stance foot of the ik with the stance foot determined by the state esstimator.
 *
 * When the reset flag is -1, the gait selection node is used so the callback should not update anything.
 * @param msg
 */
void IkSolverNode::stance_foot_callback(std_msgs::msg::Int32::SharedPtr msg)
{
    if (this->m_reset != -1) {
        m_stance_foot = msg->data;
    }
}

/**
 * When the reset flag is -1, the gait selection node is used.
 * This means that the trajectories should be set to null pointers to stop the IK from solving.
 * If the IK does not stop solving, the trajectories will conflict with those of the gait selection node.
 * @param msg
 */
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
    this->m_reset = msg->data;
}

/**
 * The timer callback starts the solving cycle.
 * It makes sure that every time interval the IK solves ands sends a trajectory to the ros2 control.
 */
void IkSolverNode::timer_callback()
{
    // Construct the state
    if (!(m_latest_foot_positions) || !(m_com_trajectory_container) || !(m_swing_trajectory_container)
        || (m_stance_foot == 0)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for input");
        return;
    } else {
        // IN THE POSE ARRAY, INDEX 1 IS RIGHT AND INDEX -1 IS LEFT
        if (m_stance_foot == 1) {
            m_desired_state.right_foot_pose << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            m_desired_state.right_foot_vel << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

            m_desired_state.left_foot_pose << m_swing_trajectory_container->velocity[m_swing_trajectory_index].x,
                m_swing_trajectory_container->velocity[m_swing_trajectory_index].y,
                m_swing_trajectory_container->velocity[m_swing_trajectory_index].z, 0.0, 0.0, 0.0;
        }
        if (m_stance_foot == -1) {
            m_desired_state.left_foot_pose << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

            m_desired_state.left_foot_vel << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

            m_desired_state.right_foot_pose << m_swing_trajectory_container->velocity[m_swing_trajectory_index].x,
                m_swing_trajectory_container->velocity[m_swing_trajectory_index].y,
                m_swing_trajectory_container->velocity[m_swing_trajectory_index].z, 0.0, 0.0, 0.0;
        }

        m_desired_state.com_pos << m_com_trajectory_container->velocity[m_com_trajectory_index].x,
            m_com_trajectory_container->velocity[m_com_trajectory_index].y,
            m_com_trajectory_container->velocity[m_com_trajectory_index].z, 0.0, 0.0, 0.0;

        Eigen::VectorXd solution_velocity = m_ik_solver.solve_for_velocity(
            m_ik_solver.get_state(), m_desired_state, static_cast<double>(m_timestep) / 1000.0, m_stance_foot);

        std::stringstream ss;
        Eigen::VectorXd solution_position
            = m_ik_solver.velocity_to_pos(solution_velocity, static_cast<double>(m_timestep) / 1000.0);
        ss << solution_velocity.format(Eigen::IOFormat(6, 0, ", ", "\n", "", ""));

        ss.clear();
        ss.str("");
        publish_ik_visualizations();
        if (std::isnan(solution_velocity(0))) {
            RCLCPP_WARN(rclcpp::get_logger("ik_solver"), "\n\nNO SOLUTION FOUND\n\n");
            return;
        }
        publish_joint_states(
            std::vector<double>(solution_position.data(), solution_position.data() + solution_position.size()));

        if (m_swing_trajectory_index >= m_swing_trajectory_container->velocity.size()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Reached end of swing trajectory");
        } else {
            m_swing_trajectory_index++;
        }

        if (m_com_trajectory_index >= m_com_trajectory_container->velocity.size()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Reached end of com trajectory");
        } else {
            m_com_trajectory_index++;
        }
    }
}

/**
 * Publish the calculated trajectory to the ros2 control.
 * @param joint_positions
 */
void IkSolverNode::publish_joint_states(std::vector<double> joint_positions)
{
    trajectory_msgs::msg::JointTrajectory trajectory = trajectory_msgs::msg::JointTrajectory();
    trajectory_msgs::msg::JointTrajectoryPoint point_prev;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    trajectory.joint_names.push_back("right_ankle");
    trajectory.joint_names.push_back("right_hip_aa");
    trajectory.joint_names.push_back("right_hip_fe");
    trajectory.joint_names.push_back("right_knee");
    trajectory.joint_names.push_back("left_ankle");
    trajectory.joint_names.push_back("left_hip_aa");
    trajectory.joint_names.push_back("left_hip_fe");
    trajectory.joint_names.push_back("left_knee");

    pinocchio::JointIndex index = 0;

    pinocchio::Model pinocchio_model = m_ik_solver.get_model();
    std::vector<double> previous_joint_positions = m_ik_solver.get_joint_pos();

    for (auto& i : trajectory.joint_names) {
        index = pinocchio_model.getJointId(i);
        point.positions.push_back(joint_positions[pinocchio_model.joints[index].idx_q()]);
    }

    point_prev = point_prev_saved;
    point_prev.time_from_start.sec = 0.0;
    point_prev.time_from_start.nanosec = 0.0;
    point.time_from_start.sec = 0;
    point.time_from_start.nanosec = m_timestep * 1e6;
    trajectory.points.push_back(point_prev);
    trajectory.points.push_back(point);
    trajectory.header.stamp = this->get_clock()->now();
    point_prev_saved = point;
    m_joint_trajectory_publisher->publish(trajectory);

    sensor_msgs::msg::JointState::SharedPtr internal_state = std::make_shared<sensor_msgs::msg::JointState>();
    internal_state->position = point.positions;
    internal_state->name = trajectory.joint_names;
    m_ik_solver.set_joint_configuration(internal_state);
    m_ik_solver.set_current_state();
    m_ik_solver.set_jacobian();
}

/**
 * An extra method to visualize the ik movements in RVIZ. This makes it easier to verify and test the IK solver.
 */
void IkSolverNode::publish_ik_visualizations()
{
    pinocchio::Model vis_model = m_ik_solver.get_model();
    pinocchio::Data vis_data = m_ik_solver.get_data();

    // Publish the joint visualizations
    visualization_msgs::msg::Marker ik_markers;
    ik_markers.type = 7;
    ik_markers.header.frame_id = "map";
    ik_markers.id = 0;

    Eigen::Matrix<double, 3, 1> frame_transform;
    geometry_msgs::msg::Point marker_container;

    for (pinocchio::FrameIndex i = 0; i < static_cast<pinocchio::FrameIndex>(vis_model.nframes); i++) {
        frame_transform = vis_data.oMf[i].translation();
        marker_container.x = frame_transform[0];
        marker_container.y = frame_transform[1];
        marker_container.z = frame_transform[2];
        ik_markers.points.push_back(marker_container);
    }

    ik_markers.action = 0;
    ik_markers.frame_locked = 1;
    ik_markers.scale.x = 0.1;
    ik_markers.scale.y = 0.1;
    ik_markers.scale.z = 0.1;
    ik_markers.pose.position.x = 0.0;
    ik_markers.pose.position.y = 0.0;
    ik_markers.pose.position.z = 0.0;
    ik_markers.pose.orientation.x = 0.0;
    ik_markers.pose.orientation.y = 0.0;
    ik_markers.pose.orientation.z = 0.0;
    ik_markers.pose.orientation.w = 1.0;
    ik_markers.ns = "exo_joint_visualization";
    ik_markers.lifetime.sec = 0;
    ik_markers.color.a = 1.0;
    ik_markers.color.b = 0.5;
    ik_markers.color.g = 0.7;
    m_ik_visualizer->publish(ik_markers);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IkSolverNode>());
    rclcpp::shutdown();
    return 0;
}
