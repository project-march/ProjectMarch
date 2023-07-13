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
    , m_stance_foot(-1)
    , hip_aa_upper_limit(0.1705329252)
    , hip_aa_lower_limit(-0.23617993878)
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
    declare_parameter("timestep", 8);
    m_timestep = this->get_parameter("timestep").as_int();

    m_solving_timer
        = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&IkSolverNode::timer_callback, this));
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
        m_swing_trajectory_index = 0;
        // Update desired state
        m_swing_trajectory_container = msg;

        // m_stance_foot=-m_stance_foot;
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
    set_foot_placement(msg);
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
    m_stance_foot = msg->data;
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
    publish_ik_visualizations();
    if (!(m_latest_foot_positions) || !(m_com_trajectory_container) || !(m_swing_trajectory_container)
        || (m_stance_foot == 0)) {
        //        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        //            "Waiting for input\nCoM input received: %s\n, swing input received: %s\n, Stance foot: %i\n",
        //            (m_com_trajectory_container) ? "true" : "false", (m_swing_trajectory_container) ? "true" :
        //            "false", m_stance_foot);
        return;
    } else {
        float swing_z_factor = 1.0;
        float swing_x_factor = 0.5;
        // IN THE POSE ARRAY, INDEX 1 IS RIGHT AND INDEX -1 IS LEFT
        if (m_stance_foot == 1) {
            m_desired_state.right_foot_pose << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            m_desired_state.right_foot_vel << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            m_desired_state.left_foot_pose
                << m_swing_trajectory_container->velocity[m_swing_trajectory_index].x * swing_x_factor,
                m_swing_trajectory_container->velocity[m_swing_trajectory_index].y,
                m_swing_trajectory_container->velocity[m_swing_trajectory_index].z * swing_z_factor, 0.0, 0.0, 0.0;
        }
        if (m_stance_foot == -1) {
            m_desired_state.left_foot_pose << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            m_desired_state.left_foot_vel << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

            m_desired_state.right_foot_pose
                << m_swing_trajectory_container->velocity[m_swing_trajectory_index].x * swing_x_factor,
                m_swing_trajectory_container->velocity[m_swing_trajectory_index].y,
                m_swing_trajectory_container->velocity[m_swing_trajectory_index].z * swing_z_factor, 0.0, 0.0, 0.0;
        }
        // RCLCPP_INFO(this->get_logger(), "Initialized stance foot");

        m_desired_state.com_pos << m_com_trajectory_container->velocity[m_com_trajectory_index].x, 0.0, 0.0, 0.0, 0.0,
            0.0;
        Eigen::VectorXd solution_velocity = m_ik_solver.solve_for_velocity(
            m_ik_solver.get_state(), m_desired_state, static_cast<double>(m_timestep) / 1000.0, m_stance_foot);

        Eigen::VectorXd solution_position
            = m_ik_solver.velocity_to_pos(solution_velocity, static_cast<double>(m_timestep) / 1000.0);
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
        publish_ik_path();
        publish_com_path();
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
        double xdif;
        double next_joint_pos_weight = 0.8;
        if (i.compare("left_hip_aa") == 0) {
            xdif = (m_com_trajectory_container->trajectory[m_com_trajectory_index].y - 0.33 / 2) * 2.5;
            if (abs(m_previous_xdif - xdif) > 0.10) {
                xdif = m_previous_xdif;
            } else {
                m_previous_xdif = xdif;
            }
            point.positions.push_back(std::max(hip_aa_lower_limit,
                std::min(joint_positions[pinocchio_model.joints[index].idx_q()] * (1 - next_joint_pos_weight)
                        + (xdif - 0.05) * next_joint_pos_weight,
                    hip_aa_upper_limit)));

        } else if (i.compare("right_hip_aa") == 0) {
            xdif = -(m_com_trajectory_container->trajectory[m_com_trajectory_index].y - 0.33 / 2) * 2.5;

            if (abs(m_previous_rxdif - xdif) > 0.8) {
                xdif = m_previous_rxdif;
            } else {
                m_previous_rxdif = xdif;
            }
            point.positions.push_back(std::max(hip_aa_lower_limit,
                std::min(joint_positions[pinocchio_model.joints[index].idx_q()] * (1 - next_joint_pos_weight)
                        + (xdif - 0.05) * next_joint_pos_weight,
                    hip_aa_upper_limit)));
        } else {
            if ((i.compare("right_ankle") == 0) || (i.compare("left_ankle") == 0)) {
                point.positions.push_back(joint_positions[pinocchio_model.joints[index].idx_q() + 0.004]);
            } else {
                point.positions.push_back(joint_positions[pinocchio_model.joints[index].idx_q()]);
            }
        }
    }

    point_prev = point_prev_saved;
    point_prev.time_from_start.sec = 0.0;
    point_prev.time_from_start.nanosec = 0.0;
    point.time_from_start.sec = 0;
    point.time_from_start.nanosec = m_timestep * 1e6;
    if (point_prev_saved.positions.size() > 0) {
        trajectory.points.push_back(point_prev);
    }
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
    ik_markers.scale.x = 0.07;
    ik_markers.scale.y = 0.07;
    ik_markers.scale.z = 0.07;
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

void IkSolverNode::publish_ik_path()
{
    pinocchio::Model vis_model = m_ik_solver.get_model();
    pinocchio::Data vis_data = m_ik_solver.get_data();
    // Publish the joint visualizations
    visualization_msgs::msg::Marker ik_markers;
    ik_markers.type = 7;
    ik_markers.header.frame_id = "map";
    ik_markers.id = 1;
    pinocchio::FrameIndex right_foot_index;
    if (m_stance_foot == -1) {
        right_foot_index = vis_model.getFrameId("R_toe");
    } else {
        right_foot_index = vis_model.getFrameId("L_toe");
    }
    Eigen::Matrix<double, 3, 1> frame_transform;
    geometry_msgs::msg::Point marker_container;
    frame_transform = vis_data.oMf[right_foot_index].translation();
    marker_container.x = frame_transform[0];
    marker_container.y = frame_transform[1];
    marker_container.z = frame_transform[2];

    for (int i = m_swing_trajectory_index; i < m_swing_trajectory_container->velocity.size(); i++) {
        marker_container.x += m_swing_trajectory_container->velocity[i].x * 8 * 1e-3;
        marker_container.y += m_swing_trajectory_container->velocity[i].y * 8 * 1e-3;
        marker_container.z += m_swing_trajectory_container->velocity[i].z * 8 * 1e-3;
        ik_markers.points.push_back(marker_container);
    }

    ik_markers.action = 0;
    ik_markers.frame_locked = 1;
    ik_markers.scale.x = 0.05;
    ik_markers.scale.y = 0.05;
    ik_markers.scale.z = 0.05;
    ik_markers.pose.position.x = 0.0;
    ik_markers.pose.position.y = 0.0;
    ik_markers.pose.position.z = 0.0;
    ik_markers.pose.orientation.x = 0.0;
    ik_markers.pose.orientation.y = 0.0;
    ik_markers.pose.orientation.z = 0.0;
    ik_markers.pose.orientation.w = 1.0;
    ik_markers.ns = "exo_joint_visualization";
    ik_markers.lifetime.sec = 0;
    ik_markers.color.a = 0.5;
    ik_markers.color.r = 0.7;
    ik_markers.color.g = 0.7;
    ik_markers.color.b = 0.6;
    m_ik_visualizer->publish(ik_markers);
}

void IkSolverNode::publish_com_path()
{
    pinocchio::Model vis_model = m_ik_solver.get_model();
    pinocchio::Data vis_data = m_ik_solver.get_data();
    // Publish the joint visualizations
    visualization_msgs::msg::Marker ik_markers;
    ik_markers.type = 7;
    ik_markers.header.frame_id = "map";
    ik_markers.id = 2;

    geometry_msgs::msg::Point marker_container;
    Eigen::VectorXd com_pos = m_ik_solver.get_state().com_pos;
    marker_container.x = com_pos(0);
    marker_container.y = com_pos(1);
    marker_container.z = com_pos(2);

    for (int i = m_com_trajectory_index; i < m_com_trajectory_container->velocity.size(); i++) {
        marker_container.x += m_com_trajectory_container->velocity[i].x * 8 * 1e-3;
        marker_container.y += m_com_trajectory_container->velocity[i].y * 8 * 1e-3;
        marker_container.z += m_com_trajectory_container->velocity[i].z * 8 * 1e-3;
        ik_markers.points.push_back(marker_container);
    }
    ik_markers.action = 0;
    ik_markers.frame_locked = 1;
    ik_markers.scale.x = 0.05;
    ik_markers.scale.y = 0.05;
    ik_markers.scale.z = 0.05;
    ik_markers.pose.position.x = 0.0;
    ik_markers.pose.position.y = 0.0;
    ik_markers.pose.position.z = 0.0;
    ik_markers.pose.orientation.x = 0.0;
    ik_markers.pose.orientation.y = 0.0;
    ik_markers.pose.orientation.z = 0.0;
    ik_markers.pose.orientation.w = 1.0;
    ik_markers.ns = "exo_joint_visualization";
    ik_markers.lifetime.sec = 0;
    ik_markers.color.a = 0.6;
    ik_markers.color.r = 0.4;
    ik_markers.color.g = 0.6;
    ik_markers.color.b = 0.9;
    m_ik_visualizer->publish(ik_markers);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IkSolverNode>());
    rclcpp::shutdown();
    return 0;
}
