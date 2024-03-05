// standard
#include "zmp_mpc_solver/zmp_mpc_solver_node.hpp"
//#include "march_shared_msgs/msg/point_stamped_list.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

SolverNode::SolverNode()
    : Node("mpc_solver_node")
    , m_zmp_solver()
    , m_desired_previous_foot_x(0.0)
    , m_desired_previous_foot_y(0.33)
{
    //    m_trajectory_publisher = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("joint_trajectory",
    //    10);
    m_com_trajectory_publisher = this->create_publisher<geometry_msgs::msg::PoseArray>("com_trajectory", 10);
    m_final_feet_publisher = this->create_publisher<geometry_msgs::msg::PoseArray>("final_feet_position", 10);
    m_com_visualizer_publisher = this->create_publisher<nav_msgs::msg::Path>("com_visualization_trajectory", 10);
    m_zmp_visualizer_publisher = this->create_publisher<nav_msgs::msg::Path>("zmp_visualization_trajectory", 10);
    m__footstep_visualizer_publisher
        = this->create_publisher<visualization_msgs::msg::Marker>("footsteps_visualization", 100);
    m_current_shooting_node_publisher = this->create_publisher<std_msgs::msg::Int32>("current_shooting_node", 10);

    m_com_subscriber = this->create_subscription<march_shared_msgs::msg::CenterOfMass>(
        "/robot_com_position", 10, std::bind(&SolverNode::com_callback, this, _1));
    m_desired_steps_subscriber = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/desired_footsteps", 10, std::bind(&SolverNode::desired_pos_callback, this, _1));
    m_feet_pos_subscriber = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/est_foot_position", 10, std::bind(&SolverNode::feet_callback, this, _1));
    m_zmp_subscriber = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/robot_zmp_position", 10, std::bind(&SolverNode::zmp_callback, this, _1));
    m_stance_foot_subscriber = this->create_subscription<std_msgs::msg::Int32>(
        "/current_stance_foot", 10, std::bind(&SolverNode::stance_foot_callback, this, _1));
    m_right_foot_on_ground_subscriber = this->create_subscription<std_msgs::msg::Bool>(
        "/right_foot_on_ground", 10, std::bind(&SolverNode::right_foot_ground_callback, this, _1));
    m_left_foot_on_ground_subscriber = this->create_subscription<std_msgs::msg::Bool>(
        "/left_foot_on_ground", 10, std::bind(&SolverNode::left_foot_ground_callback, this, _1));

    geometry_msgs::msg::Pose prev_foot_pose_container;

    m_prev_foot_msg.header.frame_id = "world";
    prev_foot_pose_container.position.x = 0.0;
    prev_foot_pose_container.position.y = 0.0;
    prev_foot_pose_container.position.z = 0.0;
    m_prev_foot_msg.poses.push_back(prev_foot_pose_container);

    // timer_callback();

    m_solving_timer = this->create_wall_timer(50ms, std::bind(&SolverNode::timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "Booted up ZMP solver node");
}

void SolverNode::com_callback(march_shared_msgs::msg::CenterOfMass::SharedPtr msg)
{
    m_zmp_solver.set_current_com(msg->position.point.x, msg->position.point.y, msg->velocity.x, msg->velocity.y);
    m_zmp_solver.set_com_height(msg->position.point.z);
}

void SolverNode::zmp_callback(geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    m_zmp_solver.set_current_zmp(msg->point.x, msg->point.y);
}

void SolverNode::desired_pos_callback(geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    // CHANGE THIS, NEED AN EXTRA TOPIC. THIS ONE IS CONNECTED TO THE FOOTSTEP PLANNER, NEED ONE FROM STATE ESTIMATION
    // OR SOMETHING FOR CURRENT FEET POSITIONS.
    m_desired_footsteps = msg;
    m_zmp_solver.set_candidate_footsteps(m_desired_footsteps);
    m_zmp_solver.set_reference_stepsize(m_zmp_solver.get_candidate_footsteps());
}

void SolverNode::feet_callback(geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    // m_zmp_solver.set_current_foot(msg->poses[1].position.x, msg->poses[1].position.y);
    // if ((m_zmp_solver.get_current_stance_foot() == -1)
    //     || (m_zmp_solver.get_current_stance_foot() == 1 && m_zmp_solver.get_m_current_shooting_node().data == 1)) {
    //     m_zmp_solver.set_previous_foot(msg->poses[1].position.x, msg->poses[1].position.y);
    // } else if ((m_zmp_solver.get_current_stance_foot() == 1)
    //     || (m_zmp_solver.get_current_stance_foot() == -1 && m_zmp_solver.get_m_current_shooting_node().data == 1)) {
    //     m_zmp_solver.set_previous_foot(msg->poses[0].position.x, msg->poses[0].position.y);
    // }

    // double m_desired_previous_foot_x;
    // double m_desired_previous_foot_y;

    // msg->poses[0] is the right foot
    // msg->poses[1] is the left foot
    if ((m_zmp_solver.get_current_stance_foot() == -1)
        || (m_zmp_solver.get_current_stance_foot() == 1
            && m_zmp_solver.get_m_current_shooting_node().data == 0)) { // if stance foot is the left foot
        // only change the desired previous footsteps when current shooting node is 1 and the footstep changes
        if (abs(m_desired_previous_foot_y - msg->poses[1].position.y) > 10e-2
            && m_zmp_solver.get_m_current_shooting_node().data == 1) {
            m_desired_previous_foot_x = msg->poses[1].position.x;
            m_desired_previous_foot_y = msg->poses[1].position.y;
        }
    }
    if (m_zmp_solver.get_current_stance_foot() == 1
        || (m_zmp_solver.get_current_stance_foot() == -1 && m_zmp_solver.get_m_current_shooting_node().data == 0)) {
        if (abs(m_desired_previous_foot_y - msg->poses[0].position.y) > 10e-2
            && m_zmp_solver.get_m_current_shooting_node().data == 1) {
            m_desired_previous_foot_x = msg->poses[0].position.x;
            m_desired_previous_foot_y = msg->poses[0].position.y;
        }
    }
    // RCLCPP_INFO(this->get_logger(), "desired x foot prev position %f",m_desired_previous_foot_x);
    m_zmp_solver.set_previous_foot(m_desired_previous_foot_x, m_desired_previous_foot_y);
}

void SolverNode::stance_foot_callback(std_msgs::msg::Int32::SharedPtr msg)
{
    m_zmp_solver.set_current_stance_foot(msg->data);
}

void SolverNode::right_foot_ground_callback(std_msgs::msg::Bool::SharedPtr msg)
{
    m_zmp_solver.set_right_foot_on_gound(msg->data);
}

void SolverNode::left_foot_ground_callback(std_msgs::msg::Bool::SharedPtr msg)
{
    m_zmp_solver.set_left_foot_on_gound(msg->data);
}

void SolverNode::timer_callback()
{
    if (!(m_desired_footsteps)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for input from footstep planner");
        // printf("prev des%i\n", m_prev_des_footsteps);
    } else {
        if (*m_desired_footsteps != m_prev_des_footsteps) {
            m_zmp_solver.reset_to_double_stance();
        }
        m_prev_des_footsteps = *m_desired_footsteps;
        m_zmp_solver.update_current_foot();
        m_zmp_solver.set_current_state();
        int solver_status = m_zmp_solver.solve_step();
        // bool weight_shift_done_node = m_zmp_solver.m_is_weight_shift_done;

        if (solver_status != 0) {
            RCLCPP_WARN(this->get_logger(), "Could not find a solution. exited with status %i", solver_status);
        } else {
            auto com_msg = geometry_msgs::msg::PoseArray();
            com_msg.header.stamp = this->get_clock()->now();
            com_msg.header.frame_id = "world";

            auto foot_msg = geometry_msgs::msg::PoseArray();
            foot_msg.header.stamp = this->get_clock()->now();
            foot_msg.header.frame_id = "world";

            geometry_msgs::msg::Pose pose_container;

            // This is all for visualization
            visualization_msgs::msg::Marker current_footsteps_marker;
            current_footsteps_marker.type = 8;
            current_footsteps_marker.header.frame_id = "world";
            current_footsteps_marker.id = 0;

            visualization_msgs::msg::Marker previous_footsteps_marker;
            previous_footsteps_marker.type = 8;
            previous_footsteps_marker.header.frame_id = "world";
            previous_footsteps_marker.id = 1;

            geometry_msgs::msg::Point marker_container;

            nav_msgs::msg::Path com_path;
            com_path.header.frame_id = "world";

            nav_msgs::msg::Path zmp_path;
            zmp_path.header.frame_id = "world";

            geometry_msgs::msg::PoseStamped com_path_wrapper;
            com_path_wrapper.header.frame_id = "world";

            geometry_msgs::msg::PoseStamped zmp_path_wrapper;
            zmp_path_wrapper.header.frame_id = "world";

            std::array<double, NX* ZMP_PENDULUM_ODE_N>* trajectory_pointer = m_zmp_solver.get_state_trajectory();

            for (int i = 0; i < (ZMP_PENDULUM_ODE_N); i++) {
                pose_container.position.x = (*trajectory_pointer)[(i * NX + 0)];
                pose_container.position.y = (*trajectory_pointer)[(i * NX + 3)];
                pose_container.position.z = m_zmp_solver.get_com_height();
                com_msg.poses.push_back(pose_container);
                com_path_wrapper.pose = pose_container;
                com_path.poses.push_back(com_path_wrapper);

                // Visualize the ZMP trajectory
                pose_container.position.x = (*trajectory_pointer)[(i * NX + 2)];
                pose_container.position.y = (*trajectory_pointer)[(i * NX + 5)];
                pose_container.position.z = m_zmp_solver.get_com_height();
                zmp_path_wrapper.pose = pose_container;
                zmp_path.poses.push_back(zmp_path_wrapper);
                // std::cout << "Pose Stamped:" << std::endl;
                // std::cout << "Position: x=" << com_path_wrapper.pose.position.x << ", y=" <<
                // com_path_wrapper.pose.position.y
                // << ", z=" << com_path_wrapper.pose.position.z << std::endl;

                // The feet
                pose_container.position.x = (*trajectory_pointer)[(i * NX + 6)];
                pose_container.position.y = (*trajectory_pointer)[(i * NX + 8)];
                foot_msg.poses.push_back(pose_container);
                if (i == ZMP_PENDULUM_ODE_N - 1) {
                    continue;
                }

                marker_container.x = (*trajectory_pointer)[(i * NX + 6)];
                marker_container.y = (*trajectory_pointer)[(i * NX + 8)];
                marker_container.z = 0.0;
                current_footsteps_marker.points.push_back(marker_container);

                marker_container.x = (*trajectory_pointer)[(i * NX + 7)];
                marker_container.y = (*trajectory_pointer)[(i * NX + 9)];
                marker_container.z = m_zmp_solver.get_com_height() / 2;
                previous_footsteps_marker.points.push_back(marker_container);
                // printf("marker itself point [%f,%f,%f]",
                // current_footsteps_marker.points[0].x,current_footsteps_marker.points[0].y,current_footsteps_marker.points[0].z);
            };

            current_footsteps_marker.action = 0;
            current_footsteps_marker.frame_locked = 1;
            current_footsteps_marker.scale.x = 0.1;
            current_footsteps_marker.scale.y = 0.1;
            current_footsteps_marker.scale.z = 0.1;
            current_footsteps_marker.pose.position.x = 0.0;
            current_footsteps_marker.pose.position.y = 0.0;
            current_footsteps_marker.pose.position.z = 0.0;
            current_footsteps_marker.pose.orientation.x = 0.0;
            current_footsteps_marker.pose.orientation.y = 0.0;
            current_footsteps_marker.pose.orientation.z = 0.0;
            current_footsteps_marker.pose.orientation.w = 1.0;
            current_footsteps_marker.ns = "current_footstep_visualization";
            current_footsteps_marker.lifetime.sec = 1;
            current_footsteps_marker.color.b = 1.0;
            current_footsteps_marker.color.a = 0.7;

            previous_footsteps_marker.action = 0;
            previous_footsteps_marker.frame_locked = 1;
            previous_footsteps_marker.scale.x = 0.1;
            previous_footsteps_marker.scale.y = 0.1;
            previous_footsteps_marker.scale.z = 0.1;
            previous_footsteps_marker.pose.position.x = 0.0;
            previous_footsteps_marker.pose.position.y = 0.0;
            previous_footsteps_marker.pose.position.z = 0.0;
            previous_footsteps_marker.pose.orientation.x = 0.0;
            previous_footsteps_marker.pose.orientation.y = 0.0;
            previous_footsteps_marker.pose.orientation.z = 0.0;
            previous_footsteps_marker.pose.orientation.w = 1.0;
            previous_footsteps_marker.ns = "previous_footstep_visualization";
            previous_footsteps_marker.lifetime.sec = 1;
            previous_footsteps_marker.color.r = 1.0;
            previous_footsteps_marker.color.a = 1.0;

            m__footstep_visualizer_publisher->publish(previous_footsteps_marker);
            m__footstep_visualizer_publisher->publish(current_footsteps_marker);
            m_com_visualizer_publisher->publish(com_path);
            m_zmp_visualizer_publisher->publish(zmp_path);
            m_com_trajectory_publisher->publish(com_msg);

            if (abs(foot_msg.poses[0].position.y - m_prev_foot_msg.poses[0].position.y)
                > 10e-2) { // only publish when foot is changing
                m_final_feet_publisher->publish(foot_msg);
                m_prev_foot_msg = foot_msg;
            }
            m_current_shooting_node_publisher->publish(m_zmp_solver.get_m_current_shooting_node());
            m_zmp_solver.update_current_shooting_node();
        }
    }
}

// void SolverNode::visualize_trajectory()
// {
//     visualization_msgs::msg::Marker com_marker;
//     com_marker.type = 4;
//     com_marker.header.frame_id = "world";
//     com_marker.id = 0;
//     geometry_msgs::msg::Point com_marker_point;
//     std::vector<double> m_real_time_com_trajectory_x = m_zmp_solver.get_real_time_com_trajectory_x();
//     std::vector<double> m_real_time_com_trajectory_y = m_zmp_solver.get_real_time_com_trajectory_y();
//     for (int ii = 0; ii < m_real_time_com_trajectory_x.size();ii++) {
//         com_marker_point.x = m_real_time_com_trajectory_x[ii];
//         com_marker_point.y = m_real_time_com_trajectory_y[ii];
//         com_marker_point.z = m_zmp_solver.get_com_height();
//         com_marker.points.push_back(com_marker_point);
//         RCLCPP_INFO(rclcpp::get_logger("CoM Marker"), "CoM Marker:[%f,%f,%f]", com_marker_point.x,
//         com_marker_point.y, com_marker_point.z);
//     }
//     RCLCPP_INFO(rclcpp::get_logger("CoM Marker:"), "Published %i markers", com_marker.points.size());
//     com_marker.action = 0;
//     com_marker.frame_locked = 1;
//     com_marker.scale.x = 3.0;
//     com_marker.scale.y = 1.0;
//     com_marker.scale.z = 1.0;
//     com_marker.pose.position.x = 0.0;
//     com_marker.pose.position.y = 0.0;
//     com_marker.pose.position.z = 0.0;
//     com_marker.pose.orientation.x = 0.0;
//     com_marker.pose.orientation.y = 0.0;
//     com_marker.pose.orientation.z = 0.0;
//     com_marker.pose.orientation.w = 1.0;
//     com_marker.ns = "MPC_CoM_trajectory_visualization";
//     com_marker.lifetime.sec = 1;
//     com_marker.color.a = 1.0;
//     m_rviz_com_publisher->publish(com_marker);
// }

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SolverNode>());
    rclcpp::shutdown();
    return 0;
}
