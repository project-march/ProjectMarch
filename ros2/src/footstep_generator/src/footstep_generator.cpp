#include "footstep_generator/footstep_generator.hpp"
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

FootstepGenerator::FootstepGenerator()
    : Node("footstep_generator_node")
    , m_steps(8)
    , m_vx(1.0)
    , m_vy(0.0)
    , m_l(0.3)
{
    m_service = this->create_service<march_shared_msgs::srv::RequestFootsteps>(
        "footstep_generator", std::bind(&FootstepGenerator::publish_foot_placements, this, _1, _2));
    m_publisher = this->create_publisher<geometry_msgs::msg::PoseArray>("footsteps", 10);
    m_subscriber_joints = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_state", 10, std::bind(&FootstepGenerator::joint_state_callback, this, _1));
    m_subscriber_imu = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu_state", 10, std::bind(&FootstepGenerator::imu_state_callback, this, _1));

void FootstepGenerator::publish_foot_placements(
    const std::shared_ptr<march_shared_msgs::srv::RequestFootsteps::Request> request,
    std::shared_ptr<march_shared_msgs::srv::RequestFootsteps::Response> response)
{
    auto footsteps = generate_foot_placements(request->stance_leg, request->gait_type);
    publish_footsteps(footsteps);
    response->status = 0;
}

void FootstepGenerator::joint_state_callback(sensor_msgs::msg::JointState::SharedPtr msg){
    //TODO: implement
}

void FootstepGenerator::imu_state_callback(sensor_msgs::msg::Imu::SharedPtr msg){
    //TODO: implement
}

geometry_msgs::msg::PoseArray FootstepGenerator::generate_foot_placements(int stance_leg, int gait_type)
{
    geometry_msgs::msg::PoseArray footstep_array;
    geometry_msgs::msg::Pose footstep;
    double x = 0.0;
    // Our frame is from the right foot, where y=0;
    // 1 is right leg
    // -1 is left leg
    double y = m_l / 2 - stance_leg * m_l / 2;
    // int8 STAND = 0
    // int8 STEP = 1
    // int8 STEP_CLOSE = 2
    // int8 WALK = 3
    switch (gait_type) {
        case 0:

            for (int i = 0; i < 1; i++) {
                x += 0;
                y += m_vy * 1.0 - stance_leg * m_l;
                stance_leg = -stance_leg;

                footstep.position.x = x;
                footstep.position.y = y;
                footstep.position.z = 0;

                footstep_array.poses.push_back(footstep);
            }
            break;

        case 1:
            for (int i = 0; i < 1; i++) {
                x += m_vx * 1.0;
                y += m_vy * 1.0 - stance_leg * m_l;
                stance_leg = -stance_leg;

                footstep.position.x = x;
                footstep.position.y = y;
                footstep.position.z = 0;

                footstep_array.poses.push_back(footstep);
            }
            break;

        case 2:
            for (int i = 0; i < m_steps; i++) {
                x += m_vx * 1.0;
                y += m_vy * 1.0 - stance_leg * m_l;
                stance_leg = -stance_leg;

                footstep.position.x = x;
                footstep.position.y = y;
                footstep.position.z = 0;

                footstep_array.poses.push_back(footstep);
            }
            // Then, add the closing step
            y += m_vy * 1.0 - stance_leg * m_l;
            stance_leg = -stance_leg;

            footstep.position.x = x;
            footstep.position.y = y;
            footstep.position.z = 0;

            footstep_array.poses.push_back(footstep);
            break;

        case 3:
            for (int i = 0; i < m_steps; i++) {
                x += m_vx * 1.0;
                y += m_vy * 1.0 - stance_leg * m_l;
                stance_leg = -stance_leg;

                footstep.position.x = x;
                footstep.position.y = y;
                footstep.position.z = 0;

                footstep_array.poses.push_back(footstep);
            }
            break;
    }

    return footstep_array;
}

void FootstepGenerator::publish_footsteps(geometry_msgs::msg::PoseArray footsteps)
{
    m_publisher->publish(footsteps);
}

int FootstepGenerator::get_steps()
{
    return m_steps;
}

double FootstepGenerator::get_velocity_x()
{
    return m_vx;
}

double FootstepGenerator::get_velocity_y()
{
    return m_vy;
}

double FootstepGenerator::get_feet_spread()
{
    return m_l;
}