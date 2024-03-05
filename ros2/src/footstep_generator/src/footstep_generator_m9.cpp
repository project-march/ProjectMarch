#include "footstep_generator/footstep_generator_m9.hpp"
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

FootstepGenerator::FootstepGenerator()
    : Node("footstep_generator_node")
    , m_steps(20) // Change this so we can interactively edit the amount of footsteps while Koengaiting
    , m_velocity_x()
    , m_velocity_y()
    , m_step_length()
    , m_current_left_foot()
    , m_current_right_foot()
{
    m_request_footsteps_service = this->create_service<march_shared_msgs::srv::RequestFootsteps>(
        "footstep_generator", std::bind(&FootstepGenerator::publishFootPlacements, this, _1, _2));
    m_footstep_publisher = this->create_publisher<geometry_msgs::msg::PoseArray>("/desired_footsteps", 10);
    m_exo_joint_state_subscriber = this->create_subscription<march_shared_msgs::msg::StateEstimation>(
        "state_estimation/state", 100, std::bind(&FootstepGenerator::currentExoJointStateCallback, this, _1)); 

    // At some point, use parameters to interactively change gait when gaiting 
    m_steps = 3; 
    m_step_length = 0.4; // meters
}

void FootstepGenerator::currentExoJointStateCallback(const march_shared_msgs::msg::StateEstimation::SharedPtr msg){
    m_current_left_foot = msg->inertial_foot_position[0]; 
    m_current_right_foot = msg->inertial_foot_position[1]; 
}

void FootstepGenerator::publishFootPlacements(
    const std::shared_ptr<march_shared_msgs::srv::RequestFootsteps::Request> request,
    std::shared_ptr<march_shared_msgs::srv::RequestFootsteps::Response> response)
{
    if (request->gait_type == 10){
        auto footsteps = generateFootPlacements(request->gait_type); 
        publishFootsteps(footsteps); 
        RCLCPP_INFO(this->get_logger(), "Footsteps published to MPC!"); 
    }
    response->status = true; 
}

geometry_msgs::msg::PoseArray FootstepGenerator::generateFootPlacements(int gait_type)
{
    geometry_msgs::msg::PoseArray footstep_array;
    geometry_msgs::msg::Pose footstep;

    footstep_array.header.stamp = this->now();
    footstep_array.header.frame_id = "world";

    switch (gait_type) {
        case 10: // VariableWalk  
        // publish 3x a footstep with the right foot 
        for (auto i = 0; i < m_steps; ++i){
            footstep.position.x = m_current_right_foot.position.x + (i+1)*m_step_length; 
            footstep.position.y = m_current_right_foot.position.y; 
            footstep.position.z = m_current_right_foot.position.z; 
            footstep_array.poses.push_back(footstep); 
        }
            break;

        case 1: // Step Close 
            break;
    }

    return footstep_array;
}

void FootstepGenerator::publishFootsteps(geometry_msgs::msg::PoseArray footsteps)
{
    m_footstep_publisher->publish(footsteps);
}

int FootstepGenerator::getSteps()
{
    return m_steps;
}

double FootstepGenerator::getVelocityX()
{
    return m_velocity_x;
}

double FootstepGenerator::getVelocityY()
{
    return m_velocity_y;
}

double FootstepGenerator::getStepLength()
{
    return m_step_length;
}

