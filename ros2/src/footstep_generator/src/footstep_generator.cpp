#include "footstep_generator/footstep_generator.hpp"
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

FootstepGenerator::FootstepGenerator()
    : Node("footstep_generator_node")
    , m_steps(20) // Change this so we can interactively edit the amount of footsteps while Koengaiting
    , m_vx(0.15)
    , m_vy(0.0)
    , m_l(0.33)
{
    m_service = this->create_service<march_shared_msgs::srv::RequestFootsteps>(
        "footstep_generator", std::bind(&FootstepGenerator::publish_foot_placements, this, _1, _2));
    m_publisher = this->create_publisher<geometry_msgs::msg::PoseArray>("/desired_footsteps", 10);
    m_swing_trajectory_command_publisher
        = this->create_publisher<std_msgs::msg::Int32>("/publish_swing_leg_command", 10);
    declare_parameter("n_footsteps", 20);
    declare_parameter("step_length", 0.1);
    m_steps = this->get_parameter("n_footsteps").as_int();
    m_vx = this->get_parameter("step_length").as_double();
}

void FootstepGenerator::publish_foot_placements(
    const std::shared_ptr<march_shared_msgs::srv::RequestFootsteps::Request> request,
    std::shared_ptr<march_shared_msgs::srv::RequestFootsteps::Response> response)
{
    m_steps = this->get_parameter("n_footsteps").as_int();
    m_vx = this->get_parameter("step_length").as_double();
    if (request->gait_type != 0) {
        auto footsteps = generate_foot_placements(request->stance_leg, request->gait_type);
        // ADD THE EMPTY REQUEST HERE
        if (request->gait_type == 3) {
            std_msgs::msg::Int32 msg;
            msg.data = 0;
            m_swing_trajectory_command_publisher->publish(msg);
        }
        m_publisher->publish(footsteps);
        response->status = true;
    }
}

geometry_msgs::msg::PoseArray FootstepGenerator::generate_foot_placements(int stance_leg, int gait_type)
{
    geometry_msgs::msg::PoseArray footstep_array;
    geometry_msgs::msg::Pose footstep;
    footstep_array.header.frame_id = "mpc_frame";
    double x = 0.0;
    // Our frame is from the right foot, where y=0;
    // 1 is right leg
    // -1 is left leg
    stance_leg = -1;
    double y = m_l / 2 - stance_leg * m_l / 2;
    // STAND = 1
    // WALK = 2
    // STEP_CLOSE = 3

    switch (gait_type) {
        case 1:
            y = 0;
            for (int i = 0; i < m_steps * 5; i++) {
                x += 0;
                y = (1 - ((y > 0) - (y < 0))) * m_l;
                footstep.position.x = x;
                footstep.position.y = y;
                footstep.position.z = 0;

                footstep_array.poses.push_back(footstep);
            }
            break;

        case 2: // CHANGE SO THAT FULL FOOTSTEP PLAN GETS SENT, STARTING WITH THE CURRENT STANCE FOOTSTEP POINT
            x = 0.0;
            y = y;
            footstep.position.x = x;
            footstep.position.y = y;
            footstep.position.z = 0;
            footstep_array.poses.push_back(footstep);

            x = 0.0;
            y += m_vy * 1.0 + stance_leg * m_l;
            footstep.position.x = x;
            footstep.position.y = y;
            footstep.position.z = 0;
            footstep_array.poses.push_back(footstep);
            stance_leg = -stance_leg;

            for (int i = 2; i < m_steps; i++) {
                x += m_vx * 1.0;
                y += m_vy * 1.0 + stance_leg * m_l;
                stance_leg = -stance_leg;

                footstep.position.x = x;
                footstep.position.y = y;
                footstep.position.z = 0;

                footstep_array.poses.push_back(footstep);
                // printf("stance_leg %i\n", stance_leg);
            }
            break;

        case 3:
            // for (int i = 0; i < 1; i++) {
            //     x += m_vx * 1.0;
            //     y += m_vy * 1.0 - stance_leg * m_l;
            //     stance_leg = -stance_leg;

            //     footstep.position.x = x;
            //     footstep.position.y = y;
            //     footstep.position.z = 0;

            //     footstep_array.poses.push_back(footstep);
            // }

            // starting step
            x = 0.0;
            y = y;
            footstep.position.x = x;
            footstep.position.y = y;
            footstep.position.z = 0;
            footstep_array.poses.push_back(footstep);

            x = 0.0;
            y += m_vy * 1.0 + stance_leg * m_l;
            footstep.position.x = x;
            footstep.position.y = y;
            footstep.position.z = 0;
            footstep_array.poses.push_back(footstep);
            stance_leg = -stance_leg;
            // Then, add the closing steps that stay on 0

            for (int i = 2; i < m_steps * 5; i++) {
                x = m_vx * 1.0;
                y += m_vy * 1.0 + stance_leg * m_l;
                stance_leg = -stance_leg;

                footstep.position.x = x;
                footstep.position.y = y;
                footstep.position.z = 0;

                footstep_array.poses.push_back(footstep);
                // printf("stance_leg %i\n", stance_leg);
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