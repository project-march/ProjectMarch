#include "footstep_generator/footstep_generator.hpp"
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

/**
 * The footstep generator takes the walking commands from the state machine, and calculated the footsteps that the
 * zmp-mpc will use. For now it just bases the steps on the desired step size, number of steps, and the current feet
 * positions.
 *
 * The requests from the state-machine come in on the footstep_generator service.
 * THe generated footsteps are published on the /desired_footstep topic.
 */
FootstepGenerator::FootstepGenerator()
    : Node("footstep_generator_node")
    , m_steps(20) // Change this so we can interactively edit the amount of footsteps while Koengaiting
    , m_vx(0.2)
    , m_vy(0.0)
    , m_l(0.33)
{
    m_service = this->create_service<march_shared_msgs::srv::RequestFootsteps>(
        "footstep_generator", std::bind(&FootstepGenerator::publish_foot_placements, this, _1, _2));
    m_publisher = this->create_publisher<geometry_msgs::msg::PoseArray>("/desired_footsteps", 10);
    m_swing_trajectory_command_publisher
        = this->create_publisher<std_msgs::msg::Int32>("/publish_swing_leg_command", 10);
    declare_parameter("n_footsteps", 20);
    declare_parameter("step_length", 0.2);
    m_steps = this->get_parameter("n_footsteps").as_int();
    m_vx = this->get_parameter("step_length").as_double();
}

/**
 * This method processes the service request from the state-machine.
 * The method also publishes the generated footsteps.
 *
 * @param request Request from the state-machine, with either stand, walk or step-and-close
 * @param response If the request was processed successfully.
 */
void FootstepGenerator::publish_foot_placements(
    const std::shared_ptr<march_shared_msgs::srv::RequestFootsteps::Request> request,
    std::shared_ptr<march_shared_msgs::srv::RequestFootsteps::Response> response)
{
    m_steps = this->get_parameter("n_footsteps").as_int();
    m_vx = this->get_parameter("step_length").as_double();
    if (request->gait_type != 1) {
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

/**
 * This method generates the desired footsteps for the different possible stances.
 *
 * @param stance_leg Our frame is from the right foot, where y=0. 1 is right leg, -1 is left leg.
 * @param gait_type The gait for which we need footsteps, 1 is STAND, 2 is WALK, 3 = STEP_CLOSE
 * @return Array with generated footsteps, empty when the request is invalid.
 */
geometry_msgs::msg::PoseArray FootstepGenerator::generate_foot_placements(int stance_leg, int gait_type)
{
    geometry_msgs::msg::PoseArray footstep_array;
    geometry_msgs::msg::Pose footstep;
    footstep_array.header.frame_id = "mpc_frame";
    double x = 0.0;

    stance_leg = -1;
    double y = m_l / 2 - stance_leg * m_l / 2;
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
        default:
            RCLCPP_ERROR(this->get_logger(), "UNABLE TO GENERATE FOOTSTEPS FOR STANCE LEG %d, AND GAIT TYPE %d",
                stance_leg, gait_type);
            RCLCPP_ERROR(this->get_logger(), "RETURNED EMPTY FOOTSTEP ARRAY");
    }

    return footstep_array;
}

/**
 *
 * @return amount of steps that will be generated for walk
 */
int FootstepGenerator::get_steps()
{
    return m_steps;
}

/**
 *
 * @return velocity of the steps in the x direction
 */
double FootstepGenerator::get_velocity_x()
{
    return m_vx;
}

/**
 *
 * @return velocity of the steps in the y direction
 */
double FootstepGenerator::get_velocity_y()
{
    return m_vy;
}

/**
 *
 * @return the length that the feet are apart
 */
double FootstepGenerator::get_feet_spread()
{
    return m_l;
}