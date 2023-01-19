#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "march_shared_msgs/srv/request_footsteps.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <cstdio>
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class FootstepGenerator : public rclcpp::Node {
public:
    FootstepGenerator()
        : Node("footstep_generator_node")
    {
        m_service = this->create_service<march_shared_msgs::srv::RequestFootsteps>(
            "footstep_generator", std::bind(&FootstepGenerator::publish_foot_placements, this, _1, _2));
        m_publisher = this->create_publisher<geometry_msgs::msg::PoseArray>("footsteps", 10);
    };

private:
    void publish_foot_placements(const std::shared_ptr<march_shared_msgs::srv::RequestFootsteps::Request> request,
        std::shared_ptr<march_shared_msgs::srv::RequestFootsteps::Response> response)
    {
        auto footsteps = generate_foot_placements(request->stance_leg);
        publish_footsteps(footsteps);
        response->status = 0;
    };

    geometry_msgs::msg::PoseArray generate_foot_placements(int stance_leg)
    {
        geometry_msgs::msg::PoseArray footstep_array;
        int steps = 8;
        double vx = 1.0;
        double vy = 1.0;
        double l = 0.3;
        double x = 0.0;
        double y = stance_leg * l / 2;

        geometry_msgs::msg::Pose footstep;
        for (int i = 0; i < steps; i++) {
            x += vx * i;
            y += vy * i - stance_leg * l;
            stance_leg = -stance_leg;

            footstep.position.x = x;
            footstep.position.y = y;
            footstep.position.z = 0;

            footstep_array.poses.push_back(footstep);
        }
        return footstep_array;
    };

    void publish_footsteps(const geometry_msgs::msg::PoseArray footsteps)
    {
        m_publisher->publish(footsteps);
    };

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr m_publisher;
    rclcpp::Service<march_shared_msgs::srv::RequestFootsteps>::SharedPtr m_service;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FootstepGenerator>());

    rclcpp::shutdown();
    return 0;
}
