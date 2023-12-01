#include "rclcpp/rclcpp.hpp"
#include "example_interface/srv/jacobian.hpp"

#include <memory>
#include <vector>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

void generate_jacobian(const std::shared_ptr<example_interface::srv::Jacobian::Request> request,
        std::shared_ptr<example_interface::srv::Jacobian::Response> response)
{
    // Eigen::MatrixXd jacobian = Eigen::MatrixXd::Random(request->m, request->n);
    Eigen::MatrixXd jacobian = Eigen::MatrixXd(request->m, request->n);
    int counter = 0;
    for (int i = 0; i < request->m; i++)
    {
        for (int j = 0; j < request->n; j++)
        {
            jacobian(i, j) = counter;
            counter++;
        }
    }
    // jacobian.resize(1, jacobian.size());
    Eigen::VectorXd jacobian_vector = Eigen::Map<Eigen::VectorXd>(jacobian.data(), jacobian.size());
    for (int i = 0; i < jacobian_vector.size(); i++)
    {
        response->jacobis.push_back(jacobian_vector(i));
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\nm: %d\nn: %d", request->m, request->n);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending back response: [%d]", response.get()->jacobis.size());
    for (int i = 0; i < response->jacobis.size(); i++)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "jacobian[%d]: %f", i, response->jacobis[i]);
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("jacobian_server");
    rclcpp::Service<example_interface::srv::Jacobian>::SharedPtr service =
        node->create_service<example_interface::srv::Jacobian>("jacobian", &generate_jacobian);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to generate jacobian.");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}