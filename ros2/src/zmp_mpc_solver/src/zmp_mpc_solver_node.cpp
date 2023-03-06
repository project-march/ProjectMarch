// standard
#include "zmp_mpc_solver/zmp_mpc_solver.hpp"

int main(int argc, char** argv)
{
    printf("hello world acados_solver package\n");
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SolverNode>());
    rclcpp::shutdown();
    return 0;
}
