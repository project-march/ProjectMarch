// standard
#include "c_generated_code/main_pendulum_ode.cpp"
#include <chrono>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>

#include "mujoco_interfaces/msg/mujoco_data_state.hpp"
#include "mujoco_interfaces/msg/mujoco_set_control.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
class MpcSolver : public rclcpp::Node {
public:
  MpcSolver()
      : Node("mpc_solver_node"), x_current({0, 3.14, 0, 0}),
        m_time_horizon(3.0), published(0) {
    m_timer = this->create_wall_timer(
        1000ms, std::bind(&MpcSolver::timer_callback, this));
    m_publisher =
        this->create_publisher<mujoco_interfaces::msg::MujocoSetControl>(
            "writer_input", 10);
    m_subscriber =
        this->create_subscription<mujoco_interfaces::msg::MujocoDataState>(
            "exo_state_for_mpc", 10,
            std::bind(&MpcSolver::subscriber_callback, this, _1));
  };
  double x_current[4];
  double u_current[NU * PENDULUM_ODE_N];
  double m_time_horizon;
  int published;

private:
  int solve_step(double (&x_cur)[4], double (&u_cur)[NU * PENDULUM_ODE_N]) {
    return solve_mpc(x_cur, u_cur);
  };
  void timer_callback(){
      // int status = solve_step(x_current, u_current);
      // std::cout<<x_current[0]<<','<<x_current[1]<<','<<x_current[2]<<','<<x_current[3]<<std::endl;
      // std::cout<<u_current[0]<<std::endl;
      // std::cout<<u_current[2]<<std::endl;
      // std::cout<<"---"<<std::endl;
      // if (status==0)
      //     {
      //     publish_control_msg();
      //     }
  };

  void
  subscriber_callback(mujoco_interfaces::msg::MujocoDataState::SharedPtr msg) {
    x_current[0] = msg->qpos[0]; // set x position
    x_current[1] = msg->qpos[1]; // set theta position
    x_current[2] = msg->qvel[0]; // set x' position
    x_current[3] = msg->qvel[1]; // set theta' position
    // RCLCPP_INFO(this->get_logger(), "mpc solver X pos: %g", x_current[0]);
    // RCLCPP_INFO(this->get_logger(), "mpc solver Theta pos: %g",
    // x_current[1]); RCLCPP_INFO(this->get_logger(), "mpc solver dX pos: %g",
    // x_current[2]); RCLCPP_INFO(this->get_logger(), "mpc solver dTheta pos:
    // %g", x_current[3]);
    int status = solve_step(x_current, u_current); // solve the mpc problem
    // RCLCPP_INFO(this->get_logger(), "mpc solver control output: %g",
    // u_current[0]); RCLCPP_INFO(this->get_logger(), "mpc solver status: %d",
    // status);
    if (status == 0 && published == 0) {
      publish_control_msg();
      published = 0;
    }
  };

  void publish_control_msg() {
    auto message = mujoco_interfaces::msg::MujocoSetControl();
    message.stamp = this->get_clock()->now();
    for (const double &control_input : u_current) {
      std::cout << control_input << std::endl;
      message.reference_control.push_back(control_input);
    }
    message.mode = 1;
    message.control_inputs = 1;
    message.dt = (m_time_horizon / PENDULUM_ODE_N);
    m_publisher->publish(message);
  };
  rclcpp::TimerBase::SharedPtr m_timer;
  rclcpp::Publisher<mujoco_interfaces::msg::MujocoSetControl>::SharedPtr
      m_publisher;
  rclcpp::Subscription<mujoco_interfaces::msg::MujocoDataState>::SharedPtr
      m_subscriber;
};

int main(int argc, char **argv) {

  printf("hello world acados_solver package\n");
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MpcSolver>());
  rclcpp::shutdown();
  return 0;
}
