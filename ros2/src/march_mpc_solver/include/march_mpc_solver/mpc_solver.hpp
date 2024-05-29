/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#ifndef MARCH_MPC_SOLVER__MPC_SOLVER_HPP_
#define MARCH_MPC_SOLVER__MPC_SOLVER_HPP_

#include "geometry_msgs/msg/pose_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "march_mpc_solver/c_generated_code/main_ZMP_pendulum_ode.cpp"
#include <array>
#include <chrono>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>

enum Gait_type { startup, continuous, stopping };

class MpcSolver {
public:
    MpcSolver();
    ~MpcSolver() = default;

    int solve_step();
    void initialize_mpc_params();
    void reset_to_double_stance();
    bool check_zmp_on_foot() const;
    void set_candidate_footsteps(const geometry_msgs::msg::PoseArray::SharedPtr footsteps);
    void set_reference_stepsize(const std::vector<geometry_msgs::msg::Point>& candidate_footsteps);
    void update_current_foot();
    inline void update_current_shooting_node() { m_current_shooting_node++; }

    inline std::array<double, NX> get_state() const { return m_x_current; }
    inline const std::array<double, NX * ZMP_PENDULUM_ODE_N>* get_state_trajectory() const { return &m_x_trajectory; }
    inline std::array<double, NU * ZMP_PENDULUM_ODE_N> get_input_current() const { return m_u_current; }
    inline double get_com_height() const { return m_com_height; }
    inline int get_current_stance_foot() const { return m_current_stance_foot; }
    inline const std::vector<geometry_msgs::msg::Point>& get_candidate_footsteps() const { return m_candidate_footsteps; }
    inline std::vector<double> get_real_time_com_trajectory_x() const { return m_real_time_com_trajectory_x; }
    inline std::vector<double> get_real_time_com_trajectory_y() const { return m_real_time_com_trajectory_y; }
    inline std_msgs::msg::Int32 get_current_shooting_node() const {
        std_msgs::msg::Int32 current_shooting_node;
        current_shooting_node.data = m_current_shooting_node;
        return current_shooting_node;
    }

    inline void set_current_shooting_node(int current_shooting_node) { m_current_shooting_node = current_shooting_node; }
    inline void set_foot_positions(const geometry_msgs::msg::PoseArray& foot_positions) { m_foot_positions = foot_positions; }
    inline void set_current_stance_leg(uint8_t current_stance_leg) { m_current_stance_leg = current_stance_leg; }
    inline void set_next_stance_leg(uint8_t next_stance_leg) { m_next_stance_leg = next_stance_leg; }
    inline void set_current_com(const double& x, const double& y, const double& x_dot, const double& y_dot) {
        m_com_current[0] = x;
        m_com_current[1] = y;
        m_com_vel_current[0] = x_dot;
        m_com_vel_current[1] = y_dot;
    }
    inline void set_com_height(const double& com_height) { m_com_height = com_height; }
    inline void set_current_zmp(const double& x, const double& y) {
        m_zmp_current[0] = x;
        m_zmp_current[1] = y;
    }
    inline void set_current_stance_foot(int current_stance_foot) { m_current_stance_foot = current_stance_foot; }
    inline void set_current_foot(const double& x, const double& y) {
        m_pos_foot_current[0] = x;
        m_pos_foot_current[1] = y;
    }
    inline void set_previous_foot(const double& x, const double& y) {
        m_pos_foot_prev[0] = x;
        m_pos_foot_prev[1] = y;
    }
    inline void set_current_state() {
        m_x_current[0] = m_com_current[0]; // - 0.11; when using real state estimator
        m_x_current[1] = m_com_vel_current[0];

        m_x_current[2] = m_zmp_current[0];

        m_x_current[3] = m_com_current[1];
        m_x_current[4] = m_com_vel_current[1];

        m_x_current[5] = m_zmp_current[1];

        m_x_current[6] = m_pos_foot_current[0];
        m_x_current[7] = m_pos_foot_prev[0];

        m_x_current[8] = m_pos_foot_current[1];
        m_x_current[9] = m_pos_foot_prev[1];

        m_x_current[10] = 0;
        m_x_current[11] = 0;
    }

private:
    int solve_zmp_mpc(std::array<double, NX>&, std::array<double, NU * ZMP_PENDULUM_ODE_N>&);

    std::array<double, NX> m_x_current;
    std::array<double, NX * ZMP_PENDULUM_ODE_N> m_x_trajectory;
    std::array<double, NU * ZMP_PENDULUM_ODE_N> m_u_current;
    std::array<double, 2> m_pos_foot_current;
    std::array<double, 2> m_pos_foot_prev;
    std::array<double, 2> m_zmp_current;
    std::array<double, 2> m_com_current;
    std::array<double, 2> m_com_vel_current;

    std::vector<geometry_msgs::msg::Point> m_candidate_footsteps;
    std::vector<double> m_reference_stepsize_x;
    std::vector<double> m_reference_stepsize_y;
    std::vector<double> m_real_time_com_trajectory_x;
    std::vector<double> m_real_time_com_trajectory_y;

    double m_time_horizon;

    // Constraints for the ZMP MPC
    int m_number_of_footsteps;

    double m_switch;
    int m_current_shooting_node;
    double m_timing_value;

    int m_current_count;
    int m_current_stance_foot;
    int m_previous_stance_foot;
    int m_step_counter;

    double m_admissible_region_x;
    double m_admissible_region_y;
    double m_foot_width_x;
    double m_foot_width_y;
    double m_step_size_x;
    double m_step_size_y;
    double m_com_height;
    const double m_gravity_const;
    double m_first_admissible_region_y;

    geometry_msgs::msg::PoseArray m_foot_positions;
    uint8_t m_current_stance_leg;
    uint8_t m_next_stance_leg;
};

#endif  // MARCH_MPC_SOLVER__MPC_SOLVER_HPP_