#include "weight_shift_buffer/weight_shift_buffer.hpp"
#include <chrono>
#include <cstdio>
#include <string>

WeightShiftBuffer::WeightShiftBuffer()
{
    m_duration_weight_shift = 0.5;
    m_duration_step = 3.0; // parameter linken aan recon2 parameter
    m_duration_reset_haa = 0.5;
    swing_leg = "R";
    m_hip_aa_position = 0.1;
}

void WeightShiftBuffer::set_incoming_joint_trajectory(trajectory_msgs::msg::JointTrajectory msg)
{
    m_incoming_joint_trajectory = msg;
    m_final_joint_trajectory.points.clear();
    m_final_joint_trajectory.header = msg.header;
    m_final_joint_trajectory.joint_names = msg.joint_names;
}

// change the HAA's during the swing to the desired hip_aa_position
void WeightShiftBuffer::update_HAA_during_step()
{
    if (swing_leg == "R") {
        for (int i = 0; i < m_incoming_joint_trajectory.points.size(); i++) {
            m_incoming_joint_trajectory.points[i].positions[1] = m_hip_aa_position; // 1&5 are the HAA joints
            m_incoming_joint_trajectory.points[i].positions[5] = -m_hip_aa_position;
        }
    } else if (swing_leg == "L") {
        for (int i = 0; i < m_incoming_joint_trajectory.points.size(); i++) {
            m_incoming_joint_trajectory.points[i].positions[1] = -m_hip_aa_position;
            m_incoming_joint_trajectory.points[i].positions[5] = m_hip_aa_position;
        }
    }
}

// Create the weight shift points at the beginning of the step.
void WeightShiftBuffer::add_weight_shift()
{
    trajectory_msgs::msg::JointTrajectoryPoint point_to_add = m_first_traj_point;
    m_final_joint_trajectory.points.push_back(point_to_add);

    if (swing_leg == "R") {
        for (int i = 0; i < m_duration_weight_shift * 10; i++) {
            point_to_add.positions[1]
                += (m_hip_aa_position - m_first_traj_point.positions[1]) / (m_duration_weight_shift * 10);
            point_to_add.positions[5]
                += (-m_hip_aa_position - m_first_traj_point.positions[5]) / (m_duration_weight_shift * 10);

            for (int i = 0; i < 8; i++) {
                point_to_add.velocities[i] = 0.0;
            }

            m_final_joint_trajectory.points.push_back(point_to_add);
            swing_leg = "L";
        }
    } else if (swing_leg == "L") {
        for (int i = 0; i < m_duration_weight_shift * 10; i++) {
            point_to_add.positions[1]
                += (-m_hip_aa_position - m_first_traj_point.positions[1]) / (m_duration_weight_shift * 10);
            point_to_add.positions[5]
                += (m_hip_aa_position - m_first_traj_point.positions[5]) / (m_duration_weight_shift * 10);

            for (int i = 0; i < 8; i++) {
                point_to_add.velocities[i] = 0.0;
            }

            m_final_joint_trajectory.points.push_back(point_to_add);
            swing_leg = "R";
        }
    }
    m_final_joint_trajectory.points.insert(m_final_joint_trajectory.points.end(),
        m_incoming_joint_trajectory.points.begin(), m_incoming_joint_trajectory.points.end());
}

// reset HAA's to homestand position at end of step
void WeightShiftBuffer::reset_HAA_at_end()
{
    trajectory_msgs::msg::JointTrajectoryPoint last_point_of_step
        = m_final_joint_trajectory.points[(m_final_joint_trajectory.points.size() - 1)];
    trajectory_msgs::msg::JointTrajectoryPoint point_to_add = last_point_of_step;

    for (int i = 0; i < m_duration_reset_haa * 10; i++) {
        //    trajectory_msgs::msg::JointTrajectoryPoint point_to_add = m_final_joint_trajectory.points[-1];
        point_to_add.positions[1]
            += (m_first_traj_point.positions[1] - last_point_of_step.positions[1]) / (m_duration_reset_haa * 10);
        point_to_add.positions[5]
            += (m_first_traj_point.positions[5] - last_point_of_step.positions[5]) / (m_duration_reset_haa * 10);
        m_final_joint_trajectory.points.push_back(point_to_add);
    }
}

void WeightShiftBuffer::fix_timings_traj()
{
    // double total_duration_traj = m_duration_weight_shift + m_duration_step + m_duration_reset_haa;
    // double duration_one_point = total_duration_traj/(m_final_joint_trajectory.points.size()-1);
    // for (int i =0; i<m_final_joint_trajectory.points.size(); i++){
    //     m_final_joint_trajectory.points[i].time_from_start.sec = i * duration_one_point;
    //     m_final_joint_trajectory.points[i].time_from_start.nanosec = (i * duration_one_point - floorf(i *
    //     duration_one_point)) * 1e8;
    //     // RCLCPP_INFO(rclcpp::get_logger("time"), "%i", m_final_joint_trajectory.points[i].time_from_start.sec);
    //     // RCLCPP_INFO(rclcpp::get_logger("time nano"), "%i",
    //     m_final_joint_trajectory.points[i].time_from_start.nanosec);
    // }

    double total_duration_traj = m_duration_weight_shift + m_duration_step + m_duration_reset_haa;
    double duration_one_point = total_duration_traj / (m_final_joint_trajectory.points.size() - 1);
    for (int i = 0; i < m_duration_weight_shift * 10; i++) {
        m_final_joint_trajectory.points[i].time_from_start.sec = (int)floor(i * duration_one_point);
        m_final_joint_trajectory.points[i].time_from_start.nanosec
            = (i * duration_one_point - floorf(i * duration_one_point)) * 1e8;
    }
    for (int i = m_duration_weight_shift * 10;
         i < (m_duration_weight_shift * 10 + m_incoming_joint_trajectory.points.size()); i++) {
        int old_time_sec = m_final_joint_trajectory.points[i].time_from_start.sec;
        uint old_time_nanosec = m_final_joint_trajectory.points[i].time_from_start.nanosec;

        int new_time_sec = (int)floor(old_time_sec + (old_time_nanosec + 5e8) / 1e9);
        uint new_time_nanosec = (old_time_nanosec + 50000000) % 100000000;

        m_final_joint_trajectory.points[i].time_from_start.sec = new_time_sec;
        m_final_joint_trajectory.points[i].time_from_start.nanosec = new_time_nanosec;

        RCLCPP_INFO(
            rclcpp::get_logger("time during step"), "%i", m_final_joint_trajectory.points[i].time_from_start.sec);
        RCLCPP_INFO(rclcpp::get_logger("time nano"), "%i", m_final_joint_trajectory.points[i].time_from_start.nanosec);
    }
    for (int i = (m_duration_weight_shift * 10 + m_incoming_joint_trajectory.points.size());
         i < m_final_joint_trajectory.points.size(); i++) {
        m_final_joint_trajectory.points[i].time_from_start.sec = i * duration_one_point;
        m_final_joint_trajectory.points[i].time_from_start.nanosec
            = (i * duration_one_point - floorf(i * duration_one_point)) * 1e8;
    }
}

// combine everything into one function
trajectory_msgs::msg::JointTrajectory WeightShiftBuffer::return_final_traj_with_weight_shift(
    trajectory_msgs::msg::JointTrajectory msg)
{
    set_incoming_joint_trajectory(msg);
    m_first_traj_point = m_incoming_joint_trajectory.points[0];
    update_HAA_during_step();
    add_weight_shift();
    reset_HAA_at_end();
    fix_timings_traj();

    return m_final_joint_trajectory;
}
//
trajectory_msgs::msg::JointTrajectory WeightShiftBuffer::add_weight_shift_during_gait()
{
    double base_offset = -0.05;
    if (swing_leg == "R") {
        for (int i = 0; i < m_duration_weight_shift * 10; i++) {
            m_incoming_joint_trajectory.points[i].positions[1]
                += (m_hip_aa_position / floor(m_duration_weight_shift * 10)
                    * i); //(m_hip_aa_position-m_first_traj_point.positions[1])/(m_duration_weight_shift*10)*i;
            m_incoming_joint_trajectory.points[i].positions[5]
                -= (m_hip_aa_position / floor(m_duration_weight_shift * 10)
                    * i); //(-m_hip_aa_position-m_first_traj_point.positions[5])/(m_duration_weight_shift*10)*i;

            m_incoming_joint_trajectory.points[m_incoming_joint_trajectory.points.size() - i - 1].positions[1]
                += (m_hip_aa_position / (m_duration_weight_shift * 10)
                    * i); //(m_hip_aa_position-m_first_traj_point.positions[1])/(m_duration_weight_shift*10)*i;
            m_incoming_joint_trajectory.points[m_incoming_joint_trajectory.points.size() - i - 1].positions[5]
                -= (m_hip_aa_position / (m_duration_weight_shift * 10) * i);
        }

        for (int i = m_duration_weight_shift * 10;
             i < m_incoming_joint_trajectory.points.size() - (m_duration_weight_shift * 10); i++) {
            m_incoming_joint_trajectory.points[i].positions[1] = base_offset+(m_hip_aa_position);
            m_incoming_joint_trajectory.points[i].positions[5] = base_offset-(m_hip_aa_position);
        }

        swing_leg = "L";
    } else if (swing_leg == "L") {
        for (int i = 0; i < m_duration_weight_shift * 10; i++) {
            m_incoming_joint_trajectory.points[i].positions[5] += (m_hip_aa_position / (m_duration_weight_shift * 10)
                * i); //(m_hip_aa_position-m_first_traj_point.positions[1])/(m_duration_weight_shift*10)*i;
            m_incoming_joint_trajectory.points[i].positions[1]
                -= (m_hip_aa_position / (m_duration_weight_shift * 10) * i);

            m_incoming_joint_trajectory.points[m_incoming_joint_trajectory.points.size() - i - 1].positions[5]
                += (m_hip_aa_position / (m_duration_weight_shift * 10)
                    * i); //(m_hip_aa_position-m_first_traj_point.positions[1])/(m_duration_weight_shift*10)*i;
            m_incoming_joint_trajectory.points[m_incoming_joint_trajectory.points.size() - i - 1].positions[1]
                -= (m_hip_aa_position / (m_duration_weight_shift * 10) * i);
        }

        for (int i = m_duration_weight_shift * 10;
             i < m_incoming_joint_trajectory.points.size() - (m_duration_weight_shift * 10); i++) {
            m_incoming_joint_trajectory.points[i].positions[5] = base_offset+(m_hip_aa_position);
            m_incoming_joint_trajectory.points[i].positions[1] = base_offset-(m_hip_aa_position);
        }

        swing_leg = "R";
    }
    m_final_joint_trajectory = m_incoming_joint_trajectory;

    return m_final_joint_trajectory;
}

void WeightShiftBuffer::set_weight_shift_duration(double duration)
{
    m_duration_weight_shift = duration;
    RCLCPP_INFO(rclcpp::get_logger("weight_shift_buffer"), "\n\n CHANGED WEIGHT SHIFT DURATION TO %f", duration);
}

void WeightShiftBuffer::set_step_size(double step_size)
{
    m_hip_aa_position = step_size;
    RCLCPP_INFO(rclcpp::get_logger("weight_shift_buffer"), "\n\n CHANGED WEIGHT SHIFT SIZE TO %f", step_size);
}