#include "joint_estimator.hpp"
#include "state_estimator.hpp"
JointEstimator::JointEstimator(StateEstimator* owner, sensor_msgs::msg::JointState initial_joint_states)
    : m_owner(owner)
{
    m_joint_child_link_map = { {
                                   "left_origin",
                                   "left_ankle",
                               },
        { "left_ankle", "left_knee" }, { "left_knee", "left_hip_fe" }, { "left_hip_fe", "left_hip_aa" },
        { "left_hip_aa", "right_hip_aa" }, { "right_origin", "map" }, { "right_ankle", "right_origin" },
        { "right_knee", "right_ankle" }, { "right_hip_fe", "right_knee" }, { "right_hip_aa", "right_hip_fe" } };

    initialize_joints(initial_joint_states);
}

void JointEstimator::set_joint_states(sensor_msgs::msg::JointState new_joint_states)
{
    tf2::Quaternion quaternion_math;
    geometry_msgs::msg::Quaternion quaternion_joint;
    int counter = 0;
    for (auto i : m_joints) {
        switch (i.hinge_axis) {
            case X:
                quaternion_math.setRPY(new_joint_states.position[counter], 0, 0);
                break;
            case Y:
                quaternion_math.setRPY(0, new_joint_states.position[counter], 0);
                break;
            case Z:
                quaternion_math.setRPY(0, 0, new_joint_states.position[counter]);
                break;
        }
        quaternion_math.normalize();
        tf2::convert(quaternion_math, quaternion_joint);
        i.frame.transform.rotation = quaternion_joint;
    };
}

void JointEstimator::set_individual_joint_state(std::string joint_name, double new_position)
{

    // NOTE: This is not an efficient function, mainly use this for debugging purposes
    std::vector<JointContainer>::iterator it
        = std::find_if(m_joints.begin(), m_joints.end(), [joint_name](const JointContainer& joint) {
              return joint.name == joint_name;
          });

    tf2::Quaternion quaternion_math;
    switch (it->hinge_axis) { // probably put this into a function?
        case X:
            quaternion_math.setRPY(new_position, 0, 0);
            break;
        case Y:
            quaternion_math.setRPY(0, new_position, 0);
            break;
        case Z:
            quaternion_math.setRPY(0, 0, new_position);
            break;
    }
    quaternion_math.normalize();
    tf2::convert(quaternion_math, it->frame.transform.rotation);
}

const std::vector<geometry_msgs::msg::TransformStamped> JointEstimator::get_joint_frames()
{
    std::vector<geometry_msgs::msg::TransformStamped> transform_frames;

    geometry_msgs::msg::TransformStamped current_frame;
    for (auto i : m_joints) {
        transform_frames.push_back(i.frame);
    };

    return transform_frames;
}

void JointEstimator::initialize_joints(sensor_msgs::msg::JointState initial_joint_states)
{
    int joint_amount = initial_joint_states.name.size();
    RCLCPP_INFO(m_owner->get_logger(), "JOINT AMOUNT %i", joint_amount);
    JointContainer joint_to_add;
    geometry_msgs::msg::TransformStamped joint_frame;
    tf2::Quaternion quaternion_math;
    geometry_msgs::msg::Quaternion quaternion_joint;
    RCLCPP_INFO(m_owner->get_logger(), "Test 2");
    try {
        for (int i = 0; i < joint_amount; i++) {
            joint_to_add.name = initial_joint_states.name[i];
            // These need to be obtained from yaml parameters
            joint_to_add.com_x = 1;
            joint_to_add.com_y = 0;
            joint_to_add.com_z = 0;
            joint_to_add.length_x = 1;
            // end of TO BE REPLACED
            joint_frame.header.frame_id = joint_to_add.name;
            joint_frame.child_frame_id = m_joint_child_link_map.find(std::string(joint_to_add.name))->second;
            joint_frame.transform.translation.x = joint_to_add.length_x;
            joint_frame.transform.translation.y = joint_to_add.length_y;
            joint_frame.transform.translation.z = joint_to_add.length_z;
            // Add rotations here
            joint_to_add.hinge_axis = X;
            switch (joint_to_add.hinge_axis) {
                case X:
                    quaternion_math.setRPY(initial_joint_states.position[i], 0, 0);
                    break;
                case Y:
                    quaternion_math.setRPY(0, initial_joint_states.position[i], 0);
                    break;
                case Z:
                    quaternion_math.setRPY(0, 0, initial_joint_states.position[i]);
                    break;
            }
            quaternion_math.normalize();
            tf2::convert(quaternion_math, quaternion_joint);
            joint_frame.transform.rotation = quaternion_joint;
            joint_to_add.frame = joint_frame;
            m_joints.push_back(joint_to_add);
        };
    } catch (...) {
        RCLCPP_WARN(m_owner->get_logger(), "Couldn't initialize joints!\n Perhaps the message was passed wrongly?");
    }
}