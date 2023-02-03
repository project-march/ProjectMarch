#include "joint_estimator.hpp"

JointEstimator::JointEstimator(StateEstimator* owner, sensor_msgs::msg::JointState initial_joint_states)
    : m_owner(owner)
    , m_joint_states(initial_joint_states)
{
    m_joint_child_link_map = {
        { "left_ankle", "testb" },
        { "left_hip_aa", "testd" },
        { "left_hip_fe", "testb" },
        { "left_knee", "testb" },
        { "right_ankle", "testb" },
        { "right_hip_aa", "testb" },
        { "right_hip_fe", "testb" },
        { "right_knee", "testb" },
    };

    initialize_joints(initial_joint_states);
}

void JointEstimator::set_joint_states(sensor_msgs::msg::JointState& new_joint_states)
{   
    tf2::Quaternion quaternion_math;
    geometry_msgs::msg::Quaternion quaternion_joint;
    int counter = 0;
    for (auto i: m_joints) {
        switch (i.hinge_axis) {
            case X:
                quaternion_math.setRPY(new_joint_states.position[counter] ,0 ,0);
                break;
            case Y:
                quaternion_math.setRPY(0, new_joint_states.position[counter] ,0);
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

sensor_msgs::msg::JointState JointEstimator::get_joint_states()
{
    return m_joint_states;
}

const std::vector<geometry_msgs::msg::TransformStamped> JointEstimator::get_joint_frames()
{
    std::vector<geometry_msgs::msg::TransformStamped> transform_frames;

    geometry_msgs::msg::TransformStamped current_frame;
    for (auto i: m_joints) {
        transform_frames.push_back(i.frame);
    };
    
    return transform_frames;
}

void JointEstimator::initialize_joints(sensor_msgs::msg::JointState initial_joint_states)
{
    int joint_amount = sizeof(initial_joint_states.name) / sizeof(initial_joint_states.name[0]) + 2;
    JointContainer joint_to_add;
    geometry_msgs::msg::TransformStamped joint_frame;
    tf2::Quaternion quaternion_math;
    geometry_msgs::msg::Quaternion quaternion_joint;
    for (int i = 0; i < joint_amount; i++) {
        joint_to_add.name = initial_joint_states.name[i];
        // These need to be obtained from yaml parameters
        joint_to_add.com_x = 1;
        joint_to_add.com_y = 0;
        joint_to_add.com_z = 0;
        joint_to_add.length_x = 1;
        // end of TO BE REPLACED
        joint_frame.child_frame_id = m_joint_child_link_map.find(std::string(joint_to_add.name))->second;
        joint_frame.transform.translation.x = joint_to_add.length_x;
        joint_frame.transform.translation.y = joint_to_add.length_y;
        joint_frame.transform.translation.z = joint_to_add.length_z;
        // Add rotations here
        joint_to_add.hinge_axis = X;
        switch (joint_to_add.hinge_axis) {
            case X:
                quaternion_math.setRPY(initial_joint_states.position[i] ,0 ,0);
                break;
            case Y:
                quaternion_math.setRPY(0, initial_joint_states.position[i] ,0);
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
}