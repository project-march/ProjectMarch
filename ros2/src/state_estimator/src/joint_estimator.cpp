#include "state_estimator/joint_estimator.hpp"
#include "state_estimator/state_estimator.hpp"
JointEstimator::JointEstimator(StateEstimator* owner)
    : m_owner(owner)
    , m_joint_child_link_map(interpret_joint_links())
{

    initialize_joints();
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

const JointContainer JointEstimator::get_individual_joint(std::string joint_name)
{

    // NOTE: This is not an efficient function, mainly use this for debugging purposes
    std::vector<JointContainer>::iterator it
        = std::find_if(m_joints.begin(), m_joints.end(), [joint_name](const JointContainer& joint) {
              return joint.name == joint_name;
          });

    return *it;
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

std::unordered_map<std::string, std::string> JointEstimator::interpret_joint_links()
{
    // We must declare these specific parameters here because they're called before the constructor
    m_owner->declare_parameter("joint_estimator.links", std::vector<std::string>(11, "default"));
    m_owner->declare_parameter("joint_estimator.base_links", std::vector<std::string>(11, "default"));
    auto link_list = m_owner->get_parameter("joint_estimator.links").as_string_array();
    auto child_link_list = m_owner->get_parameter("joint_estimator.base_links").as_string_array();

    std::unordered_map<std::string, std::string> joint_link_map;
    for (int i = 0; i < link_list.size(); i++) {
        joint_link_map.insert(std::pair<std::string, std::string> { link_list[i], child_link_list[i] });
        RCLCPP_INFO(m_owner->get_logger(), "match %s with %s", link_list[i].c_str(), child_link_list[i].c_str());
    }

    return joint_link_map;
}

void JointEstimator::initialize_joints()
{

    // We must declare these specific parameters here because they're called before the constructor
    // It's sadly a long list but it only happens once at initialization so it doesn't need a separate function
    // It just looks very ugly but it's necessary because it's ROS2 :(

    m_owner->declare_parameter("joint_estimator.link_hinge_axis", std::vector<int64_t>(6, 0));
    m_owner->declare_parameter("joint_estimator.link_length_x", std::vector<double>(5, 0.0));
    m_owner->declare_parameter("joint_estimator.link_length_y", std::vector<double>(6, 0.0));
    m_owner->declare_parameter("joint_estimator.link_length_z", std::vector<double>(6, 0.0));
    m_owner->declare_parameter("joint_estimator.link_mass", std::vector<double>(6, 0.0));
    m_owner->declare_parameter("joint_estimator.link_com_x", std::vector<double>(6, 0.0));
    m_owner->declare_parameter("joint_estimator.link_com_y", std::vector<double>(6, 0.0));
    m_owner->declare_parameter("joint_estimator.link_com_z", std::vector<double>(6, 0.0));
    m_owner->declare_parameter("joint_estimator.starting_position", std::vector<double>(6, 0.0));
    auto link_hinges = m_owner->get_parameter("joint_estimator.link_hinge_axis").as_integer_array();
    auto link_length_x = m_owner->get_parameter("joint_estimator.link_length_x").as_double_array();
    auto link_length_y = m_owner->get_parameter("joint_estimator.link_length_y").as_double_array();
    auto link_length_z = m_owner->get_parameter("joint_estimator.link_length_z").as_double_array();
    auto link_com_mass = m_owner->get_parameter("joint_estimator.link_mass").as_double_array();
    auto link_com_x = m_owner->get_parameter("joint_estimator.link_com_x").as_double_array();
    auto link_com_y = m_owner->get_parameter("joint_estimator.link_com_y").as_double_array();
    auto link_com_z = m_owner->get_parameter("joint_estimator.link_com_z").as_double_array();

    auto link_names = m_owner->get_parameter("joint_estimator.links").as_string_array();
    auto link_position = m_owner->get_parameter("joint_estimator.starting_position").as_double_array();

    int joint_amount = link_names.size();

    JointContainer joint_to_add;
    geometry_msgs::msg::TransformStamped joint_frame;
    tf2::Quaternion quaternion_math;
    geometry_msgs::msg::Quaternion quaternion_joint;
    try {
        for (int i = 0; i < joint_amount; i++) {
            joint_to_add.name = link_names[i];
            // These need to be obtained from yaml parameters
            joint_to_add.com.mass = link_com_mass[i];
            joint_to_add.com.position.point.x = link_com_x[i];
            joint_to_add.com.position.point.y = link_com_y[i];
            joint_to_add.com.position.point.z = link_com_z[i];
            joint_to_add.length_x = link_length_x[i];
            joint_to_add.length_y = link_length_y[i];
            joint_to_add.length_z = link_length_z[i];

            joint_frame.header.frame_id = joint_to_add.name;
            joint_frame.child_frame_id = m_joint_child_link_map.find(std::string(joint_to_add.name))->second;
            joint_frame.transform.translation.x = joint_to_add.length_x;
            joint_frame.transform.translation.y = joint_to_add.length_y;
            joint_frame.transform.translation.z = joint_to_add.length_z;
            // Add rotations here
            double nan_guard = 1e-8; // We add this value to every initial joint to avoid complete zeroes, which
                                     // sometimes cause NaN errors
            joint_to_add.hinge_axis = static_cast<Rotation>(link_hinges[i]);
            switch (joint_to_add.hinge_axis) {
                case X:
                    quaternion_math.setRPY(link_position[i] + nan_guard, 0, 0);
                    break;
                case Y:
                    quaternion_math.setRPY(0, link_position[i] + nan_guard, 0);
                    break;
                case Z:
                    quaternion_math.setRPY(0, 0, link_position[i] + nan_guard);
                    break;
            }
            quaternion_math.normalize();
            tf2::convert(quaternion_math, quaternion_joint);
            joint_frame.transform.rotation = quaternion_joint;
            joint_to_add.frame = joint_frame;
            m_joints.push_back(joint_to_add);
            // RCLCPP_INFO(m_owner->get_logger(), "%s", link_names[i].c_str());
        };
    } catch (...) {
        RCLCPP_WARN(m_owner->get_logger(), "Couldn't initialize joints!\n Perhaps the message was passed wrongly?");
    }
}

std::vector<CenterOfMass> JointEstimator::get_joint_com_positions(std::string coordinate_frame)
{
    std::vector<CenterOfMass> com_positions;
    CenterOfMass com_to_add;
    geometry_msgs::msg::PointStamped joint_com_stamped;
    geometry_msgs::msg::TransformStamped com_transform;

    for (auto i : m_joints) {
        try {
            com_transform = m_owner->get_frame_transform(coordinate_frame, i.name);
            joint_com_stamped.point = i.com.position.point;
            joint_com_stamped.header.stamp = m_owner->get_clock()->now();
            tf2::doTransform(joint_com_stamped, com_to_add.position, com_transform);
            com_positions.push_back(com_to_add);
        } catch (const tf2::TransformException& ex) {
            // RCLCPP_WARN(m_owner->get_logger(), "Error in get_joint_com_position: %s", ex.what());
            return com_positions;
        }
    };
    return com_positions;
}