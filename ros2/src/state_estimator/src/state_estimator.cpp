#include "state_estimator/state_estimator.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

StateEstimator::StateEstimator()
    : Node("state_estimator_node")
    , m_joint_estimator(this)
    , m_com_estimator()
    , m_cop_estimator(create_pressure_sensors())
    , m_imu_estimator()
    , m_zmp_estimator()
    , m_footstep_estimator()
    , m_current_stance_foot(0)
{
    m_upper_imu_subscriber = this->create_subscription<sensor_msgs::msg::Imu>(
        "/upper_imu", 10, std::bind(&StateEstimator::upper_imu_callback, this, _1));
    m_lower_imu_subscriber = this->create_subscription<sensor_msgs::msg::Imu>(
        "/lower_imu", 10, std::bind(&StateEstimator::lower_imu_callback, this, _1));
    m_pressure_sole_subscriber = this->create_subscription<march_shared_msgs::msg::PressureSolesData>(
        "/march/pressure_sole_data", 10, std::bind(&StateEstimator::pressure_sole_callback, this, _1));
    m_state_subscriber = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, std::bind(&StateEstimator::state_callback, this, _1));

    m_tf_joint_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    m_com_pos_publisher = this->create_publisher<march_shared_msgs::msg::CenterOfMass>("robot_com_position", 100);

    m_cop_pos_publisher = this->create_publisher<geometry_msgs::msg::PointStamped>("robot_cop_position", 100);

    m_foot_pos_publisher = this->create_publisher<geometry_msgs::msg::PoseArray>("est_foot_position", 100);

    m_zmp_pos_publisher = this->create_publisher<geometry_msgs::msg::PointStamped>("robot_zmp_position", 100);

    m_stance_foot_publisher = this->create_publisher<std_msgs::msg::Int32>("current_stance_foot", 100);

    m_feet_height_publisher
        = this->create_publisher<march_shared_msgs::msg::FeetHeightStamped>("robot_feet_height", 100);

    m_joint_state_publisher = this->create_publisher<sensor_msgs::msg::JointState>("measured_joint_states", 100);

    m_rviz_publisher = this->create_publisher<visualization_msgs::msg::Marker>("joint_visualizations", 100);

    declare_parameter("state_estimator_config.refresh_rate", 1000);
    auto refresh_rate = this->get_parameter("state_estimator_config.refresh_rate").as_int();
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(refresh_rate), std::bind(&StateEstimator::publish_robot_frames, this));

    m_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    m_tf_joint_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);

    // Declare parameters
    // IMU parameters
    declare_parameter("imu_estimator.lower_imu.IMU_exo_base_link", std::string("default"));
    declare_parameter("imu_estimator.lower_imu.IMU_exo_position", std::vector<double>(3, 0.0));
    declare_parameter("imu_estimator.lower_imu.IMU_exo_rotation", std::vector<double>(3, 0.0));
    declare_parameter("imu_estimator.upper_imu.IMU_exo_base_link", std::string("default"));
    declare_parameter("imu_estimator.upper_imu.IMU_exo_position", std::vector<double>(3, 0.0));
    declare_parameter("imu_estimator.upper_imu.IMU_exo_rotation", std::vector<double>(3, 0.0));

    declare_parameter("footstep_estimator.left_foot.size", std::vector<double>(6, 2));
    declare_parameter("footstep_estimator.right_foot.size", std::vector<double>(6, 2));
    declare_parameter("footstep_estimator.on_ground_threshold", 0.5);
    // Footstep parameters
    auto left_foot_size = this->get_parameter("footstep_estimator.left_foot.size").as_double_array();
    auto right_foot_size = this->get_parameter("footstep_estimator.right_foot.size").as_double_array();
    auto on_ground_threshold = this->get_parameter("footstep_estimator.on_ground_threshold").as_double();
    m_footstep_estimator.set_foot_size(left_foot_size[0], left_foot_size[1], "l");
    m_footstep_estimator.set_foot_size(right_foot_size[0], right_foot_size[1], "r");
    m_footstep_estimator.set_threshold(on_ground_threshold);

    m_foot_pos_publisher->publish(m_current_stance_foot);

    initialize_imus();
}

void StateEstimator::lower_imu_callback(sensor_msgs::msg::Imu::SharedPtr msg)
{
    m_imu_estimator.update_imu(*msg, LOWER);
}

void StateEstimator::upper_imu_callback(sensor_msgs::msg::Imu::SharedPtr msg)
{
    m_imu_estimator.update_imu(*msg, UPPER);
}

void StateEstimator::state_callback(sensor_msgs::msg::JointState::SharedPtr msg)
{
    this->m_joint_estimator.set_joint_states(msg);
    // m_joint_estimator.set_individual_joint_state("right_knee", 0.5);
    m_joint_state_publisher->publish(*msg);
}

void StateEstimator::pressure_sole_callback(march_shared_msgs::msg::PressureSolesData::SharedPtr msg)
{

    this->m_cop_estimator.update_sensor_pressures(update_pressure_sensors_data(msg->names, msg->pressure_values));
}

void StateEstimator::initialize_imus()
{
    IMU imu_to_set;
    auto imu_base_link = get_parameter("imu_estimator.lower_imu.IMU_exo_base_link").as_string();
    auto imu_position = get_parameter("imu_estimator.lower_imu.IMU_exo_position").as_double_array();
    auto imu_rotation = get_parameter("imu_estimator.lower_imu.IMU_exo_rotation").as_double_array();
    imu_to_set.data.header.frame_id = "lowerIMU";
    imu_to_set.base_frame = imu_base_link;
    imu_to_set.imu_location.translation.x = imu_position[0];
    imu_to_set.imu_location.translation.y = imu_position[1];
    imu_to_set.imu_location.translation.z = imu_position[2];
    tf2::Quaternion tf2_imu_rotation;
    tf2_imu_rotation.setRPY(imu_rotation[0], imu_rotation[1], imu_rotation[2]);
    tf2_imu_rotation.normalize();
    tf2::convert(tf2_imu_rotation, imu_to_set.imu_location.rotation);
    m_imu_estimator.set_imu(imu_to_set, LOWER);
    //
    imu_base_link = get_parameter("imu_estimator.upper_imu.IMU_exo_base_link").as_string();
    imu_position = get_parameter("imu_estimator.upper_imu.IMU_exo_position").as_double_array();
    imu_rotation = get_parameter("imu_estimator.upper_imu.IMU_exo_rotation").as_double_array();
    imu_to_set.data.header.frame_id = "upperIMU";
    imu_to_set.base_frame = imu_base_link;
    imu_to_set.imu_location.translation.x = imu_position[0];
    imu_to_set.imu_location.translation.y = imu_position[1];
    imu_to_set.imu_location.translation.z = imu_position[2];
    tf2_imu_rotation.setRPY(imu_rotation[0], imu_rotation[1], imu_rotation[2]);
    tf2_imu_rotation.normalize();
    tf2::convert(tf2_imu_rotation, imu_to_set.imu_location.rotation);
    m_imu_estimator.set_imu(imu_to_set, UPPER);
}

void StateEstimator::update_foot_frames()
{
    // This script assumes the base foot frames are named LEFT_ORIGIN and RIGHT_ORIGIN;
    // obtain the origin joint
    // IMU& imu = m_imu_estimator.get_imu(LOWER);
    try {
        geometry_msgs::msg::TransformStamped measured_hip_base_angle
            = m_tf_buffer->lookupTransform("lowerIMU", "map", tf2::TimePointZero);
        geometry_msgs::msg::TransformStamped expected_hip_base_angle
            = m_tf_buffer->lookupTransform("hip_base", "map", tf2::TimePointZero);
        //
        tf2::Quaternion tf2_measured_hip_base_angle;
        tf2::Quaternion tf2_expected_hip_base_angle;
        tf2::fromMsg(measured_hip_base_angle.transform.rotation, tf2_measured_hip_base_angle);
        tf2::fromMsg(expected_hip_base_angle.transform.rotation, tf2_expected_hip_base_angle);
        tf2_measured_hip_base_angle.normalize();
        tf2_expected_hip_base_angle.normalize();
        tf2::Quaternion tf2_angle_difference = tf2_measured_hip_base_angle - tf2_expected_hip_base_angle;
        tf2_angle_difference.normalize();
        geometry_msgs::msg::Quaternion angle_difference;
        tf2::convert(tf2_angle_difference, angle_difference);
        // testing
        tf2::Matrix3x3 m(tf2_angle_difference);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        RCLCPP_DEBUG(this->get_logger(), "The difference in angle is %f, %f, %f", roll, pitch, yaw);
        pitch = 0.0;
        m_joint_estimator.set_individual_joint_state("right_origin", pitch);
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "error in update_foot_frames: %s", ex.what());
    }
}

void StateEstimator::publish_com_frame()
{
    geometry_msgs::msg::TransformStamped com_transform;
    // All center of mass calculations are already done in the base frame,
    // So we simply update the headers and the translation component :)
    com_transform.header.frame_id = "map";
    com_transform.child_frame_id = "com";
    CenterOfMass com = m_com_estimator.get_com_state();

    com_transform.transform.translation.x = com.position.point.x;
    com_transform.transform.translation.y = com.position.point.y;
    com_transform.transform.translation.z = com.position.point.z;
    // The unit quaternion
    com_transform.transform.rotation.x = 0.0;
    com_transform.transform.rotation.y = 0.0;
    com_transform.transform.rotation.z = 0.0;
    com_transform.transform.rotation.w = 1.0;

    m_tf_joint_broadcaster->sendTransform(com_transform);

    // Here, we also publish to the robot com position
    march_shared_msgs::msg::CenterOfMass center_of_mass;
    center_of_mass.position = com.position.point;
    center_of_mass.velocity = m_zmp_estimator.get_com_velocity();

    m_com_pos_publisher->publish(center_of_mass);
}

void StateEstimator::publish_robot_frames()
{
    // publish IMU frames
    IMU& imu = m_imu_estimator.get_imu(LOWER);
    m_tf_joint_broadcaster->sendTransform(imu.get_imu_rotation());
    update_foot_frames();
    // publish joint frames
    RCLCPP_DEBUG(this->get_logger(), "Number of frames is %i", m_joint_estimator.get_joint_frames().size());
    for (auto i : m_joint_estimator.get_joint_frames()) {
        m_tf_joint_broadcaster->sendTransform(i);
        RCLCPP_DEBUG(this->get_logger(),
            ("\n Set up link " + i.header.frame_id + "\n with child link " + i.child_frame_id).c_str());
    }
    // Publish each joint center of mass
    std::vector<CenterOfMass> joint_com_positions = m_joint_estimator.get_joint_com_positions("map");

    // Update and publish the actual, full center of mass
    m_com_estimator.set_com_state(joint_com_positions);
    publish_com_frame();
    // Update COP
    try {
        geometry_msgs::msg::TransformStamped left_foot_frame
            = m_tf_buffer->lookupTransform("map", "left_origin", tf2::TimePointZero);
        geometry_msgs::msg::TransformStamped right_foot_frame
            = m_tf_buffer->lookupTransform("map", "right_origin", tf2::TimePointZero);
        m_cop_estimator.set_cop_state(m_cop_estimator.get_sensors(), { right_foot_frame, left_foot_frame });
        // Update ZMP
        m_zmp_estimator.set_com_states(m_com_estimator.get_com_state(), this->get_clock()->now());
        m_zmp_estimator.set_zmp();
        m_zmp_pos_publisher->publish(m_zmp_estimator.get_zmp());
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "Error during publishing of Center of Pressure: %s", ex.what());
    }

    // Update the feet
    m_footstep_estimator.update_feet(m_cop_estimator.get_sensors());
    // Publish the feet
    // The first foot is always the right foot
    // The second foot is always the left foot
    geometry_msgs::msg::PoseArray foot_positions;
    foot_positions.header.frame_id = "map";
    foot_positions.poses.push_back(m_footstep_estimator.get_foot_position("r"));
    foot_positions.poses.push_back(m_footstep_estimator.get_foot_position("l"));
    m_foot_pos_publisher->publish(foot_positions);

    // double stance
    if (m_footstep_estimator.get_foot_on_ground("l") && m_footstep_estimator.get_foot_on_ground("r")) {

        // We always take the front foot as the stance foot :)
        if (foot_positions.poses[0].position.x > foot_positions.poses[1].position.x && m_current_stance_foot != 1) {
            m_current_stance_foot = 1;
            m_foot_pos_publisher->publish(m_current_stance_foot);
        } else if (foot_positions.poses[0].position.x < foot_positions.poses[1].position.x
            && m_current_stance_foot != -1) {
            m_current_stance_foot = -1;
            m_foot_pos_publisher->publish(m_current_stance_foot);
        }
    }
    // left
    if (m_footstep_estimator.get_foot_on_ground("l") && !m_footstep_estimator.get_foot_on_ground("r")
        && m_current_stance_foot != -1) {
        m_current_stance_foot = -1;
        m_stance_foot_publisher->publish(m_current_stance_foot);
    }
    // right
    if (!m_footstep_estimator.get_foot_on_ground("l") && m_footstep_estimator.get_foot_on_ground("r")
        && m_current_stance_foot != 1) {
        m_current_stance_foot = 1;
        m_stance_foot_publisher->publish(m_current_stance_foot);
    }

    // Update and publish feet height
    march_shared_msgs::msg::FeetHeightStamped feet_height_msg;
    feet_height_msg.header.frame_id = "map";
    feet_height_msg.heights = m_joint_estimator.get_feet_height();
    m_feet_height_publisher->publish(feet_height_msg);

    visualize_joints();
}

geometry_msgs::msg::TransformStamped StateEstimator::get_frame_transform(
    const std::string& target_frame, const std::string& source_frame)
{
    geometry_msgs::msg::TransformStamped frame_transform;
    try {
        frame_transform = m_tf_buffer->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
        return frame_transform;
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "Source frame: %s", source_frame.c_str());
        RCLCPP_WARN(this->get_logger(), "Target frame: %s", target_frame.c_str());
        RCLCPP_WARN(this->get_logger(), "error in get_frame_transform: %s", ex.what());
        return frame_transform;
    }
}

std::vector<PressureSensor> StateEstimator::create_pressure_sensors()
{
    this->declare_parameter("cop_estimator.names", std::vector<std::string>(16, ""));
    this->declare_parameter("cop_estimator.x_positions", std::vector<double>(16, 0.0));
    this->declare_parameter("cop_estimator.y_positions", std::vector<double>(16, 0.0));
    this->declare_parameter("cop_estimator.z_positions", std::vector<double>(16, 0.0));
    auto names = this->get_parameter("cop_estimator.names").as_string_array();
    auto x_positions = this->get_parameter("cop_estimator.x_positions").as_double_array();
    auto y_positions = this->get_parameter("cop_estimator.y_positions").as_double_array();
    auto z_positions = this->get_parameter("cop_estimator.z_positions").as_double_array();
    std::vector<PressureSensor> sensors;
    for (size_t i = 0; i < names.size(); i++) {
        PressureSensor sensor;
        const char initial = names.at(i)[0];
        if (initial != 'l' && initial != 'r') {
            RCLCPP_WARN(this->get_logger(),
                "Pressure Sensor %i has incorrect initial character %s. Required: 'l' or 'r'", i, initial);
        }
        sensor.name = initial;
        CenterOfPressure cop;
        if (sensor.name.find("l_") != std::string::npos) {
            cop.position.header.frame_id = "leftPressureSole";
        }
        if (sensor.name.find("r_") != std::string::npos) {
            cop.position.header.frame_id = "rightPressureSole";
        }

        cop.position.point.x = x_positions.at(i);
        cop.position.point.y = y_positions.at(i);
        cop.position.point.z = z_positions.at(i);
        sensors.push_back(sensor);
    }
    // Read the pressure sensors from the hardware interface
    return sensors;
}

std::map<std::string, double> StateEstimator::update_pressure_sensors_data(
    std::vector<std::string> names, std::vector<double> pressure_values)
{
    std::map<std::string, double> pressure_values_map;
    for (size_t i = 0; i < names.size(); i++) {
        pressure_values_map.emplace(names.at(i), pressure_values.at(i));
    }
    return pressure_values_map;
}

void StateEstimator::visualize_joints()
{
    // Publish the joint visualizations
    visualization_msgs::msg::Marker joint_markers;
    joint_markers.type = 5;
    joint_markers.header.frame_id = "map";
    joint_markers.id = 0;
    std::vector<JointContainer> joints = m_joint_estimator.get_joints();
    geometry_msgs::msg::TransformStamped joint_transform;
    geometry_msgs::msg::Point marker_container;
    tf2::Quaternion tf2_joint_rotation;
    // Joint_endpoint is in local joint coordinates, we transform it to obtain global coordinates
    tf2::Vector3 joint_endpoint;
    try {
        for (auto i : joints) {
            joint_transform = m_tf_buffer->lookupTransform("map", i.frame.header.frame_id, tf2::TimePointZero);
            marker_container.x = joint_transform.transform.translation.x;
            marker_container.y = joint_transform.transform.translation.y;
            marker_container.z = joint_transform.transform.translation.z;
            joint_markers.points.push_back(marker_container);
            // We have to set up the joint transform manually because none of the transform functions work >:(
            tf2_joint_rotation
                = tf2::Quaternion(joint_transform.transform.rotation.x, joint_transform.transform.rotation.y,
                    joint_transform.transform.rotation.z, joint_transform.transform.rotation.w);
            joint_endpoint = tf2::quatRotate(tf2_joint_rotation, tf2::Vector3(i.length_x, i.length_y, i.length_z));
            marker_container.x += joint_endpoint.getX();
            marker_container.y += joint_endpoint.getY();
            marker_container.z += joint_endpoint.getZ();
            joint_markers.points.push_back(marker_container);
            RCLCPP_DEBUG(
                this->get_logger(), "Marker:[%f,%f,%f]", marker_container.x, marker_container.y, marker_container.z);
        }

    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "error in visualize_joints: %s", ex.what());
    }
    RCLCPP_DEBUG(this->get_logger(), "Published %i markers", joint_markers.points.size());
    joint_markers.action = 0;
    joint_markers.frame_locked = 1;
    joint_markers.scale.x = 0.2;
    joint_markers.scale.y = 1.0;
    joint_markers.scale.z = 1.0;
    joint_markers.pose.position.x = 0.0;
    joint_markers.pose.position.y = 0.0;
    joint_markers.pose.position.z = 0.0;
    joint_markers.pose.orientation.x = 0.0;
    joint_markers.pose.orientation.y = 0.0;
    joint_markers.pose.orientation.z = 0.0;
    joint_markers.pose.orientation.w = 1.0;
    joint_markers.ns = "exo_joint_visualization";
    joint_markers.lifetime.sec = 1;
    joint_markers.color.a = 1.0;
    m_rviz_publisher->publish(joint_markers);
}