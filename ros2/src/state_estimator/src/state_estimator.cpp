#include "state_estimator/state_estimator.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

StateEstimator::StateEstimator()
    : Node("state_estimator_node")
    , m_joint_estimator(this)
    , m_com_estimator()
    , m_imu_estimator()
    , m_cop_estimator(CopEstimator(get_pressure_sensors()))
{
    m_state_publisher = this->create_publisher<march_shared_msgs::msg::RobotState>("robot_state", 10);
    m_sensor_subscriber = this->create_subscription<sensor_msgs::msg::Imu>(
        "/lower_imu", 10, std::bind(&StateEstimator::sensor_callback, this, _1));
    m_state_subscriber = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_state", 10, std::bind(&StateEstimator::state_callback, this, _1));
    m_tf_joint_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    m_com_pos_publisher = this->create_publisher<geometry_msgs::msg::PointStamped>("robot_com_position", 100);

    timer_ = this->create_wall_timer(1000ms, std::bind(&StateEstimator::publish_robot_frames, this));

    m_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    m_tf_joint_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);

    // Declare parameters
    // IMU parameters
    declare_parameter("imu_estimator.IMU_exo_base_link", std::string("default"));
    declare_parameter("imu_estimator.IMU_exo_position", std::vector<double>(3, 0.0));
    declare_parameter("imu_estimator.IMU_exo_rotation", std::vector<double>(3, 0.0));
    // declare_parameter("joint_estimator.link_hinge_axis", std::vector<int64_t>(6, 1));
    // declare_parameter("joint_estimator.link_length_x", std::vector<float>(5, 0.0));
    // declare_parameter("joint_estimator.link_length_y", std::vector<double>(6, 0.0));
    // declare_parameter("joint_estimator.link_length_z", std::vector<double>(6, 0.0));
    // declare_parameter("joint_estimator.link_mass", std::vector<double>(6, 0.0));
    // declare_parameter("joint_estimator.link_com_x", std::vector<double>(6, 0.0));
    // declare_parameter("joint_estimator.link_com_y", std::vector<double>(6, 0.0));
    // declare_parameter("joint_estimator.link_com_z", std::vector<double>(6, 0.0));

    initialize_imus();
}

// sensor_msgs::msg::JointState StateEstimator::get_initial_joint_states()
// {
//     sensor_msgs::msg::JointState initial_joint_state;
//     // change it so the names are obtained from the parameter
//     initial_joint_state.name = { "right_ankle", "right_knee", "right_hip_fe", "right_hip_aa", "left_ankle",
//     "left_knee",
//         "left_hip_fe", "left_hip_aa", "right_origin", "left_origin" };
//     initial_joint_state.position = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
//     return initial_joint_state;
// }

void StateEstimator::sensor_callback(sensor_msgs::msg::Imu::SharedPtr msg)
{
    m_imu_estimator.update_imu(*msg);
}

void StateEstimator::state_callback(sensor_msgs::msg::JointState::SharedPtr msg)
{
}

void StateEstimator::initialize_imus()
{
    IMU imu_to_set;
    auto imu_base_link = get_parameter("imu_estimator.IMU_exo_base_link").as_string();
    auto imu_position = get_parameter("imu_estimator.IMU_exo_position").as_double_array();
    auto imu_rotation = get_parameter("imu_estimator.IMU_exo_rotation").as_double_array();
    imu_to_set.data.header.frame_id = "lowerIMU";
    imu_to_set.base_frame = imu_base_link;
    imu_to_set.imu_location.translation.x = imu_position[0];
    imu_to_set.imu_location.translation.x = imu_position[1];
    imu_to_set.imu_location.translation.x = imu_position[2];
    tf2::Quaternion tf2_imu_rotation;
    tf2_imu_rotation.setRPY(imu_rotation[0], imu_rotation[1], imu_rotation[2]);
    tf2_imu_rotation.normalize();
    tf2::convert(tf2_imu_rotation, imu_to_set.imu_location.rotation);
    m_imu_estimator.set_imu(imu_to_set);
}

void StateEstimator::update_foot_frames()
{
    // This script assumes the base foot frames are named LEFT_ORIGIN and RIGHT_ORIGIN;
    // obtain the origin joint
    IMU& imu = m_imu_estimator.get_imu();
    try {
        geometry_msgs::msg::TransformStamped measured_hip_base_angle
            = m_tf_buffer->lookupTransform(imu.get_imu_rotation().header.frame_id, "map", tf2::TimePointZero);
        geometry_msgs::msg::TransformStamped expected_hip_base_angle
            = m_tf_buffer->lookupTransform("hip_base", "map", tf2::TimePointZero);
        //
        tf2::Quaternion tf2_measured_hip_base_angle; 
        tf2::Quaternion tf2_expected_hip_base_angle;
        tf2::fromMsg(measured_hip_base_angle.transform.rotation,tf2_measured_hip_base_angle);
        tf2::fromMsg(expected_hip_base_angle.transform.rotation,tf2_expected_hip_base_angle);
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

        RCLCPP_INFO(this->get_logger(), "The difference in angle is %f, %f, %f", roll, pitch, yaw);
        m_joint_estimator.set_individual_joint_state("right_origin", roll);
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "error in update_foot_frames: %s", ex.what());
    }
}

void StateEstimator::publish_robot_state()
{
    auto msg = march_shared_msgs::msg::RobotState();
    msg.stamp = this->get_clock()->now();
    msg.joint_names.push_back("");
    msg.joint_pos.push_back(0);
    msg.joint_vel.push_back(0);
    msg.sensor_names.push_back("");
    msg.sensor_data.push_back(0);

    m_state_publisher->publish(msg);
}

void StateEstimator::publish_robot_frames()
{
    //debug function
    m_joint_estimator.set_individual_joint_state("right_knee", 3.14);
    // publish IMU frames
    IMU& imu = m_imu_estimator.get_imu();
    m_tf_joint_broadcaster->sendTransform(imu.get_imu_rotation());
    update_foot_frames();
    // publish joint frames
    RCLCPP_DEBUG(this->get_logger(), "Number of frames is %i", m_joint_estimator.get_joint_frames().size());
    for (auto i : m_joint_estimator.get_joint_frames()) {
        m_tf_joint_broadcaster->sendTransform(i);
        RCLCPP_DEBUG(this->get_logger(),
            ("\n Set up link " + i.header.frame_id + "\n with child link " + i.child_frame_id).c_str());
    }
    std::vector<CenterOfMass> test = m_joint_estimator.get_joint_com_positions("right_knee");
    RCLCPP_DEBUG(this->get_logger(), "Array size is %i", test.size());
    for (auto com : test) {
        RCLCPP_DEBUG(this->get_logger(), ("\n Publishing COM"));
        RCLCPP_DEBUG(this->get_logger(), "\n Publishing COM with pos x = %f", com.position.point.x);
        m_com_pos_publisher->publish(com.position);
    }
    // RCLCPP_INFO(this->get_logger(), "Test amount: %i", test[0].point.x);
}

geometry_msgs::msg::TransformStamped StateEstimator::get_frame_transform(
    std::string& target_frame, std::string& source_frame)
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

std::vector<PressureSensor> StateEstimator::get_pressure_sensors()
{
    std::vector<PressureSensor> sensors;
    // Read the pressure sensors from the hardware interface
    PressureSensor mock_sensor;
    mock_sensor.name = "mock_sensor";
    CenterOfPressure cop;
    cop.position.point.x = 0;
    cop.position.point.y = 0;
    cop.position.point.z = 0;
    cop.pressure = 1;
    sensors.push_back(mock_sensor);
    return sensors;
}

geometry_msgs::msg::Point transform_point(
    std::string& target_frame, std::string& source_frame, geometry_msgs::msg::Point& point_to_transform)
{
}