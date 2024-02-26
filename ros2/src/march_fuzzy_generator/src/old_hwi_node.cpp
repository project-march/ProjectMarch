// // Class to receive the fuzzy weights and set the weights of the joints.
// // TODO: create a separate controller and get rid of this class in this file.
// class FuzzyWeightsNode : public rclcpp::Node {
// public:
//     explicit FuzzyWeightsNode()
//         : Node("fuzzy_weights_node")
//     {
//         m_weight_subscription = this->create_subscription<march_shared_msgs::msg::FuzzyWeights>(
//             "fuzzy_weights", 10, std::bind(&FuzzyWeightsNode::setJointWeights, this, _1));

//         m_measured_torque_publisher
//             = this->create_publisher<control_msgs::msg::JointTrajectoryControllerState>("measured_torque", 10);

//         m_measure_torque_subscription = this->create_subscription<std_msgs::msg::Int32>(
//             "measure_torque", 10, std::bind(&FuzzyWeightsNode::averageTorqueCallback, this, _1));
//     }

//     // Method to receive and set the weights of a single joint
//     void setJointWeights(march_shared_msgs::msg::FuzzyWeights::SharedPtr msg) {
        
//         bool found_joint = false;

//         for (march_hardware_interface::JointInfo& jointInfo : *joints_info_) {
//             if (jointInfo.name == msg->joint_name) {
//                 // TODO: ask the girls about this check.
//                 if (jointInfo.torque_weight > std::numeric_limits<float>::epsilon() && (!jointInfo.target_torque || std::isnan(jointInfo.target_torque))) {
//                     RCLCPP_FATAL_STREAM(this->get_logger(), "No torque setpoint found for " << msg->joint_name << ". No torque weight will be applied.");
//                     return;
//                 };

//                 jointInfo.torque_weight = msg->torque_weight;
//                 jointInfo.position_weight = msg->position_weight;
//                 found_joint = true;
//                 break;
//             }
//         }
//         if (!found_joint) {
//             RCLCPP_WARN(this->get_logger(), "Couldn't find: ", msg->joint_name);
//         }
//     }


//     // Method to publish the measured torque
//     // Gets called by the hardware interface's write() method
//     void publish_measured_torque() {

//         control_msgs::msg::JointTrajectoryControllerState torque_points;
//         trajectory_msgs::msg::JointTrajectoryPoint point;

//         for (march_hardware_interface::JointInfo& jointInfo : *joints_info_) {
//             torque_points.joint_names.push_back(jointInfo.name);
//             point.effort.push_back(jointInfo.torque);
//         }

//         point.time_from_start.sec = 0;
//         point.time_from_start.nanosec = 8 * 1e6;
//         torque_points.actual = point;
//         torque_points.header.stamp = this->get_clock()->now();

//         m_measured_torque_publisher->publish(torque_points);
//     }

//     // Method to calculate the average torque and use this to set the target_position 
//     // TODO: check where the msg is coming from > from the rqt input device, should ask the girls about this
//     void averageTorqueCallback(std_msgs::msg::Int32::SharedPtr msg) {

//         std::map<std::string, std::vector<float>> measured_torques;

//         for (const auto& joint : *joints_info_) {
//             measured_torques.emplace(joint.name, std::vector<float>());
//         }
            
//         auto start = std::chrono::steady_clock::now();
//         auto work_duration = std::chrono::seconds {msg->data};

//         while ((std::chrono::steady_clock::now() - start) < work_duration) {
//             for (const auto& joint : *joints_info_) {
//                 measured_torques[joint.name].push_back(joint.torque);
//             }
//         }

//         for (march_hardware_interface::JointInfo& jointInfo : *joints_info_) {
//             const std::vector<float>& total = measured_torques[jointInfo.name];
//             const float avg_torque = std::accumulate(total.begin(), total.end(), 0.0) / total.size();
                
//             RCLCPP_INFO_STREAM(this->get_logger(),
//                 "joint " << jointInfo.name << " has average torque " << avg_torque << " measured over " << total.size()
//                          << " values");

//             jointInfo.target_torque = avg_torque;
//         }
//     }

//     std::vector<JointInfo>* joints_info_;
//     std::optional<float> delta; // TODO: check if this is used

// private:
//     rclcpp::Subscription<march_shared_msgs::msg::FuzzyWeights>::SharedPtr m_weight_subscription;
//     rclcpp::Publisher<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr m_measured_torque_publisher;
//     rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr m_measure_torque_subscription;
// };