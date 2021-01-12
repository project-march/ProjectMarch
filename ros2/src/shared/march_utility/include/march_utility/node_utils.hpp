// Copyright 2020 Project March.
#ifndef MARCH_NODE_UTILS_H
#define MARCH_NODE_UTILS_H

namespace node_utils {
    // Get the Joint names from the robot information node.
    std::vector<std::string> get_joint_names(rclcpp::Node& node);
}

#endif  // MARCH_NODE_UTILS_H
