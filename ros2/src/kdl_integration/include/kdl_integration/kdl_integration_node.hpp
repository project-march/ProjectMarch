#ifndef KDL_INTEGRATION_NODE_HPP
#define KDL_INTEGRATION_NODE_HPP

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "kdl_parser/kdl_parser.hpp"

class KDLIntegrationNode : public rclcpp::Node
{
  public:
    KDLIntegrationNode();
    ~KDLIntegrationNode() = default;

  private:
    KDL::Tree kdl_tree_;
};

#endif  // KDL_INTEGRATION_NODE_HPP