#ifndef KDL_INTEGRATION_NODE_HPP
#define KDL_INTEGRATION_NODE_HPP

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>

class KDLIntegrationNode : public rclcpp::Node
{
  public:
    KDLIntegrationNode();
    ~KDLIntegrationNode() = default;

  private:
    KDL::Tree kdl_tree_;
    KDL::Chain kdl_chain_left_;
    KDL::Chain kdl_chain_right_;
};

#endif  // KDL_INTEGRATION_NODE_HPP