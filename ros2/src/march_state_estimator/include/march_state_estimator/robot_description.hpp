#ifndef MARCH_STATE_ESTIMATOR__ROBOT_DESCRIPTION_HPP_
#define MARCH_STATE_ESTIMATOR__ROBOT_DESCRIPTION_HPP_

#include <string>
#include <vector>

#include "march_state_estimator/robot_node.hpp"
#include "urdf/model.h"

class RobotDescription
{
public:
  RobotDescription() = default;
  ~RobotDescription();

  void parseURDF(const std::string & urdf_path);
  void configureRobotNodes();
  void parameterizeRobotNode();
  std::vector<std::string> getNodeNames();
  std::vector<std::string> getParentNames();
  std::vector<RobotNode*> findNodes(std::vector<std::string> names);
  std::vector<Eigen::Vector3d> getNodesPosition(std::vector<std::string> joint_names, std::vector<double> joint_angles);
  std::vector<Eigen::Matrix3d> getNodesRotation(std::vector<std::string> joint_names, std::vector<double> joint_angles);

private:
    
    urdf::Model m_urdf_model;
    std::vector<RobotNode*> m_robot_nodes;

};

#endif  // MARCH_STATE_ESTIMATOR__ROBOT_DESCRIPTION_HPP_