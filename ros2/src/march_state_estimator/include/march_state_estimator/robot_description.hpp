/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#ifndef MARCH_STATE_ESTIMATOR__ROBOT_DESCRIPTION_HPP_
#define MARCH_STATE_ESTIMATOR__ROBOT_DESCRIPTION_HPP_

#include <string>
#include <vector>
#include <unordered_map>

#include "march_state_estimator/robot_node.hpp"
#include "urdf/model.h"
#include "yaml-cpp/yaml.h"

// struct RobotPartData
// {
//     std::string name;
//     std::string type;
//     std::vector<double> joint_axis;
//     GiNaC::matrix global_rotation;
//     GiNaC::matrix global_linear_position;
//     GiNaC::matrix global_linear_position_jacobian;
//     GiNaC::matrix global_rotation_jacobian;
// };

struct RobotPartData
{
    std::string name;
    std::string type;
    std::vector<double> joint_axis;
    std::vector<std::string> global_rotation;
    std::vector<std::string> global_linear_position;
    std::vector<std::string> global_linear_position_jacobian;
    std::vector<std::string> global_rotation_jacobian;
};

class RobotDescription
{
public:
  RobotDescription() = default;
  ~RobotDescription() = default;

  void parseURDF(const std::string & urdf_path);
  void parseYAML(const std::string & yaml_path);
  void createRobotPart(const RobotPartData & robot_part_data);
  void setRobotPart(const std::shared_ptr<RobotNode> robot_node, const RobotPartData & robot_part_data);
  void configureRobotNodes();
  std::vector<std::string> getAllNodeNames() const;
  std::vector<std::string> getAllParentNames() const;
  std::vector<std::shared_ptr<RobotNode>> findNodes(std::vector<std::string> names);
  std::vector<Eigen::Vector3d> getAllNodesPosition(const std::unordered_map<std::string, double> & joint_positions);
  std::vector<Eigen::Matrix3d> getAllNodesRotation(const std::unordered_map<std::string, double> & joint_positions);

private:
  std::vector<std::string> vectorizeExpressions(
    const YAML::Node & yaml_node, const unsigned int & rows, const unsigned int & cols);
  GiNaC::matrix convertToGiNaCMatrix(const std::vector<std::string> & expressions, 
    const unsigned int & rows, const unsigned int & cols);

  urdf::Model m_urdf_model;
  // std::vector<std::shared_ptr<RobotNode>> m_robot_nodes;
  std::vector<std::shared_ptr<RobotNode>> m_robot_nodes;
  std::vector<std::shared_ptr<RobotNode>> m_robot_node_ptrs;
  std::unordered_map<std::string, std::shared_ptr<RobotNode>> m_robot_nodes_map;
};

#endif  // MARCH_STATE_ESTIMATOR__ROBOT_DESCRIPTION_HPP_