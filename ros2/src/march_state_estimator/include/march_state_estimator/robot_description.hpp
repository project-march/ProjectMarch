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

private:
    
    urdf::Model urdf_model_;
    std::vector<RobotNode*> robot_nodes_;

};

#endif  // MARCH_STATE_ESTIMATOR__ROBOT_DESCRIPTION_HPP_