#ifndef MARCH_STATE_ESTIMATOR__ROBOT_DESCRIPTION_HPP_
#define MARCH_STATE_ESTIMATOR__ROBOT_DESCRIPTION_HPP_

#include <string>
#include <vector>

#include "march_state_estimator/robot_node.hpp"
#include "urdf/model.h"

class RobotDescription
{
public:
  RobotDescription();
  ~RobotDescription() = default;

  void parse(const std::string & urdf_path);  

private:
    
    urdf::Model urdf_model_;

};

#endif  // MARCH_STATE_ESTIMATOR__ROBOT_DESCRIPTION_HPP_