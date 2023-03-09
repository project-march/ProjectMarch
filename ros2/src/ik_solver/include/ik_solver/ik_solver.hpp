//
// Created by Tessel & Jack March 6th 2023
//

#include <Eigen/Core>
#include <vector>
#include <iostream>

#ifndef IK_SOLVER_JACOBIAN_GENERATOR_HPP
#define IK_SOLVER_JACOBIAN_GENERATOR_HPP

class JacobianGenerator {
private: 
    int number_of_links = 7;
    std::vector<double> masses = std::vector<double>(number_of_links) ;
    std::vector<double> lengths = std::vector<double>(number_of_links);
    std::vector<double> distances = std::vector<double>(number_of_links);
    
public: 
    std::vector<double> joint_angles = std::vector<double>(number_of_links);

};


#endif // IK_SOLVER_JACOBIAN_GENERATOR_HPP