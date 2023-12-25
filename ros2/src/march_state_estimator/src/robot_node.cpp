#include "march_state_estimator/robot_node.hpp"

#include "rclcpp/rclcpp.hpp"
#include <sstream>
#include "math.h"

void RobotNode::setParent(RobotNode * parent)
{
    parent_ = parent;
}

void RobotNode::addChild(RobotNode * child)
{
    children_.push_back(child);
}

void RobotNode::setOriginPosition(const Eigen::Vector3d & position)
{
    Eigen::MatrixXd position_matrix(WORKSPACE_DIM, 1);
    position_matrix = position;
    origin_position_vector_ = utilConvertEigenToGiNaC(position_matrix);
}

void RobotNode::setOriginRotation(const Eigen::Matrix3d & rotation)
{
    origin_rotation_matrix_ = utilConvertEigenToGiNaC(rotation);
}

std::string RobotNode::getName() const
{
    return name_;
}

uint64_t RobotNode::getId() const
{
    return id_;
}

uint64_t RobotNode::getId(const std::string & name) const
{
    return std::hash<std::string>{}(name);
}

char RobotNode::getType() const
{
    return type_;
}

GiNaC::matrix RobotNode::getOriginPosition() const
{
    return origin_position_vector_;
}

GiNaC::matrix RobotNode::getOriginRotation() const
{
    return origin_rotation_matrix_;
}

GiNaC::symbol RobotNode::getJointAngle() const
{
    return joint_angle_;
}

RobotNode * RobotNode::getParent() const
{
    return parent_;
}

std::vector<RobotNode*> RobotNode::getChildren() const
{
    return children_;
}

Eigen::Vector3d RobotNode::getGlobalPosition(std::vector<std::string> joint_names, std::vector<double> joint_angles) const
{
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Test 1, %s", name_.c_str());
    GiNaC::lst substitutions = GiNaC::lst();
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Test 2");
    // for (long unsigned int i = 0; i < joint_angles_.size(); i++)
    // {
    //     // std::stringstream ss;
    //     // ss << "Joint angle: " << joint_angles_[i];
    //     // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), ss.str().c_str());
    //     substitutions.append(joint_angles_[i] == M_PI_4f64);
    // }
    // for (long unsigned int i = 0; i < joint_names.size(); i++)
    // {
    //     // TODO: This is a hack. The joint angle should be a symbol, not a string.
    //     // TODO: Replace joint_angles_ with vector of RobotJoint objects.
    //     std::stringstream q_name;
    //     for (auto & joint_angle : joint_angles_)
    //     {
    //         q_name.str("");
    //         q_name << joint_angle;
    //         std::string j_name = "q_{" + joint_names[i] + "}";
    //         // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Joint angle: %s", q_name.str().c_str());
    //         // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Joint name: %f", j_name);
    //         if (q_name.str() == j_name)
    //         {
    //             substitutions.append(joint_angle == joint_angles[i]);
    //             // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Joint angle: %s", q_name.str().c_str());
    //             // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Joint angle value: %f", joint_angles[i]);
    //         }
    //     }
    // }

    // for (long unsigned int i = 0; i < joint_names.size(); i++)
    // {
    //     // TODO: This is a hack. The joint angle should be a symbol, not a string.
    //     // TODO: Replace joint_angles_ with vector of RobotJoint objects.
    //     std::stringstream q_name;
    //     for (auto & joint_angle : joint_angles_)
    //     {
    //         q_name.str("");
    //         q_name << joint_angle;
    //         std::string j_name = "q_{" + joint_names[i] + "}";
    //         if (q_name.str() == j_name)
    //         {
    //             substitutions.append(joint_angle == joint_angles[i]);
    //         }
    //     }
    // }

    for (auto & joint_node : joint_nodes_)
    {
        for (long unsigned int i = 0; i < joint_names.size(); i++)
        {
            if (joint_node->getName() == joint_names[i])
            {
                substitutions.append(joint_node->getJointAngle() == joint_angles[i]);
                break;
            }
        }
    }
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Test 3");

    Eigen::Vector3d global_position = Eigen::Vector3d::Zero();
    for (int i = 0; i < WORKSPACE_DIM; i++)
    {
        // std::stringstream ss;
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Test 3.%d.1", i);
        // ss << global_position_vector_(i, 0);
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), ss.str().c_str());
        GiNaC::ex global_position_component = GiNaC::evalf(global_position_vector_(i, 0).subs(substitutions));
        // ss.str("");
        // ss << global_position_component;
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Test 3.%d.2", i);
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), ss.str().c_str());
        global_position(i) = GiNaC::ex_to<GiNaC::numeric>(global_position_component).to_double();
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Test 3.%d.3", i);
    }
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Test 4");

    return global_position;
}

Eigen::Matrix3d RobotNode::getGlobalRotation() const
{
    double dummy_joint_angle = M_PI;
    GiNaC::lst substitutions = GiNaC::lst();
    for (long unsigned int i = 0; i < joint_angles_.size(); i++)
    {
        substitutions.append(joint_angles_[i] == 1.0);
    }

    Eigen::Matrix3d global_rotation = Eigen::Matrix3d::Zero();
    for (int i = 0; i < WORKSPACE_DIM; i++)
    {
        for (int j = 0; j < WORKSPACE_DIM; j++)
        {
            GiNaC::ex global_rotation_component = GiNaC::evalf(global_rotation_matrix_(i, j).subs(substitutions));
            global_rotation(i, j) = GiNaC::ex_to<GiNaC::numeric>(global_rotation_component).to_double();
        }
    }

    return global_rotation;
}

std::vector<GiNaC::symbol> RobotNode::getJointAngles() const
{
    std::vector<GiNaC::symbol> joint_angles;
    RobotNode * parent = parent_;
    while (parent != nullptr)
    {
        if (parent->getType() == 'J')
        {
            joint_angles.push_back(parent->getJointAngle());
        }
        parent = parent->getParent();
    }
    return joint_angles;
}

std::vector<RobotNode*> RobotNode::getJointNodes() const
{
    std::vector<RobotNode*> joint_nodes;
    RobotNode * parent = parent_;
    while (parent != nullptr)
    {
        if (parent->getType() == 'J')
        {
            joint_nodes.push_back(parent);
        }
        parent = parent->getParent();
    }
    return joint_nodes;
}

void RobotNode::expressKinematics()
{
    GiNaC::matrix global_position = expressGlobalPosition();
    GiNaC::matrix global_rotation = expressGlobalRotation();

    for (unsigned int i = 0; i < global_position.rows(); i++)
    {
        global_position(i, 0) = global_position(i, 0).simplify_indexed();
    }
    // for (unsigned int i = 0; i < global_rotation.rows(); i++)
    // {
    //     for (unsigned int j = 0; j < global_rotation.cols(); j++)
    //     {
    //         global_rotation(i, j) = global_rotation(i, j).simplify_indexed();
    //     }
    // }

    // joint_angles_ = getJointAngles();
    joint_nodes_ = getJointNodes();
    global_position_vector_ = global_position;
    global_rotation_matrix_ = global_rotation;

    // std::stringstream temp_stringstream;

    // temp_stringstream << "Global position: " 
    //     << global_position_vector_(0, 0) << ", " 
    //     << global_position_vector_(1, 0) << ", " 
    //     << global_position_vector_(2, 0);
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), temp_stringstream.str().c_str());

    // temp_stringstream.str("");
    // temp_stringstream << "Global rotation: " 
    //     << global_rotation_matrix_(0, 0) << ", " 
    //     << global_rotation_matrix_(0, 1) << ", " 
    //     << global_rotation_matrix_(0, 2) << ", " 
    //     << global_rotation_matrix_(1, 0) << ", " 
    //     << global_rotation_matrix_(1, 1) << ", " 
    //     << global_rotation_matrix_(1, 2) << ", " 
    //     << global_rotation_matrix_(2, 0) << ", " 
    //     << global_rotation_matrix_(2, 1) << ", " 
    //     << global_rotation_matrix_(2, 2);
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), temp_stringstream.str().c_str());
}

GiNaC::matrix RobotNode::expressGlobalPosition() const
{
    GiNaC::matrix global_position(WORKSPACE_DIM, 1);
    global_position = origin_position_vector_;
    RobotNode * parent = parent_;
    while (parent != nullptr)
    {
        global_position = parent->getOriginRotation().mul(global_position);
        global_position = global_position.add(parent->getOriginPosition());

        parent = parent->getParent();
    }
    return global_position;
}

GiNaC::matrix RobotNode::expressGlobalRotation() const
{
    GiNaC::matrix global_rotation(WORKSPACE_DIM, WORKSPACE_DIM);
    global_rotation = origin_rotation_matrix_;
    RobotNode * parent = parent_;
    while (parent != nullptr)
    {
        global_rotation = parent->getOriginRotation().mul(global_rotation);

        parent = parent->getParent();
    }
    return global_rotation;
}

GiNaC::matrix RobotNode::utilConvertEigenToGiNaC(const Eigen::MatrixXd & matrix) const
{
    GiNaC::matrix ginac_matrix(matrix.rows(), matrix.cols());
    for (unsigned int i = 0; i < matrix.rows(); i++)
    {
        for (unsigned int j = 0; j < matrix.cols(); j++)
        {
            ginac_matrix(i, j) = matrix(i, j);
        }
    }
    return ginac_matrix;
}

Eigen::Matrix3d RobotNode::utilConvertGiNaCToEigen(const GiNaC::matrix & matrix) const
{
    Eigen::Matrix3d eigen_matrix = Eigen::Matrix3d::Zero();
    for (unsigned int i = 0; i < matrix.rows(); i++)
    {
        for (unsigned int j = 0; j < matrix.cols(); j++)
        {
            eigen_matrix(i, j) = GiNaC::ex_to<GiNaC::numeric>(matrix(i, j)).to_double();
        }
    }
    return eigen_matrix;
}