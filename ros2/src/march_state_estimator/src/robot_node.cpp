#include "march_state_estimator/robot_node.hpp"

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
    origin_position_ = position;
}

void RobotNode::setOriginRotation(const Eigen::Matrix3d & rotation)
{
    origin_rotation_ = rotation;
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

Eigen::Vector3d RobotNode::getOriginPosition() const
{
    return origin_position_;
}

Eigen::Matrix3d RobotNode::getOriginRotation() const
{
    return origin_rotation_;
}

RobotNode * RobotNode::getParent() const
{
    return parent_;
}

std::vector<RobotNode*> RobotNode::getChildren() const
{
    return children_;
}

Eigen::Vector3d RobotNode::getGlobalPosition() const
{
    Eigen::Vector3d global_position = origin_position_;
    RobotNode * parent = parent_;
    while (parent != nullptr)
    {
        global_position = parent->getOriginRotation() * global_position + parent->getOriginPosition();
        parent = parent->getParent();
    }
    return global_position;
}

Eigen::Matrix3d RobotNode::getGlobalRotation() const
{
    Eigen::Matrix3d global_rotation = origin_rotation_;
    RobotNode * parent = parent_;
    while (parent != nullptr)
    {
        global_rotation = parent->getOriginRotation() * global_rotation;
        parent = parent->getParent();
    }
    return global_rotation;
}