#include "march_state_estimator/robot_node.hpp"

void RobotNode::setParent(RobotNode * parent)
{
    parent_ = parent;
}

void RobotNode::addChild(RobotNode * child)
{
    children_.push_back(child);
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

RobotNode * RobotNode::getParent() const
{
    return parent_;
}

std::vector<RobotNode*> RobotNode::getChildren() const
{
    return children_;
}