/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#include "march_state_estimator/robot_description/robot_zmp.hpp"
#include <boost/algorithm/clamp.hpp>
#include <cmath>

RobotZMP::RobotZMP()
{
    m_name = "zmp";
    m_type = 'Z';
    setNormOrder(2); // TODO: Make this a parameter
}

Eigen::Vector3d RobotZMP::getGlobalPosition(const JointNameToValueMap& joint_positions) const
{
    Eigen::Vector3d global_position = Eigen::Map<Eigen::Vector3d>(
        evaluateExpression(m_global_position_expressions, m_joint_nodes, WORKSPACE_DIM, 1, joint_positions).data());
    global_position.noalias() = m_inertial_orientation * global_position;
    return normalizeZmp(global_position);
}

Eigen::MatrixXd RobotZMP::getGlobalPositionJacobian(const JointNameToValueMap& joint_positions) const
{
    Eigen::MatrixXd global_position_jacobian = evaluateExpression(
        m_global_position_jacobian_expressions, m_joint_nodes, WORKSPACE_DIM, m_joint_nodes.size(), joint_positions);
    global_position_jacobian.noalias() = m_inertial_orientation.toRotationMatrix() * global_position_jacobian;
    return global_position_jacobian.block(0, 0, XY_DIM, global_position_jacobian.cols());
}

Eigen::MatrixXd RobotZMP::getFootBoundingPolygonUnionCoefficients() const
{
    return m_foot_bounding_polygon;
}

void RobotZMP::setNormOrder(const unsigned int& norm_order)
{
    m_norm_order = 2 * norm_order - 1;
    if (m_norm_order < 1) {
        m_norm_order = 1;
    }
}

void RobotZMP::setFootPositions(const Eigen::Vector3d& left_foot_position, const Eigen::Vector3d& right_foot_position)
{
    m_left_foot_position = left_foot_position.head<XY_DIM>();
    m_right_foot_position = right_foot_position.head<XY_DIM>();
}

void RobotZMP::setStanceLeg(const uint8_t& stance_leg)
{
    Eigen::MatrixXd polygon1 = getFootBoundingPolygon(m_left_foot_position);
    Eigen::MatrixXd polygon2 = getFootBoundingPolygon(m_right_foot_position);
    if ((~stance_leg & 0b10) >> 1) {
        polygon2 = getFootBoundingPolygon(m_left_foot_position);
    } else if ((~stance_leg & 0b01)) {
        polygon1 = getFootBoundingPolygon(m_right_foot_position);
    }
    m_foot_bounding_polygon = computePolygonUnionCoefficients(polygon1, polygon2);
}

void RobotZMP::setInertialOrientation(const Eigen::Quaterniond& inertial_orientation)
{
    m_inertial_orientation = inertial_orientation;
}

Eigen::MatrixXd RobotZMP::getFootBoundingPolygon(const Eigen::Vector2d& foot_position) const
{
    Eigen::MatrixXd polygon(NUM_POLYGON_VERTICES, NUM_POLYGON_AXES);

    polygon.row(0).noalias() = foot_position + Eigen::Vector2d(FOOT_LENGTH_2, FOOT_WIDTH_2);
    polygon.row(1).noalias() = foot_position + Eigen::Vector2d(-FOOT_LENGTH_2, FOOT_WIDTH_2);
    polygon.row(2).noalias() = foot_position + Eigen::Vector2d(-FOOT_LENGTH_2, -FOOT_WIDTH_2);
    polygon.row(3).noalias() = foot_position + Eigen::Vector2d(FOOT_LENGTH_2, -FOOT_WIDTH_2);

    return polygon;
}

Eigen::MatrixXd RobotZMP::computePolygonUnionCoefficients(
    const Eigen::MatrixXd& polygon1, const Eigen::MatrixXd& polygon2) const
{
    // Compute the coefficients of the union of two polygons.
    const uint8_t NUM_COLS = 3;

    Eigen::MatrixXd union_polygon(XY_DIM, NUM_COLS);
    union_polygon(0, 0) = std::min(polygon1.col(0).minCoeff(), polygon2.col(0).minCoeff());
    union_polygon(0, 1) = std::max(polygon1.col(0).maxCoeff(), polygon2.col(0).maxCoeff());
    union_polygon(1, 0) = std::min(polygon1.col(1).minCoeff(), polygon2.col(1).minCoeff());
    union_polygon(1, 1) = std::max(polygon1.col(1).maxCoeff(), polygon2.col(1).maxCoeff());
    union_polygon(0, 2) = Eigen::Block<Eigen::MatrixXd, 1, 2>(union_polygon, 0, 0).mean();
    union_polygon(1, 2) = Eigen::Block<Eigen::MatrixXd, 1, 2>(union_polygon, 1, 0).mean();
    return union_polygon;
}

Eigen::Vector3d RobotZMP::normalizeZmp(const Eigen::Vector3d& zmp) const
{
    Eigen::Vector3d normalized_position = Eigen::Vector3d::Zero();

    normalized_position.x() = normalizeZmpByAxis(zmp.x(), m_foot_bounding_polygon.row(0));
    normalized_position.y() = normalizeZmpByAxis(zmp.y(), m_foot_bounding_polygon.row(1));
    return normalized_position;
}

double RobotZMP::normalizeZmpByAxis(const double& position, const Eigen::Vector3d& axis) const
{
    double norm = std::pow((2 * position - axis.x() - axis.y()) / std::abs(axis.y() - axis.x()), m_norm_order);
    return boost::algorithm::clamp(norm, -1.0, 1.0);
}