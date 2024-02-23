/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#ifndef MARCH_STATE_ESTIMATOR__ROBOT_ZMP_HPP_
#define MARCH_STATE_ESTIMATOR__ROBOT_ZMP_HPP_

#include "march_state_estimator/robot_node.hpp"

#define XY_DIM 2

class RobotZMP : public RobotNode {
public:
    RobotZMP();
    ~RobotZMP() = default;

    Eigen::Vector3d getGlobalPosition(const JointNameToValueMap& joint_positions) const override;
    Eigen::MatrixXd getGlobalPositionJacobian(const JointNameToValueMap& joint_positions) const override;
    Eigen::MatrixXd getFootBoundingPolygonUnionCoefficients() const;

    void setNormOrder(const unsigned int& norm_order);
    void setStanceLeg(const uint8_t& stance_leg);
    void setFootPositions(const Eigen::Vector3d& left_foot_position, const Eigen::Vector3d& right_foot_position);
    void setInertialOrientation(const Eigen::Quaterniond& inertial_orientation);

    // TODO: Create friend class for testing
    Eigen::MatrixXd getFootBoundingPolygon(const Eigen::Vector2d& foot_position) const;
    Eigen::MatrixXd computePolygonUnionCoefficients(
        const Eigen::MatrixXd& polygon1, const Eigen::MatrixXd& polygon2) const;
    Eigen::Vector3d normalizeZmp(const Eigen::Vector3d& zmp) const;
    double normalizeZmpByAxis(const double& position, const Eigen::Vector3d& axis) const;

private:
    // TODO: To be YAML parsed
    const double FOOT_WIDTH = 0.125; // [m]
    const double FOOT_LENGTH = 0.30; // [m]
    const double FOOT_WIDTH_2 = 0.0675; // 0.125 / 2[m]
    const double FOOT_LENGTH_2 = 0.15; // 0.30 / 2[m]
    const uint8_t NUM_POLYGON_VERTICES = 4;
    const uint8_t NUM_POLYGON_AXES = 2;
    const uint8_t NUM_FOOTS = 2;

    unsigned int m_norm_order;
    Eigen::Vector2d m_left_foot_position;
    Eigen::Vector2d m_right_foot_position;
    Eigen::MatrixXd m_foot_bounding_polygon;
    Eigen::Quaterniond m_inertial_orientation;
};

#endif // MARCH_STATE_ESTIMATOR__ROBOT_ZMP_HPP_