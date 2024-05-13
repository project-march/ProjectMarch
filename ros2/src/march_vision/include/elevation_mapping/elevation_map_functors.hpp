/*
 * P.Fankhauser, M.Bloesch, and M.Hutter, 
 * "Probabilistic Terrain Mapping for Mobile Robots with Uncertain Localization",
 * in IEEE Robotics and Automation Letters (RA-L)
 *
 * MARCH functionality and ROS2 porting by Alexander Andonov
 */

#pragma once

namespace elevation_mapping {

template <typename Scalar>
struct VarianceClampOperator {
  VarianceClampOperator(const Scalar& minVariance, const Scalar& maxVariance) : minVariance_(minVariance), maxVariance_(maxVariance) {}
  const Scalar operator()(const Scalar& x) const {
    return x < minVariance_ ? minVariance_ : (x > maxVariance_ ? std::numeric_limits<float>::infinity() : x);
  }
  Scalar minVariance_, maxVariance_;
};

}  // namespace elevation_mapping
