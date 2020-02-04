#include "gmock/gmock.h"  // Brings in Google Mock.
#include "march_hardware/Joint.h"

class MockJoint : public march::Joint
{
public:
  MOCK_METHOD0(getAngle, float());
  MOCK_METHOD0(getTemperature, float());
  MOCK_METHOD0(getTemperatureGESSlaveIndex, int());
  MOCK_METHOD0(getIMotionCubeSlaveIndex, int());
  MOCK_METHOD0(hasIMotionCube, bool());
  MOCK_METHOD0(hasTemperatureGES, bool());

  MOCK_METHOD1(hasTemperatureGES, void(double effort));
};
