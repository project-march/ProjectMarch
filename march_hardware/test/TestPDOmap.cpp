// Copyright 2018 Project March.

#include "gtest/gtest.h"
#include <gmock/gmock.h>

#include <march_hardware/PDOmap.h>
#include <march_hardware/March4.h>

class PDOTest : public ::testing::Test
{
protected:
};

TEST_F(PDOTest, sortPDOmap)
{
  march4cpp::PDOmap pdoMapMISO = march4cpp::PDOmap();
  pdoMapMISO.addObject(march4cpp::IMCObjectName::StatusWord);
  pdoMapMISO.addObject(march4cpp::IMCObjectName::ActualPosition);
  std::map<march4cpp::IMCObjectName, int> misoByteOffsets = pdoMapMISO.map(1, march4cpp::dataDirection::miso);

  ASSERT_EQ(0, misoByteOffsets[march4cpp::IMCObjectName::ActualPosition]);
  ASSERT_EQ(4, misoByteOffsets[march4cpp::IMCObjectName::StatusWord]);
}

TEST_F(PDOTest, multipleAddObjects)
{
  march4cpp::PDOmap pdoMapMISO = march4cpp::PDOmap();

  pdoMapMISO.addObject(march4cpp::IMCObjectName::ActualPosition);
  pdoMapMISO.addObject(march4cpp::IMCObjectName::StatusWord);
  pdoMapMISO.addObject(march4cpp::IMCObjectName::StatusWord);
  std::map<march4cpp::IMCObjectName, int> misoByteOffsets = pdoMapMISO.map(1, march4cpp::dataDirection::miso);
  ASSERT_EQ(2, misoByteOffsets.size());
}

TEST_F(PDOTest, ObjectCounts)
{
    march4cpp::PDOmap pdoMapMISO = march4cpp::PDOmap();
    pdoMapMISO.addObject(march4cpp::IMCObjectName::CurrentLimit);
    std::map<march4cpp::IMCObjectName, int> misoByteOffsets = pdoMapMISO.map(1, march4cpp::dataDirection::miso);
    ASSERT_EQ(1, misoByteOffsets.count(march4cpp::IMCObjectName::CurrentLimit));
    ASSERT_EQ(0, misoByteOffsets.count(march4cpp::IMCObjectName::DCLinkVoltage));
}

TEST_F(PDOTest, exceedMaxSize)
{
  march4cpp::PDOmap pdoMapMISO = march4cpp::PDOmap();
  pdoMapMISO.addObject(march4cpp::IMCObjectName::StatusWord);
  pdoMapMISO.addObject(march4cpp::IMCObjectName::ActualPosition);
  pdoMapMISO.addObject(march4cpp::IMCObjectName::ControlWord);
  pdoMapMISO.addObject(march4cpp::IMCObjectName::DetailedErrorRegister);
  pdoMapMISO.addObject(march4cpp::IMCObjectName::MotionErrorRegister);
  pdoMapMISO.addObject(march4cpp::IMCObjectName::TargetPosition);
  pdoMapMISO.addObject(march4cpp::IMCObjectName::DCLinkVoltage);
  pdoMapMISO.addObject(march4cpp::IMCObjectName::MotorPosition);
  pdoMapMISO.addObject(march4cpp::IMCObjectName::ActualTorque);
  pdoMapMISO.addObject(march4cpp::IMCObjectName::QuickStopDeceleration);
  pdoMapMISO.addObject(march4cpp::IMCObjectName::CurrentLimit);
  pdoMapMISO.addObject(march4cpp::IMCObjectName::DriveTemperature);
  pdoMapMISO.addObject(march4cpp::IMCObjectName::QuickStopOption);
  ASSERT_THROW(pdoMapMISO.map(1, march4cpp::dataDirection::miso), std::exception);
}
