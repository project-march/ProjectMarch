// Copyright 2019 Project March.

#include "gtest/gtest.h"
#include <gmock/gmock.h>

#include <march_gait_generator/GaitGenerator.h>
#include <QApplication>

/**
 * The main method which runs all the tests
 */
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  auto res = RUN_ALL_TESTS();

    QApplication app( argc, argv );

    GaitGenerator* gaitGenerator = new GaitGenerator();
    gaitGenerator->show();

    app.exec();

  return res;
}
