// Copyright 2019 Project March

#ifndef GAITGENERATOR_H
#define GAITGENERATOR_H

#include <QWidget>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QLineEdit>
#include <QTableWidget>

#include <urdf/model.h>

#include <robot_state_publisher/robot_state_publisher.h>

#include <kdl_parser/kdl_parser.hpp>

#include <sensor_msgs/JointState.h>

#include <march_gait_generator/widgets/FancySlider.h>
#include <march_gait_generator/Gait.h>

namespace rviz
{
class Display;
class RenderPanel;
class VisualizationManager;
}

enum PoseOption
{
  position,
  velocity
};

class GaitGenerator : public QWidget
{
  Q_OBJECT

public:
  int keyFrameCounter;
  Gait gait;

  explicit GaitGenerator(QWidget* parent = 0);
  explicit GaitGenerator(Gait gait, QWidget* parent = 0);
  virtual ~GaitGenerator();

  QString appendKeyFrameCounter(const std::string& base);
  void loadGaitEditor();
  QGroupBox* createKeyFrameSettings();

private Q_SLOTS:

private:
  ros::NodeHandle n;
  ros::Publisher joint_pub;
  robot_state_publisher::RobotStatePublisher* robotStatePublisher;
  QHBoxLayout* main_layout_;
  QTableWidget* gaitEditor_;
  urdf::Model* model_;
  KDL::Tree kdlTree_;

  void loadUrdf();
  void initLayout();

  void publishPose(int keyFrameIndex);

  QGroupBox* createPoseView(PoseStamped poseStamped, int index);
  QGroupBox* createPoseEditor(Pose pose, int poseIndex);

  void connectSlider(std::string jointName, int poseIndex, FancySlider* slider, QLineEdit* value, PoseOption option);
};
#endif  // GAITGENERATOR_H
