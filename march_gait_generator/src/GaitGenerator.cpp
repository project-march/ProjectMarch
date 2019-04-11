// Copyright 2019 Project March

#include <QSlider>
#include <QLabel>
#include <QHeaderView>
#include <QGridLayout>

#include <QVBoxLayout>

#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/view_manager.h"
#include "rviz/display.h"

#include <march_gait_generator/GaitGenerator.h>
#include <march_gait_generator/UIBuilder.h>

GaitGenerator::GaitGenerator(Gait gait, QWidget* parent) : gait(gait), QWidget(parent)
{
  this->loadUrdf();

  this->initLayout();
  this->loadGaitEditor();
}

GaitGenerator::GaitGenerator(QWidget* parent)
{
  this->loadUrdf();

  // Create an empty Gait based on the joints found in the URDF.
  std::vector<std::string> joints;
  for (auto it = model_->joints_.begin(); it != model_->joints_.end(); ++it)
  {
    std::string jointName = it->first;
    joints.push_back(jointName);
  }
  this->gait = Gait();
  gait.addPoseStamped(PoseStamped(joints));
  gait.addPoseStamped(PoseStamped(joints));
  gait.addPoseStamped(PoseStamped(joints));
  gait.addPoseStamped(PoseStamped(joints));
  gait.addPoseStamped(PoseStamped(joints));
  this->initLayout();
  this->loadGaitEditor();
}

// Destructor.
GaitGenerator::~GaitGenerator()
{
  delete main_layout_;
  delete model_;
}

void GaitGenerator::initLayout()
{
  keyFrameCounter = 0;
  main_layout_ = new QHBoxLayout();

  this->setLayout(main_layout_);

  gaitEditor_ = new QTableWidget();
  gaitEditor_->setRowCount(3);
  gaitEditor_->setColumnCount(this->gait.poseList.size());

  gaitEditor_->setHorizontalScrollMode(QAbstractItemView::ScrollPerPixel);
  gaitEditor_->setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);
  gaitEditor_->verticalHeader()->setVisible(false);
  gaitEditor_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  gaitEditor_->setSelectionMode(QAbstractItemView::NoSelection);
  gaitEditor_->setShowGrid(false);
  gaitEditor_->horizontalHeader()->setSectionResizeMode(QHeaderView::Fixed);
  main_layout_->addWidget(gaitEditor_);
}

void GaitGenerator::loadGaitEditor()
{
  for (int i = 0; i < this->gait.poseList.size(); i++)
  {
    QGroupBox* poseView = this->createPoseView(gait.poseList.at(i), i);
    gaitEditor_->setCellWidget(0, i, poseView);
  }

  gaitEditor_->setSpan(2, 0, 1, this->gait.poseList.size());

  gaitEditor_->setCellWidget(2, 0, createFooter(this->gait.name, this->gait.comment, this->gait.version));
  gaitEditor_->resizeColumnsToContents();
  gaitEditor_->resizeRowsToContents();
}

QGroupBox* GaitGenerator::createPoseView(PoseStamped poseStamped, int index)
{
  // Construct and lay out render panel.
  rviz::RenderPanel* renderPanel = new rviz::RenderPanel();

  QGroupBox* poseEditor = this->createPoseEditor(poseStamped.pose, index);
  poseEditor->setTitle(QString("Pose"));
  poseEditor->layout()->addWidget(renderPanel);

  poseEditor->setFixedWidth(400);

  rviz::VisualizationManager* manager = new rviz::VisualizationManager(renderPanel);
  renderPanel->initialize(manager->getSceneManager(), manager);
  manager->initialize();
  manager->startUpdate();

  manager->getViewManager()->getCurrent()->subProp("Distance")->setValue("1.4");
  manager->getViewManager()->getCurrent()->subProp("Yaw")->setValue("1.57");
  manager->getViewManager()->getCurrent()->subProp("Pitch")->setValue("0");
  manager->getViewManager()->getCurrent()->subProp("Focal Point")->subProp("X")->setValue("0");
  manager->getViewManager()->getCurrent()->subProp("Focal Point")->subProp("Y")->setValue("0");
  manager->getViewManager()->getCurrent()->subProp("Focal Point")->subProp("Z")->setValue("0.7");

  QString fixedFrame = QString("pose").append(QString::number(index)).append("/world");

  manager->setFixedFrame(fixedFrame);

  rviz::Display* grid = manager->createDisplay("rviz/Grid", appendKeyFrameCounter("grid"), true);
  ROS_ASSERT(grid != NULL);

  grid->subProp("Line Style")->setValue("Lines");
  grid->subProp("Color")->setValue(QColor(Qt::yellow));
  grid->subProp("Plane")->setValue("XZ");
  grid->subProp("Cell Size")->setValue(0.1);
  grid->subProp("Plane Cell Count")->setValue(30);

  rviz::Display* robotmodel = manager->createDisplay("rviz/RobotModel", "robot_description", true);
  robotmodel->subProp("TF Prefix")->setValue(QString("pose").append(QString::number(index)));

  return poseEditor;
}

QGroupBox* GaitGenerator::createPoseEditor(Pose pose, int poseIndex)
{
  QGroupBox* poseEditor = new QGroupBox();
  QGridLayout* poseEditorLayout = new QGridLayout();
  poseEditor->setLayout(poseEditorLayout);

  for (int i = 0; i < pose.name.size(); i++)
  {
    std::string jointName = pose.name.at(i);
    auto joint = model_->getJoint(jointName);
    ROS_ASSERT_MSG(joint != nullptr, "Joint %s does not exist in the robot description", jointName.c_str());

    if (joint->limits == nullptr)
    {
      ROS_WARN("Skipping joint %s as limits are missing.", jointName.c_str());
      continue;
    }

    QGroupBox* jointSetting = createJointSetting(jointName, joint->limits, pose.getJointPosition(jointName),
                                                 pose.getJointVelocity(jointName));

    poseEditorLayout->addWidget(jointSetting, i, 0);

    auto positionSlider = jointSetting->findChild<FancySlider*>("PositionSlider");
    auto positionValue = jointSetting->findChild<QLineEdit*>("PositionValue");

    this->connectSlider(jointName, poseIndex, positionSlider, positionValue, PoseOption::position);

    auto velocitySlider = jointSetting->findChild<FancySlider*>("VelocitySlider");
    auto velocityValue = jointSetting->findChild<QLineEdit*>("VelocityValue");

    this->connectSlider(jointName, poseIndex, velocitySlider, velocityValue, PoseOption::velocity);
  }

  return poseEditor;
}

void GaitGenerator::loadUrdf()
{
  model_ = new urdf::Model();
  model_->initParam("robot_description");

  if (!kdl_parser::treeFromUrdfModel(*model_, kdlTree_))
  {
    ROS_ERROR("Failed to construct kdl tree");
  }

  this->robotStatePublisher = new robot_state_publisher::RobotStatePublisher(kdlTree_, *model_);
}

QString GaitGenerator::appendKeyFrameCounter(const std::string& base)
{
  std::ostringstream os;
  os << base << keyFrameCounter;
  return QString::fromStdString(os.str());
}

void GaitGenerator::publishPose(int poseIndex)
{
  std::string prefix = QString("pose").append(QString::number(poseIndex)).toStdString();
  std::string prefixOld = QString("pose").append(QString::number(poseIndex - 1)).toStdString();

  this->robotStatePublisher->publishFixedTransforms(prefix);
  this->robotStatePublisher->publishTransforms(this->gait.poseList.at(poseIndex).pose.toPositionMap(), ros::Time::now(),
                                               prefix);
}

void GaitGenerator::connectSlider(std::string jointName, int poseIndex, FancySlider* slider, QLineEdit* valueDisplay,
                                  PoseOption option)
{
  // Have to create the connections here or else we can't connect it to the gait.
  connect(slider, &FancySlider::valueChanged, [=]() {
    float value = slider->value();
    valueDisplay->setText(QString::number(value / slider->MULTIPLICATION_FACTOR));
    if (option == PoseOption::position)
    {
      this->gait.poseList.at(poseIndex).pose.setJointPosition(jointName, value / slider->MULTIPLICATION_FACTOR);
    }
    else if (option == PoseOption::velocity)
    {
      this->gait.poseList.at(poseIndex).pose.setJointVelocity(jointName, value / slider->MULTIPLICATION_FACTOR);
    }
    else
    {
      ROS_WARN("Pose option is not known");
    }
    this->publishPose(poseIndex);
  });

  connect(valueDisplay, &QLineEdit::editingFinished, [=]() {
    QString text = valueDisplay->text();
    bool success;
    float value = text.toFloat(&success);
    if (success)
    {
      slider->setValue(value * slider->MULTIPLICATION_FACTOR);
      publishPose(poseIndex);
    }
    else
    {
      ROS_WARN("Text %s is not valid.", text.toStdString().c_str());
      valueDisplay->setText(QString::number(slider->value() / slider->MULTIPLICATION_FACTOR));
    }
  });
}
