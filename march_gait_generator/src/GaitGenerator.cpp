/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <QSlider>
#include <QLabel>
#include <QGridLayout>
#include <QVBoxLayout>
#include <QLineEdit>

#include <tf/transform_broadcaster.h>

#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"

#include <robot_state_publisher/robot_state_publisher.h>

#include <march_gait_generator/GaitGenerator.h>
#include <march_gait_generator/widgets/FancySlider.h>
#include <march_gait_generator/UIBuilder.h>

// BEGIN_TUTORIAL
// Constructor for GaitGenerator.  This does most of the work of the class.
GaitGenerator::GaitGenerator(Gait gait, QWidget* parent )
        : gait(gait), QWidget( parent )
{
    initUrdf();

    keyFrameCounter = 0;
    main_layout_ = new QHBoxLayout;


    // Set the top-level layout for this GaitGenerator widget.
    setLayout( main_layout_ );

    this->loadGaitEditor();
}

GaitGenerator::GaitGenerator( QWidget* parent): GaitGenerator(Gait(), parent){
}

// Destructor.
GaitGenerator::~GaitGenerator()
{
//    delete manager_;
}

void GaitGenerator::loadGaitEditor(){
    for( int i = 0; i<this->gait.poseList.size(); i++){
        this->addPoseView(this->gait.poseList.at(i));
    }
}

void GaitGenerator::addPoseView(PoseStamped poseStamped){
    // Construct and lay out render panel.
    rviz::RenderPanel* renderPanel = new rviz::RenderPanel();

    renderPanel->getViewController();

    QGridLayout* poseEditor = this->createPoseEditor(poseStamped.pose);
//    controls_layout->addWidget( renderPanel, 0, 0, 1, 5);

    main_layout_->addLayout(poseEditor);


    rviz::VisualizationManager* manager = new rviz::VisualizationManager( renderPanel );
    renderPanel ->initialize( manager->getSceneManager(), manager );
    manager->initialize();
    manager->startUpdate();

    manager->setFixedFrame("world");


    rviz::Display* grid = manager->createDisplay( "rviz/Grid", appendKeyFrameCounter("grid") , true );
    ROS_ASSERT( grid != NULL );

    // Configure the GridDisplay the way we like it.
    grid->subProp( "Line Style" )->setValue( "Lines" );
    grid->subProp( "Color" )->setValue( QColor(Qt::yellow) );
    grid->subProp( "Plane" )->setValue( "XZ");
    grid->subProp( "Cell Size" )->setValue( 0.1);
    grid->subProp( "Plane Cell Count" )->setValue( 20);
    rviz::Display* robotmodel = manager->createDisplay( "rviz/RobotModel", appendKeyFrameCounter("robotmodel"), true );
    manager->createDisplay( "rviz/TF", "sd", true );
//    robotmodel->subProp("TF Prefix")->setValue(QString("frame").append(QString::number(keyFrameCounter)));
}

QGridLayout* GaitGenerator::createPoseEditor(Pose pose){
    QGridLayout* poseEditor = new QGridLayout();
    int frameIndex = keyFrameCounter;

//    for( auto it = jointMap.begin(); it != jointMap.end(); ++it) {
    for(int i =0; i< pose.name.size(); i++){
        std::string jointName = pose.name.at(i);
        auto test = model_->getJoint(jointName);

        ROS_INFO_STREAM(test);
//        double lowerLimit = limits->lower;
//        double upperLimit = limits->upper;
//
//        QGridLayout* jointSetting = createJointSetting()
//
//        if ( lowerLimit == 0 and upperLimit == 0){
//            ROS_WARN("Skipping joint %s as limits are 0.", it->first.c_str());
//            continue;
//        }
//
//        connect(fancySlider, &FancySlider::valueChanged, [=]() {
//            float value = fancySlider->value();
//            actualValue->setText(QString::number(value/fancySlider->MULTIPLICATION_FACTOR));
//            publishKeyFrame(frameIndex, getJointStateFromKeyFrame(frameIndex));
//        });
//
//        connect(actualValue, &QLineEdit::editingFinished, [=]() {
//            QString text = actualValue->text();
//            bool succes;
//            float value = text.toFloat(&succes);
//            if(succes){
//                fancySlider->setValue(value*fancySlider->MULTIPLICATION_FACTOR);
//                publishKeyFrame(frameIndex, getJointStateFromKeyFrame(frameIndex));
//
//            } else {
//                ROS_WARN("Text %s is not a valid position.", text.toStdString().c_str());
//                actualValue->setText(QString::number(fancySlider->value()/fancySlider->MULTIPLICATION_FACTOR));
//            }
//        });
//
//        fancySlider->setValue(0);
//
//        gridRow += 2;
    }

    return poseEditor;
}


void GaitGenerator::initUrdf(){
    model_ = new urdf::Model();
    model_->initParam("robot_description");
}

QString GaitGenerator::appendKeyFrameCounter(const std::string& base){
    std::ostringstream os;
    os << base << keyFrameCounter;
    return QString::fromStdString(os.str());

}

void GaitGenerator::publishKeyFrame(int keyFrameIndex, sensor_msgs::JointState jointState) {
    std::string topic = QString("frame").append(QString::number(keyFrameIndex)).append("/joint_states").toStdString();
    joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);

    joint_pub.publish(jointState);
}

sensor_msgs::JointState GaitGenerator::getJointStateFromKeyFrame(int keyFrameIndex) {
    QString keyName = QString("frame").append(QString::number(keyFrameIndex));
    QGridLayout* layout = main_layout_->findChild<QGridLayout*>(keyName);
    QList<QLineEdit *> values = layout->findChildren<QLineEdit*>();

    sensor_msgs::JointState jointState;
    jointState.header.stamp = ros::Time::now();
    for (int i = 0; i < layout->count(); ++i)
    {
        QLineEdit *widget = dynamic_cast<QLineEdit *>(layout->itemAt(i)->widget());

        if (widget != NULL)
        {
            double position = widget->text().toDouble();
            std::string jointName = widget->objectName().replace(keyName, "").toStdString();
            jointState.name.push_back(jointName);
            jointState.position.push_back(position);
        }
    }
    return jointState;
}
