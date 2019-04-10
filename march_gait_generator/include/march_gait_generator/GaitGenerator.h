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
#ifndef GAITGENERATOR_H
#define GAITGENERATOR_H

#include <QWidget>
#include <QHBoxLayout>

#include <urdf/model.h>

#include <sensor_msgs/JointState.h>
#include <march_gait_generator/Gait.h>


namespace rviz
{
    class Display;
    class RenderPanel;
    class VisualizationManager;
}

// BEGIN_TUTORIAL
// Class "GaitGenerator" implements the top level widget for this example.
class GaitGenerator: public QWidget
{
Q_OBJECT

public:
    int keyFrameCounter;
    Gait gait;

    explicit GaitGenerator(QWidget* parent = 0 );
    explicit GaitGenerator(Gait gait,  QWidget* parent = 0);
    virtual ~GaitGenerator();

private Q_SLOTS:
    void addKeyFramePanel();
    QGridLayout* createKeyFrameSettings();
    void initUrdf();
    void addKeyFrameUI();

    QString appendKeyFrameCounter(const std::string& base);

private:
    ros::NodeHandle n;
    ros::Publisher joint_pub;

    QHBoxLayout* main_layout_;
    urdf::Model* model_;
    void publishKeyFrame(int keyFrameIndex, sensor_msgs::JointState jointState);

    sensor_msgs::JointState getJointStateFromKeyFrame(int keyFrameIndex);

};
// END_TUTORIAL
#endif // GAITGENERATOR_H
