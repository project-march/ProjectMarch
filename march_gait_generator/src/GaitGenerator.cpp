
#include <QSlider>
#include <QLabel>
#include <QGridLayout>
#include <QDialog>
#include <QVBoxLayout>

#include <tf/transform_broadcaster.h>

#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"

#include <robot_state_publisher/robot_state_publisher.h>

#include <march_gait_generator/GaitGenerator.h>
#include <march_gait_generator/UIBuilder.h>

GaitGenerator::GaitGenerator(Gait gait, QWidget* parent )
        : gait(gait), QWidget( parent )
{
    this->loadUrdf();

    this->initLayout();
    this->loadGaitEditor();
}

GaitGenerator::GaitGenerator( QWidget* parent){
    this->loadUrdf();

    // Create an empty Gait based on the joints found in the URDF.
    std::vector<std::string> joints;
    for( auto it = model_->joints_.begin(); it != model_->joints_.end(); ++it) {
        std::string jointName = it->first;
        joints.push_back(jointName);
    }
    this->gait = Gait();
    gait.addPoseStamped(PoseStamped(joints));
    gait.addPoseStamped(PoseStamped(joints));
    this->initLayout();
    this->loadGaitEditor();

}

// Destructor.
GaitGenerator::~GaitGenerator()
{
//    delete manager_;
}

void GaitGenerator::initLayout(){
    keyFrameCounter = 0;
    main_layout_ = new QHBoxLayout();


    // Set the top-level layout for this GaitGenerator widget.
    this->setLayout( main_layout_ );


    QWidget *poseViewList_ = new QWidget;
    QScrollArea *scrollArea_ = new QScrollArea;
    QVBoxLayout *layout = new QVBoxLayout(poseViewList_);
    scrollArea_->setWidget(poseViewList_);
    scrollArea_->setWidgetResizable(true);

    main_layout_->addWidget(poseViewList_);

}

void GaitGenerator::loadGaitEditor(){
    for( int i = 0; i<this->gait.poseList.size(); i++){
        this->addPoseView(this->gait.poseList.at(i), i);
    }
}

void GaitGenerator::addPoseView(PoseStamped poseStamped, int index){
    // Construct and lay out render panel.
    rviz::RenderPanel* renderPanel = new rviz::RenderPanel();

    renderPanel->getViewController();

    QGroupBox* poseEditor = this->createPoseEditor(poseStamped.pose, index);
    poseEditor->setTitle(QString("Pose"));
    poseEditor->layout()->addWidget(renderPanel);

    poseViewList_->layout()->addWidget(poseEditor);


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

QGroupBox* GaitGenerator::createPoseEditor(Pose pose, int poseIndex){
    QGroupBox* poseEditor = new QGroupBox();
    QGridLayout* poseEditorLayout = new QGridLayout();
    poseEditor->setLayout(poseEditorLayout);

    for(int i =0; i< pose.name.size(); i++){
        std::string jointName = pose.name.at(i);
        auto joint = model_->getJoint(jointName);
        ROS_ASSERT_MSG(joint != nullptr, "Joint %s does not exist in the robot description", jointName.c_str());

        if ( joint->limits->lower == 0 and joint->limits->upper == 0){
            ROS_WARN("Skipping joint %s as limits are 0.", jointName.c_str());
            continue;
        }

        QGroupBox* jointSetting = createJointSetting(jointName, joint->limits, pose.getJointPosition(jointName), pose.getJointVelocity(jointName));

        poseEditorLayout->addWidget(jointSetting, i, 0);

        auto positionSlider = jointSetting->findChild<FancySlider*>("PositionSlider");
        auto positionValue = jointSetting->findChild<QLineEdit*>("PositionValue");

        this->connectSlider(jointName, poseIndex, positionSlider, positionValue, PoseOption::position);

        auto velocitySlider = jointSetting->findChild<FancySlider*>("VelocitySlider");
        auto velocityValue = jointSetting->findChild<QLineEdit*>("VelocityValue");

        this->connectSlider(jointName, poseIndex, velocitySlider, velocityValue, PoseOption::velocity);

//
//        fancySlider->setValue(0);
//
//        gridRow += 2;
    }

    return poseEditor;
}


void GaitGenerator::loadUrdf(){
    model_ = new urdf::Model();
    model_->initParam("robot_description");
}

QString GaitGenerator::appendKeyFrameCounter(const std::string& base){
    std::ostringstream os;
    os << base << keyFrameCounter;
    return QString::fromStdString(os.str());

}

void GaitGenerator::publishPose(int poseIndex) {
    std::string topic = QString("pose").append(QString::number(poseIndex)).append("/joint_states").toStdString();
    joint_pub = n.advertise<sensor_msgs::JointState>(topic, 1);
    joint_pub.publish(this->gait.poseList.at(poseIndex).pose.toJointState());
}

void GaitGenerator::connectSlider(std::string jointName, int poseIndex, FancySlider *slider, QLineEdit *valueDisplay, PoseOption option) {
    // Have to create the connections here or else we can't connect it to the gait.
    connect(slider, &FancySlider::valueChanged, [=]() {
        float value = slider->value();
        valueDisplay->setText(QString::number(value/slider->MULTIPLICATION_FACTOR));
        if(option == PoseOption::position){
            this->gait.poseList.at(poseIndex).pose.setJointPosition(jointName, value/slider->MULTIPLICATION_FACTOR);
        } else if (option == PoseOption::velocity) {
            this->gait.poseList.at(poseIndex).pose.setJointVelocity(jointName, value/slider->MULTIPLICATION_FACTOR);
        } else {
            ROS_WARN("Pose option is not known");
        }
        this->publishPose(poseIndex);
    });

    connect(valueDisplay, &QLineEdit::editingFinished, [=]() {
        QString text = valueDisplay->text();
        bool success;
        float value = text.toFloat(&success);
        if(success){
            slider->setValue(value*slider->MULTIPLICATION_FACTOR);
            publishPose(poseIndex);

        } else {
            ROS_WARN("Text %s is not valid.", text.toStdString().c_str());
            valueDisplay->setText(QString::number(slider->value()/slider->MULTIPLICATION_FACTOR));
        }
    });
}
