
#ifndef MARCH_GAIT_GENERATOR_UIBUILDER_H
#define MARCH_GAIT_GENERATOR_UIBUILDER_H


#include <QtWidgets/QGridLayout>
#include <QFile>
#include <string>
#include <march_gait_generator/widgets/FancySlider.h>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>

QGroupBox* createJointSetting(std::string jointName, const urdf::JointLimitsSharedPtr& limits, float position, float velocity){
    QGroupBox* jointSetting = new QGroupBox();
    QGridLayout* jointSettingLayout = new QGridLayout();


    jointSetting->setTitle(QString::fromStdString(jointName));
    jointSetting->setLayout(jointSettingLayout);

    FancySlider* positionSlider = new FancySlider( Qt::Horizontal);
    positionSlider->setMinimum( limits->lower * positionSlider->MULTIPLICATION_FACTOR);
    positionSlider->setMaximum( limits->upper * positionSlider->MULTIPLICATION_FACTOR);
    positionSlider->setObjectName("PositionSlider");
    positionSlider->setFixedWidth(200);

    FancySlider* velocitySlider = new FancySlider( Qt::Horizontal);
    velocitySlider->setMinimum( -limits->velocity * velocitySlider->MULTIPLICATION_FACTOR);
    velocitySlider->setMaximum( limits->velocity * velocitySlider->MULTIPLICATION_FACTOR);
    velocitySlider->setObjectName("VelocitySlider");
    velocitySlider->setFixedWidth(200);

    QLineEdit* positionValue = new QLineEdit();
    positionValue->setText(QString::number(position));
    positionValue->setObjectName("PositionValue");


    QLineEdit* velocityValue = new QLineEdit();
    velocityValue->setText(QString::number(velocity));
    velocityValue->setObjectName("VelocityValue");

    QLabel* positionLabel = new QLabel("Position");
    QLabel* velocityLabel = new QLabel("Velocity");

    jointSettingLayout->addWidget(positionLabel, 0, 0);
    jointSettingLayout->addWidget(positionSlider, 0, 1, 1, 2);
    jointSettingLayout->addWidget(positionValue, 0, 3);

    jointSettingLayout->addWidget(velocityLabel, 1, 0);
    jointSettingLayout->addWidget(velocitySlider, 1, 1, 1, 2);
    jointSettingLayout->addWidget(velocityValue, 1, 3);
    return jointSetting;
}

QGroupBox* createFooter(std::string name, std::string comment, std::string version){
    QGroupBox* footer = new QGroupBox();
    QVBoxLayout* footerLayout = new QVBoxLayout();
    footer->setLayout(footerLayout);

    footerLayout->addWidget(new QLabel("Gait:"));
    footerLayout->addWidget(new QLineEdit(QString::fromStdString(name)));
    footerLayout->addWidget(new QLabel("Description:"));
    footerLayout->addWidget(new QLabel("Version:"));

    return footer;

}

#endif //MARCH_GAIT_GENERATOR_UIBUILDER_H
