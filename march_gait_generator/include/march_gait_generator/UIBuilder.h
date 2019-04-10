
#ifndef MARCH_GAIT_GENERATOR_UIBUILDER_H
#define MARCH_GAIT_GENERATOR_UIBUILDER_H


#include <QtWidgets/QGridLayout>
#include <string>
#include <march_gait_generator/widgets/FancySlider.h>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>

QGridLayout* createJointSetting(std::string jointName, float minValue, float maxValue, float value){
    QGridLayout* jointSetting = new QGridLayout();

    FancySlider* fancySlider = new FancySlider( Qt::Horizontal);

    QLabel* minLabel = new QLabel(QString::fromStdString(std::to_string(minValue)), fancySlider);
    QLabel* maxLabel = new QLabel(QString::fromStdString(std::to_string(maxValue)), fancySlider);

    fancySlider->setMinimum( minValue * fancySlider->MULTIPLICATION_FACTOR);
    fancySlider->setMaximum( maxValue * fancySlider->MULTIPLICATION_FACTOR);

    QLineEdit* actualValue = new QLineEdit();
    // Set objectname to later retrieve the jointname.
    actualValue->setText(QString::number(0/fancySlider->MULTIPLICATION_FACTOR));

    QLabel* jointLabel = new QLabel(QString::fromStdString(jointName));


    jointSetting->addWidget(jointLabel, 0, 0);


    jointSetting->addWidget(actualValue, 0, 1, 1, 2);
    jointSetting->addWidget(minLabel, 1, 0);
    jointSetting->addWidget(fancySlider, 1, 1);
    jointSetting->addWidget(maxLabel, 1, 2);
    return jointSetting;
}

#endif //MARCH_GAIT_GENERATOR_UIBUILDER_H
