#ifndef FANCYSLIDER_H
#define FANCYSLIDER_H

#include <QSlider>

class FancySlider : public QSlider
{
Q_OBJECT
public:
    int MULTIPLICATION_FACTOR = 1000000;
    explicit FancySlider(QWidget *parent = 0);
    explicit FancySlider(Qt::Orientation orientation, QWidget *parent = 0);

protected:
    virtual void sliderChange(SliderChange change);
};

#endif // FANCYSLIDER_H