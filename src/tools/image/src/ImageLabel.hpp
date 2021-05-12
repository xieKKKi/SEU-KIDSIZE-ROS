#ifndef __IMAGE_LABEL_HPP
#define __IMAGE_LABEL_HPP

#include <QtWidgets>

class ImageLabel: public QLabel
{
    Q_OBJECT
public:
    ImageLabel(int w = 640, int h = 480);
    void set_image(QImage im);
    void set_size(int w, int h);
protected:
    void paintEvent(QPaintEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);

signals:
    void shot(QRect);
private:
    bool drawing;
    QPoint startPoint;
    QPoint endPoint;
    QRect shotRect;
    QImage image;
};

class Slider: public QWidget
{
    Q_OBJECT 
public:
    Slider(Qt::Orientation orientation, QString name, QWidget *parent = Q_NULLPTR);
    void setRange(int mini, int maxi);
    int value() const
    {
        return slider->value();
    }
public slots:
    void procValueChanged(int v);

signals:
    void changed(int v);

private:
    QLabel *valueLabel;
    QSlider *slider;
};



#endif
