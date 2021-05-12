#include "ImageLabel.hpp"

using namespace std;

ImageLabel::ImageLabel(int w, int h)
{
    this->setFixedSize(w, h);
    this->setStyleSheet("QLabel{background:black}");
    drawing = false;
}

void ImageLabel::set_image(QImage im)
{
    image = QImage(im);
    this->update();
}

void ImageLabel::set_size(int w, int h)
{
    this->setFixedSize(w, h);
}

void ImageLabel::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton)
    {
        drawing = true;
        startPoint = event->pos();
    }
}

void ImageLabel::mouseMoveEvent(QMouseEvent *event)
{
    if (drawing)
    {
        endPoint.setX(event->pos().x());
        endPoint.setY(event->pos().y());
        shotRect = QRect(startPoint, endPoint);
        this->update();
    }
}

void ImageLabel::mouseReleaseEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton)
    {
        shotRect = QRect(startPoint, endPoint);
        emit shot(shotRect);
        drawing = false;
        this->update();
        shotRect.setWidth(0);
        shotRect.setHeight(0);
    }
}

void ImageLabel::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);

    if (!image.isNull())
    {
        QPixmap pixmap = QPixmap::fromImage(image);
        painter.drawPixmap(0, 0, image.width(), image.height(), pixmap);

        if (drawing)
        {
            painter.setPen(QPen(Qt::red, 4, Qt::SolidLine, Qt::FlatCap));
            painter.drawRect(shotRect);
            painter.setPen(QPen(Qt::red, 2, Qt::SolidLine, Qt::FlatCap));
            painter.drawLine(shotRect.topLeft(), shotRect.bottomRight());
            painter.drawLine(shotRect.topRight(), shotRect.bottomLeft());
        }
    }
}


Slider::Slider(Qt::Orientation orientation, QString name, QWidget *parent)
    : QWidget(parent)
{
    valueLabel = new QLabel();
    if(orientation == Qt::Horizontal)
    {
        QHBoxLayout *layout = new QHBoxLayout();
        slider = new QSlider(Qt::Horizontal);
        layout->addWidget(new QLabel(name));
        layout->addWidget(slider);
        layout->addWidget(valueLabel);
        setLayout(layout);
    }
    else
    {
        QVBoxLayout *layout = new QVBoxLayout();
        slider = new QSlider(Qt::Vertical);
        layout->addWidget(valueLabel);
        layout->addWidget(slider);
        layout->addWidget(new QLabel(name));
        setLayout(layout);
    }
    slider->setRange(0, 200);
    connect(slider, &QSlider::valueChanged, this, &Slider::procValueChanged);
}

void Slider::setRange(int mini, int maxi)
{
    slider->setRange(mini, maxi);
}

void Slider::procValueChanged(int v)
{
    valueLabel->setText(QString::number(v));
    emit changed(v);
}