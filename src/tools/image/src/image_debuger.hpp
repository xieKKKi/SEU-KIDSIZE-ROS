#ifndef __IMAGE_DEBUGER_HPP
#define __IMAGE_DEBUGER_HPP

#include <bits/stdc++.h>
#include <QtWidgets>
#include "ImageLabel.hpp"
#include <opencv2/opencv.hpp>
#include <darknet/network.h>
#include <darknet/parser.h>
#include <seuimage/seuimage.hpp>
#include <ros/ros.h>

class ImageDebuger: public QMainWindow
{
    Q_OBJECT
public:
    ImageDebuger();
public slots:
    void procTimer();
    void procBtnLoad();
    void procBtnLast();
    void procBtnNext();
    void procBtnAuto();
    void procShot(QRect rect);
    void procFrmSlider(int v);
    void procSlider(int v);
    
private:
    void procImage(const unsigned int &index);
    void showSrc();
    void showDst();
    void Reset();
    void calHSVThresh(QRect rect);

    bool NetworkInit(std::string cfg, std::string wts);
    void BallAndPostDet();
    
    template<typename T>
    T Mean(std::vector<T> &data)
    {
        int sum=0;
        for(size_t i=0; i<data.size(); i++)
            sum+=data[i];
        return static_cast<T>(sum/data.size());
    }
    
    std::vector<int> H, S, V;
    int H_low, H_high, S_low, S_high, V_low, V_high;

    QPushButton *btnLoad, *btnNext, *btnLast, *btnAutoPlay;
    QCheckBox *autoSaveBox;
    ImageLabel *srcLab, *dstLab;
    QLabel *infoLab;
    QTimer *timer;
    QSlider *frmSlider, *fieldSlider;
    Slider *sld1, *sld2, *sld3;
    QLabel *ftLabel;
    QLineEdit *delayEdit;
    QCheckBox *ballpostBox, *fieldBox;
    unsigned int curr_index_;
    QString curr_dir_;
    QStringList image_names_;
    cv::Mat rgb_src_, hsv_src_, rgb_dst_;
    int width_;
    int height_;

    network yolo;
    seuimage::CudaMatC netMat;
    seuimage::CudaMatF netfMat;
    seuimage::CudaMatF yoloMat;
};



#endif
