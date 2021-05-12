#include "image_debuger.hpp"
#include <opencv2/opencv.hpp>
#include <ros/package.h>
#include <common/datadef.hpp>
#include <seuimage/ransac.hpp>

using namespace cv;
using namespace std;
using namespace seuimage;

ImageDebuger::ImageDebuger()
{
    height_ = 480;
    width_ = 640;
    std::string cfgpath = ros::package::getPath("config")+"/conf/";
    string yolocfg = cfgpath+"vision/robocup.cfg";
    string wcfg = cfgpath+"vision/robocup.weights";
    if(!NetworkInit(yolocfg, wcfg))
    {
        cout<<"NetworkInit failed"<<endl;
        exit(0);
    }
    srcLab = new ImageLabel(width_, height_);
    dstLab = new ImageLabel(width_, height_);
    curr_index_ = 0;
    infoLab = new QLabel("0/0");
    statusBar()->addWidget(infoLab);

    QHBoxLayout *imageLayout = new QHBoxLayout;
    imageLayout->addWidget(srcLab);
    imageLayout->addWidget(dstLab);

    ballpostBox = new QCheckBox("Ball_Post");
    fieldBox = new QCheckBox("Field");
    btnLoad = new QPushButton("Open Folder");
    btnLast = new QPushButton("<--");
    btnNext = new QPushButton("-->");
    btnAutoPlay = new QPushButton("<>");
    autoSaveBox = new QCheckBox("Save");
    btnAutoPlay->setFixedWidth(120);
    delayEdit = new QLineEdit("1000");
    delayEdit->setPlaceholderText("delay of auto play (ms)");
    delayEdit->setFixedWidth(50);
    QHBoxLayout *ctrlLayout = new QHBoxLayout;
    ctrlLayout->addWidget(ballpostBox);
    ctrlLayout->addWidget(fieldBox);
    ctrlLayout->addWidget(btnLoad);
    ctrlLayout->addWidget(btnLast);
    ctrlLayout->addWidget(btnNext);
    ctrlLayout->addWidget(btnAutoPlay);
    ctrlLayout->addWidget(delayEdit);
    ctrlLayout->addWidget(autoSaveBox);

    frmSlider = new QSlider(Qt::Horizontal);
    frmSlider->setEnabled(false);
    fieldSlider = new QSlider(Qt::Horizontal);
    fieldSlider->setRange(10, 100);
    ftLabel = new QLabel();
    QHBoxLayout *frdLayout = new QHBoxLayout();
    frdLayout->addWidget(fieldSlider);
    frdLayout->addWidget(ftLabel);
    sld1 = new Slider(Qt::Horizontal, "1");
    sld2 = new Slider(Qt::Horizontal, "2");
    sld3 = new Slider(Qt::Horizontal, "3");

    QVBoxLayout *mainLayout = new QVBoxLayout();
    mainLayout->addLayout(imageLayout);
    mainLayout->addLayout(ctrlLayout);
    mainLayout->addWidget(frmSlider);
    //mainLayout->addLayout(frdLayout);
    /*
    mainLayout->addWidget(sld1);
    mainLayout->addWidget(sld2);
    mainLayout->addWidget(sld3);
    */
    

    QWidget *mainWidget  = new QWidget();
    mainWidget->setLayout(mainLayout);
    this->setCentralWidget(mainWidget);

    timer = new QTimer;
    connect(timer, &QTimer::timeout, this, &ImageDebuger::procTimer);
    connect(btnLoad, &QPushButton::clicked, this, &ImageDebuger::procBtnLoad);
    connect(btnLast, &QPushButton::clicked, this, &ImageDebuger::procBtnLast);
    connect(btnNext, &QPushButton::clicked, this, &ImageDebuger::procBtnNext);
    connect(btnAutoPlay, &QCheckBox::clicked, this, &ImageDebuger::procBtnAuto);
    connect(frmSlider, &QSlider::valueChanged, this, &ImageDebuger::procFrmSlider);
    connect(fieldSlider, &QSlider::valueChanged, this, &ImageDebuger::procSlider);
    connect(sld1, &Slider::changed, this, &ImageDebuger::procSlider);
    connect(sld2, &Slider::changed, this, &ImageDebuger::procSlider);
    connect(sld3, &Slider::changed, this, &ImageDebuger::procSlider);
    connect(srcLab, &ImageLabel::shot, this, &ImageDebuger::procShot);
    Reset();
}

bool ImageDebuger::NetworkInit(std::string cfg, std::string wts)
{
    yolo.gpu_index = 0;
    yolo = parse_network_cfg_custom((char*)(cfg.c_str()), 1, 0);
    load_weights(&yolo, const_cast<char *>((char*)(wts.c_str())));
    set_batch_network(&yolo, 1);
    fuse_conv_batchnorm(yolo);
    calculate_binary_weights(yolo);
    srand((unsigned int)time(0));
    bool ret;
    ret = netMat.create(yolo.h, yolo.w, 3);
    if(!ret) return false;

    ret = netfMat.create(yolo.h, yolo.w, 3);
    if(!ret) return false;

    ret = yoloMat.create(yolo.h, yolo.w, 3);
    if(!ret) return false;
    
    cudaSetDevice(0);
    return true;
}

void ImageDebuger::showSrc()
{
    Mat bgr = imread(String((curr_dir_+image_names_.at(curr_index_-1)).toStdString()));
    cvtColor(bgr, rgb_src_, CV_BGR2RGB);
    height_ = bgr.size().height;
    width_ = bgr.size().width;
    srcLab->set_size(bgr.size().width, bgr.size().height);
    dstLab->set_size(bgr.size().width, bgr.size().height);
    rgb_dst_ = rgb_src_.clone();
    cvtColor(bgr, hsv_src_, CV_BGR2HSV_FULL);
    QImage srcImage(rgb_src_.data, rgb_src_.cols, rgb_src_.rows, QImage::Format_RGB888);
    srcLab->set_image(srcImage);
}

void ImageDebuger::showDst()
{
    if(rgb_dst_.empty()) return;
    QImage dstImage(rgb_dst_.data, rgb_dst_.cols, rgb_dst_.rows, QImage::Format_RGB888);
    dstLab->set_image(dstImage);
    Mat bgr;
    cvtColor(rgb_dst_, bgr, CV_RGB2BGR);
    if(autoSaveBox->isChecked())
    {
        imwrite(image_names_[curr_index_-1].split("/").back().toStdString(), bgr);
    }
}

void ImageDebuger::BallAndPostDet()
{
    CudaMatC rgbMat;
    rgbMat.create(height_, width_, 3);
    rgbMat.upload(rgb_dst_);
    Resize(rgbMat, netMat);
    RGB8uTo32fNorm(netMat, netfMat);
    PackedToPlanar(netfMat, yoloMat);
    layer l = yolo.layers[yolo.n - 1];
    network_predict1(yolo, yoloMat.data());
    int nboxes = 0;
    float nms = 0.45;
    detection *dets = get_network_boxes(&yolo, width_, height_, 
                    0.5, 0.5, 0, 1, &nboxes, 0);

    if (nms)
    {
        do_nms_sort(dets, nboxes, l.classes, nms);
    }

    std::vector<DetObject> ball_dets, post_dets;
    ball_dets.clear();
    post_dets.clear();

    for (int i = 0; i < nboxes; i++)
    {
        if (dets[i].prob[0] > dets[i].prob[1])
        {
            if (dets[i].prob[0] >= 0.6)
            {
                int bx = (dets[i].bbox.x - dets[i].bbox.w / 2.0) * width_;
                int by = (dets[i].bbox.y - dets[i].bbox.h / 2.0) * height_;
                int bw = dets[i].bbox.w * width_, bh = dets[i].bbox.h * height_ + 1;
                ball_dets.push_back(DetObject(0, dets[i].prob[0], bx, by, bw, bh));
            }
        }
        else
        {
            if (dets[i].prob[1] >= 0.5)
            {
                int px = (dets[i].bbox.x - dets[i].bbox.w / 2.0) * width_;
                int py = (dets[i].bbox.y - dets[i].bbox.h / 2.0) * height_;
                int pw = dets[i].bbox.w * width_, ph = dets[i].bbox.h * height_;
                post_dets.push_back(DetObject(1, dets[i].prob[1], px, py, pw, ph));
            }
        }
    }
    std::sort(ball_dets.rbegin(), ball_dets.rend());
    std::sort(post_dets.rbegin(), post_dets.rend());
    free_detections(dets, nboxes);
    for(auto &det:ball_dets)
    {
        cv::rectangle(rgb_dst_, cv::Point(det.x, det.y), cv::Point(det.x + det.w,
                    det.y + det.h), cv::Scalar(255, 0, 0), 2);
    }
    for(auto &det:post_dets)
    {
        cv::rectangle(rgb_dst_, cv::Point(det.x, det.y), cv::Point(det.x + det.w,
                    det.y + det.h), cv::Scalar(0, 0, 255), 2);
    }
}

void ImageDebuger::procImage(const unsigned int &index)
{
    if (index < 1 || index > image_names_.size())
    {
        return;
    }

    curr_index_ = index;
    infoLab->setText(QString::number(curr_index_) + "/" + QString::number(image_names_.size()) 
                    + "   " + image_names_.at(curr_index_-1));
    frmSlider->setValue(index);
    showSrc();
    
    //label color
    for(size_t i=0; i<rgb_dst_.rows; i++)
    {
        for(size_t j=0; j<rgb_dst_.cols; j++)
        {
            Vec3b tmp = hsv_src_.at<Vec3b>(i, j);
            // if((tmp[0]>=H_low && tmp[0]<= H_high)
            //     && (tmp[1]>=S_low && tmp[1]<= S_high)
            //     && (tmp[2]>=V_low && tmp[2]<= V_high))
            if(tmp[0]>=255.0/8 && tmp[0]<= 255.0*0.5 && tmp[1]>255*0.2 && tmp[2]>255*0.1)
            {
                rgb_dst_.at<Vec3b>(i, j) = Vec3b(0, 255, 0);
            }
            else if(tmp[1]<255*0.4 && tmp[2]>255*0.3)
            {
                rgb_dst_.at<Vec3b>(i, j) = Vec3b(255, 255, 255);
            }
        }
    }

    if (ballpostBox->isChecked()) //ball and post detection
    {
        BallAndPostDet();
    }
    if (fieldBox->isChecked()) //field detection
    {
        vector<Point2f> points;
        for(size_t j=0; j<rgb_dst_.cols; j++)
        {
            size_t row = 0;
            for(int i=0; i<rgb_dst_.rows; i++)
            {
                Vec3b tmp = hsv_src_.at<Vec3b>(i, j);
                if((tmp[0]>=H_low && tmp[0]<= H_high)
                    && (tmp[1]>=S_low && tmp[1]<= S_high)
                    && (tmp[2]>=V_low && tmp[2]<= V_high))
                {
                    //row = i;
                }
                else
                {
                    if((i-row)<fieldSlider->value())
                        row = i;
                }
            }
            rgb_dst_.at<Vec3b>(row, j) = Vec3b(255, 0, 0);
            //field.at<uint8_t>(Point(j, row)) = 255;
            points.push_back(Point2f(j, row));
        }
        Vec4f _line;
        //HoughLinesP(field, lines, 1, CV_PI/180, sld1->value(), sld2->value(), sld3->value());
        int numForEstimate = 5;
        float successProbability = 0.999f;
        float maxOutliersPercentage = 0.2; 
        Ransac(points, _line, numForEstimate, successProbability, maxOutliersPercentage);
        float a = _line[1]/_line[0];
        float b = _line[3] - a*_line[2];
        int x1=0, x2=640;
        int y1 = a*x1+b, y2=a*x2+b;
        line(rgb_dst_, Point(x1, y1), Point(x2, y2), Scalar(255, 0, 0));
    }
    showDst();
}

void ImageDebuger::procBtnLast()
{
    curr_index_--;

    if (curr_index_ < 1)
    {
        curr_index_ = image_names_.size();
    }

    procImage(curr_index_);
}

void ImageDebuger::procBtnNext()
{
    curr_index_++;

    if (curr_index_ > image_names_.size())
    {
        if(btnAutoPlay->text() == "Stop")
        {
            procBtnAuto();
            curr_index_ = image_names_.size();
            return;
        }
        else curr_index_ = 1;
    }

    procImage(curr_index_);
}

void ImageDebuger::procBtnLoad()
{
    timer->stop();
    curr_dir_ = QFileDialog::getExistingDirectory(nullptr, "Open image directory", QDir::homePath())+"/";
    if (curr_dir_.isEmpty())
    {
        return;
    }

    QDir dir(curr_dir_);
    QStringList nameFilters;
    nameFilters << "*.jpg" << "*.png";
    image_names_.clear();
    image_names_ = dir.entryList(nameFilters, QDir::Files|QDir::Readable, QDir::Name);
    if (!image_names_.empty())
    {
        frmSlider->setEnabled(true);
        frmSlider->setMinimum(1);
        frmSlider->setMaximum(image_names_.size());
        procImage(1);
    }
}

void ImageDebuger::procBtnAuto()
{
    if (btnAutoPlay->text() == "Play")
    {
        int delay = delayEdit->text().toInt();

        if (delay < 10)
        {
            delay = 10;
        }

        timer->start(delay);
        btnAutoPlay->setText("Stop");
    }
    else
    {
        timer->stop();
        btnAutoPlay->setText("Play");
    }
}

void ImageDebuger::procShot(QRect rect)
{
    if(rgb_src_.empty()) return;
    calHSVThresh(rect);
    procImage(curr_index_);
}

void ImageDebuger::calHSVThresh(QRect rect)
{
    if (rect.width() > 10 && rect.height() > 10)
    {
        int x, y, w, h;
        x = rect.left();
        y = rect.top();
        w = rect.width();
        h = rect.height();
        
        if (x + w < width_ && y + h < height_)
        {
            for(int i=y; i<=y+h; i++)
            {
                for(int j=x; j<=x+w; j++)
                {
                    Vec3b tmp = hsv_src_.at<Vec3b>(i, j);
                    H.push_back(tmp[0]);
                    S.push_back(tmp[1]);
                    V.push_back(tmp[2]);
                }
            }
            sort(H.begin(), H.end());
            sort(S.begin(), S.end());
            sort(V.begin(), V.end());
            size_t n = H.size();
            size_t o = n*0.05;
            vector<int> NH,NS,NV;
            for(int i=o; i<n-o; i++)
            {
                NH.push_back(H[i]);
                NS.push_back(S[i]);
                NV.push_back(V[i]);
            }
            int MH=Mean(NH), MS=Mean(NS), MV=Mean(NV);
            float SH=0.0, SS=0.0, SV=0.0;
            size_t nn = NH.size();
            for(size_t i=0; i<nn; i++)
            {
                SH += pow(NH[i]-MH, 2);
                SS += pow(NS[i]-MS, 2);
                SV += pow(NV[i]-MV, 2);
            }
            SH = sqrt(SH/nn);
            SS = sqrt(SS/nn);
            SV = sqrt(SV/nn);
            H_low = MH-3*SH; H_high = MH+3*SH;
            S_low = MS-4*SS; S_high = MS+4*SS;
            V_low = MV-5*SV; V_high = MV+5*SV;
            printf("H: (%d, %d)\nS: (%d, %d)\nV: (%d, %d)\n", 
                H_low, H_high, S_low, S_high, V_low, V_high);
        }
    }
    else{
        Reset();
    }
}

void ImageDebuger::Reset()
{
    H_low = 255; H_high = 255;
    S_low = 255; S_high = 255;
    V_low = 255; V_high = 255;
    H.clear();
    S.clear();
    V.clear();
}

void ImageDebuger::procFrmSlider(int v)
{
    procImage(v);
}

void ImageDebuger::procSlider(int v)
{
    //ftLabel->setText(QString::number(v));
    procImage(curr_index_);
}


void ImageDebuger::procTimer()
{
    procBtnNext();
}
