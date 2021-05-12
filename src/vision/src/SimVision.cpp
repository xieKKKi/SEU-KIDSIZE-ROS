#include "camera/TpCamera.hpp"
#include <ros/ros.h>
#include <common/SetInt.h>
#include <common/ImageResult.h>
#include <common/ImageSnap.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>

using namespace common;

void Run(const ros::TimerEvent& event);

void ImagePublish(const cv::Mat &bgr);
bool SendTypeService(SetInt::Request &req, SetInt::Response &res);
bool ImageSnapService(ImageSnap::Request &req, ImageSnap::Response &res);

const int width=640;
const int height=480;
cv::Mat srcMat;

std::atomic_int send_type(0);
std::shared_ptr<Camera> camera;
ros::Publisher imagePublisher;
ros::Publisher resultPublisher;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision");
    ros::NodeHandle node;
    camera = std::make_shared<TpCamera>(node, "/sensor/image");
    camera->Open();
    imagePublisher = node.advertise<sensor_msgs::CompressedImage>("/result/vision/compressed", 1);
    resultPublisher = node.advertise<common::ImageResult>("/result/vision/imgproc", 1);
    ros::ServiceServer typeServer = node.advertiseService("/setting/sendtype", SendTypeService);
    ros::ServiceServer imgSnapSrv = node.advertiseService("/debug/image/snap", ImageSnapService);
    camera->Run();
    ros::Timer timer = node.createTimer(ros::Duration(0.05), Run);
    ros::spin();
    camera->Close();
    return 0;
}

cv::Point2i PointsMean(const std::vector<cv::Point2i> &points)
{
    uint32_t x=0, y=0;
    uint32_t n = points.size();
    if(n==0) return cv::Point2i(-1, -1);
    for (size_t i = 0; i < n; i++)
    {
        const cv::Point2i &p=points[i];
        x += p.x;
        y += p.y;
    }
    return cv::Point2i(x/n, y/n);
}

bool isBall(const cv::Vec3b &hsv)
{
    int H = hsv[0], S = hsv[1], V = hsv[2];
    if((H>=19 && H<=24) && (S>=239 && S<=250) 
        && (V>=221 && V<=263))
        return true;
    else return false;
}

bool isPost(const cv::Vec3b &hsv)
{
    int H = hsv[0], S = hsv[1], V = hsv[2];
    if((S>=171 && S<=220) 
        && (V>=216))
        return true;
    else return false;
}

void Run(const ros::TimerEvent& event)
{
    if(!ros::ok()) return;
    bool ret;
    uint32_t image_time;
    std::vector<cv::Point2i> ballPoints, postPoints[2];
    srcMat = camera->GetData(image_time).clone();
    cv::Mat hsvMat;
    int pidx = 0;
    int no_post = 0;
    cv::cvtColor(srcMat, hsvMat, CV_RGB2HSV);
    for (size_t j = 0; j < hsvMat.cols; j++)
    {
        bool has_post = false;
        for (size_t i = 0; i < hsvMat.rows; i++)
        {
            const cv::Vec3b hsv = hsvMat.at<cv::Vec3b>(i, j);
            if(isBall(hsv)){
                ballPoints.push_back(cv::Point2i(j, i));
            }
            if(isPost(hsv)){
                has_post = true;
                postPoints[pidx].push_back(cv::Point2i(j ,i));
            }
        }
        if(!postPoints[0].empty() && !has_post) no_post++;
        if(no_post>20) pidx = 1;
    }
    cv::Point2i ball = PointsMean(ballPoints);
    cv::Point2i post1 = PointsMean(postPoints[0]);
    cv::Point2i post2 = PointsMean(postPoints[1]);
    
    common::ImageResult imgResult;
    imgResult.stamp = image_time;
    if (ball.x!=-1)
    {
        imgResult.has_ball = true;
        imgResult.ball.x = ball.x;
        imgResult.ball.y = ball.y;
    }
    if (post1.x!=-1)
    {
        imgResult.has_post = true;
        ObjInfo tmp;
        tmp.x = post1.x;
        tmp.y = post1.y;
        imgResult.posts.push_back(tmp);
    }
    if (post2.x!=-1)
    {
        imgResult.has_post = true;
        ObjInfo tmp;
        tmp.x = post2.x;
        tmp.y = post2.y;
        imgResult.posts.push_back(tmp);
    }
    resultPublisher.publish(imgResult);

    if(send_type>0)
    {   
        if(send_type>1)
        {
            cv::circle(srcMat, ball, 5, cv::Scalar(0, 0, 255), 2);
            cv::circle(srcMat, post1, 5, cv::Scalar(255, 0, 0), 2);
            cv::circle(srcMat, post2, 5, cv::Scalar(255, 0, 0), 2);
        }
        ImagePublish(srcMat);
    }
}

void ImagePublish(const cv::Mat &rgb)
{
    cv::Mat bgr;
    cv::cvtColor(rgb, bgr, CV_RGB2BGR);
    sensor_msgs::CompressedImage image;
    std::vector<uint8_t> buf;
    cv::imencode(".jpg", bgr, buf);
    image.header.stamp = ros::Time::now();
    image.header.frame_id = "vision";
    image.format = "jpeg";
    image.data.resize(buf.size());
    memcpy(&image.data[0], &(buf[0]), buf.size());
    imagePublisher.publish(image);

}

bool SendTypeService(SetInt::Request &req, SetInt::Response &res)
{
    send_type = req.number;
    ros::param::set("image", req.number);
    return true;
}

bool ImageSnapService(ImageSnap::Request &req, ImageSnap::Response &res)
{
    return true;
}