/** 
*  Copyright (C) 2021, SEU Robot Lab. All rights reserved.
*  @file     Vision.cpp                                                  
*  @brief    1. 识别球，发布识别结果
             2. 发布压缩图片
*  @author   刘川 谢家鹏
*  @todo     
*/
#include "camera/MvCamera.hpp"
#include "camera/UvCamera.hpp"
#include "camera/TpCamera.hpp"
#include <ros/ros.h>
#include <seuimage/seuimage.hpp>
#include <darknet/network.h>
#include <darknet/parser.h>
#include <config/basic_parser.hpp>
#include <common/datadef.hpp>
#include <std_srvs/SetBool.h>
#include <common/SetInt.h>
#include <common/ImageResult.h>
#include <common/ImageSnap.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>

using namespace seuimage;
using namespace common;

bool MallocMemory();
void NetworkInit(std::string cfg, std::string wts);
bool VisionInit();
bool CameraInit();
void Run(const ros::TimerEvent& event);

void ImagePublish(const cv::Mat &bgr);
bool SendTypeService(SetInt::Request &req, SetInt::Response &res);
bool ImageSnapService(ImageSnap::Request &req, ImageSnap::Response &res);  //得到快照，还没写完

const int width=640;
const int height=480;
CudaMatC srcMat;
CudaMatC rgbMat;
CudaMatC dstMat;
CudaMatC hsvMat;
CudaMatC relMat;
CudaMatC netMat;
CudaMatF netfMat;
CudaMatF yoloMat;
network yolo;

ObjectFilter ballFilter, postFilter;
std::atomic_int send_type(0);
std::shared_ptr<Camera> camera;
ros::Publisher imagePublisher;
ros::Publisher resultPublisher;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision");
    ros::NodeHandle node;
    if(!CameraInit()) {
        camera = std::make_shared<TpCamera>(node, "/sensor/image");
        camera->Open();
    }
    VisionInit();
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

void Run(const ros::TimerEvent& event)
{
    
    if(!ros::ok()) return;
    bool ret;
    uint32_t image_time;
    ret = srcMat.upload(camera->GetData(image_time));
    
    if(!ret)
    {
        ROS_ERROR("upload error");
        return;
    }
    if (GET_CHANNELS(camera->Type()) == 1) // Bayer
    {
        ret = CudaBayerToRGB(srcMat, rgbMat, 1.12, 0.85, 1.37);
        //ret = BayerToRGB(srcMat, rgbMat);
        ret = Resize(rgbMat, dstMat);
        ret = dstMat.copyTo(relMat);
    }
    else if(GET_CHANNELS(camera->Type()) == 2) // YUYV
    {
        ret = YUV422ToRGB(srcMat, rgbMat);
        ret = Resize(rgbMat, dstMat);
        ret = dstMat.copyTo(relMat);
    }
    else if(GET_CHANNELS(camera->Type()) == 3) // RGB
    {
        ret = Resize(srcMat, dstMat);
        ret = dstMat.copyTo(relMat);
    }
    else return;

    ret = RGBToHSV(relMat, hsvMat);
    
    ret = Resize(relMat, netMat);
    if(!ret)
    {
        ROS_ERROR("Resize to net error");
        return;
    }
    ret = RGB8uTo32fNorm(netMat, netfMat);
    if(!ret)
    {
        ROS_ERROR("RGB8uTo32fNorm error");
        return;
    }
    ret = PackedToPlanar(netfMat, yoloMat);
    if(!ret)
    {
        ROS_ERROR("PackedToPlanar error");
        return;
    }

    layer l = yolo.layers[yolo.n - 1];
    network_predict1(yolo, yoloMat.data());
    int nboxes = 0;
    float nms = 0.45;
    detection *dets = get_network_boxes(&yolo, width, height, 
                    0.65, 0.5, 0, 1, &nboxes, 0);

    if (nms)
    {
        do_nms_sort(dets, nboxes, l.classes, nms);
    }

    std::vector<DetObject> ball_dets, post_dets;
    ball_dets.clear();
    post_dets.clear();

    for (int i = 0; i < nboxes; i++)
    {
        int id = 0;
        for(int j=0; j<l.classes; j++)
        {
            if(dets[i].prob[j]>dets[i].prob[id])
                id = j;
        }
        if (id == 0)
        {
            if (dets[i].prob[id] >= ballFilter.min_prob)
            {
                int bx = (dets[i].bbox.x - dets[i].bbox.w / 2.0) * width;
                int by = (dets[i].bbox.y - dets[i].bbox.h / 2.0) * height;
                int bw = dets[i].bbox.w * width, bh = dets[i].bbox.h * height + 1;
                ball_dets.push_back(DetObject(0, dets[i].prob[id], bx, by, bw, bh));
            }
        }
        else if(id == 1)
        {
            if (dets[i].prob[id] >= postFilter.min_prob)
            {
                int px = (dets[i].bbox.x - dets[i].bbox.w / 2.0) * width;
                int py = (dets[i].bbox.y - dets[i].bbox.h / 2.0) * height;
                int pw = dets[i].bbox.w * width, ph = dets[i].bbox.h * height;
                post_dets.push_back(DetObject(1, dets[i].prob[id], px, py, pw, ph));
            }
        }
    }
    std::sort(ball_dets.rbegin(), ball_dets.rend());
    std::sort(post_dets.rbegin(), post_dets.rend());
    free_detections(dets, nboxes);


    common::ImageResult imgResult;
    imgResult.stamp = image_time;
    if (!ball_dets.empty())
    {
        imgResult.has_ball = true;
        imgResult.ball.x = ball_dets[0].x+ball_dets[0].w/2;
        imgResult.ball.y = ball_dets[0].y+ball_dets[0].h;
    }
    for (uint32_t i = 0; i < post_dets.size() && i<2; i++)
    {
        imgResult.has_post = true;
        ObjInfo tmp;
        tmp.x = post_dets[i].x+post_dets[i].w/2;
        tmp.y = post_dets[i].y+post_dets[i].h/3*2;
        imgResult.posts.push_back(tmp);
    }
    resultPublisher.publish(imgResult);


    send_type = 2;
    if(send_type>0)
    {   
        cv::Mat rgb(height, width, CV_8UC3);
        if(!dstMat.download(rgb))
        {
            ROS_ERROR("download error");
            return;
        }
        
        if(send_type>1)
        {
            if (!ball_dets.empty())
            {
                cv::rectangle(rgb, cv::Point(ball_dets[0].x, ball_dets[0].y), cv::Point(ball_dets[0].x + ball_dets[0].w,
                            ball_dets[0].y + ball_dets[0].h), cv::Scalar(255, 0, 0), 2);
            }

            for (uint32_t i = 0; i < post_dets.size(); i++)
            {
                if (i >= 2)
                {
                    break;
                }

                cv::rectangle(rgb, cv::Point(post_dets[i].x, post_dets[i].y), cv::Point(post_dets[i].x + post_dets[i].w,
                            post_dets[i].y + post_dets[i].h), cv::Scalar(0, 0, 255), 2);
            }
        }
        ImagePublish(rgb);
    }
}

bool CameraInit()
{
    std::string cfgfile;
    try{
        ros::param::get("camera_config_file", cfgfile);
    }
    catch(ros::InvalidNameException &e){
        ROS_ERROR("%s", e.what());
        return false;
    }
    camera = std::make_shared<MvCamera>(cfgfile);
    camera->Open();
    if(camera->Available()) return true;
    camera = std::make_shared<UvCamera>(0);
    camera->Open();
    if(camera->Available()) return true;
    return false;
}

bool VisionInit()
{
    std::string visionfile;
    try{
        ros::param::get("vision_file", visionfile);
    }
    catch(ros::InvalidNameException &e){
        ROS_ERROR("%s", e.what());
        return false;
    }
    
    std::string vision_root = visionfile.substr(0, visionfile.find_last_of("/")+1);
    std::string yolo_cfg, weights;
    if(visionfile.empty()) return false;
    common::bpt::ptree pt;
    if(!common::get_tree_from_file(visionfile, pt))
    {
        ROS_ERROR("parse file: [%s] failed", visionfile.c_str());
        return false;
    }
    yolo_cfg = vision_root+pt.get<std::string>("cfg_file");
    weights = vision_root+pt.get<std::string>("weights_file");
    ballFilter.min_prob = pt.get<float>("dets.prob.ball");
    ballFilter.r_range = get_config_vector<float, 2>(pt, "dets.boxes.ball.ratio");
    ballFilter.w_range = get_config_vector<int, 2>(pt, "dets.boxes.ball.width");
    ballFilter.h_range = get_config_vector<int, 2>(pt, "dets.boxes.ball.height");
    postFilter.min_prob = pt.get<float>("dets.prob.post");
    postFilter.r_range = get_config_vector<float, 2>(pt, "dets.boxes.post.ratio");
    postFilter.w_range = get_config_vector<int, 2>(pt, "dets.boxes.post.width");
    postFilter.h_range = get_config_vector<int, 2>(pt, "dets.boxes.post.height");

    yolo.gpu_index = 0;
    yolo = parse_network_cfg_custom((char*)(yolo_cfg.c_str()), 1, 0);
    load_weights(&yolo, const_cast<char *>((char*)(weights.c_str())));
    set_batch_network(&yolo, 1);
    fuse_conv_batchnorm(yolo);
    calculate_binary_weights(yolo);
    srand(2222222);
    cudaSetDevice(0);

    if(!MallocMemory()){
        ROS_ERROR("MallocMemory failed");
        return false;
    }
    return true;
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

bool MallocMemory()
{
    if(camera.get()==nullptr) return false;
    bool ret;
    ret = srcMat.create(camera->Height(), camera->Width(), GET_CHANNELS(camera->Type()));
    if(!ret) return false;

    ret = rgbMat.create(camera->Height(), camera->Width(), 3);
    if(!ret) return false;

    ret = dstMat.create(height, width, 3);
    if(!ret) return false;

    ret = relMat.create(height, width, 3);
    if(!ret) return false;

    ret = hsvMat.create(height, width, 3);
    if(!ret) return false;

    ret = netMat.create(yolo.h, yolo.w, 3);
    if(!ret) return false;

    ret = netfMat.create(yolo.h, yolo.w, 3);
    if(!ret) return false;

    ret = yoloMat.create(yolo.h, yolo.w, 3);
    if(!ret) return false;
    
    return true;
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