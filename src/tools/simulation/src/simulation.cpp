#include <common/BodyAngles.h>
#include <common/HeadAngles.h>
#include <common/ImuData.h>
#include <common/GetAngles.h>
#include <common/common.hpp>
#include <seumath/math.hpp>
#include <bits/stdc++.h>
#include <ros/ros.h>
#include "SimRobot.hpp"

using namespace std;
using namespace common;
using namespace seumath;
using namespace Eigen;

std::shared_ptr<SimRobot> player;
uint32_t start_clock = 0;

int main(int argc, char **argv)
{
    setenv("WEBOTS_ROBOT_NAME", "maxwell", 0);
    ros::init(argc, argv, "maxwell");
    ros::NodeHandle node;
    bool ok=false;
    int i=0;
    while(ros::ok() && i++<20)
    {
      FILE   *stream;  
      FILE    *wstream;
      char   buf[1024]; 
      memset( buf, '\0', sizeof(buf) );//初始化buf
      stream = popen( "ls /tmp | grep webots-" , "r" );
      fread( buf, sizeof(char), sizeof(buf),  stream);  //将刚刚FILE* stream的数据流读取到buf中
      pclose( stream ); 
      string sbuf(buf);
      if(sbuf.find("webots") != string::npos)
      {
        string pf = "/tmp/"+sbuf.substr(0, sbuf.find("\n"))+"/WEBOTS_SERVER";
        ROS_WARN("%s", pf.c_str());
        if(access(pf.c_str(), F_OK)!=-1)
        {
          ok = true;
          break;
        }
      }
      ROS_WARN("waiting for webots ......");
      usleep(1000000);
    }
    if(!ok) return 0;
    ros::service::waitForService("/get_angles");
    player = std::make_shared<SimRobot>(&node);
    start_clock = get_clock();
    int ret = 0;
    while (ros::ok() && ret >= 0)
    {
        GetAngles getsrv;
        ros::service::call("/get_angles", getsrv);
        player->mAngles = getsrv.response.body;
        player->mHAngles = getsrv.response.head;
        ret = player->myStep();
        ros::spinOnce();
    }
    return 0;
}

