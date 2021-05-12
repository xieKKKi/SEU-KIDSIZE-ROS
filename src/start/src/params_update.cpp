/** 
*  Copyright (C) 2021, SEU Robot Lab. All rights reserved.
*  @file     params_update.cpp                                                  
*  @brief    read config files and update params.
*  @author   刘川 谢家鹏
*  @todo     
*/
#include <ros/ros.h>
#include <ros/package.h>
#include <config/basic_parser.hpp>

using namespace std;
using namespace common;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "config");
    ros::NodeHandle node;
    std::string cfgpath = ros::package::getPath("config")+"/conf/";
    const std::string root_cfg_file = "config.conf";
    string cfgfile(cfgpath+root_cfg_file);
    bpt::ptree pt;
    if(!parse_file(cfgfile, pt)){
        ROS_ERROR("parser cfgfile: %s failed", cfgfile.c_str());
        return false;
    }
    bool gamectrl;
    node.param<bool>("gamectrl", gamectrl, false);
    ros::param::set("gamectrl", gamectrl);
    ros::param::set("image", 0);
    int id = pt.get<int>("id");
    try
    {
        for(auto p:pt)
        {
            if(!p.second.data().empty())
            {
                string data=p.second.data();
                if(p.first.find("file")!=string::npos){
                    data = cfgpath+data;
                }
                if(data == "true"){
                    ros::param::set(p.first, true);
                }else if(data == "false"){
                    ros::param::set(p.first, false);
                }else{
                    bool ok=false;
                    try{
                        int d = std::stoi(data);
                        ros::param::set(p.first, d);
                        ok = true;
                    }catch(std::invalid_argument&){
                    }
                    if(!ok){
                        try{
                            float d = std::stof(data);
                            ros::param::set(p.first, d);
                            ok = true;
                        }catch(std::invalid_argument&){
                        }
                    }
                    if(!ok) ros::param::set(p.first, data);
                }
            }
        }
        std::string temp = "players." + std::to_string(id);
        bpt::ptree pr = pt.get_child(temp);
        for (auto p : pr)
        {
            string data=p.second.data();
            if(p.first.find("file")!=string::npos){
                data = cfgpath+data;
            }
            ros::param::set(p.first, data);
        }
    }
    catch (bpt::ptree_error &e)
    {
        ROS_ERROR("%s", e.what());
        exit(0);
    }
    ros::param::set("params", true);
    return 0;
}