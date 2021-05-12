# 基于ROS的机器人控制系统
适用于SEU-UniRobot的机器人控制系统，包含机器人运动控制，机器人图像处理，机器人决策，机器人调试工具，机器人仿真系统等；主要开发语言为C/C++，脚本语言为Python。
## 依赖项  
- 编译环境  
    + CMake >= 3.14  
    + gcc/g++  
    + nvcc

- C++ 
    + ROS  
    + CUDA >= 9.0  
    + cuDNN >= 7.0   
    + libeigen3-dev
    + MVSDK (摄像头SDK)  
    + rosbridge_server

- Python3
    + paramiko  
    + PyQt5  
    + pydot  
    + pygraphviz  
    + roslibpy  

- 仿真环境
    + Webots(安装位置必须是/usr/local/webots)  
    + 需要在～/.bashrc中添加： export WEBOTS_HOME=/usr/local/webots  


## 开发工具   
+ VS Code (插件: C/C++, Python, ROS, CMake)  
+ astyle (代码格式化工具)  

## 架构说明  
本架构分成六个部分： 上下层交互、运动控制、视觉、决策、比赛控制器和仿真，下面对每个部分进行详细解释。  
### 上下层交互  
上下层交互对应的文件夹为player，负责与机器人的下位机交互，其工作原理是：下位机定时向上层系统发送请求，并携带了按键状态、电源状态、IMU数据信息。player在接收到请求信息后，先向下位机发送舵机的关节角度信息，然后再根据需要发送其他信息，最后再从motion里获取下一次的关节角度信息。  
### 运动控制  
运动控制对应的文件夹为motion，负责机器人的运动生成，其工作原理是：内部维持了一个关节角度队列，每次生成一个动作时，就将动作过程中的所有角度信息保存到队列中，player每请求一次关节角度，就弹出队列最前端的角度信息，待队列内的个数小于一定数量时，生成下一步的动作。motion订阅了BodyTask和HeadTask以及ImuData等话题，每次先检测ImuData，判断是否处于摔倒状态，摔倒则调用起身动作，否则根据Task执行相关的动作。  
### 视觉  
视觉对应的文件夹为vision，负责机器人的图像处理，其工作原理是：初始化摄像头后开启一个摄像头获取图像信息，然后以10Hz的频率对图像进行处理，处理结果以ImageResult信息发布出去，其具体数据可以根据需要自行定制。  
### 决策  
决策对应的文件夹为control，负责机器人的决策制定。
### 比赛控制器  
比赛控制器对应的文件夹是gamectrl，负责获取比赛控制器的信息并将关键信息以话题的方式发布出来，同时还负责向比赛控制器回发信息。  
### 仿真 
仿真对应的文件夹为simulation，负责机器人仿真，仿真平台使用的是Webots。仿真平台的控制器对应的功能是player的功能和摄像头。  

## 目录说明  
+ common: 数据结构定义  
+ config: 配置文件存放及解析  
+ gamectrl: 比赛控制器通信  
+ libraries: 自己实现的库  
    - seuimage: 图像处理相关  
    - seumath: 基于Eigen的数学库  
    - robot: 机器人相关功能  
    - darknet: 神经网络框架  
+ motion: 机器人运动控制  
+ player: 机器人控制器  
+ vision: 机器人视觉  
+ control: 机器人决策控制  
+ tools: 机器人调试工具  
    - simulation: 基于Webots的仿真系统
    - action: 动作调试  
    - debuger: 基于roslibpy的调试功能整合  
    - image: 离线图像调试  
    - team_monitor: 全组机器人状态监测  
    - easy_start: 快捷启动工具  
+ [start](src/start/start.md): 启动文件 
