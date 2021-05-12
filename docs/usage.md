# 使用手册

## 首次使用

### 如何连接到机器人

以下 *电脑* 以 Ubuntu 20.04 为例

推荐准备 一个 HDMI 显示器，一根网线，一套 USB 键鼠，一个 USB 拓展坞

参考教程[Ubuntu16.04下通过一根网线 ssh终端登录Linux设备（树莓派、TX2、Manifold妙算等）](https://blog.csdn.net/X_kh_2001/article/details/80487987)

1. 用网线将机器人与电脑相连
2. 电脑上将 与机器人的有线连接 设为 *Shared to other Computers*
3. 若 与机器人的有线连接 下，电脑的 ip 为 `192.168.0.1` ，则应当已经完成。反之进行以下配置
4. 假设 与机器人的有线连接 下，电脑的 ip 为 `10.42.0.1`，则需将机器人的静态 ip 配置为 `10.42.0.x`，其中 1 < x < 255。配置方法为 设置 -> 网络 -> 右下角options -> ip 选项卡

验证方法：`ping 192.168.0.133`，后面的 ip 为机器人ip

目前的机器人默认 ip 分别为 `192.168.0.131` `192.168.0.132` `192.168.0.133`

也有可能为 `10.42.0.13x`

> 机器人只能连接 2.4GHz 的 WiFi
> 
> 扫描子网设备：`arp -a`

## 启动

```Bash
sudo su # 否则无法打开 tty 串口
cd /home/nvidia/SEU-Kidsize
sudo apt install ros-melodic-rosbridge-server
# 若在机器人上编译则须删除 src/tools/simulation 文件夹
catkin_make # 首次编译可能需要重复四次
source devel/setup.bash
roslaunch start start_robot.launch
```

## tools

以下在电脑上进行

### easy_start
管理编译代码easy_start其实大可不必，推荐使用vscode远程登录

```Bash
python3 src/tools/easy_start/scripts/easy_start.py
```

可能需要在 [config.py](../src/tools/easy_start/scripts/config.py) 中修改 用户名和密码

### debuger

```Bash
sudo apt install graphviz-dev
pip3 install -r src/tools/debuger/scripts/requirements.txt
python3 src/tools/debuger/scripts/main.py
```

### 

- 机器人：`start_robot_remote`
- 电脑：
  1. `roscore`
  2. `rosrun debuger main.py`
  3. 点击 *动作调试器*，第一次应报错：*无法连接到本地调试器*，第二次运行则正常 #现在报错： Reason: new node registered with same name，待解决