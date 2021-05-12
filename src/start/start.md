# start 包说明  
该包定义了各种launch文件和参数更新节点  

## 文件说名：
+ src  
    - [params_update.cpp](src/params_update.cpp): 参数更新节点，在运行前必须先更新参数。  
+ launch  
    - [update_params.launch](launch/update_params.launch): 参数更新节点的launch文件   
    - [start_debug_server.launch](launch/start_debug_server.launch): 在线调试的launch文件  
    - [start_action_debug_simulation.launch](launch/start_action_debug_simulation.launch): 动作生成调试的launch文件   
    - [start_action_debug_robot.launch](launch/start_action_debug_robot.launch): 动作生成调试的launch文件  
    - [start_simulation.launch](launch/start_simulation.launch): 仿真的launch文件，启动仿真相关节点  
    - [start_robot.launch](launch/start_robot.launch): 机器人的launch文件，启动所有相关节点   
    - [start_simulation_remote.launch](launch/start_simulation_remote.launch): 以遥控模式启动仿真机器人，不会运行control包内的节点       
    - [start_robot_remote.launch](launch/start_robot_remote.launch): 以遥控模式启动机器人，不会运行control包内的节点     
    - [start_without_vision_simulation.launch](launch/start_without_vision_simulation.launch): 机器人的launch文件，但是不启动视觉相关节点  
    - [start_without_vision_robot.launch](launch/start_without_vision_robot.launch): 机器人的launch文件，但是不启动视觉相关节点   
    
      


## 各个文件启动节点详情  
|             launch              | params_update | player | simulation |  motion  | vision |  gamectrl | control | debug_server|
| :------------: | :------------: | :------------: | :------------: | :------------: | :------------: | :------------: | :------------: | :------------: |
| params_update   | ●  |   |   |   |   |   |   |   |
|  start_debug_server |   |   |   |   |   |   |   | ●   |
|  start_action_debug_simulation |  ●  |   | ●   |  ●  |   |   |   |   |
|  start_action_debug_robot |  ●  | ●   |   |  ●  |   |   |   |   |
|  start_simulation |  ● |   | ●  | ●  |  ● |  ● |  ● |   |
|  start_robot |  ● |  ● |   |  ● | ●  |  ● | ●  |   |
|  start_simulation_remote |  ● |   | ●  | ●  | ●  |   |   |  ● |
|  start_robot_remote |  ● |  ● |   | ●  | ●  |   |   |  ● |
|  start_without_vision_simulation |  ● |   | ●   | ●  |   |   | ●  |   |
|  start_without_vision_robot |  ● |  ● |    | ●  |   |   | ●  |   |   |



