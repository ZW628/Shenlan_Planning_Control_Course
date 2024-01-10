# Project III LQR 轨迹跟踪算法实现

本项目希望大家根据 LQR 算法实现车辆的横向控制，结合 PID 算法实现轨迹跟踪。基本的
系统框架已经给出，仅需要完成 `lqr_control/src/lqr_controller.cpp` 中 todo 部分。

代码运行方式：

控制台 1：启动 carla
```bash
cd <root-to-carla>
./CarlaUE4.sh
```

控制台 2：启动 `lqr_control` 节点
```bash
rosdep install --from-paths src --ignore-src
catkin build
roslaunch lqr_control demo.launch
```