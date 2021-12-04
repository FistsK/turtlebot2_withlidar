# turtlebot2_withlidar 
## 本工程是基于turtlebot2开源代码修改,加入了激光雷达传感器,下载文件覆盖掉相应文件夹  
```bash
roslaunch turtlebot_gazebo turtlebot_go.launch  
代替
roslaunch turtlebot_gazebo turtlebot_world.launch  
将kinec相机伪点云输出话题由原有的scan改为scan_camera 
新增激光雷达输出话题scan 
```
## turtlebot2源码配置 
环境:Turtlebot2+Ubuntu18.04+melodic 
1.安装依赖 

```cpp
 sudo apt-get install ros-melodic-kobuki-* 
 sudo apt-get install ros-melodic-ecl-streams 
 sudo apt-get install libusb-dev 
 sudo apt-get install libspnav-dev 
 sudo apt-get install ros-melodic-joystick-drivers 
 sudo apt-get install bluetooth 
 sudo apt-get install libbluetooth-dev 
 sudo apt-get install libcwiid-dev 
```

2.新建工作空间,准备相关包 

```cpp
 mkdir -p ~/turtlebot_ws/src  
 cd ~/turtlebot_ws/src  

 git clone https://github.com/turtlebot/turtlebot_simulator 
 git clone https://github.com/turtlebot/turtlebot.git 
 git clone https://github.com/turtlebot/turtlebot_apps.git 
 git clone https://github.com/udacity/robot_pose_ekf 
 git clone https://github.com/ros-perception/depthimage_to_laserscan.git  
 git clone https://github.com/yujinrobot/kobuki_msgs.git 
 git clone https://github.com/yujinrobot/kobuki_desktop.git 
 cd kobuki_desktop/ 
 rm -r kobuki_qtestsuite 
 git clone https://github.com/toeklk/orocos-bayesian-filtering.git 
 git clone https://github.com/turtlebot/turtlebot_msgs.git 
 git clone https://github.com/ros-drivers/joystick_drivers.git 
```

3.复制kobuki和yujin_ocs依赖库到turtlebot_ws/src工作空间下 

```cpp
 mkdir -p ~/repos/ 
 cd ~/repos/ 
 git clone https://github.com/yujinrobot/kobuki.git 
 cp -r kobuki/* ~/turtlebot_ws/src/ 
 git clone https://github.com/yujinrobot/yujin_ocs.git 
 cp -r yujin_ocs/yocs_cmd_vel_mux/  yujin_ocs/yocs_controllers  yujin_ocs/yocs_velocity_smoother ~/turtlebot_ws/src/ 
```

4.编译工作空间

```cpp
 cd ~/turtlebot_ws 
 catkin_make 
```

5.添加工作空间到bashrc文件 

```cpp
echo "source ~/turtlebot_ws/devel/setup.bash" >> ~/.bashrc 
```

6.启用环境变量

```cpp
source ~/.bashrc  
```

测试

```cpp
roslaunch turtlebot_gazebo turtlebot_world.launch 
roslaunch turtlebot_teleop keyboard_teleop.launch  
```
