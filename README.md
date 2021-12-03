# turtlebot2_withlidar
## 本工程是基于turtlebot2开源代码修改,加入了激光雷达传感器,下载文件覆盖掉相应文件夹 
```bash
roslaunch turtlebot_gazebo turtlebot_go.launch 
代替
roslaunch turtlebot_gazebo turtlebot_world.launch 
将kinec相机伪点云输出话题由原有的scan改为scan_camera
新增激光雷达输出话题scan
```
