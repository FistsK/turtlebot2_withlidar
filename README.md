# turtlebot2_withlidar 
## 本工程是基于turtlebot2开源代码修改,加入了激光雷达传感器,下载文件覆盖掉相应文件夹  
```bash
roslaunch turtlebot_gazebo turtlebot_go.launch  
代替
roslaunch turtlebot_gazebo turtlebot_world.launch  
将kinec相机伪点云输出话题由原有的scan改为scan_camera 
新增激光雷达输出话题scan 
```
# 前言
在仿真的过程中发现Turtlebot2仿真环境是以Kinect摄像头为传感器，进行伪激光数据输出，这在跑ROS中Gmapping算法是会导致特征点比较少（激光雷达360度，深度相机由于视场角问题，特征点非常少）会导致在某个区域机器人实际在移动但是gmapping算法认为机器人原地不动，出现卡顿现象，这是看[古月居课程：ROS常用SLAM功能包使用指南 · 古月 ](https://class.guyuehome.com/detail/p_5ed700a841cc8_UpE7PGXW/6)知道的。所以打算加入激光雷达传感器代替SCAN原有的伪激光话题。然后按照古月课程整理了一节《实验一 SLAM地图构建与保存 》
# <center>实验一 SLAM地图构建与保存 
## 一.实验内容 
本次实验通过Turtlebot仿真环境,配置ROS中几种常见的SLAM算法功能包实现对机器人周围环境的地图构建. 
## 二.实验目标 
1.了解SLAM算法的基本框架 
2.掌握几种常见SLAM算法功能包的配置  
3.了解不同SLAM算法之间的差异和各自适用的场景 
## 三.实验环境 
·Ubuntu18.04 desktop 
·ROS Melodic 
·TurtleBot2 仿真功能包 
## 四.实验原理  
### 4.1Gmapping 
![在这里插入图片描述](https://img-blog.csdnimg.cn/254648c18ef54b02bde600c5116425c6.png?x-oss-process=image/watermark,type_d3F5LXplbmhlaQ,shadow_50,text_Q1NETiBAUm9ja1dhbmcu,size_20,color_FFFFFF,t_70,g_se,x_16#pic_center) 

Gmapping是基于粒子滤波的算法。 
优点: 
1.可以实时构建室内地图，在构建小场景地图所需的计算量较小且精度较高。 
2.相比Hector SLAM对激光雷达频率要求低、鲁棒性高（Hector 在机器人快速转向时很容易发生错误匹配，建出的地图发生错位，原因主要是优化算法容易陷入局部最小值）。 
3.而相比Cartographer在构建小场景地图时，Gmapping不需要太多的粒子并且没有回环检测，因此计算量小于Cartographer而精度并没有差太多。 
4.在长廊及低特征场景中建图效果好；在地况较为平整时采用Gmapping方法较多。 
缺点： 
1.依赖里程计（odometry），无法适用无人机及地面小车不平坦区。 
2.严重依赖里程计，无法适应无人机及地面不平坦的区域，无回环（激光SLAM很难做回环检测），大的场景，粒子较多的情况下，特别消耗资源。 
3.随着场景增大所需的粒子增加，因为每个粒子都携带一幅地图，因此在构建大地图时所需内存和计算量都会增加。因此不适合构建大场景地图。 
4.没有回环检测，因此在回环闭合时可能会造成地图错位，虽然增加粒子数目可以使地图闭合但是以增加计算量和内存为代价。 
所以Gmapping不能像cartographer那样构建大的地图。 
### 4.2 Hector SLAM 
![在这里插入图片描述](https://img-blog.csdnimg.cn/dbd637e1182d46c2a7982d627167d8d7.png?x-oss-process=image/watermark,type_d3F5LXplbmhlaQ,shadow_50,text_Q1NETiBAUm9ja1dhbmcu,size_20,color_FFFFFF,t_70,g_se,x_16#pic_center) 
PS:博客中懒得自己画了，图片直接截取古月居的课件，感觉课件中的很详细了 
[古月居官网](https://www.guyuehome.com/) 

基于优化的算法（解最小二乘问题），优缺点：不需要里程计，但对于雷达帧率要求很高40Hz，估计6自由度位姿，可以适应空中或者地面不平坦的情况。初值的选择对结果影响很大，所以要求雷达帧率较高。 Hector_SLAM提出的初衷是为了解决非平坦地区，非结构化环境下的地图构建。 
优点： 
1.SLAM经过多年的发展，在结构化场景下建图已经非常成熟，但是都离不开高精度的里程计数据，而Hector_SLAM完全摆脱了里程计，仅仅依赖于高频率的激光。不需要使用里程计，所以使得空中无人机及地面小车在不平坦区域建图存在运用的可行性。 
2.利用已经获得的地图对激光束点阵进行优化, 估计激光点在地图的表示,和占据网格的概率。 
3.利用高斯牛顿方法解决scan-matching 问题，获得激光点集映射到已有地图的刚体变换。 
4.为避免局部最小而非全局最优，使用多分辨率地图；导航中的状态估计加入惯性测量系统（IMU），利用EKF滤波。 
5.Hector_SLAM灵活高、可扩展性强，可用于二维和三维地图的创建，很好地适应了微型处理器，降低了SLAM算法对计算机硬件的要求，在自主移动机器人导航领域应用越来越多。 
缺点： 
1.需要雷达（LRS）的更新频率较高，测量噪声小。所以在制图过程中，需要robot速度控制在比较低的情况下，建图效果才会比较理想，这也是它没有回环（loop close）的一个后遗症。 
2.且在里程计数据比较精确的时候，无法有效利用里程计信息。 
实验表明：在大地图，低特征（distinctive landmarks）场景中，hector的建图误差高于gmapping。这是由于hector过分依赖scan-match。特别是在长廊问题中，误差更加明显。 
补充：hector_slam通过最小二乘法匹配扫描点，且依赖高精度的激光雷达数据，因此扫描角很小且噪声较大的Kinect是不行的，匹配时会陷入局部点，地图比较混乱。 
### 4.3 Cartographer 
![在这里插入图片描述](https://img-blog.csdnimg.cn/147202ed047e41c4bed8a7582fb17dda.png?x-oss-process=image/watermark,type_d3F5LXplbmhlaQ,shadow_50,text_Q1NETiBAUm9ja1dhbmcu,size_20,color_FFFFFF,t_70,g_se,x_16#pic_center) 

累计误差较前两种算法低，能天然的输出协方差矩阵，成本较低的雷达也能跑出不错的效果。先由一定数量的laser scan构建submap，由submap拼接成地图，具有回环检测，间隔一定数量的扫描进行一次所有submap的图优化（SPA，运用了分支定界原理进行加速）。 


## 五.实验步骤 
5.1 Gmapping建图 
ROS中默认的SLAM定位算法为Gmapping,因此在编译好工作空间后,在终端直接运行: 

```cpp
roslaunch turtlebot_gazebo turtlebot_go.launch 
```

![在这里插入图片描述](https://img-blog.csdnimg.cn/87906f4c2160475e89b6e608bcd6290b.png?x-oss-process=image/watermark,type_d3F5LXplbmhlaQ,shadow_50,text_Q1NETiBAUm9ja1dhbmcu,size_20,color_FFFFFF,t_70,g_se,x_16) 

新建终端输入 

```cpp
roslaunch turtlebot_gazebo gmapping_demo.launch 
```

![在这里插入图片描述](https://img-blog.csdnimg.cn/712038584d8a48648e3b4fe9ac141ea9.png?x-oss-process=image/watermark,type_d3F5LXplbmhlaQ,shadow_50,text_Q1NETiBAUm9ja1dhbmcu,size_20,color_FFFFFF,t_70,g_se,x_16) 


再新建终端输入 

```cpp
roslaunch turtlebot_teleop keyboard_teleop.launch 
```

![在这里插入图片描述](https://img-blog.csdnimg.cn/6d7d3fde5ec04f09a7f058bc385a7c82.png?x-oss-process=image/watermark,type_d3F5LXplbmhlaQ,shadow_50,text_Q1NETiBAUm9ja1dhbmcu,size_20,color_FFFFFF,t_70,g_se,x_16) 


根据终端提示,将鼠标定位在此终端(按键控制终端,否则无法控制),通过Rviz左下角摄像头第一视角,按键盘”I”键控制机器人前进,”J”和”L”分别为左转和右转,”,”键为后退控制机器人对周围环境进行建图,并构建一张完整地图,用于后续机器人导航定位. 

控制机器人运行一周后,见下图: 
![在这里插入图片描述](https://img-blog.csdnimg.cn/26a262b052d94451ba715c9a2357869d.png?x-oss-process=image/watermark,type_d3F5LXplbmhlaQ,shadow_50,text_Q1NETiBAUm9ja1dhbmcu,size_20,color_FFFFFF,t_70,g_se,x_16) 

进入/home/用户名/turtlebot_ws/src/turtlebot_simulator/turtlebot_gazebo/maps目录下 

```cpp
rosrun map_server map_saver -f map 
```

注释:rosrun map_server map_saver -f [要保存成地图的地图名,此处要保存为map] 
生成map.pgm和map.yaml两个文件,其中map.pgm为地图,map.yaml为地图配置文件里面包括地图的路径,分辨率等. 
![在这里插入图片描述](https://img-blog.csdnimg.cn/b9d90caefaed47ee892cf27a7a254b45.png?x-oss-process=image/watermark,type_d3F5LXplbmhlaQ,shadow_50,text_Q1NETiBAUm9ja1dhbmcu,size_20,color_FFFFFF,t_70,g_se,x_16) 

最终生成地图: 
![在这里插入图片描述](https://img-blog.csdnimg.cn/6d82462a26b9434dbb2d8de2bd67fd7e.png?x-oss-process=image/watermark,type_d3F5LXplbmhlaQ,shadow_50,text_Q1NETiBAUm9ja1dhbmcu,size_20,color_FFFFFF,t_70,g_se,x_16) 
5.2 Hector建图 
Hector SLAM算法在Turtlebot2工作空间中已经配置完成，对配置感兴趣的同学可以查看hector.launch这个文件中的内容。 
利用Hector进行建图： 
新建终端： 

```cpp
roslaunch turtlebot_gazebo turtlebot_go.launch  
```

![在这里插入图片描述](https://img-blog.csdnimg.cn/2406bfdcd17c46fbadd0b8f4e0322904.png?x-oss-process=image/watermark,type_d3F5LXplbmhlaQ,shadow_50,text_Q1NETiBAUm9ja1dhbmcu,size_20,color_FFFFFF,t_70,g_se,x_16) 

新建终端输入： 

```cpp
roslaunch turtlebot_gazebo hector.launch  
```

![在这里插入图片描述](https://img-blog.csdnimg.cn/d101fa251ed64801ba9f5782526695c6.png?x-oss-process=image/watermark,type_d3F5LXplbmhlaQ,shadow_50,text_Q1NETiBAUm9ja1dhbmcu,size_20,color_FFFFFF,t_70,g_se,x_16) 

新建终端打开按键控制： 

```cpp
roslaunch turtlebot_teleop keyboard_teleop.launch 
```

![在这里插入图片描述](https://img-blog.csdnimg.cn/d4700ba33cde400bb171dec043079fee.png?x-oss-process=image/watermark,type_d3F5LXplbmhlaQ,shadow_50,text_Q1NETiBAUm9ja1dhbmcu,size_20,color_FFFFFF,t_70,g_se,x_16) 

鼠标定位在此终端，根据提示控制机器人对周围环境进行建图。 
![在这里插入图片描述](https://img-blog.csdnimg.cn/280779179731432eb28e7272d28c81f1.png?x-oss-process=image/watermark,type_d3F5LXplbmhlaQ,shadow_50,text_Q1NETiBAUm9ja1dhbmcu,size_20,color_FFFFFF,t_70,g_se,x_16) 

进入/home/用户名/turtlebot_ws/src/turtlebot_simulator/turtlebot_gazebo/maps目录下 

```cpp
rosrun map_server map_saver -f map_hector 
```

注释:rosrun map_server map_saver -f [要保存成地图的地图名,此处要保存为map] 
生成map_hector.pgm和map_hector.yaml两个文件,其中map_hector.pgm为地图,map_hector.yaml为地图配置文件里面包括地图的路径,分辨率等. 

![在这里插入图片描述](https://img-blog.csdnimg.cn/516c3171b2be47a8ae641454f4813176.png?x-oss-process=image/watermark,type_d3F5LXplbmhlaQ,shadow_50,text_Q1NETiBAUm9ja1dhbmcu,size_20,color_FFFFFF,t_70,g_se,x_16)![在这里插入图片描述](https://img-blog.csdnimg.cn/534133fb1fa24e749289312d873b86f5.png?x-oss-process=image/watermark,type_d3F5LXplbmhlaQ,shadow_50,text_Q1NETiBAUm9ja1dhbmcu,size_20,color_FFFFFF,t_70,g_se,x_16) 



5.3 Cartographer建图 
安装功能包： 

```cpp
sudo apt-get install  ros-melodic-cartographer-* 
```

新建终端打开Gazebo仿真环境： 

```cpp
roslaunch turtlebot_gazebo turtlebot_go.launch 
```

![在这里插入图片描述](https://img-blog.csdnimg.cn/12f42116b0f94126b1d1a762c59674b5.png?x-oss-process=image/watermark,type_d3F5LXplbmhlaQ,shadow_50,text_Q1NETiBAUm9ja1dhbmcu,size_20,color_FFFFFF,t_70,g_se,x_16) 

新建终端打开cartographer节点： 

```cpp
roslaunch turtlebot_gazebo cartographer_demo.launch  
```

![在这里插入图片描述](https://img-blog.csdnimg.cn/6e12f8154ffd417480812fcd2d483a16.png?x-oss-process=image/watermark,type_d3F5LXplbmhlaQ,shadow_50,text_Q1NETiBAUm9ja1dhbmcu,size_20,color_FFFFFF,t_70,g_se,x_16) 

新建终端打开按键控制： 

```cpp
roslaunch turtlebot_teleop keyboard_teleop.launch 
```

![在这里插入图片描述](https://img-blog.csdnimg.cn/a6fb259fc90b44918eed0e3273cfbcc5.png?x-oss-process=image/watermark,type_d3F5LXplbmhlaQ,shadow_50,text_Q1NETiBAUm9ja1dhbmcu,size_20,color_FFFFFF,t_70,g_se,x_16) 

鼠标控制机器人进行建图： 
![在这里插入图片描述](https://img-blog.csdnimg.cn/140b2e6eb3cb4c46a25294d958a14110.png?x-oss-process=image/watermark,type_d3F5LXplbmhlaQ,shadow_50,text_Q1NETiBAUm9ja1dhbmcu,size_20,color_FFFFFF,t_70,g_se,x_16) 

保存地图： 
新建终端依次输入下列三条指令： 

```cpp
rosservice  call /finish_trajectory "trajectory_id: 0"  
rosservice  call /write_state "filename: '${HOME}/mymap.pbstream'"  
rosrun cartographer_ros cartographer_pbstream_to_ros_map -map_filestem=${HOME}/mymap -pbstream_filename=${HOME}/mymap.pbstream -resolution=0.05 
```

在Home目录下出现： 
![在这里插入图片描述](https://img-blog.csdnimg.cn/6a3f74095eb34ba78a7f2395f89b7cf3.png?x-oss-process=image/watermark,type_d3F5LXplbmhlaQ,shadow_50,text_Q1NETiBAUm9ja1dhbmcu,size_20,color_FFFFFF,t_70,g_se,x_16) 

 1. mymap.pbstream  
 2. mymap.pgm  
 3. mymap.yaml 

其中mymap.pgm和mymap.yaml是我们所需要的地图文件，并将其复制到/home/用户名/turtlebot_ws/src/turtlebot_simulator/turtlebot_gazebo/maps目录下。 
![在这里插入图片描述](https://img-blog.csdnimg.cn/8b6d9b02afb54cf59676c9d26dd90f1e.png?x-oss-process=image/watermark,type_d3F5LXplbmhlaQ,shadow_50,text_Q1NETiBAUm9ja1dhbmcu,size_20,color_FFFFFF,t_70,g_se,x_16) 

Cartographer所建地图效果： 

![在这里插入图片描述](https://img-blog.csdnimg.cn/95b6620f8a514672919a7c309f88898c.png?x-oss-process=image/watermark,type_d3F5LXplbmhlaQ,shadow_50,text_Q1NETiBAUm9ja1dhbmcu,size_20,color_FFFFFF,t_70,g_se,x_16) 


5.4建图效果对比 
![在这里插入图片描述](https://img-blog.csdnimg.cn/bf0eaab3d1094dee93f972abbd97e159.png?x-oss-process=image/watermark,type_d3F5LXplbmhlaQ,shadow_50,text_Q1NETiBAUm9ja1dhbmcu,size_19,color_FFFFFF,t_70,g_se,x_16#pic_center) 

<center>gmapping建图效果 

![在这里插入图片描述](https://img-blog.csdnimg.cn/03a3bad5c8a24f0fbeb6f372a32baaf2.png?x-oss-process=image/watermark,type_d3F5LXplbmhlaQ,shadow_50,text_Q1NETiBAUm9ja1dhbmcu,size_20,color_FFFFFF,t_70,g_se,x_16#pic_center) 




<center>hector建图效果 

![在这里插入图片描述](https://img-blog.csdnimg.cn/bbfefea643a84717acdaab2c1c0c5263.png?x-oss-process=image/watermark,type_d3F5LXplbmhlaQ,shadow_50,text_Q1NETiBAUm9ja1dhbmcu,size_18,color_FFFFFF,t_70,g_se,x_16#pic_center) 

<center>cartographer建图效果 

5.5 RTAB建图 
安装功能包： 

```cpp
sudo apt-get install ros-melodic-rtabmap ros-melodic-rtabmap-ros 
```

启动Gazebo仿真环境： 

```cpp
roslaunch turtlebot_gazebo turtlebot_go.launch 
```

![在这里插入图片描述](https://img-blog.csdnimg.cn/2481e887b83442d0abaf6dfd107409d5.png?x-oss-process=image/watermark,type_d3F5LXplbmhlaQ,shadow_50,text_Q1NETiBAUm9ja1dhbmcu,size_20,color_FFFFFF,t_70,g_se,x_16) 


运行RTAB节点： 

```cpp
roslaunch turtlebot_gazebo rtabmap_demo.launch  
```

![在这里插入图片描述](https://img-blog.csdnimg.cn/cbd76f5886484eda89a0109ef1ddd2b7.png?x-oss-process=image/watermark,type_d3F5LXplbmhlaQ,shadow_50,text_Q1NETiBAUm9ja1dhbmcu,size_20,color_FFFFFF,t_70,g_se,x_16)![在这里插入图片描述](https://img-blog.csdnimg.cn/013780dfb1f5477abcc9408633b7a83a.png?x-oss-process=image/watermark,type_d3F5LXplbmhlaQ,shadow_50,text_Q1NETiBAUm9ja1dhbmcu,size_20,color_FFFFFF,t_70,g_se,x_16) 



配置rviz设置： 
添加机器人模型：Add->By dispaly type->rviz->RobotModel  点击OK 
![在这里插入图片描述](https://img-blog.csdnimg.cn/5d85343f73cb4886be3a920177ec9c20.png?x-oss-process=image/watermark,type_d3F5LXplbmhlaQ,shadow_50,text_Q1NETiBAUm9ja1dhbmcu,size_20,color_FFFFFF,t_70,g_se,x_16) 

左侧rviz设置按照下图勾选： 
![在这里插入图片描述](https://img-blog.csdnimg.cn/887235400f2a4eaeae606cbda5a8afa0.png?x-oss-process=image/watermark,type_d3F5LXplbmhlaQ,shadow_50,text_Q1NETiBAUm9ja1dhbmcu,size_20,color_FFFFFF,t_70,g_se,x_16) 

启动键盘控制节点： 

```cpp
roslaunch turtlebot_teleop keyboard_teleop.launch 
```

![在这里插入图片描述](https://img-blog.csdnimg.cn/a1b94cb48e464855aa5518a9e9c553de.png?x-oss-process=image/watermark,type_d3F5LXplbmhlaQ,shadow_50,text_Q1NETiBAUm9ja1dhbmcu,size_20,color_FFFFFF,t_70,g_se,x_16) 

通过键盘控制机器人进行建图： 
![在这里插入图片描述](https://img-blog.csdnimg.cn/8aeb2e061a0a40159298b251a3a25856.png?x-oss-process=image/watermark,type_d3F5LXplbmhlaQ,shadow_50,text_Q1NETiBAUm9ja1dhbmcu,size_20,color_FFFFFF,t_70,g_se,x_16) 


关闭所有终端，新建终端查看构建地图输入： 

```cpp
rtabmap-databaseViewer ~/.ros/rtabmap.db 
```
![在这里插入图片描述](https://img-blog.csdnimg.cn/9749053a83814fe8b7d67c585a93d14b.png?x-oss-process=image/watermark,type_d3F5LXplbmhlaQ,shadow_50,text_Q1NETiBAUm9ja1dhbmcu,size_20,color_FFFFFF,t_70,g_se,x_16) 

点击view中 Graph view查看2D地图 
![在这里插入图片描述](https://img-blog.csdnimg.cn/5e1b0ac64c7f4dddbc04cce85901c4d2.png?x-oss-process=image/watermark,type_d3F5LXplbmhlaQ,shadow_50,text_Q1NETiBAUm9ja1dhbmcu,size_20,color_FFFFFF,t_70,g_se,x_16) 


点击view中 Occupancy Grid查看3D地图 
![在这里插入图片描述](https://img-blog.csdnimg.cn/832dde02b38141c0a8602850eff2efbb.png?x-oss-process=image/watermark,type_d3F5LXplbmhlaQ,shadow_50,text_Q1NETiBAUm9ja1dhbmcu,size_20,color_FFFFFF,t_70,g_se,x_16) 


## 六.实验小结  

 1. 有些安装包的下载需要科学上网，这也是学习过程中必备的技能。 
 2. 通过本实验，掌握ROS中几种常见的建图方法，并学习各个算法原理体会不同算法之间的差异性以及各自适用的场景。 
 3. 有兴趣的同学可以在TurtleBot2源码工程文件基础上，修改配置文件，搭建自己的仿真环境。 
 4. 遇到问题时，分析问题可能出现的原因，有顺序条理地去解决，例如配置文件都正确，修改不起作用，需要检查当前环境变量是否设置正确。 

# 附录
[代码开源地址](https://github.com/FistsK/turtlebot2_withlidar.git) 

```cpp
git clone https://github.com/FistsK/turtlebot2_withlidar.git 
```
这两个文件夹是我已经配置好的，下载到你的turtlebot2源码配置工作区，覆盖掉相应文件夹即可。至于turtlebot2源码配置 
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
