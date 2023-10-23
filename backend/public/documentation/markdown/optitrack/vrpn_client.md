# 在ROS中接收动捕数据




## 一、在ROS1中接收动捕数据

1. **安装ros vrpn库**

   执行以下命令安装ros的VRPN库，其中 `${ROS_DISTRO}` 替换为实际使用的ROS1版本：
   
   ```bash
      sudo apt install ros-${ROS_DISTRO}-vrpn*
   ```

2. **运行ros包：vrpn_client_ros**

   执行以下命令运行vrpn_client_ros，其中 `${VRPN_SERVER}` 替换动捕服务所在设备在局域网中的IP：

   ```bash
      source /opt/ros/${ROS_DISTRO}/setup.bash
      roslaunch vrpn_client_ros sample.launch server:=${VRPN_SERVER}
   ```

   执行该指令后，VRPN动捕数据将被转发为ROS1话题




## 二、在ROS2中接收动捕数据

ROS2的官方并未提供VRPN client端的相关库，因此我们提供一个在ROS2中的VRPN client实现，需要从github仓库拉取代码并编译执行。
1. **拉取代码**

   执行以下命令拉取代码：

   ```bash
      git clone https://github.com/efc-robot/vrpn_client_ros2
   ```

2. **编译和执行**
   
   拉取完毕后按照 `README.md` 文档进行安装和使用即可，此处略