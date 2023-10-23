# Quick Start

云测试场提供foxGlove作为rviz的网页端替代，由于DDS协议的限制，目前暂时仅支持在网页端显示真实机器人中发布的ROS1话题数据。使用网页端的rviz分为4步：

1. **创建虚拟机器人作为ROS Master节点**
   
   * 参考 `使用文档` -> `虚拟机器人` -> `作为ROS Master节点` 文档创建虚拟机器人（**注意，在创建虚拟机器人时需要添加一个端口映射用于Nginx服务，在创建虚拟机器人时，云测试场默认配置了 `9091` 端口的公网映射，因此对在本示例中，我们使用 `9091` 端口作为Nginx代理端口**）
   * 在虚拟机器人中执行下面指令：
     ```bash
       source /opt/ros/melodic/setup.bash
       roscore
     ```
     以启动matser节点，需要注意的是，在这里需要根据使用的虚拟机器人镜像及实际需求，将指令 `source /opt/ros/melodic/setup.bash` 中的“melodic”修改为合适的ros版本名称
   
2. **发布话题数据**
   
   * 在发送话题数据的机器人、虚拟机器人或控制节点中，将该虚拟机器人设置为真实机器人的ROS master节点
   * 按照自身需求，发布话题数据

3. **配置Nginx转发**
   
   由于websocket协议的限制，无法直接通过端口映射将9090端口发布到公网，因此需要通过以下步骤在虚拟机器人中通过nginx对9090端口进行反向代理。

   * 执行指令安装Nginx：

     ```bash
        sudo apt install nginx
     ```

   * 在 `/etc/nginx/conf.d` 下创建配置文件 `foxGlove.conf` ，并在其中写入以下内容:

     ```
        server {
             listen 9091;

             location / {
                 proxy_pass http://0.0.0.0:9090;
                 proxy_http_version 1.1;
                 proxy_set_header Upgrade $http_upgrade;
                 proxy_set_header Connection "Upgrade";
             }
        }
     ```
    
    * 执行以下命令重载Nginx：
      
      ```bash
        sudo service nginx restart
      ```

      重载完成和虚拟机器人的9090端口被映射到虚拟机器人的9091端口。

4. **启动ROS bridge**
   
   在作为ROS Master节点的虚拟机器人中执行指令：
   
   ```bash
      roslaunch rosbridge_server rosbridge_websocket.launch
   ```
   
   将rosbridge启动在9090端口上，该端口将被Nginx转发至9091端口。

5. **在foxglove添加数据源进行可视化**
   
   * 在虚拟机器人列表中找到作为ROS Master节点的虚拟机器人，查看为该虚拟机器人的9091端口分配的公网端口映射，例如在这里可以看到9091端口的公网端口映射为49215

     <div align=center>
         <img src="http://192.168.37.130:8000/files/documentation/images/visual_robot_created.png" width="50%">
     </div>

   * 点击 `开发工具` 页面的 `Rviz Web` 卡片中的 `Open` 按钮，即可跳转至foxglove页面

     <div align=center>
         <img src="http://192.168.37.130:8000/files/documentation/images/foxglove.png" width="50%">
     </div>

   * 点击 `Open Connection` 选项，选择 `Rosbrigde (ROS 1 & 2)` ，将 `WebSocket URL` 中的“localhost”修改为云测试场的公网地址，将端口号9090修改成为虚拟机器人的9091端口分配的公网映射端口，例如云测试场公网IP为111.207.104.144，虚拟机器人9091端口的公网端口映射为49215，则此处配置为 `ws://111.207.104.144:49215`

     <div align=center>
         <img src="http://192.168.37.130:8000/files/documentation/images/foxglove_config_connection.png" width="50%">
     </div>
    
     配置完成后，点击Open按钮完成数据源配置，可以开始在foxglove页面中查看话题数据信息

     <div align=center>
         <img src="http://192.168.37.130:8000/files/documentation/images/foxglove_data_loaded.png" width="50%">
     </div>