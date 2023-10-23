# 在虚拟机器人中使用gazebo

> 在虚拟机器人中使用gazebo的原理是使用gzweb把gazebo的场景数据转换至网页端显示，需要注意的是由于虚拟机器人中并没有显示器（其实可以通过VNC打开虚拟机器人的图形界面，从而使用桌面版的gazbeo等其他图形界面应用，但这一点放到在虚拟机器人中使用VNC时再说），所以在这里启动的并不是gazebo而是gzserver。若是暂时不理解这一段话的含义也没关系，按照下面的步骤做一遍再回过头来看或许会有更多的理解。

在虚拟机器人中使用gazebo需要四步：

1. **开放容器端口**
   
   首先需要至少在创建虚拟机器人时向公网暴露一个端口用于访问gzweb（也就是gazebo的web版），gzweb默认监听的端口是8080，所以一般来说需要在创建虚拟机器人时设置向公网暴露8080端口，但是如果你有其他的需求或特殊的偏好，也可以自定义gzweb监听的端口，当然也就需要向公网暴露相应端口。
   <div align=center>
     <img src="http://192.168.37.130:8000/files/documentation/images/gazebo_expose_ports.png" width="50%">
   </div>
   在这里，可以看到向公网暴露了5900、8080、8443、9060、9091、9190六个端口，其中8443是VSCode-web的服务端口，5900是VNC的服务端口，其他四个都可以自由使用。

2. **启动gzserver**
   
   通过VSCode-web进入虚拟机器人后，在命令行输入以下命令即启动了gzserver：
   ```bash
      gzserver
   ```
   这时候没有任何输出是正常的，不要慌张，在执行以上命令之后会发现VSCode-web命令行Tab右侧的端口Tab出现了一个带有数字2的气泡，进入这个Tab会发现11345和36801端口被占用，这就证明gzserver的服务已经成功启动。

   <div align=center>
     <img src="http://192.168.37.130:8000/files/documentation/images/gazebo_gzserver_listening_ports.png" width="50%">
   </div>

   当然如果仅仅是这样启动gzserver，gazebo的世界里“白茫茫大地真干净”，另外网页端的gazebo对场景的载入和设置支持的并不好（例如甚至不支持物体缩放），所以很有必要在启动gazebo时导入场景模型。在启动gazebo时导入场景模型的命令是下面这句：

   ```bash
      gzserver [path to word file]
   ```

   这里[path to word file]代表`.world`场景文件的路径，绝对路径和相对路径都可以。以下是一个简单的场景文件`test_site.world`，相关图片资源已经预置在虚拟机器人中，可以直接使用。
   
   ```xml
      <?xml version ='1.0'?>
      <sdf version ='1.4'>

          <world name='default'>

              <include>
                  <uri>model://my_ground_plane</uri>
              </include>

              <include>
                  <uri>model://sun</uri>
              </include>

              <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
                  <ros>
                      <namespace>/gazebo</namespace>
                  </ros>
                  <update_rate>1.0</update_rate>
              </plugin> 

          </world>

      </sdf>
   ```

   此事我们还看不到gazebo的仿真环境，但是不要着急，如果一切顺利马上就能看到可视化的场景。
   
3. **启动gzweb**
   
   gzweb的源码已经被放置到根目录下，并且安装了相关依赖，执行以下命令即可启动gzweb：

   ```bash
      cd /gzweb
      port=8080 npm start
   ```

   在这里port=8080表示将gzweb服务启动在虚拟机器人的8080端口，可以指定成任何未被占用的端口（当然这个端口需要在创建虚拟机器人的时候暴露到公网，否则没有意义），如果缺省，默认值也是8080。

4. **访问页面**
   
   如果一切顺利，到这里就可以访问你的虚拟机器人中启动的gzweb了。首先回到远程开发界面，找到你的虚拟机器人，查看gzweb服务所在端口的公网映射信息。例如在这里，通过 <kbd>49226->9060</kbd> ，以及云测试场服务器的公网地址IP是192.168.37.130，可以知道我们需要访问 <kbd>http://192.168.37.130:49226</kbd> 。

   <div align=center>
     <img src="http://192.168.37.130:8000/files/documentation/images/gazebo_port_mapping_detail.png" width="90%">
   </div>

   打开该网页后，就可以看到gazebo场景了，该页面等价于一个几乎全功能的gazebo，页面中显示场景是由前面提到的.world文件描述的，通过修改场景文件或者直接拖放模型，可以变更gazebo场景中的布置，通过ROS向gazebo场景添加的机器人也会在gzweb上同步显示。

   <details>
      <summary>图片-gzweb页面</summary>
      <div align=center>
        <img src="http://192.168.37.130:8000/files/documentation/images/gazebo_gzweb.png" width="90%">
      </div>
   </details>