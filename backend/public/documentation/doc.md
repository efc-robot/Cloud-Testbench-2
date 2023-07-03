# Cloud Testbench
<div align=center>
  <img src="http://192.168.37.130:8000/files/documentation/images/banner.png" width="70%">
</div>
云测试场是一套远程对机器人进行软件开发的工作平台，提供对远程机器人中的代码进行开发和运行，远程查看机器人ROS话题数据，通过数字孪生远程监控机器人位姿信息三项主要功能。

## 虚拟机器人

云测试场中的虚拟机器人可以理解为云测试场局域网内的一台虚拟机（实际上是一个docker容器，如果有这方面的知识更好，如果没有的话直接理解为虚拟机即可），虚拟机器人上运行ubuntu20.04系统，预装了ros noetic，ros galactic，ros foxy三个版本的ros。

虚拟机器人环境信息：

| 环境 |  |
| :----: | --- |
| ubuntu版本 | `20.04` |
| ROS版本 | `noetic` `galactic` `foxy` |
| gazebo版本 | `11.11.0` |
| nginx | ✅ |

在云测试场中，虚拟机器人主要起到以下几个作用：

1. 虚拟机器人首先可以单纯地作为一个运行ubuntu系统的ROS开发环境虚拟机，用来验证你的代码和算法，或者任何其他在ubuntu系统下进行的开发工作，唯一的区别是暂时没有图形化界面，可能需要你具备通过命令行使用ubuntu系统的基本知识。
2. 其次虚拟机器人可以作为ROS1的master节点，当你创建一个虚拟机器人后，真实机器人可以将master节点设置为该虚拟机器人，这样做地好处首先是真实机器人不需要承担作为master节点的运算负载，其次由于虚拟机器人依托于场地服务器硬件存在，稳定性和性能都更有保障。
3. 由于真实机器人处在局域网中并不对外暴露，当你想要通过rviz-web(foxGlove)查看可视化的真实机器人激光雷达数据和节点图时，需要借助虚拟机器人作为桥梁，关于这一点的细节后面的文档中会详细说明。

### 创建虚拟机器人

当需要创建虚拟机器人时，在 <kbd>机器人远程开发</kbd> -> <kbd>虚拟机器人</kbd> 界面，点击 <kbd>添加虚拟机器人</kbd> 按钮，会弹出虚拟机器人配置界面，按照需求进行配置并提交即可
<details>
 <summary>图片-虚拟机器人列表</summary>
  <div align=center>
     <img src="http://192.168.37.130:8000/files/documentation/images/visual_robot_list.png" width="90%">
   </div>
</details>
<details>
 <summary>图片-设置虚拟机器人配置</summary>
   <div align=center>
     <img src="http://192.168.37.130:8000/files/documentation/images/visual_robot_form.png" width="90%">
   </div>
</details>

以下是虚拟机器人配置界面各配置项的作用：

| 配置项 | 必选 | 作用 |
| :----: | :----: | ---- |
| 机器人名称 | ✅ | 机器人的昵称。机器人的唯一标识是机器人id，但是由于id的可读性差，故提供自定义昵称功能，便于识别 |
| ubuntu密码 | ✅ | 在这里配置的密码将作为虚拟机器人中运行的ubuntu系统中root用户的密码 |
| VSCode-web密码 |  | 如果在这里配置了密码，在打开VSCode-web页面时将需要输入该密码，如果缺省该项，打开VSCode-web时将直接进入主界面，无需输入密码 |
| 默认工作目录 |  | 打开VSCode-web页面时，VSCode默认打开的工作目录，默认填写了<kbd>/</kbd>也即根目录，可以根据自己的喜好进行更改，但不能输入不存在的路径，当此项缺省时，默认工作目录将被设置为<kbd>/conifg</kbd> |
| 开放端口 |  | 由于虚拟机器人处在云测试场局域网中，在公网是无法访问虚拟机器人上的端口的，如果需要在公网访问虚拟机器人的某个端口，需要在此处设置向公网开放该端口,云测试场将为开放端口分配一个公网映射端口映射。需要注意的是，开放端口配置必须要在创建虚拟机器人时添加，当创建完毕后不能再新增开放端口。预设的两个开放端口8443和9091分别是VSCode-web和nginx服务的预留端口，禁止移除8443的端口开放配置，否则将无法访问虚拟机器人，建议不要对9091的端口开放配置进行改动，除非你非常明确你的操作结果。 |

(在后续的**虚拟机器人的端口映射**部分将对开放端口进行进一步说明。)

### 使用虚拟机器人

创建虚拟机器人后，可以在 <kbd>机器人远程开发</kbd> -> <kbd>虚拟机器人</kbd> 的表格中看到该虚拟机器人
<details>
  <summary>图片-列表中显示创建的机器人</summary>
  <div align=center>
    <img src="http://192.168.37.130:8000/files/documentation/images/visual_robot_created.png" width="90%">
  </div>
</details>

云测试场以VSCode-web作为虚拟机器人的操作入口。在表格的最右侧的**操作**中点击VSCode-web，则会打开新的窗口，进入该虚拟机器人的VSCode-web操作界面。机器人刚创建的数秒内进入VSCode-web显示找不到网页是正常的，这是因为容器创建需要一些时间，稍等几秒即可。

* 如果在创建虚拟机器人时设置了VSCode-web密码，需要在密码界面输入该密码。
   <details>
      <summary>图片-VSCode-web密码界面</summary>
      <div align=center>
        <img src="http://192.168.37.130:8000/files/documentation/images/visual_robot_vscode_web_password.png" width="90%">
      </div>
   </details>
 * 进入虚拟机器人的VSCode-web操作界面，等效于在虚拟机器人的ubuntu系统中打开的一个VSCode应用，左侧按照创建机器人时配置的默认工作目录，将工作目录配置到相应位置。
   <details>
      <summary>图片-VSCode-web主界面</summary>
      <div align=center>
        <img src="http://192.168.37.130:8000/files/documentation/images/visual_robot_vscode_web_main.png" width="90%">
      </div>
   </details>
 * 在VSCode-web界面，可以通过图形界面选项，或直接使用快捷键<kbd>ctrl</kbd>+<kbd>`</kbd>即可打开命令行，这一点以及其他的VSCode功能和桌面端的VSCode是一致的。包括在VSCode-web中，你也可以安装各种你需要的各种插件。
   <details>
      <summary>图片-VSCode-web命令行</summary>
      <div align=center>
        <img src="http://192.168.37.130:8000/files/documentation/images/visual_robto_vscode_web_terminal.png" width="90%">
      </div>
   </details>

另外，进入VSCode-web之后，在创建或修改文件时，会发现除了/config目录外都没有编辑权限，这是为了避免以root用户权限创建文件导致的一些问题。但是没关系，如果确实需要使用某个目录或文件，使用刚才创建虚拟机器人时设置的ubuntu密码修改你需要使用的文件和目录的权限即可。

### 虚拟机器人的端口映射

<div align=center>
  <img src="http://192.168.37.130:8000/files/documentation/images/visual_robot_port_mapping.png" width="20%">
</div>

表格的**端口映射**中会列出创建虚拟机器人时配置的开放端口向公网的映射情况。该列中的每一项代表着一条端口映射规则，例如 <kbd>49155 -> 9091</kbd> 表示将虚拟机器人的9091端口映射到云测试场的49155公网端口，也就是说例如云测试场的公网ip是111.207.104.144，此时访问111.207.104.144:49155即相当于访问机器人的9091端口。

### 销毁虚拟机器人

在表格的最右侧的**操作**中点击**销毁**按钮，即可销毁该虚拟机器人，该操作会永久销毁该虚拟机器人及其虚拟环境下的所有文件和环境配置。除非服务器重启或用户主动删除，你创建的虚拟机器人不会随着退出云测试场而被销毁，你可以保留虚拟机器人以重复利用在该机器人中配置的开发环境，但是由于虚拟机器人对场地服务器的存储和运算都会造成一定的负荷，建议及时销毁不再使用的虚拟机器人。

### 虚拟机器人作为真实机器人的ROS master节点（ROS1）

将虚拟机器人设置为真实机器人的ROS master节点分为3步：
1. **获取虚拟机器人在测试场局域网中的IP**

   虚拟机器人与真实机器人共同处在云测试场真实场地的局域网中，在VSCode-web中打开命令行，通过<kbd>ifconfig</kbd>命令可以查看虚拟机器人的网络配置，其中eth1网卡的IP地址是虚拟机器人在测试场局域网中的IP，例如在下面这个例子中，虚拟机器人在测试场局域网中的IP为<kbd>172.16.0.2</kbd>。
   ```bash
   abc@16aeb21e2dc0:/$ ifconfig 
   eth0: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
           inet 172.17.0.2  netmask 255.255.0.0  broadcast 172.17.255.255
           ether 02:42:ac:11:00:02  txqueuelen 0  (Ethernet)
           RX packets 1046  bytes 312691 (312.6 KB)
           RX errors 0  dropped 0  overruns 0  frame 0
           TX packets 982  bytes 4764438 (4.7 MB)
           TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

   eth1: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
           inet 172.16.0.2  netmask 255.255.255.0  broadcast 172.16.0.255
           ether 02:42:ac:10:00:02  txqueuelen 0  (Ethernet)
           RX packets 10  bytes 600 (600.0 B)
           RX errors 0  dropped 0  overruns 0  frame 0
           TX packets 0  bytes 0 (0.0 B)
           TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

   lo: flags=73<UP,LOOPBACK,RUNNING>  mtu 65536
           inet 127.0.0.1  netmask 255.0.0.0
           loop  txqueuelen 1000  (Local Loopback)
           RX packets 12  bytes 841 (841.0 B)
           RX errors 0  dropped 0  overruns 0  frame 0
           TX packets 12  bytes 841 (841.0 B)
           TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0
   ```
2. **设置ROS_MASTER_URI**
   假设虚拟机器人在局域网在IP地址为172.16.0.2，在虚拟机器人端和真实机器人端通过命令:
   ```bash
      export ROS_MASTER_URI=172.16.0.2
   ```
   将虚拟机器人设置为master节点
3. **启动master节点**
   在虚拟机器人端执行以下命令，启动ROS master节点：
   ```bash
      source /opt/ros/noetic/setup.bash
      roscore
   ```

<!-- （可以放到rviz-web的文档中） -->
### 以虚拟机器人作为中介使用rviz-web（ROS1）
> 目前暂时仅支持ROS1下的以虚拟机器人作为中介使用rviz-web

以虚拟机器人作为中介使用rviz-web分为4步：
1. **设置虚拟机器人为master节点**
   参考<kbd>虚拟机器人作为真实机器人的ROS master节点</kbd>文档，将虚拟机器人设置为真实机器人的ROS master节点，并启动matser节点
2. **启动rosbridge**
   执行以下命令，启动rosbridge，将ROS的话题信息发布到虚拟机器人的9090端口：
   ```bash
      source /opt/ros/noetic/setup.bash
      roslaunch rosbridge_server rosbridge_websocket.launch
   ```
3. **打开nginx服务**
   由于websocket协议的限制，无法直接通过端口映射将9090端口发布到公网，因此需要在虚拟机器人中通过nginx对9090端口进行反向代理。执行以下命令启动nginx服务，即可将虚拟机器人的9090端口映射到虚拟机器人的9091端口：
   ```bash
      sudo service nginx start
   ```
   这个映射规则是由预置在<kbd>/etc/nginx/conf.d</kbd>下的配置文件<kbd>foxGlove.conf</kbd>确定的，而<kbd>9091</kbd>端口是预设的虚拟机器人开放端口之一，在虚拟机器人创建时已经被映射至公网。
4. **参考端口映射规则，设置rviz-web数据源**
   在虚拟机器人列表中，找到当前虚拟机器人的9091端口的映射信息，将其填入rviz-web的数据源配置中即可。假设云测试场的公网地址是<kbd>111.207.104.144</kbd>，并且虚拟机器人的端口映射信息为<kbd>49155 -> 9091</kbd>，则在云测试场的<kbd>rviz-web</kbd> -> <kbd>open connection</kbd> -> <kbd>Rosbridge</kbd> -> <kbd>WebSocket URL</kbd>中填入<kbd>ws://111.207.104.144:49155</kbd>，即可在rviz-web中查看到ROS的话题数据。

### 在虚拟机器人中使用gazebo

> 在虚拟机器人中使用gazebo的原理是使用gzweb把gazebo的场景数据转换至网页端显示，需要注意的是由于虚拟机器人中并没有显示器（其实可以通过VNC打开虚拟机器人的图形界面，从而使用桌面版的gazbeo等其他图形界面应用，但这一点放到在虚拟机器人中使用VNC时再说），所以在这里启动的并不是gazebo而是gzserver。若是暂时不理解这一段话的含义也没关系，按照下面的步骤做一遍再回过头来看或许会有更多的理解。

在虚拟机器人中使用gazebo需要四步：

1. **开放容器端口**
   
   首先需要至少在创建虚拟机器人时向公网暴露一个端口用于访问gzweb（也就是gazebo的web版），gzweb默认监听的端口是8080，所以一般来说需要在创建虚拟机器人时设置向公网暴露8080端口，但是如果你有其他的需求或特殊的偏好，也可以自定义gzweb监听的端口，当然也就需要向公网暴露相应端口。
   <details>
      <summary>图片-设置向公网暴露的端口</summary>
      <div align=center>
        <img src="http://192.168.37.130:8000/files/documentation/images/gazebo_expose_ports.png" width="90%">
      </div>
   </details>
   在这里，可以看到向公网暴露了5900、8080、8443、9060、9091、9190六个端口，其中8443是VSCode-web的服务端口，5900是VNC的服务端口，其他四个都可以自由使用。

2. **启动gzserver**
   
   通过VSCode-web进入虚拟机器人后，在命令行输入以下命令即启动了gzserver：
   ```bash
      gzserver
   ```
   这时候没有任何输出是正常的，不要慌张，在执行以上命令之后会发现VSCode-web命令行Tab右侧的端口Tab出现了一个带有数字2的气泡，进入这个Tab会发现11345和36801端口被占用，这就证明gzserver的服务已经成功启动。

   <details>
      <summary>图片-查看gzserver占用端口</summary>
      <div align=center>
        <img src="http://192.168.37.130:8000/files/documentation/images/gazebo_gzserver_listening_ports.png" width="90%">
      </div>
   </details>

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