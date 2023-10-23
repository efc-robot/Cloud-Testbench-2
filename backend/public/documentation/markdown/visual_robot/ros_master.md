# 虚拟机器人作为真实机器人的ROS master节点（ROS1）

虚拟机器人与真实机器人处在同一个局域网内，因此可以将虚拟机器人设置为真实机器人的ROS master节点，需要进行以下3步操作：




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