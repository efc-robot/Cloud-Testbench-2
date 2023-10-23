# 快速开始

完成以下三个步骤，动捕系统将捕获机器人的位姿信息并通过vrpn实时传输：

1. **粘贴标记点**
   
   在机器人上粘贴标记球，需要保证至少有不少于3个的标记球可以被动捕系统捕捉到。

   <div align=center>
     <img src="http://192.168.37.130:8000/files/documentation/images/akm_bot.jpeg" width="30%">
   </div>




2. **创建刚体**
   
   打开动捕软件，在动捕软件中选中标记点，右键打开刚体创建选项，创建刚体。需要注意的是，需要选中不少于3个标记点才可以创建一个刚体。

   <div align=center>
     <img src="http://192.168.37.130:8000/files/documentation/images/optitrack_select_rigid.png" width="50%">
   </div>

   刚体创建后，可以在右侧的资产列表中查看。

   <div align=center>
     <img src="http://192.168.37.130:8000/files/documentation/images/optitrack_rigid_created.png" width="50%">
   </div>




3. **发送数据**
   
   在软件界面的数据传输面板中配置传输参数，点击开始传输按钮，动捕软件开始实时传输场景中刚体位姿信息。

   <div align=center>
     <img src="http://192.168.37.130:8000/files/documentation/images/optitrack_transport_config_1.png" width="30%">
   </div>

   <div align=center>
     <img src="http://192.168.37.130:8000/files/documentation/images/optitrack_transport_config_2.png" width="30%">
   </div>