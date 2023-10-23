# 真实机器人接入云测试场




## 一、在云测试场中注册

> 注意： 将机器人注册到云测试场需要云测试场管理员权限

1. **注册机器人信息**
   
   在云测试场的 `云测试场管理` -> `机器人管理` 页面点击 `添加机器人` 按钮，填写机器人信息并提交。

   <div align=center>
      <img src="http://192.168.37.130:8000/files/documentation/images/real_robot_register.png" width="50%">
   </div>

   注册完成后机器人信息将被添加到 `云测试场管理` -> `机器人管理` 页面的列表中。

   * **表单配置项说明：**

     | 配置项 | 必选 | 作用 |
     | :----: | :----: | ---- |
     | 机器人名称 | ✅ | 机器人的昵称。机器人的唯一标识是机器人UUID，但是由于UUID的可读性差，故提供自定义昵称功能，便于识别 |
     | 机器人类型 | ✅ | 在云测试场的预置机器人类型中进行选择 |
     | 机器人IP | ✅ | 机器人在云测试场局域网内的IP |


2. **配置机器人FTP服务信息**

    在云测试场的 `云测试场管理` -> `ftp配置管理` 页面点击 `添加ftp配置` 按钮，填写机器人的FTP服务配置信息并提交。

   <div align=center>
      <img src="http://192.168.37.130:8000/files/documentation/images/ftp_config_register.png" width="50%">
   </div>

   注册完成后机器人信息将被添加到 `云测试场管理` -> `机器人管理` 页面的列表中。

   * **表单配置项说明：**

     | 配置项 | 必选 | 作用 |
     | :----: | :----: | ---- |
     | 机器人UUID | ✅ | 机器人在云测试场系统中的UUID，用于识别该FTP服务配置从属于哪一台机器人，UUID在机器人注册时分配，可以在查看机器人列表页面查看。 |
     | 机器人用户名 | ✅ | 机器人本地操作系统的用户名 |
     | 机器人密码 | ✅ | 机器人本地操作系统的密码 |
     | 机器人本地工作目录 | ✅ | 在此处配置的机器人本地文件系统路径将在申请使用该机器人时被挂载到控制节点（VSCode-web）中 |
     | FTP服务端口 | ✅ | 机器人端FTP服务所监听的端口 |




## 二、配置机器人环境

1. **安装基本依赖**

   ``` bash
       sudo apt install vsftpd curl
   ```

2. **配置FTP服务**
   
   编辑FTP服务配置文件`/etc/vsftpd.conf`，找到配置项`write_enable=YES`，如果该配置项被注释，取消该配置项的注释。执行指令：

   ```bash
      sudo /etc/init.d/vsftpd restart
   ```

   重启FTP服务。

3. **配置机器人上线脚本**
   
   编写上线脚本 `online.sh` ，注意：
   * `HOST`和`PORT`需要修改为云测试场服务在局域网内的IP和服务端口
   * `ROBOT_UUID`需要修改为机器人在云测试场中注册的UUID，该信息可以在云测试场的`查看机器人列表`页面该机器人的注册信息中看到
   * `ROBOT_USERNAME`需要修改为机器人本地操作系统使用的用户名。
   
   ```bash
      HOST="192.168.3.36"
      PORT="8000"
      ROUTER="/robot/online"
      ROBOT_UUID="ae860164-1b05-11ee-bf2f-6f6bd544c18f"
      ROBOT_USERNAME="nvidia"

      while true; do
              curl -X POST http://${HOST}:${PORT}${ROUTER} \
              --header 'Content-Type: application/json' \
              --data-raw '{"robot_uuid": "'${ROBOT_UUID}'","robot_username": "'${ROBOT_USERNAME}'"}'
              sleep 2s
      done
   ```

4. **将机器人上线脚本添加到系统启动项**
   
   创建服务单元文件`/etc/systemd/system/ct-robot-online.service`，其中第9行中的 “/home/nvidia/cloud_testbench/robot_online” 需要修改为实际存放上线脚本`online.sh`的目录。

   ```bash
      [Unit]
      Description=ct-robot-online

      Wants=vsftpd.service
      After=vsftpd.service

      [Service]
      Type=simple
      ExecStart=/bin/bash -c "cd /home/nvidia/cloud_testbench/robot_online && ./online.sh"

      [Install]
      WantedBy=multi-user.target
   ```

   执行以下命令重新加载systemd并启用服务：

   ```bash
      sudo systemctl daemon-reload          # 重新加载 systemd 服务配置
      sudo systemctl enable ct-robot-online # 启用服务
      sudo systemctl start ct-robot-online
      sudo systemctl status ct-robot-online # 确认服务状态
   ```