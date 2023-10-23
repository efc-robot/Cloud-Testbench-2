# 真实机器人申请及使用




## 一、申请机器人

用户登录云测试场后，点击 `查看机器人列表` 页面的表格中找到想要申请的机器人，在该机器人所在行的 `操作` 栏下的 `申请` 按钮，在弹出窗口中填写机器人控制节点配置信息并提交。（ 需要注意的是，被申请的机器人需要在线且未被分配，否则申请按钮将处在禁用状态。 ）

<div align=center>
    <img src="http://192.168.37.130:8000/files/documentation/images/real_robot_apply.png" width="50%">
</div>

* **表单配置项说明：**

  | 配置项 | 必选 | 作用 |
  | :----: | :----: | ---- |
  | 机器人UUID | ✅ | 标识申请的机器人 |
  | 机器人用户名 | ✅ | 云测试场将以此处配置的用户名进行机器人本地文件系统的FTP挂载 |
  | VSCode密码 |  | 进入VSCode-web时的密码，同时也将作为VSCode-web内部操作系统密码 |
  | VSCode-web工作空间 | ✅ | 在此处配置将机器人的系统目录挂载到VSCode-web中的哪个目录 |

申请完成后该机器人将被添加到 `机器人远程开发` 页面的 `真实机器人` 表格中。同时该机器人在 `查看机器人列表` 页面中的状态将变更为 `占用` ,其他用户无法再进行申请。

<div align=center>
    <img src="http://192.168.37.130:8000/files/documentation/images/real_robot_allocated.png" width="50%">
</div>




## 二、对机器人端代码进行开发

完成机器人申请后，在 `机器人远程开发` 页面的 `真实机器人` 表格中，点击机器人所在行的 `操作` 栏下的 `VSCode-web` 按钮，将跳转至VSCode-web页面。

<div align=center>
    <img src="http://192.168.37.130:8000/files/documentation/images/my_real_robot.png" width="50%">
</div>

在VSCode-web页面中左侧的文件浏览面板中显示的是挂载到VSCode-web容器中的机器人本地目录，在此处打开文件，即可在VSCode-web中直接对连接到云测试场的机器人本地的文件进行编辑。

<div align=center>
    <img src="http://192.168.37.130:8000/files/documentation/images/real_robot_vscode_web.png" width="50%">
</div>




## 三、机器人远程控制

1. 在VSCode-web中使用快捷键 <kbd>Ctrl</kbd> + <kbd>`</kbd> 或点击左上角三条杠形状的菜单栏，选择 <kbd>Terminal</kbd> -> <kbd>New Terminal</kbd> 打开命令行。

2. 执行指令：
   
   ```bash
      ssh haro@192.168.3.12
   ```

   并根据提示输入密码，通过ssh接入真实机器人。

   <div align=center>
       <img src="http://192.168.37.130:8000/files/documentation/images/real_robot_ssh.png" width="50%">
   </div>

   * 命令中的`haro`需要修改为你所使用的真实机器人的用户名，该配置可以在`机器人远程开发`页面中的`真实机器人`表格中相应机器人信息的`机器人端OS用户名`中找到
   * `192.168.3.12`需要修改为你所使用的真实机器人的IP，该配置可以在`机器人远程开发`页面中的`真实机器人`表格中相应机器人信息的`机器人局域网IP`中找到
   * 执行该命令是提示输入的机器人密码可以在`机器人远程开发`页面中的`真实机器人`表格中相应机器人信息的`机器人端OS密码`中找到

   > 在VSCode-web打开的命令行，以及在命令行中建立的ssh连接，不会随着VSCode-web页面的关闭而丢失，在下次打开VSCode-web页面，再次呼出命令行时会恢复到上次退出时的状态。
   > 但它会在释放真实机器人使用权时，以及机器人离线时所导致的VSCode-web控制节点销毁而销毁




## 四、释放真实机器人

完成机器人申请后，在 `机器人远程开发` 页面的 `真实机器人` 表格中，点击机器人所在行的 `操作` 栏下的 `释放` 按钮，即可释放机器人使用权，机器人将恢复到未分配状态，同时将销毁申请机器人时创建的VSCode-web控制节点。

<div align=center>
    <img src="http://192.168.37.130:8000/files/documentation/images/my_real_robot.png" width="50%">
</div>
