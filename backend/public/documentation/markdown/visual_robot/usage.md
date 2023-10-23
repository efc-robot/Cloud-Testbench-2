# 虚拟机器人的使用

## 一、创建虚拟机器人

在 <kbd>机器人远程开发</kbd> -> <kbd>虚拟机器人</kbd> 界面，点击 <kbd>添加虚拟机器人</kbd> ，按照需求进行配置并提交。

<div align=center>
   <img src="http://192.168.37.130:8000/files/documentation/images/visual_robot_form.png" width="50%">
</div>

当提示虚拟机器人创建成功，该虚拟机器人将会被加入到 <kbd>机器人远程开发</kbd> -> <kbd>虚拟机器人</kbd> 的表格中。

<div align=center>
   <img src="http://192.168.37.130:8000/files/documentation/images/visual_robot_created.png" width="50%">
</div>

| 配置项 | 必选 | 作用 |
| :----: | :----: | ---- |
| 容器镜像 | ✅ | 虚拟机器人使用的docker容器镜像，选择的镜像决定了虚拟机器人的操作系统版本，以及基本环境配置 |
| 机器人名称 | ✅ | 机器人的昵称。机器人的唯一标识是机器人UUID，但是由于UUID的可读性差，故提供自定义昵称功能，便于识别 |
| ubuntu密码 | ✅ | 在这里配置的密码将作为虚拟机器人中运行的ubuntu系统中root用户的密码 |
| VSCode-web密码 |  | 如果在这里配置了密码，在打开VSCode-web页面时将需要输入该密码，如果缺省该项，打开VSCode-web时将直接进入主界面，无需输入密码 |
| 默认工作目录 |  | 打开VSCode-web页面时，VSCode默认打开的工作目录，默认填写了<kbd>/</kbd>也即根目录，可以根据自己的喜好进行更改，但不能输入不存在的路径，当此项缺省时，默认工作目录将被设置为<kbd>/conifg</kbd> |
| 开放端口 |  | 虚拟机器人内部需要进行公网映射的端口 |




## 二、虚拟机器人的端口映射

由于虚拟机器人处在云测试场局域网中，在公网是无法访问虚拟机器人上的端口的，如果需要在公网访问虚拟机器人的某个端口，需要在创建虚拟机器人的表格中配置开放端口，云测试场将为开放端口分配一个公网映射端口映射。需要注意的是，由于docker容器的特性，开放端口配置必须要在创建虚拟机器人时添加，当创建完毕后不能再新增开放端口。

预设的三个个开放端口`8443`、`5900`和`9091`分别是VSCode-web、VNC服务和nginx服务的预留端口，禁止移除8443的端口开放配置，否则将无法访问虚拟机器人，建议不要对`5900`和`9091`的端口开放配置进行改动，除非你非常明确你的操作结果。

<div align=center>
  <img src="http://192.168.37.130:8000/files/documentation/images/visual_robot_port_mapping.png" width="20%">
</div>

虚拟机器人列表的**端口映射**中会列出创建虚拟机器人时配置的开放端口向公网的映射情况。该列中的每一项代表着一条端口映射规则。例如 <kbd>49209 -> 9091</kbd> 表示将虚拟机器人的9091端口映射到云测试场的49209公网端口，也就是说例如云测试场的公网ip是111.207.104.144，此时访问111.207.104.144:49209即相当于访问机器人的9091端口。直接点击映射规则将跳转至相应url，例如点击<kbd>49209 -> 9091</kbd>将直接跳转至`http://111.207.104.144:49209`




## 三、使用虚拟机器人

在 <kbd>机器人远程开发</kbd> -> <kbd>虚拟机器人</kbd> 的表格查看当前用户创建的虚拟机器人
<div align=center>
   <img src="http://192.168.37.130:8000/files/documentation/images/visual_robot_created.png" width="50%">
</div>

云测试场以VSCode-web作为虚拟机器人的操作入口。在表格的最右侧的`操作`中点击`VSCode-web`，则会打开新的窗口，进入该虚拟机器人的VSCode-web操作界面。机器人刚创建的数秒内进入VSCode-web显示找不到网页是正常的，这是因为容器创建需要一些时间，稍等几秒即可。

<div align=center>
   <img src="http://192.168.37.130:8000/files/documentation/images/visual_robot_vscode_web_main.png" width="50%">
</div>

**注意：**
* 如果在创建虚拟机器人时设置了VSCode-web密码，需要在密码界面输入该密码。
   <details>
      <summary>图片-VSCode-web密码界面</summary>
      <div align=center>
        <img src="http://192.168.37.130:8000/files/documentation/images/visual_robot_vscode_web_password.png" width="90%">
      </div>
   </details>
* 在VSCode-web界面，可以通过图形界面选项，或直接使用快捷键<kbd>ctrl</kbd>+<kbd>`</kbd>即可打开命令行，这一点以及其他的VSCode功能和桌面端的VSCode是一致的。包括在VSCode-web中，你也可以安装各种你需要的各种插件。
   <details>
      <summary>图片-VSCode-web命令行</summary>
      <div align=center>
        <img src="http://192.168.37.130:8000/files/documentation/images/visual_robto_vscode_web_terminal.png" width="90%">
      </div>
   </details>
* 进入虚拟机器人的VSCode-web操作界面，等效于在虚拟机器人的ubuntu系统中打开的一个VSCode应用，左侧按照创建机器人时配置的默认工作目录，将工作目录配置到相应位置。
* 进入VSCode-web之后，在创建或修改文件时，会发现除了/config目录外都没有编辑权限，这是为了避免以root用户权限创建文件导致的一些问题。但是没关系，如果确实需要使用某个目录或文件，使用刚才创建虚拟机器人时设置的ubuntu密码修改你需要使用的文件和目录的权限即可。




## 四、销毁虚拟机器人

在表格的最右侧的**操作**中点击**销毁**按钮，即可销毁该虚拟机器人，该操作会永久销毁该虚拟机器人及其虚拟环境下的所有文件和环境配置。除非服务器重启或用户主动删除，你创建的虚拟机器人不会随着退出云测试场而被销毁，你可以保留虚拟机器人以重复利用在该机器人中配置的开发环境，但是由于虚拟机器人对场地服务器的存储和运算都会造成一定的负荷，建议及时销毁不再使用的虚拟机器人。
