# 云测试场网盘webdav挂载

云测试场网盘支持webdav协议，因此可以通过webdav协议挂载云测试场网盘目录。在云测试场中，可以在虚拟机器人、VSCode控制节点、真实机器人中进行云测试场网盘的webdav挂载。




## 一、通过webdav挂载云测试场网盘

1. **安装依赖**

   执行以下指令安装基础依赖：

   ```bash
      sudo apt-get install davfs2 -y
   ```

2. **创建挂载路径**
   
   假设我们要将云测试场网盘挂载到 `/home/seafile` 目录下，执行以下命令创建挂载点：

   ```bash
      mkdir -p /home/seafile
   ```

3. **执行挂载指令**
   
   假设云测试场的公网IP为 `111.207.104.144` ，执行以下命令将云测试场网盘目录挂载到 `/home/seafile` 目录：

   ```bash
      sudo mount -t davfs -o noexec http://111.207.104.144:9080/seafdav /home/seafile
   ```

   执行该指令后，会提示输入账号密码，在此处输入你的云测试场网盘账号和密码即可




## 二、解除云测试场网盘挂载

1. **执行解除挂载指令**
   
   执行以下命令解除 `/home/seafile` 目录下的云测试场网盘目录挂载：

   ```bash
      sudo umount /home/seafile
   ```
   
   当虚拟机器人销毁、VSCode控制节点销毁、真实机器人下线时，云测试场网盘挂载都会自动解除。