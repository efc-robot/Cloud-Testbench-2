# VNC的使用




## 一、打开VNC远程桌面

1. 创建虚拟机器人
   * 创建机器人时需要选择带有VNC的镜像（例如ct_visual_robot:bionic_vnc_dav_sc_v2）
   * 创建虚拟机器人时需要添加端口`5900`
2. 启动VNC服务
   * 将`/home/scripts/vnc_run.sh`中的`DISPLAY_INDEX`修改为2
   * 执行`/home/scripts/vnc_run.sh`
3. 进入VNC
   * 点击`虚拟机器人`页面对应虚拟机器人的`端口映射`栏中对应5900端口的按钮打开VNC页面
   * 点击VNC初始页面中的`vnc.html`
   * 点击`Connect`按钮
   * 输入密码（密码：thudrone）


## 二、VNC虚拟桌面相关配置

1. 修改VNC远程桌面分辨率
   * 将`/home/scripts/vnc_run.sh`中的`GEOMETRY`配置项修改为想要的分辨率
   * 重启VNC服务
