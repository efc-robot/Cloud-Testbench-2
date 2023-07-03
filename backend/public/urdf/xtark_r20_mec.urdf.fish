<?xml version="1.0" encoding="utf-8"?>

<robot name="xtark_robot">

    <!-- 机器人底部中心原点 -->
    <link name="base_footprint">
        <visual>
            <geometry>
                <sphere radius="0.001" />
            </geometry>
        </visual>
    </link>

    <!-- 机器人底盘 -->
    <link name="base_link">
        <visual>
            <geometry>
                <mesh filename="/home/xtark/xtark/ros2_ws/src/xtark_description/meshes/xtark_description/meshes/base_r10_mec.stl" />
            </geometry>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <material name="base_color">
                <!-- <color rgba="0.2196 0.557 0.557 1" /> -->
                <!-- <color rgba="0.545 0.506 0.298 1" /> -->
                <color rgba="0.8 0.8 0.8 1" />
               
            </material>
        </visual>
    </link>    

    <!-- 机器人雷达 -->
    <link name="laser_link">
        <visual>
            <geometry>
                <mesh filename="/home/xtark/xtark/ros2_ws/src/xtark_description/meshes/xtark_description/meshes/laser.stl" />
            </geometry>
            <origin xyz="0 0 -0.06" rpy="0 0 0" />
            <material name="laser_color">
                <color rgba="0.4 0.4 0.4 1" />
            </material>
        </visual>
    </link> 

    <!-- 机器人摄像头 -->
    <link name="camera_link">
        <visual>
            <geometry>
                <mesh filename="/home/xtark/xtark/ros2_ws/src/xtark_description/meshes/xtark_description/meshes/depthcamera.stl" />
            </geometry>
            <origin xyz="-0.037 0 0" rpy="0 0 0" />
            <material name="camera_color">
                <color rgba="0.15 0.15 0.15 1" />
            </material>
        </visual>
    </link>

    <!-- 机器人IMU传感器 -->
    <link name="imu_link">
        <visual>
            <geometry>
                <sphere radius="0.010" />
            </geometry>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <material name="imu_color">
                <color rgba="1 1 0 1" />
            </material>
        </visual>
    </link>

    <!-- 底盘与原点连接的关节 -->
    <joint name="baselink2footprint" type="fixed">
        <origin xyz="0 0 0.1255" rpy="0 0 0"/>
        <parent link="base_footprint" />
        <child  link="base_link" />
    </joint>

    <!-- 雷达到与底盘连接的关节 -->
    <joint name="laser2base" type="fixed">
        <origin xyz="0 0 0.06" rpy="0 0 0"/>
        <parent link="base_link" />
        <child  link="laser_link" />
    </joint>

    <!-- 深度摄像头与底盘连接的关节 -->
    <joint name="camera2base" type="fixed">
        <origin xyz="0.105 0 0.0285" rpy="0 0 0"/>
        <parent link="base_link" />
        <child  link="camera_link" />
    </joint>

    <!-- IMU传感器与底盘连接的关节 -->
    <joint name="imu2base" type="fixed">
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <parent link="base_link" />
        <child  link="imu_link" />
    </joint>

</robot>