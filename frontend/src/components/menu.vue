<template>
  <el-menu
    default-active="1"
    class="el-menu-vertical-demo"
    style="height:100%"
  >
    <el-menu-item index="1">
      <el-icon><EcosystemIcon /></el-icon>
      <router-link to="/digitalTwins" class="menu-label">场地数字孪生</router-link>
    </el-menu-item>
    <el-menu-item index="2">
      <el-icon><Box /></el-icon>
      <router-link to="/robots" class="menu-label">查看机器人列表</router-link>
    </el-menu-item>
    <el-menu-item index="3" v-if="isLoggedIn">
      <el-icon><MagicStick /></el-icon>
      <router-link to="/develop" class="menu-label">机器人远程开发</router-link>
    </el-menu-item>
    <el-menu-item index="4">
      <el-icon><KnifeFork /></el-icon>
      <router-link to="/dev_tools" class="menu-label">开发工具</router-link>
    </el-menu-item>
    <el-sub-menu index="5">
      <template #title>
        <el-icon><Guide /></el-icon>
        <span>使用文档</span>
      </template>
      <el-menu-item index="5-1"><router-link to="/documentation/markdown/intro" class="menu-label">云测试场介绍</router-link></el-menu-item>
      <el-sub-menu index="5-2">
        <template #title><span>真实机器人</span></template>
        <el-menu-item index="5-2-1"><router-link to="/documentation/markdown/real_robot/access_cloud_testbench" class="menu-label">接入云测试场</router-link></el-menu-item>
        <el-menu-item index="5-2-2"><router-link to="/documentation/markdown/real_robot/apply_and_use" class="menu-label">申请及使用</router-link></el-menu-item>
      </el-sub-menu>
      <el-sub-menu index="5-3">
        <template #title><span>虚拟机器人</span></template>
        <el-menu-item index="5-3-1"><router-link to="/documentation/markdown/visual_robot/intro" class="menu-label">虚拟机器人介绍</router-link></el-menu-item>
        <el-menu-item index="5-3-1"><router-link to="/documentation/markdown/visual_robot/usage" class="menu-label">创建及使用</router-link></el-menu-item>
        <el-menu-item index="5-3-2"><router-link to="/documentation/markdown/visual_robot/ros_master" class="menu-label">作为ROS Master</router-link></el-menu-item>
        <el-menu-item index="5-3-2"><router-link to="/documentation/markdown/visual_robot/gzweb" class="menu-label">gazebo</router-link></el-menu-item>
      </el-sub-menu>
      <el-sub-menu index="5-4">
        <template #title><span>云测试场网盘</span></template>
        <el-menu-item index="5-4-1"><router-link to="/documentation/markdown/seafile/usage" class="menu-label">Quick Start</router-link></el-menu-item>
        <el-menu-item index="5-4-1"><router-link to="/documentation/markdown/seafile/webdav" class="menu-label">webdav</router-link></el-menu-item>
      </el-sub-menu>
      <el-sub-menu index="5-5">
        <template #title><span>远程调试工具</span></template>
        <el-menu-item index="5-5-1"><router-link to="/documentation/markdown/foxglove/intro" class="menu-label">foxglove介绍</router-link></el-menu-item>
        <el-menu-item index="5-5-2"><router-link to="/documentation/markdown/foxglove/quick_start" class="menu-label">Quick Start</router-link></el-menu-item>
      </el-sub-menu>
      <el-sub-menu index="5-6">
        <template #title><span>OptiTrack 定位系统</span></template>
        <el-menu-item index="5-6-1"><router-link to="/documentation/markdown/optitrack/intro" class="menu-label">OptiTrack介绍</router-link></el-menu-item>
        <el-menu-item index="5-6-2"><router-link to="/documentation/markdown/optitrack/quick_start" class="menu-label">Quick Start</router-link></el-menu-item>
        <el-menu-item index="5-6-3"><router-link to="/documentation/markdown/optitrack/vrpn_client" class="menu-label">ROS接收动捕数据</router-link></el-menu-item>
      </el-sub-menu>
    </el-sub-menu>
    <el-menu-item index="6">
      <el-icon><MessageBox /></el-icon>
      <router-link to="/issues" class="menu-label">意见箱</router-link>
    </el-menu-item>
    <el-sub-menu index="7" v-if="role=='admin'">
      <template #title>
        <el-icon><SetUp /></el-icon>
        <span>云测试场管理</span>
      </template>
      <el-menu-item index="7-1">
        <el-icon><User /></el-icon>
        <router-link to="/userManage" class="menu-label">用户管理</router-link>
      </el-menu-item>
      <el-menu-item index="7-2">
        <el-icon><Cpu /></el-icon>
        <router-link to="/robotManage" class="menu-label">机器人管理</router-link>
      </el-menu-item>
      <el-menu-item index="7-3">
        <el-icon><Connection /></el-icon>
        <router-link to="/ftpManage" class="menu-label">ftp配置管理</router-link>
      </el-menu-item>
      <el-menu-item index="7-4">
        <el-icon><EcosystemIcon /></el-icon>
        <router-link to="/digitalTwinsManage" class="menu-label">数字孪生场景管理</router-link>
      </el-menu-item>
    </el-sub-menu>
  </el-menu>
</template>

<script setup>
import EcosystemIcon from './icons/IconEcosystem.vue'
import router from '../router';
import {
  Box,
  MagicStick,
  KnifeFork,
  SetUp,
  User,
  Cpu,
  Connection,
  MessageBox,
  Guide
} from '@element-plus/icons-vue'
const handleClick = (key, keyPath) => {
  console.log(key, keyPath)
  router.push("robots")
};
</script>

<script>
import { mapState } from 'vuex'
export default {
  name: "Menu",
  computed: {
    ...mapState(['isLoggedIn']),
    ...mapState(['username']),
    ...mapState(['role']),
  },
}
</script>

<style scoped>
.menu-label {
  color:black;
  width:100%
}
</style>