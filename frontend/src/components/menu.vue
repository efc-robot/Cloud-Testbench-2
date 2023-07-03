<template>
  <el-menu
    default-active="1"
    class="el-menu-vertical-demo"
    style="height:100%"
  >
    <el-menu-item index="1">
      <el-icon><EcosystemIcon /></el-icon>
      <router-link to="/digitalTwins" style="color:black;width:100%">场地数字孪生</router-link>
    </el-menu-item>
    <el-menu-item index="2">
      <el-icon><Box /></el-icon>
      <router-link to="/robots" style="color:black;width:100%">查看机器人列表</router-link>
    </el-menu-item>
    <el-menu-item index="3" v-if="isLoggedIn">
      <el-icon><MagicStick /></el-icon>
      <router-link to="/develop" style="color:black;width:100%">机器人远程开发</router-link>
    </el-menu-item>
    <el-menu-item index="4">
      <el-icon><KnifeFork /></el-icon>
      <router-link to="/dev_tools" style="color:black;width:100%">开发工具</router-link>
    </el-menu-item>
    <el-menu-item index="5">
      <el-icon><Guide /></el-icon>
      <router-link to="/documentation" style="color:black;width:100%">使用文档</router-link>
    </el-menu-item>
    <el-menu-item index="6">
      <el-icon><MessageBox /></el-icon>
      <router-link to="/issues" style="color:black;width:100%">意见箱</router-link>
    </el-menu-item>
    <el-sub-menu index="7" v-if="role=='admin'">
      <template #title>
        <el-icon><SetUp /></el-icon>
        <span>云测试场管理</span>
      </template>
      <el-menu-item index="7-1">
        <el-icon><User /></el-icon>
        <router-link to="/userManage" style="color:black;width:100%">用户管理</router-link>
      </el-menu-item>
      <el-menu-item index="7-2">
        <el-icon><Cpu /></el-icon>
        <router-link to="/robotManage" style="color:black;width:100%">机器人管理</router-link>
      </el-menu-item>
      <el-menu-item index="7-3">
        <el-icon><Connection /></el-icon>
        <router-link to="/ftpManage" style="color:black;width:100%">ftp配置管理</router-link>
      </el-menu-item>
      <el-menu-item index="7-4">
        <el-icon><EcosystemIcon /></el-icon>
        <router-link to="/digitalTwinsManage" style="color:black;width:100%">数字孪生场景管理</router-link>
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