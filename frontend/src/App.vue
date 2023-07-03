<script setup>
import Menu from './components/menu.vue';
// import Header from './components/header.vue';
import Footer from './components/footer.vue';
import router from './router';
import {
  Box,
  Failed,
  MagicStick
} from '@element-plus/icons-vue'
import {BASE_URL} from "@/config/api.js"
</script>

<template>
  <div id="app" class="common-layout" style="height:100vh">
    <el-container style="height:100%">
      <el-header style="height:auto;padding:5px;padding-left:20px;padding-right:20px;background-color:#00000005;background">
        <el-row class="mb-4" style="float:right">
            <div v-if="!isLoggedIn">
              <el-button round @click="onSignInBtnClick()">Sign in</el-button>
              <el-button round @click="loginPanelVisible = true">Login</el-button>
            </div>
            <div v-else>
              <el-button round text>
                <el-icon><Medal /></el-icon>
                <span>{{ username }}</span>
              </el-button>
              <el-button round @click="logout()">logout</el-button>
            </div>
        </el-row>
      </el-header>
      <el-container>
        <el-aside width="200px">
          <Menu></Menu>
        </el-aside>
        <el-container>
          <el-main>
            <!-- <DigitalTwins></DigitalTwins> -->
            <!-- <Robots></Robots> -->
            <!-- <Develop></Develop> -->
            <router-view/>
          </el-main>
          <el-footer style="height:auto;padding:5px;background-color:#00000005;background">
            <Footer></Footer>
          </el-footer>
        </el-container>
      </el-container>
    </el-container>
    <el-dialog
        v-model="loginPanelVisible"
        title="登录"
        width="30%"
        align-center
        style="padding-left:30px;padding-right:30px;padding-top:30px;"
    >
      <el-form :model="loginForm" label-width="60px" v-on:keyup.enter="login">
        <el-form-item label="用户名">
            <el-input v-model="loginForm.username" />
        </el-form-item>
        <el-form-item label="密码">
            <el-input v-model="loginForm.password" type="password" />
        </el-form-item>
      </el-form>
      <template #footer>
        <span class="dialog-footer">
            <el-button @click="loginPanelVisible = false">取消</el-button>
            <el-button type="primary" @click="login()">登录</el-button>
        </span>
      </template>
    </el-dialog> 
    <el-dialog
        v-model="signInPanelVisible"
        title="注册"
        width="30%"
        align-center
        style="padding-left:30px;padding-right:30px;padding-top:30px;"
    >
      <el-form :model="signInForm" label-width="120px" v-on:keyup.enter="signIn">
        <el-form-item label="用户名" :rules="{required: true, message: '用户名不能为空', trigger: 'blur'}" prop="username">
            <el-input v-model="signInForm.username" />
        </el-form-item>
        <el-form-item label="密码" :rules="{required: true, message: '密码不能为空', trigger: 'blur'}" prop="password">
            <el-input v-model="signInForm.password" type="password" />
        </el-form-item>
        <el-form-item label="确认密码" :rules="{required: true, message: '再次输入密码以确认', trigger: 'blur'}" prop="password_confirm">
            <el-input v-model="signInForm.password_confirm" type="password" />
        </el-form-item>
      </el-form>
      <template #footer>
        <span class="dialog-footer">
            <el-button @click="signInPanelVisible = false">取消</el-button>
            <el-button type="primary" @click="signIn()">注册</el-button>
        </span>
      </template>
    </el-dialog> 
  </div>
</template>

<script>
import axios from 'axios'
import { reactive } from 'vue'
import { Medal } from '@element-plus/icons-vue'
import { ElMessage } from 'element-plus'
import { handleErr, resetLoginStatus } from "@/js/errorProcessor.js"
import { mapState, mapGetters, mapActions } from 'vuex'

export default {
  name: 'App',
  router,
  data() {
    return {
      loginPanelVisible: false,
      signInPanelVisible: false,
      loginForm: reactive({
          "username": "",
          "password": ""
      }),
      signInForm: reactive({
          "username": "",
          "password": "",
          "password_confirm": "",
      }),
    }
  },
  created() {
    this.verify_token()
    this.$store.dispatch('initializeApp');
  },
  mounted() {
    // this.verify_token()
    // console.log(this.isLoggedIn)
    // console.log(typeof(this.isLoggedIn))
  },
  computed: {
    ...mapState(['isLoggedIn']),
    ...mapState(['username']),
    ...mapState(['role']),
  },
  methods: {
    login() {
      axios({
          method: "post",
          data: {
              "username": this.loginForm.username,
              "password": this.loginForm.password,
          },
          headers: {
              Authorization: ""
          },
          url: BASE_URL + '/login/'
      })
      .then(res => {
        let username = res.data["username"]
        let role = res.data["role"]
        let token = res.data["token"]
        localStorage.setItem('token', token)
        localStorage.setItem('username', username)
        localStorage.setItem('role', role)
        localStorage.setItem('isLoggedIn', true)
        this.$store.commit('REFREASH_USER_INFO')
        this.loginPanelVisible = false
      })
      .catch(error => {
          console.log(error)
          ElMessage.error(error.response.data.detail)
      })
    },
    signIn() {
      if (this.signInForm.password_confirm != this.signInForm.password) {
        ElMessage.error("两次输入的密码不一致");
        return;
      }
      axios({
          method: "post",
          data: {
              "username": this.signInForm.username,
              "password": this.signInForm.password,
              "role": "user",
          },
          headers: {
              Authorization: ""
          },
          url: BASE_URL + '/user'
      })
      .then(res => {
        console.log(res)
        this.signInPanelVisible = false
        ElMessage.success("注册成功")
      })
      .catch(error => {
          console.log(error)
          // ElMessage.error(error.response.data.detail)
          ElMessage.error("注册失败，用户名已被占用")
      })
    },
    logout() {
      axios({
          method: "post",
          headers: {
              Authorization: localStorage.getItem('token')
          },
          url: BASE_URL + '/logout/'
      })
      .then(res => {
        resetLoginStatus()
      })
      .catch(error => {
          console.log(error)
          ElMessage.error(error.response.data.detail)
      })
    },
    verify_token() {
      axios({
          method: "post",
          headers: {
              Authorization: localStorage.getItem('token')
          },
          url: BASE_URL + '/verify_token/'
      })
      .then(res => {
        console.log(res.data)
      })
      .catch(err => {
        handleErr(err)
      })
    },
    onSignInBtnClick() {
      this.signInPanelVisible = true;
    }
  }
}
</script>
