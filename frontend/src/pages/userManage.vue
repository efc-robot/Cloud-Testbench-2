<template>
    <div style="height:100%;width:100%;">
        <div style="height:100%;width:100%;padding:15px">
            <div style="height:100px;width:100%;padding:5px"></div>
            <div style="height:auto;width:100%;padding:15px">
                <el-button type="primary" @click="onAddBtnClick()">添加用户</el-button>
            </div>
            <div style="height:auto;width:100%;background-color:#edecec5c;padding:15px">
                <el-table :data="tableData" style="width: 100%">
                    <el-table-column prop="username" label="用户名" min-width="180" />
                    <el-table-column prop="password" label="密码" min-width="180" />
                    <el-table-column prop="role" label="角色" min-width="140" />
                    <el-table-column prop="uuid" label="UUID" min-width="300" />
                    <el-table-column fixed="right" label="操作" width="180" >
                        <template #default="scope">
                            <el-button link type="primary" size="small" @click="onInfoBtnClick(scope.row)">查看</el-button>
                            <el-button link type="primary" size="small" @click="onEditBtnClick(scope.row)">编辑</el-button>
                            <el-button link type="primary" size="small" @click="del(scope.row)">删除</el-button>
                        </template>
                    </el-table-column>
                </el-table>
            </div>
        </div>
        <el-dialog
            v-model="addCardVisible"
            title="添加用户"
            width="50%"
            align-center
            style="padding-left:30px;padding-right:30px;padding-top:30px;"
        >
            <el-form :model="userInfo" label-width="160px">
                <el-form-item label="用户名">
                    <el-input v-model="userInfo.username" placeholder="设置用户名" />
                </el-form-item>
                <el-form-item label="密码">
                    <el-input v-model="userInfo.password" type="password" placeholder="设置密码"/>
                </el-form-item>
                <el-form-item label="权限">
                    <el-select v-model="userInfo.role" placeholder="设置角色权限" style="width:100%">
                        <el-option v-for="(role, roleName) in roleTypes" :key="roleName" :label="roleName" :value="role" />
                    </el-select>
                </el-form-item>
            </el-form>
            <template #footer>
                <span class="dialog-footer">
                    <el-button @click="addCardVisible = false">取消</el-button>
                    <el-button type="primary" @click="add(userInfo)">提交</el-button>
                </span>
            </template>
        </el-dialog>
        <el-dialog
            v-model="settingCardVisible"
            title="修改用户信息"
            width="50%"
            align-center
            style="padding-left:30px;padding-right:30px;padding-top:30px;"
        >
            <el-form :model="userInfo" label-width="160px">
                <el-form-item label="UUID">
                    <el-input v-model="userInfo.uuid" placeholder="用户UUID" disabled />
                </el-form-item>
                <el-form-item label="用户名">
                    <el-input v-model="userInfo.username" placeholder="设置用户名" />
                </el-form-item>
                <el-form-item label="密码">
                    <el-input v-model="userInfo.password" type="password" placeholder="设置密码"/>
                </el-form-item>
                <el-form-item label="权限">
                    <el-select v-model="userInfo.role" placeholder="设置角色权限" style="width:100%">
                        <el-option v-for="(role, roleName) in roleTypes" :key="roleName" :label="roleName" :value="role" />
                    </el-select>
                </el-form-item>
            </el-form>
            <template #footer>
                <span class="dialog-footer">
                    <el-button @click="settingCardVisible = false">取消</el-button>
                    <el-button type="primary" @click="edit(userInfo)">提交</el-button>
                </span>
            </template>
        </el-dialog>
        <el-dialog
            v-model="infoCardVisible"
            title="查看用户信息"
            width="50%"
            align-center
            style="padding-left:30px;padding-right:30px;padding-top:30px;"
        >
            <el-form :model="userInfo" label-width="160px">
                <el-form-item label="UUID :">{{ userInfo.uuid }}</el-form-item>
                <el-form-item label="用户名 :">{{ userInfo.username }}</el-form-item>
                <el-form-item label="密码 :">{{ userInfo.password }}</el-form-item>
                <el-form-item label="权限 :">{{ userInfo.role }}</el-form-item>
            </el-form>
        </el-dialog>
    </div>
</template>

<script>
import axios from 'axios'
import { reactive } from 'vue'
import { ElMessage } from 'element-plus'
import {BASE_URL} from "@/config/api.js"


export default {
    data() {
        return {
            tableData: [], 
            ftpTableData: [], 
            addCardVisible: false,
            settingCardVisible: false,
            infoCardVisible: false,
            roleTypes: {
                "普通用户": "user",
                "管理员": "admin"
            },
            userInfo: reactive({
                "uuid": "",
                "username": "",
                "password": "",
                "role": ""
            }),
        }
    },
    mounted() {
        this.getTableData()
    },
    watch: {
        // 'tableData': {
        //     handler(newVal, oldVal) {
        //         this.$forceUpdate()
        //     }
        // }
    },
    methods: {
        onAddBtnClick() {
            this.userInfo = {}
            this.addCardVisible = true
        },
        onEditBtnClick(row) {
            this.userInfo.uuid = row.uuid
            this.userInfo.username = row.username
            this.userInfo.password = row.password
            this.userInfo.role = row.role
            this.settingCardVisible = true
        },
        onInfoBtnClick(row) {
            this.userInfo.uuid = row.uuid
            this.userInfo.ip = row.ip
            this.userInfo.name = row.name
            this.userInfo.type = row.type
            this.infoCardVisible = true
        },
        edit(userInfo) {
            axios({
                method: "patch",
                data: userInfo,
                headers: {
                    Authorization: localStorage.getItem('token')
                },
                url: BASE_URL + '/user'
            })
            .then(res => {
                console.log(res)
                this.getTableData()
            })
            .catch(error => {
                console.log(error)
                ElMessage.error(error.response.data.detail)
            })
            this.settingCardVisible = false
        },
        add(userInfo) {
            axios({
                method: "post",
                data: userInfo,
                headers: {
                    Authorization: localStorage.getItem('token')
                },
                url: BASE_URL + '/user'
            })
            .then(res => {
                console.log(res)
                this.getTableData()
            })
            .catch(error => {
                console.log(error)
                ElMessage.error(error.response.data.detail)
            })
            this.addCardVisible = false
        },
        del(row) {
            axios({
                method: "delete",
                params: {"uuid":row.uuid},
                headers: {
                    Authorization: localStorage.getItem('token')
                },
                url: BASE_URL + '/user'
            })
            .then(res => {
                console.log(res)
                this.getTableData()
            })
            .catch(error => {
                console.log(error)
                ElMessage.error(error.response.data.detail)
            })
        },
        getTableData(){
            axios.get(BASE_URL + '/user/all')
            .then(res => {
                this.tableData = res.data
            })
            .catch(error => {
                console.log(error)
                ElMessage.error(error.response.data.detail)
            })
        }
    }
}
</script>

<style scoped>
.dialog-footer button:first-child {
  margin-right: 10px;
}
</style>