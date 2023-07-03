<template>
    <div style="height:100%;width:100%;">
        <div style="height:100%;width:100%;padding:15px">
            <div style="height:100px;width:100%;padding:5px"></div>
            <div style="height:auto;width:100%;padding:15px">
                <el-button type="primary" @click="onAddBtnClick()">添加FTP配置</el-button>
            </div>
            <div style="height:auto;width:100%;background-color:#edecec5c;padding:15px">
                <el-table :data="tableData" style="width: 100%">
                    <el-table-column prop="robot_uuid" label="机器人UUID" min-width="300" />
                    <el-table-column prop="username" label="机器人用户名" min-width="150" />
                    <el-table-column prop="password" label="机器人登录密码" min-width="180" />
                    <el-table-column prop="port" label="FTP服务端口" width="120" />
                    <el-table-column prop="remote_dir" label="工作目录" min-width="300" >
                        <template #default="scope">/home/{{ scope.row.username }}{{ scope.row.remote_dir }}</template>
                    </el-table-column>
                    <el-table-column prop="mount_point" label="服务器FTP挂载点" min-width="550" />
                    <el-table-column fixed="right" label="操作" width="180" >
                        <template #default="scope">
                            <el-button link type="primary" size="small" @click="onInfoBtnClick(scope.row)">查看</el-button>
                            <el-button link type="primary" size="small" @click="onEditBtnClick(scope.row)" disabled>编辑</el-button>
                            <el-button link type="primary" size="small" @click="del(scope.row)">删除</el-button>
                        </template>
                    </el-table-column>
                </el-table>
            </div>
        </div>
        <el-dialog
            v-model="addCardVisible"
            title="添加FTP配置"
            width="50%"
            align-center
            style="padding-left:30px;padding-right:30px;padding-top:30px;"
        >
            <el-form :model="ftpInfo" label-width="160px">
                <el-form-item label="机器人UUID">
                    <el-input v-model="ftpInfo.robot_uuid" placeholder="机器人UUID" />
                </el-form-item>
                <el-form-item label="机器人用户名">
                    <el-input v-model="ftpInfo.username" placeholder="填写与机器人进行FTP连接时使用的用户名" />
                </el-form-item>
                <el-form-item label="机器人登录密码">
                    <el-input v-model="ftpInfo.password" placeholder="填写与机器人进行FTP连接时使用的密码"/>
                </el-form-item>
                <el-form-item label="机器人本地工作目录">
                    <el-input v-model="ftpInfo.remote_dir" placeholder="设置希望在vscode-web中访问的机器人系统目录">
                        <template #prepend>/home/{{ ftpInfo.username }}</template>
                    </el-input>
                </el-form-item>
                <el-form-item label="FTP服务端口">
                    <el-input v-model="ftpInfo.port" placeholder="设置FTP服务端口"/>
                </el-form-item>
            </el-form>
            <template #footer>
                <span class="dialog-footer">
                    <el-button @click="addCardVisible = false">取消</el-button>
                    <el-button type="primary" @click="add(ftpInfo)">提交</el-button>
                </span>
            </template>
        </el-dialog>
        <el-dialog
            v-model="settingCardVisible"
            title="修改FTP配置"
            width="50%"
            align-center
            style="padding-left:30px;padding-right:30px;padding-top:30px;"
        >
            <el-form :model="ftpInfo" label-width="160px">
                <el-form-item label="机器人UUID">
                    <el-input v-model="ftpInfo.robot_uuid" placeholder="机器人UUID" />
                </el-form-item>
                <el-form-item label="机器人用户名">
                    <el-input v-model="ftpInfo.username" placeholder="设置机器人用户名" />
                </el-form-item>
                <el-form-item label="机器人登录密码">
                    <el-input v-model="ftpInfo.password" placeholder="设置机器人登录密码"/>
                </el-form-item>
                <el-form-item label="机器人本地工作目录">
                    <el-input v-model="ftpInfo.remote_dir" placeholder="设置希望在vscode-web中访问的机器人系统目录">
                        <template #prepend>/home</template>
                    </el-input>
                </el-form-item>
                <el-form-item label="FTP服务端口">
                    <el-input v-model="ftpInfo.port" placeholder="设置FTP服务端口"/>
                </el-form-item>
            </el-form>
            <template #footer>
                <span class="dialog-footer">
                    <el-button @click="settingCardVisible = false">取消</el-button>
                    <el-button type="primary" @click="edit(ftpInfo)">提交</el-button>
                </span>
            </template>
        </el-dialog>
        <el-dialog
            v-model="infoCardVisible"
            title="查看FTP配置"
            width="50%"
            align-center
            style="padding-left:30px;padding-right:30px;padding-top:30px;"
        >
            <el-form :model="ftpInfo" label-width="160px">
                <el-form-item label="机器人UUID :">{{ ftpInfo.robot_uuid }}</el-form-item>
                <el-form-item label="机器人用户名 :">{{ ftpInfo.username }}</el-form-item>
                <el-form-item label="机器人登录密码 :">{{ ftpInfo.password }}</el-form-item>
                <el-form-item label="机器人本地工作目录 :">/home{{ ftpInfo.remote_dir }}</el-form-item>
                <el-form-item label="工作目录挂载点 :">{{ ftpInfo.mount_point }}</el-form-item>
                <el-form-item label="FTP服务端口 :">{{ ftpInfo.port }}</el-form-item>
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
            ftpInfo: reactive({
                "robot_uuid": "",
                "username": "",
                "password": "",
                "remote_dir": "",
                "mount_point": "",
                "port": ""
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
            this.ftpInfo = {}
            this.ftpInfo.remote_dir = '/'
            this.ftpInfo.port = 21
            this.addCardVisible = true
        },
        onEditBtnClick(row) {
            this.ftpInfo.robot_uuid = row.robot_uuid
            this.ftpInfo.username = row.username
            this.ftpInfo.password = row.password
            this.ftpInfo.remote_dir = row.remote_dir
            this.ftpInfo.port = row.port
            this.settingCardVisible = true
        },
        onInfoBtnClick(row) {
            this.ftpInfo.robot_uuid = row.robot_uuid
            this.ftpInfo.username = row.username
            this.ftpInfo.password = row.password
            this.ftpInfo.remote_dir = row.remote_dir
            this.ftpInfo.mount_point = row.mount_point
            this.ftpInfo.port = row.port
            this.infoCardVisible = true
        },
        edit(ftpInfo) {
            axios({
                method: "patch",
                data: ftpInfo,
                headers: {
                    Authorization: localStorage.getItem('token')
                },
                url: BASE_URL + '/ftp_info'
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
        add(ftpInfo) {
            axios({
                method: "post",
                data: ftpInfo,
                headers: {
                    Authorization: localStorage.getItem('token')
                },
                url: BASE_URL + '/ftp_info'
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
                data: {
                    "robot_uuid": row.robot_uuid,
                    "username": row.username
                },
                headers: {
                    Authorization: localStorage.getItem('token')
                },
                url: BASE_URL + '/ftp_info'
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
            axios.get(BASE_URL + '/ftp_info/all')
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