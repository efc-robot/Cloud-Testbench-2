<template>
    <div style="height:100%;width:100%;">
        <div style="height:100%;width:100%;padding:15px">
            <div style="height:100px;width:100%;padding:5px"></div>
            <div style="height:auto;width:100%;padding:15px">
                <el-button type="primary" @click="onAddBtnClick()">添加机器人</el-button>
            </div>
            <div style="height:auto;width:100%;background-color:#edecec5c;padding:15px">
                <el-table :data="robotsTableData" style="width: 100%">
                    <el-table-column prop="name" label="名称" width="180" />
                    <el-table-column prop="type" label="类型" width="140" />
                    <el-table-column prop="uuid" label="UUID" min-width="300" />
                    <el-table-column prop="ip" label="IP" min-width="180" />
                    <el-table-column prop="online" label="在线" min-width="100" />
                    <el-table-column prop="allocated" label="已分配" min-width="100" />
                    <el-table-column fixed="right" label="操作" width="180" >
                        <template #default="scope">
                            <el-button link type="primary" size="small" @click="onInfoBtnClick(scope.row)">查看</el-button>
                            <el-button link type="primary" size="small" @click="onEditSettingsBtnClick(scope.row)">编辑</el-button>
                            <el-button link type="primary" size="small" @click="delRobot(scope.row)">删除</el-button>
                        </template>
                    </el-table-column>
                </el-table>
            </div>
        </div>
        <el-dialog
            v-model="addCardVisible"
            title="添加机器人"
            width="50%"
            align-center
            style="padding-left:30px;padding-right:30px;padding-top:30px;"
        >
            <el-form :model="robotInfo" label-width="160px">
                <el-form-item label="机器人名称">
                    <el-input v-model="robotInfo.name" placeholder="设置登机器人名称" />
                </el-form-item>
                <el-form-item label="机器人类型">
                    <el-select v-model="robotInfo.type" placeholder="设置登录机器人类型" style="width:100%">
                        <el-option v-for="(type, index) in robotTypes" :key="index" :label="type" :value="type" />
                    </el-select>
                </el-form-item>
                <el-form-item label="机器人IP">
                    <el-input v-model="robotInfo.ip" placeholder="设置登录机器人IP"/>
                </el-form-item>
            </el-form>
            <template #footer>
                <span class="dialog-footer">
                    <el-button @click="addCardVisible = false">取消</el-button>
                    <el-button type="primary" @click="addRobot(robotInfo)">提交</el-button>
                </span>
            </template>
        </el-dialog>
        <el-dialog
            v-model="settingCardVisible"
            title="修改机器人信息"
            width="50%"
            align-center
            style="padding-left:30px;padding-right:30px;padding-top:30px;"
        >
            <el-form :model="robotInfo" label-width="160px">
                <el-form-item label="机器人UUID">
                    <el-input v-model="robotInfo.uuid" disabled />
                </el-form-item>
                <el-form-item label="机器人名称">
                    <el-input v-model="robotInfo.name" placeholder="设置机器人名称" />
                </el-form-item>
                <el-form-item label="机器人类型">
                    <el-select v-model="robotInfo.type" placeholder="设置登录机器人类型" style="width:100%">
                        <el-option v-for="(type, index) in robotTypes" :key="index" :label="type" :value="type" />
                    </el-select>
                </el-form-item>
                <el-form-item label="机器人IP">
                    <el-input v-model="robotInfo.ip" placeholder="设置登录机器人IP"/>
                </el-form-item>
            </el-form>
            <template #footer>
                <span class="dialog-footer">
                    <el-button @click="settingCardVisible = false">取消</el-button>
                    <el-button type="primary" @click="editRobot(robotInfo)">提交</el-button>
                </span>
            </template>
        </el-dialog>
        <el-dialog
            v-model="infoCardVisible"
            title="查看机器人信息"
            width="50%"
            align-center
            style="padding-left:30px;padding-right:30px;padding-top:30px;"
        >
            <el-form :model="robotInfo" label-width="160px">
                <el-form-item label="机器人UUID :">{{ robotInfo.uuid }}</el-form-item>
                <el-form-item label="机器人名称 :">{{ robotInfo.name }}</el-form-item>
                <el-form-item label="机器人类型 :">{{ robotInfo.type }}</el-form-item>
                <el-form-item label="机器人IP :">{{ robotInfo.ip }}</el-form-item>
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
            robotsTableData: [], 
            ftpTableData: [], 
            addCardVisible: false,
            settingCardVisible: false,
            infoCardVisible: false,
            robotTypes: [
                "mock bot",
                "akm"
            ],
            robotInfo: reactive({
                "uuid": "",
                "ip": "",
                "name": "",
                "type": ""
            }),
        }
    },
    mounted() {
        this.getRobotsTable()
    },
    methods: {
        onAddBtnClick() {
            this.robotInfo = {}
            this.addCardVisible = true
        },
        onEditSettingsBtnClick(row) {
            this.robotInfo.uuid = row.uuid
            this.robotInfo.ip = row.ip
            this.robotInfo.name = row.name
            this.robotInfo.type = row.type
            this.settingCardVisible = true
        },
        onInfoBtnClick(row) {
            this.robotInfo.uuid = row.uuid
            this.robotInfo.ip = row.ip
            this.robotInfo.name = row.name
            this.robotInfo.type = row.type
            this.infoCardVisible = true
        },
        editRobot(robotInfo) {
            axios({
                method: "patch",
                data: robotInfo,
                headers: {
                    Authorization: localStorage.getItem('token')
                },
                url: BASE_URL + '/robot'
            })
            .then(res => {
                console.log(res)
                this.getRobotsTable()
            })
            .catch(error => {
                console.log(error)
                ElMessage.error(error.response.data.detail)
            })
            this.settingCardVisible = false
        },
        addRobot(robotInfo) {
            axios({
                method: "post",
                data: robotInfo,
                headers: {
                    Authorization: localStorage.getItem('token')
                },
                url: BASE_URL + '/robot'
            })
            .then(res => {
                console.log(res)
                this.getRobotsTable()
                this.addCardVisible = false
            })
            .catch(error => {
                console.log(error)
                ElMessage.error(error.response.data.detail)
            })
        },
        delRobot(row) {
            axios({
                method: "delete",
                params: {"uuid":row.uuid},
                headers: {
                    Authorization: localStorage.getItem('token')
                },
                url: BASE_URL + '/robot'
            })
            .then(res => {
                console.log(res)
                this.getRobotsTable()
            })
            .catch(error => {
                console.log(error)
                ElMessage.error(error.response.data.detail)
            })
        },
        getRobotsTable(){
            axios.get(BASE_URL+'/robot/all')
            .then(res => {
                    this.robotsTableData = res.data
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