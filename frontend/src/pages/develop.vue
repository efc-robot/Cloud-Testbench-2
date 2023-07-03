<script setup>
    import { ElMessage } from 'element-plus'
    import {BASE_URL} from "@/config/api.js"
    import {HOST} from "@/config/backend.js"
    import {
        RemoveFilled,
        CirclePlusFilled
    } from '@element-plus/icons-vue'
</script>

<template>
    <div style="height:100%;width:100%;">
        <div style="height:100%;width:100%;padding:15px">
            <div style="height:100px;width:100%;padding:5px"></div>
            <el-tabs type="border-card">
                <el-tab-pane label="真实机器人">
                    <div style="height:auto;width:100%;background-color:#edecec5c;padding:5px">
                        <el-table :data="myRobotsTableData" style="width: 100%">
                            <el-table-column prop="robot_name" label="机器人名称" width="150" />
                            <el-table-column prop="robot_type" label="机器人类型" min-width="150" />
                            <el-table-column prop="robot_ip" label="机器人局域网IP" min-width="150" />
                            <el-table-column prop="robot_username" label="机器人端OS用户名" min-width="150" />
                            <el-table-column prop="robot_password" label="机器人端OS密码" min-width="150" />
                            <el-table-column prop="robot_workspace" label="机器人端工作目录" min-width="200">
                                <template #default="scope">/home/{{ scope.row.robot_username }}{{ scope.row.robot_workspace }}</template>
                            </el-table-column>
                            <el-table-column prop="code_server_workspace" label="vscode-web工作目录" min-width="200" />
                            <el-table-column prop="code_server_password" label="vscode-web密码" min-width="150" />
                            <el-table-column prop="robot_uuid" label="机器人UUID" min-width="300" />
                            <el-table-column fixed="right" label="操作" width="150" >
                                <template #default="scope">
                                    <el-button link type="primary" size="small" @click="openServerService(scope.row.code_server_port)">VSCode-web</el-button>
                                    <el-button link type="primary" size="small" @click="releaseRobot(scope.row)" :disabled="scope.row.allocated">释放</el-button>
                                </template>
                            </el-table-column>
                        </el-table>
                    </div>
                </el-tab-pane>
                <el-tab-pane label="虚拟机器人">
                    <div style="height:auto;width:100%;background-color:#edecec5c;padding:5px">
                        <el-table :data="myVisualRobotsTableData" style="width: 100%">
                            <el-table-column prop="name" label="机器人名称" width="150" />
                            <el-table-column prop="sudo_password" label="ubuntu系统密码" min-width="150" />
                            <el-table-column prop="default_workspace" label="vscode-web默认工作目录" min-width="250" />
                            <el-table-column prop="password" label="vscode-web密码" min-width="150" />
                            <el-table-column prop="expose_ports" label="端口映射（公网端口->容器端口）" min-width="400">
                                <template #default="scope">
                                    <el-button
                                        v-for="(port, host_port) in scope.row.expose_ports" 
                                        type="warning" plain
                                        size="small"
                                        @click="openServerService(port)" 
                                        style="margin:2px"
                                    >{{port}} -> {{host_port}}</el-button>
                                    <!-- <div
                                        style="height:auto;width:auto;background-color:burlywood;float:left;margin:2px 4px 2px 0;padding:1px;border-radius:4px;" 
                                        v-for="(port, host_port) in scope.row.expose_ports"
                                    >{{port}}->{{host_port}}</div> -->
                                </template>
                            </el-table-column>
                            <el-table-column prop="id" label="机器人id" width="600" />
                            <el-table-column prop="docker_image" label="docker镜像" min-width="250" />
                            <el-table-column fixed="right" label="操作" width="150" >
                                <template #default="scope">
                                    <el-button link type="primary" size="small" @click="openVSCodeWeb(scope.row.expose_ports)">VSCode-web</el-button>
                                    <el-button link type="primary" size="small" @click="destoryVisualBot(scope.row.id)">销毁</el-button>
                                </template>
                            </el-table-column>
                        </el-table>
                    </div>
                    <div style="height:auto;width:100%;padding:15px 15px 0 0">
                        <el-button type="primary" @click="onAddBtnClick()">添加虚拟机器人</el-button>
                    </div>
                </el-tab-pane>
            </el-tabs>
        </div>
        <el-dialog
            v-model="createVBotCardVisible"
            title="添加虚拟机器人"
            width="50%"
            align-center
            style="padding-left:30px;padding-right:30px;padding-top:30px;"
        >
            <el-form :model="vBotConfig" label-width="160px">
                <el-form-item label="容器镜像" :rules="{required: true, message: '需要设置机器人容器镜像', trigger: 'blur'}" prop="docker_image" >
                    <el-select v-model="vBotConfig.docker_image" placeholder="设置虚拟机器人使用的容器镜像" style="width:100%;">
                        <el-option
                            v-for="item in dockerImageOptions"
                            :key="item.name"
                            :label="getImageDescriptionText(item.name, item.description)"
                            :value="item.name">
                        </el-option>
                    </el-select>
                </el-form-item>
                <el-form-item label="机器人名称" :rules="{required: true, message: '机器人名称不能为空', trigger: 'blur'}" prop="name" >
                    <el-input v-model="vBotConfig.name" placeholder="设置登机器人名称" />
                </el-form-item>
                <el-form-item label="ubuntu密码" :rules="{required: true, message: '需要设置ubuntu系统密码', trigger: 'blur'}" prop="sudo_password" >
                    <el-input v-model="vBotConfig.sudo_password" placeholder="设置ubuntu系统root用户密码" />
                </el-form-item>
                <el-form-item label="vscode-web密码"><el-input v-model="vBotConfig.password" placeholder="设置vscode-web登录密码" /></el-form-item>
                <el-form-item label="默认工作目录"><el-input v-model="vBotConfig.default_workspace" placeholder="设置vscode-web默认工作目录" /></el-form-item>
                <el-form-item label="开放端口">
                    <el-input
                        style="width:150px;margin:0 5px 5px 0;" 
                        v-model="tmpValue_exposePort" 
                        type="number" 
                        class="mo-input--number"
                    >
                        <template #append>
                            <el-button @click="addExposePorts(tmpValue_exposePort)">
                                <el-icon style="color:#409EFF;"><CirclePlusFilled/></el-icon>
                            </el-button>
                        </template>
                    </el-input>
                    <el-input
                        v-for="(port, index) in vBotConfig.expose_ports" 
                        v-model="vBotConfig.expose_ports[index]"
                        style="width:150px;margin:0 5px 5px 0;" 
                        type="number" 
                        class="mo-input--number"
                    >
                        <template #append>
                            <el-button @click="delExposePorts(index)">
                                <el-icon style="color:#F56C6C"><RemoveFilled/></el-icon>
                            </el-button>
                        </template>
                    </el-input>
                </el-form-item>
            </el-form>
            <template #footer>
                <span class="dialog-footer">
                    <el-button @click="createVBotCardVisible = false">取消</el-button>
                    <el-button type="primary" @click="createVisualRobot(vBotConfig)">提交</el-button>
                </span>
            </template>
        </el-dialog>
    </div>
</template>

<script>
import axios from 'axios'
import { reactive } from 'vue'
import {handleErr} from "@/js/errorProcessor.js"


export default {
    data() {
        return {
            myRobotsTableData: [], 
            myVisualRobotsTableData: [], 
            createVBotCardVisible: false,
            vBotConfig: {},
            allowCreateVBotReq: true,
            tmpValue_exposePort: null,
            dockerImageOptions: [
                {
                    name: 'ros:cloud_test_bench',
                    description: '系统：ubuntu20.04，ROS1版本：noetic，ROS2版本：galactic，foxy',
                },
                {
                    name: 'ros:visual_robot_u1804_test',
                    description: '系统：ubuntu18.04，ROS1版本：melodic，ROS2版本：eloquent',
                },
                {
                    name: 'ct_visual_robot:bionic',
                    description: '系统：ubuntu18.04，ROS1版本：melodic',
                },
            ],
        }
    },
    mounted() {
        this.getMyRobotsTableData()
        this.getMyVisualRobotsTableData()
    },
    methods: {
        releaseRobot(row) {
            axios({
                method: "post",
                data: {
                    "robot_uuid": row.robot_uuid,
                    "robot_username": row.robot_username,
                },
                headers: {
                    Authorization: localStorage.getItem('token')
                },
                url: BASE_URL + '/robot/release'
            })
            .then(res => {
                console.log(res)
                this.getMyRobotsTableData()
            })
            .catch(error => {
                handleErr(error)
                ElMessage.error(error.response.data.detail)
            })
        },
        getMyRobotsTableData() {
            axios({
                method: "get",
                headers: {
                    Authorization: localStorage.getItem('token')
                },
                url: BASE_URL + '/robot/get_my_robots'
            })
            .then(res => {
                this.myRobotsTableData = res.data
            })
            .catch(error => {
                handleErr(error)
                ElMessage.error(error.response.data.detail)
            })
        },
        getMyVisualRobotsTableData() {
            axios({
                method: "get",
                headers: {
                    Authorization: localStorage.getItem('token')
                },
                url: BASE_URL + '/v_bot/my_vbot'
            })
            .then(res => {
                this.myVisualRobotsTableData = res.data
            })
            .catch(error => {
                handleErr(error)
                ElMessage.error(error.response.data.detail)
            })
        },
        destoryVisualBot(id) {
            axios({
                method: "delete",
                data: {
                    "id": id,
                },
                headers: {
                    Authorization: localStorage.getItem('token')
                },
                url: BASE_URL + '/v_bot'
            })
            .then(res => {
                console.log(res)
                this.getMyVisualRobotsTableData()
            })
            .catch(error => {
                handleErr(error)
                ElMessage.error(error.response.data.detail)
            })
        },

        onAddBtnClick() {
            this.vBotConfig = {
                "default_workspace":"/",
                "expose_ports":[8443, 9091],
            }
            this.tmpValue_exposePort = null
            this.createVBotCardVisible = true
        },
        createVisualRobot(vBotConfig) {
            if (this.allowCreateVBotReq) {
                this.allowCreateVBotReq = false
                axios({
                    method: "post",
                    data: vBotConfig,
                    headers: {
                        Authorization: localStorage.getItem('token')
                    },
                    url: BASE_URL + '/v_bot'
                })
                .then(res => {
                    this.createVBotCardVisible = false
                    this.getMyVisualRobotsTableData()
                    this.allowCreateVBotReq = true
                    console.log(res)
                })
                .catch(error => {
                    handleErr(error)
                    ElMessage.error(error.response.data.detail)
                    this.allowCreateVBotReq = true
                })
            }
        },
        openVSCodeWeb(expose_ports) {
            let service_port = expose_ports[8443]
            this.openServerService(service_port)
        },
        openServerService(service_port) {
            let url = "http://" + HOST + ":" + service_port
            window.open(url)
        },
        addExposePorts(port) {
            if (port) {
                this.vBotConfig["expose_ports"].push(port)
                this.tmpValue_exposePort = null
            }
        },
        delExposePorts(index) {
            console.log(index)
            this.vBotConfig["expose_ports"].splice(index, 1)
        },
        getImageDescriptionText(name, description) {
            return name + "<" + description + ">"
        }
    }
}
</script>

<style scoped>
.dialog-footer button:first-child {
  margin-right: 10px;
}
</style>
<style>
.mo-input--number input::-webkit-outer-spin-button,
.mo-input--number input::-webkit-inner-spin-button {
-webkit-appearance: none;
}
.mo-input--number input[type="number"] {
-moz-appearance: textfield;
}
</style>