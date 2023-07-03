<template>
    <div style="height:100%;width:100%;">
        <div style="height:100%;width:100%;padding:15px">
            <div style="height:100px;width:100%;padding:5px"></div>
            <div style="height:auto;width:100%;background-color:#edecec5c;padding:15px">
                <el-card class="box-card">
                    <template #header>
                        <div class="card-header">
                            <span>向场景添加模型</span>
                            <el-button class="button" type="primary" @click="addEntity()">添加模型</el-button>
                        </div>
                    </template>
                    <el-form :model="form_addEntity" label-width="160px">
                        <el-form-item label="机器人名称">
                            <el-input v-model="form_addEntity.entity_name" placeholder="机器人名称" />
                        </el-form-item>
                        <el-form-item label="机器人类型">
                            <el-select v-model="form_addEntity.model_type" placeholder="机器人类型" style="width:100%;">
                                <el-option
                                    v-for="item in modelTypes"
                                    :key="item.name"
                                    :label=item.description
                                    :value="item.type">
                                </el-option>
                            </el-select>
                        </el-form-item>
                    </el-form>
                </el-card>
            </div>
            <div style="height:auto;width:100%;background-color:#edecec5c;padding:15px">
                <el-card class="box-card">
                    <template #header>
                        <div class="card-header">
                            <span>移除场景中模型</span>
                            <el-button class="button" type="primary" @click="delEntity()">移除模型</el-button>
                        </div>
                    </template>
                    <el-form :model="form_delEntity" label-width="160px">
                        <el-form-item label="机器人名称">
                            <el-input v-model="form_delEntity.entity_name" placeholder="机器人名称" />
                        </el-form-item>
                    </el-form>
                </el-card>
            </div>
        </div>
    </div>
</template>

<script>
import axios from 'axios'
import { reactive } from 'vue'
import { ElMessage } from 'element-plus'
import {DIGITAL_TWINS_WS_URL} from "@/config/digitalTwins.js"


export default {
    data() {
        return {
            websocket: null, 
            modelTypes: [
                {
                    "name": "xtark r20 mec",
                    "type": "xtark_r20_mec",
                    "description": "xtark r20 麦克纳姆轮",
                }
            ],
            form_addEntity: reactive({
                "entity_name": "",
                "model_type": "",
            }),
            form_delEntity: reactive({
                "entity_name": "",
            }),
        }
    },
    mounted() {
        this.createWsConnect()
    },
    beforeDestroy() {
        this.websocket.close();
    },
    methods: {
        addEntity() {
            if (this.websocket.readyState === WebSocket.OPEN) {
                let message = {
                    "op": "call_service",
                    "service": "/spawn_robot",
                    "args": {
                        "entity_name": this.form_addEntity.entity_name,
                        "model_type": this.form_addEntity.model_type,
                        "robot_namespace": "",
                        "reference_frame": "world"
                    }
                }
                let jsonMessage = JSON.stringify(message);
                this.websocket.send(jsonMessage);
            } else {
                console.error('WebSocket connection not open');
            }
        },
        delEntity() {
            if (this.websocket.readyState === WebSocket.OPEN) {
                let message = {
                    "op": "call_service",
                    "service": "/delete_robot",
                    "args": {"entity_name": this.form_delEntity.entity_name}
                }
                let jsonMessage = JSON.stringify(message);
                this.websocket.send(jsonMessage);
            } else {
                console.error('WebSocket connection not open');
            }
        },
        createWsConnect() {
            this.websocket = new WebSocket(DIGITAL_TWINS_WS_URL);
            this.websocket.onopen = this.onWebSocketOpen;
            this.websocket.onmessage = this.onWebSocketMessage;
            this.websocket.onclose = this.onWebSocketClose;
            this.websocket.onerror = this.onWebSocketError;
        },
        onWebSocketOpen() {
            console.log('WebSocket to digital twins backend connected');
        },
        onWebSocketMessage(event) {
            console.log('get ws msg from digital twins backend: ', event.data);
        },
        onWebSocketClose() {
            console.log('WebSocket to digital twins backend closed');
        },
        onWebSocketError(error) {
            console.log('get error from digital twins backend: ', error);
        },
    }
}
</script>

<style scoped>
.dialog-footer button:first-child {
  margin-right: 10px;
}


.card-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
}

.text {
  font-size: 14px;
}

.item {
  margin-bottom: 18px;
}

.box-card {
  /* width: 480px; */
}

</style>