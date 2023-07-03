<template>
    <div style="height:100%;width:100%;">
        <div style="height:100%;width:100%;padding:15px">
            <div style="height:100px;width:100%;padding:5px"></div>
            <div style="height:auto;width:100%;padding:15px">
                <el-button type="primary" @click="issueDialogVisible = true">我要提意见</el-button>
            </div>
            <div style="height:auto;width:100%;background-color:#edecec5c;padding:15px">
                <el-table :data="issueTable" style="width:100%">
                    <el-table-column prop="time" label="时间" width="300" />
                    <el-table-column prop="issue" label="意见" min-width="150" />
                    <el-table-column prop="reporter" label="反馈者" width="350" />
                    <el-table-column prop="solved" label="已解决" width="100" />
                    <el-table-column fixed="right" label="操作" width="150">
                        <template #default="scope">
                            <el-button link type="primary" size="small" disabled>修改</el-button>
                            <el-button link type="primary" size="small" @click="delIssue(scope.row.uuid)">删除</el-button>
                        </template>
                    </el-table-column>
                </el-table>
            </div>
        </div>
        <el-dialog
            v-model="issueDialogVisible"
            title="意见反馈"
            width="50%"
            align-center
            style="padding-left:30px;padding-right:30px;padding-top:30px;"
        >
            <el-input type="textarea" :autosize="{ minRows: 4 }" placeholder="" v-model="issueText" />
            <template #footer>
                <span class="dialog-footer">
                    <el-button @click="issueDialogVisible = false">取消</el-button>
                    <el-button type="primary" @click="commitIssue()">提交</el-button>
                </span>
            </template>
        </el-dialog>
    </div>
</template>

<script>
import axios from 'axios'
import {BASE_URL} from "@/config/api.js"

export default {
    data() {
        return {
            issueTable: [],
            issueText: "",
            issueDialogVisible: false,
        }
    },
    mounted() {
        this.getIssuesTableData()
    },
    methods: {
        getIssuesTableData() {
            axios({
                method: "get",
                headers: {
                    Authorization: localStorage.getItem('token')
                },
                url: BASE_URL + '/issues/all'
            })
            .then(res => {
                this.issueTable = res.data
            })
            .catch(error => {
                handleErr(error)
                ElMessage.error(error.response.data.detail)
            })
        },
        commitIssue() {
            axios({
                method: "post",
                data: {
                    "issue": this.issueText,
                },
                headers: {
                    Authorization: localStorage.getItem('token')
                },
                url: BASE_URL + '/issues/'
            })
            .then(res => {
                this.issueDialogVisible = false
                this.issueText = ""
                console.log(res)
                this.getIssuesTableData()
                ElMessage.success("反馈意见已提交")
            })
            .catch(error => {
                console.log(error)
                ElMessage.error(error.response.data.detail)
            })
        },
        delIssue(issueUUID) {
            axios({
                method: "delete",
                params: {"uuid":issueUUID},
                headers: {
                    Authorization: localStorage.getItem('token')
                },
                url: BASE_URL + '/issues/'
            })
            .then(res => {
                console.log(res)
                this.getIssuesTableData()
            })
            .catch(error => {
                console.log(error)
                ElMessage.error(error.response.data.detail)
            })
        },
    },
}
</script>