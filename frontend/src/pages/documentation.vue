<template>
    <div v-html="markdown" class="markdown-body"></div>
</template>

<script>
import {marked} from 'marked';
import axios from 'axios'
import {BASE_URL} from "@/config/api.js"
import 'github-markdown-css';

export default {
    data() {
        return {
            markdown: ""
        };
    },
    mounted() {
        this.loadMarkdownFile();
    },
    watch: {
        '$route':'loadMarkdownFile'
    },
    methods: {
        async loadMarkdownFile() {
            let router = this.$route.path;
            let url = BASE_URL + '/files' + router + '.md';
            axios({
                method: "get",
                url: url
            })
            .then(res => {
                const markdownContent = marked(res.data);
                this.markdown = markdownContent;
            })
            .catch(error => {
                console.log(error)
                ElMessage.error(error.response.data.detail)
            })
        },
    },
}
</script>