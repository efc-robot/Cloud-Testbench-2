<template>
  <div style="height:100%;padding: 15px;">
    <!-- <iframe :src="externalUrl" width="100%" height="100%"></iframe> -->
    <el-row :gutter="20">
      <el-col v-for="item in toolList" :span="6" >
        <el-card :body-style="{ padding: '0px' }">
          <div class="img_container">
            <img :src=item.image_url class="image"/>
          </div>
          <div style="padding: 14px">
            <span>{{item.name}}</span>
            <div class="bottom">
              <span class="description">{{ item.description }}</span>
              <el-button type="primary" class="button" round @click="openServerService(item.service_url)" >Open</el-button>
            </div>
          </div>
        </el-card>
      </el-col>
    </el-row>
  </div>
</template>

<script>
import { ref } from 'vue'
import {RVIZWEB_URL} from "@/config/rvizWeb.js"
import {SEAFILE_URL} from "@/config/seafile.js"
import {BASE_URL} from "@/config/api.js"

export default {
  // setup() {
  //   const externalUrl = ref(RVIZWEB_URL)
  //   return {
  //       externalUrl,
  //   }
  // },
  data() {
    return {
      toolList: [
        {
          "name": "Rviz Web",
          "description": "Rviz的web版本",
          "image_url": BASE_URL + "/files/images/foxglove.png",
          "service_url": RVIZWEB_URL,
        },
        {
          "name": "Seafile",
          "description": "seafile网盘",
          "image_url": BASE_URL + "/files/images/seafile.png",
          "service_url": SEAFILE_URL,
        },
      ],
    }
  },
  methods: {
    openServerService(service_url) {
       window.open(service_url)
    },
  },
}
</script>

<style scoped>
.description {
  font-size: 12px;
  color: #999;
}

.bottom {
  margin-top: 13px;
  line-height: 12px;
  display: flex;
  justify-content: space-between;
  align-items: center;
}

.button {
  /* padding: 0; */
  min-height: auto;
}

.image {
  /* height: 100%;
  display: inline; */
  position: absolute;
  width: 100%;
  top: 0;
  bottom: 0;
  left: 0;
  right: 0;
}

.img_container {
  position: relative;
  width: 100%;
  height: 0px;
  padding-bottom: 90%;
}
.img_container::before {
  content: "";
  display: block;
  padding-top: 100%;
}
</style>