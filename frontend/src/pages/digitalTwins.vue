<template>
  <div style="height:100%">
    <iframe :src="externalUrl" width="100%" height="100%"></iframe>
  </div>
</template>

<script>
import { ref } from 'vue'
import {GZWEB_URL} from "@/config/gzweb.js"

export default {
  setup() {
    const externalUrl = ref(GZWEB_URL)

    return {
      externalUrl
    }
  }
}
</script>