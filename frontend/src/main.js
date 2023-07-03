import { createApp } from 'vue'
import App from './App.vue'
import router from './router'
import ElementPlus from 'element-plus'
import store from './vuex.js';

import './assets/main.css'
import 'element-plus/dist/index.css'
import 'element-plus/theme-chalk/index.css'

const app = createApp(App)
app.use(router)
app.use(ElementPlus)
app.use(store);
app.mount('#app')
