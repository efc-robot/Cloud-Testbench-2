import { createRouter, createWebHistory } from 'vue-router';
import DigitalTwins from '../pages/digitalTwins.vue';
import Robots from '../pages/robots.vue';
import Develop from '../pages/develop.vue';
import DevTools from '../pages/devTools.vue';
import UserManage from '../pages/userManage.vue';
import RobotManage from '../pages/robotManage.vue';
import DigitalTwinsManage from '../pages/digitalTwinsManage.vue';
import FtpManage from '../pages/ftpManage.vue';
import issues from '../pages/issues.vue';
import documentation from '../pages/documentation.vue';

const routes = [
  {
    path: '/',
    name: 'Home',
    component: DigitalTwins
  },
  {
    path: '/digitalTwins',
    name: 'DigitalTwins',
    component: DigitalTwins
  },
  {
    path: '/robots',
    name: 'Robots',
    component: Robots
  },
  {
    path: '/develop',
    name: 'Develop',
    component: Develop
  },
  {
    path: '/dev_tools',
    name: 'DevTools',
    component: DevTools
  },
  {
    path: '/userManage',
    name: 'UserManage',
    component: UserManage
  },
  {
    path: '/robotManage',
    name: 'RobotManage',
    component: RobotManage
  },
  {
    path: '/ftpManage',
    name: 'FtpManage',
    component: FtpManage
  },
  {
    path: '/digitalTwinsManage',
    name: 'DigitalTwinsManage',
    component: DigitalTwinsManage
  },
  {
    path: '/issues',
    name: 'issues',
    component: issues
  },
  // {
  //   path: '/documentation/intro',
  //   name: 'documentation',
  //   component: documentation
  // },
  {
    path: '/documentation',
    name: 'documentation',
    component: documentation,
    children: [
      { path: 'markdown/intro', component: documentation, },
      {
        path: 'markdown/real_robot',
        component: documentation,
        children: [
          { path: 'access_cloud_testbench', component: documentation, },
          { path: 'apply_and_use', component: documentation, },
        ]
      },
      {
        path: 'markdown/visual_robot',
        component: documentation,
        children: [
          { path: 'intro', component: documentation, },
          { path: 'usage', component: documentation, },
          { path: 'ros_master', component: documentation, },
          { path: 'gzweb', component: documentation, },
        ]
      },
      {
        path: 'markdown/seafile',
        component: documentation,
        children: [
          { path: 'usage', component: documentation, },
          { path: 'webdav', component: documentation, },
        ]
      },
      {
        path: 'markdown/foxglove',
        component: documentation,
        children: [
          { path: 'intro', component: documentation, },
          { path: 'quick_start', component: documentation, },
        ],
      },
      {
        path: 'markdown/optitrack',
        component: documentation,
        children: [
          { path: 'intro', component: documentation, },
          { path: 'quick_start', component: documentation, },
          { path: 'vrpn_client', component: documentation, },
        ],
      },
    ]
  },
  {
    path: '/issues',
    name: 'issues',
    component: issues
  }
];

const router = createRouter({
  history: createWebHistory(),
  routes
});

export default router;