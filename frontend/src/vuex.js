import { createStore } from 'vuex';

const store = createStore({
  state() {
    return {
        isLoggedIn: false,
        username: null, 
        role: null
    };
  },
  mutations: {
    SET_LOGIN_STATE(state, isLoggedIn) {
        state.isLoggedIn = isLoggedIn;
    },
    SET_USER(state, username) {
        state.username = username;
    },
    SET_ROLE(state, role) {
        state.role = role;
    },
    REFREASH_USER_INFO(state) {
        const isLoggedIn = eval(localStorage.getItem('isLoggedIn'))
        const username = localStorage.getItem('username')
        const role = localStorage.getItem('role')
        state.isLoggedIn = isLoggedIn;
        state.username = username;
        state.role = role;
    }
    // setUser(state, user) {
    //   state.user = user;
    // },
    // setPermissions(state, permissions) {
    //   state.permissions = permissions;
    // },
  },
  actions: {
    initializeApp({ commit }) {
        const isLoggedIn = eval(localStorage.getItem('isLoggedIn'))
        const username = localStorage.getItem('username')
        const role = localStorage.getItem('role')
        if (isLoggedIn && username) {
            commit('SET_LOGIN_STATE', isLoggedIn)
            commit('SET_USER', username)
            commit('SET_ROLE', role)
        }
    }
    // updateUser({ commit }, user) {
    //   commit('setUser', user);
    // },
    // updatePermissions({ commit }, permissions) {
    //   commit('setPermissions', permissions);
    // },
  },
//   getters: {
//     isAuthenticated(state) {
//       return state.user !== null;
//     },
//     hasPermission: (state) => (permission) => {
//       return state.permissions.includes(permission);
//     },
//   },
});

export default store;