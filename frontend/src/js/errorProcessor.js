import store from '../vuex.js';

function handleErr(err) {
    let errCode = err.response.status
    if (errCode === 401) {
        handleErr401(err)
    }
}

function handleErr401(err) {
    resetLoginStatus()
};

function resetLoginStatus() {
    localStorage.setItem('token', null)
    localStorage.setItem('username', null)
    localStorage.setItem('role', null)
    localStorage.setItem('isLoggedIn', false)
    store.commit('REFREASH_USER_INFO')
};

export { handleErr, resetLoginStatus }