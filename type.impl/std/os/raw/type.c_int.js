(function() {
    var type_impls = Object.fromEntries([["assimp_sys",[]],["glutin_egl_sys",[]],["glutin_glx_sys",[]],["r2r_rcl",[]],["x11_dl",[]]]);
    if (window.register_type_impls) {
        window.register_type_impls(type_impls);
    } else {
        window.pending_type_impls = type_impls;
    }
})()
//{"start":55,"fragment_lengths":[17,22,22,15,14]}