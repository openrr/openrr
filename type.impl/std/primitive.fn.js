(function() {
    var type_impls = Object.fromEntries([["clang_sys",[]],["glutin_egl_sys",[]],["wayland_sys",[]],["x11rb",[]],["x11rb_protocol",[]]]);
    if (window.register_type_impls) {
        window.register_type_impls(type_impls);
    } else {
        window.pending_type_impls = type_impls;
    }
})()
//{"start":55,"fragment_lengths":[16,22,19,13,22]}