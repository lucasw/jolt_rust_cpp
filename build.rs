use cmake;

fn main() {
    // force rebuild if this file changes
    println!("cargo:rerun-if-changed=NULL");

    // let dst = cmake::build("JoltPhysics/Build");
    let dst = cmake::Config::new("JoltPhysics/Build")
        // TODO(lucasw) still only works in debug mode
        // in release get undefined reference to `JPH::AssertFailed'
        // no matter which settings of JPH_ENABLE_ASSERTS used
        // "USE_ASSERTS" -> -DUSE_ASERTS on command line -> USE_ASSERTS in cmake file
        .define("USE_ASSERTS", "ON")
        .build();

    println!("cargo:rustc-link-search=native={}", dst.display());

    cxx_build::bridge("src/lib.rs")
        .file("src/misc.cpp")
        .include("JoltPhysics")
        // these need to sync with defaults in JoltPhysics cmake
        .define("JPH_DEBUG_RENDERER", Some("1"))
        .define("JPH_OBJECT_STREAM", Some("1"))
        .define("JPH_PROFILE_ENABLED", Some("1"))
        // TODO(lucasw) there is an issue here with debug builds vs release builds,
        // only debug builds are currently working
        .define("JPH_ENABLE_ASSERTS", Some("1"))
        .std("c++20")
        .compile("vehicle_jolt");

    println!("cargo:rerun-if-changed=src/misc.cpp");
    println!("cargo:rerun-if-changed=src/misc.h");

    let out_dir = std::path::PathBuf::from(std::env::var("OUT_DIR").unwrap());
    println!("cargo:rustc-link-search=native={}/lib", out_dir.display());
    println!("cargo:rustc-link-lib=Jolt");

    // On macOS and Linux, we need to explicitly link against the C++ standard
    // library here to avoid getting missing symbol errors from Jolt/JoltC.
    if cfg!(target_os = "macos") {
        println!("cargo:rustc-flags=-l dylib=c++");
    }

    if cfg!(target_os = "linux") {
        println!("cargo:rustc-link-lib=dylib=stdc++");
    }
}
