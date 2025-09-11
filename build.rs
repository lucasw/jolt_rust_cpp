fn main() {
    cxx_build::bridge("src/main.rs")
        .file("src/vehicle.cpp")
        // .file("src/HelloWorld.cpp")
        // this needs to be the same as what was built by the joltc in cargo
        // could have a submodule here to make sure it's the same
        // Or is it possible to get the path to the header files in target/release/joltc-sys/...?
        .include("../jolt-rust/crates/joltc-sys/JoltC/JoltPhysics")
        // This doesn't eliminate the 'Version mismatch' error message
        // maybe it's because jolt-sys has different settings, try matching those
        // or need to recompile all of jolt, don't use jolt-sys at all
        // .include("target/release/build/joltc-sys-5fa7919ad5c36a20/out/include")
        .define("JPH_DEBUG_RENDERER", Some("1"))
        .define("JPH_PROFILE_ENABLED", Some("1"))
        .std("c++20")
        .compile("vehicle_jolt");

    println!("cargo:rerun-if-changed=src/vehicle.cpp");
    println!("cargo:rerun-if-changed=src/vehicle.h");
    // println!("cargo:rerun-if-changed=src/HelloWorld.cpp");

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
