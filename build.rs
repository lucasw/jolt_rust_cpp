fn main() {
    cxx_build::bridge("src/main.rs")
        .file("src/vehicle.cpp")
        // TODO(lucasw) this depends on a source checkout of JoltPhysics adjacent to this project
        // but there is one inside the current build dir (?), need to include that one instead
        // .include("../JoltPhysics")
        // this needs to be the same as what was built by the joltc in cargo
        .include("../jolt-rust/crates/joltc-sys/JoltC/JoltPhysics")
        .std("c++20")
        .compile("vehicle_jolt");

    println!("cargo:rerun-if-changed=src/vehicle.cpp");
    println!("cargo:rerun-if-changed=src/vehicle.h");

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
