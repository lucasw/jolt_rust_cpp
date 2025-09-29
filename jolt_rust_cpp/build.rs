use cmake;

fn main() {
    // force rebuild if this file changes
    println!("cargo:rerun-if-changed=NULL");

    let profile = std::env::var("PROFILE").unwrap();
    if profile == "debug" {
        println!("cargo:warning=Running build.rs in DEBUG mode");
        let mut config = cmake::Config::new("../JoltPhysics/Build");
        println!("cargo:warning=config {}", config.get_profile());
        let dst = config.build();

        println!("cargo:rustc-link-search=native={}", dst.display());

        cxx_build::bridge("src/lib.rs")
            .file("src/misc.cpp")
            // TODO(lucasw) any way to sync these with whatever Jolt.cmake used?
            .define("JPH_DEBUG_RENDERER", Some("1"))
            .define("JPH_ENABLE_ASSERTS", Some("1"))
            .define("JPH_OBJECT_STREAM", Some("1"))
            .define("JPH_PROFILE_ENABLED", Some("1"))
            .include(dst.join("include"))
            .std("c++20")
            .compile("vehicle_jolt");
    } else if profile == "release" {
        println!("cargo:warning=Running build.rs in RELEASE mode");
        let mut config = cmake::Config::new("../JoltPhysics/Build");
        println!("cargo:warning=config {}", config.get_profile());
        let dst = config
            // TODO(lucasw) build release doesn't work without this, need to figure out how to turn it
            // off
            // "USE_ASSERTS" -> -DUSE_ASSERTS on command line -> USE_ASSERTS in cmake file
            // .define("USE_ASSERTS", "OFF")
            // .define("GENERATE_DEBUG_SYMBOLS", "OFF")
            .define("PROFILER_IN_DEBUG_AND_RELEASE", "OFF")
            .build();

        println!("cargo:rustc-link-search=native={}", dst.display());

        // https://github.com/jrouwe/JoltPhysics/discussions/1332
        cxx_build::bridge("src/lib.rs")
            .file("src/misc.cpp")
            .define("JPH_ENABLE_ASSERTS", None)
            .define("JPH_DISABLE_CUSTOM_ALLOCATOR", None)
            .define("JPH_TRACK_BROADPHASE_STATS", None)
            .define("JPH_TRACK_NARROWPHASE_STATS", None)
            .define("JPH_EXTERNAL_PROFILE", None)
            .define("JPH_PROFILE_ENABLED", None)
            .define("NDEBUG", Some("1"))
            .define("_DEBUG", None)
            /*
            .define("DEBUG", None)
            */
            .include(dst.join("include"))
            .std("c++20")
            .compile("vehicle_jolt");

    } else {
        println!("cargo:warning=Running build.rs with unknown profile: {}", profile);
    }

    // TODO(lucasw) can't really make use of this with cxx bridge
    /*
    // TODO(lucasw) load these from environmental variables or input text file
    let num_terrain = 196;
    let num_terrain2 = num_terrain * num_terrain;
    let num_rays = 1000;

    let out_dir = std::env::var_os("OUT_DIR").unwrap();

    let path = std::path::Path::new(&out_dir).join("constants.rs");
    let mut text = format!("const NUM: usize = {num_terrain};\n");
    text += &format!("const NUM2: usize = {};\n", num_terrain2);
    text += &format!("const NUM_RAYS: usize = {};\n", num_rays);
    std::fs::write(&path, text).unwrap();
    */

    // cxx bridge can't use this with include!()
    /*
    let path = std::path::Path::new(&out_dir).join("array_structs.rs");
    let mut text = format!("");
    text += r###"
    #[derive(Clone)]
    struct CRayCastConfig {
        // TODO(lucasw) hard-coded to vehicle reference frame for now
        offset: CVec3,
    "###;

    text += &format!("    directions: [CVec3; {num_rays}],");
    text += r###"
        num_rays: usize,
    }

    #[derive(Clone)]
    struct CTerrain {
        cell_size: f32,
        offset: CVec3,
        num: usize,
        "###;
    text += &format!("heights: [f32; {num_terrain2}],\n    }}");
    std::fs::write(&path, text).unwrap();
    */

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
