#[cxx::bridge(namespace = "jolt_rust_cpp")]
mod ffi {
    unsafe extern "C++" {
        /*
        include!("jolt_rust_cpp/src/vehicle.h");
        type SimSystem;
        fn new_sim_system() -> UniquePtr<SimSystem>;
        */
        include!("jolt_rust_cpp/src/HelloWorld.h");
        fn hello_world() -> i64;
    }
}

fn main() {
    // let _sim_system = ffi::new_sim_system();
    println!("Hello, world!");
    ffi::hello_world();
    println!("done");
}
