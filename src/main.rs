#[cxx::bridge(namespace = "jolt_rust_cpp")]
mod ffi {
    unsafe extern "C++" {
        /*
        include!("jolt_rust_cpp/src/vehicle.h");
        type SimSystem;
        fn new_sim_system() -> UniquePtr<SimSystem>;
        */
        include!("jolt_rust_cpp/src/misc.h");
        fn init() -> i64;
    }
}

fn main() {
    println!("jolt_rust_cpp");
    // let _sim_system = ffi::new_sim_system();
    ffi::init();
    println!("done");
}
