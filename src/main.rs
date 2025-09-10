#[cxx::bridge(namespace = "jolt_rust_cpp")]
mod ffi {
    unsafe extern "C++" {
        include!("jolt_rust_cpp/src/vehicle.h");
        type SimSystem;
        fn new_sim_system() -> UniquePtr<SimSystem>;
    }
}

fn main() {
    let _sim_system = ffi::new_sim_system();
    println!("Hello, world!");
}
