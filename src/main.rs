#[cxx::bridge(namespace = "jolt_rust_cpp")]
mod ffi {
    unsafe extern "C++" {
        /*
        include!("jolt_rust_cpp/src/vehicle.h");
        */
        include!("jolt_rust_cpp/src/misc.h");
        type SimSystem;
        fn new_sim_system(max_num_bodies: u32) -> UniquePtr<SimSystem>;
        // fn init(max_num_bodies: u32) -> i64;
    }
}

fn main() {
    println!("jolt_rust_cpp");
    let sim_system = ffi::new_sim_system(8000);
    /*
    for i in 0..100 {
        sim_system.step();
    }
    */
    // sim_system.init(8000);
    // ffi::init(8000);
    println!("done");
}
