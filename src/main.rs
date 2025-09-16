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
        fn update(self: Pin<&mut SimSystem>);
        // fn update(&self);
        fn close(self: Pin<&mut SimSystem>);
        // fn close(&self);
    }
}

fn main() {
    println!("jolt_rust_cpp");
    let mut sim_system = ffi::new_sim_system(8000);
    for _i in 0..300 {
        sim_system.as_mut().unwrap().update();
    }
    // sim_system.init(8000);
    // sim_system.close();
    sim_system.as_mut().unwrap().close();
    println!("done");
}
