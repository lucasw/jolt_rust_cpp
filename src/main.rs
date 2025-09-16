#[cxx::bridge(namespace = "jolt_rust_cpp")]
mod ffi {
    #[derive(Clone)]
    struct CVec3 {
        x: f32,
        y: f32,
        z: f32,
    }

    unsafe extern "C++" {
        /*
        include!("jolt_rust_cpp/src/vehicle.h");
        */
        include!("jolt_rust_cpp/src/misc.h");
        type SimSystem;
        fn new_sim_system(
            max_num_bodies: u32,
            // floor_pos_x: f32, floor_pos_y: f32, floor_pos_z: f32,
            floor_pos: CVec3,
        ) -> UniquePtr<SimSystem>;
        // fn init(max_num_bodies: u32) -> i64;
        fn update(self: Pin<&mut SimSystem>);
        // fn update(&self);
        fn close(self: Pin<&mut SimSystem>);
        // fn close(&self);
    }
}

fn main() -> Result<(), anyhow::Error> {
    println!("jolt_rust_cpp");

    let rec = rerun::RecordingStreamBuilder::new("jolt_rust_cpp").spawn()?;

    let floor_pos = ffi::CVec3 {
        x: 0.0,
        y: 0.0,
        z: -1.0,
    };
    // TODO(lucasw) use quat in SimSystem
    let floor_quat = (0.0, 0.0, 0.0, 1.0);

    let mut sim_system = ffi::new_sim_system(8000, floor_pos.clone());
    let floor_half_extent = 50.0;
    let floor_half_height = 1.0;

    rec.log_static(
        "world/floor",
        &rerun::Boxes3D::from_centers_and_half_sizes(
            [(floor_pos.x, floor_pos.y, floor_pos.z)],
            [(floor_half_extent, floor_half_extent, floor_half_height)],
        )
        // .with_fill_mode(rerun::FillMode::Solid)
        .with_quaternions([rerun::Quaternion::from_xyzw([
            floor_quat.0,
            floor_quat.1,
            floor_quat.2,
            floor_quat.3,
        ])]),
    )?;

    for _i in 0..300 {
        sim_system.as_mut().unwrap().update();
    }
    // sim_system.init(8000);
    // sim_system.close();
    sim_system.as_mut().unwrap().close();
    println!("done");

    Ok(())
}
