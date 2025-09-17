#[cxx::bridge(namespace = "jolt_rust_cpp")]
mod ffi {
    #[derive(Clone)]
    struct CVec3 {
        x: f32,
        y: f32,
        z: f32,
    }

    #[derive(Clone)]
    struct CQuat {
        x: f32,
        y: f32,
        z: f32,
        w: f32,
    }

    #[derive(Clone)]
    struct CTf {
        pos: CVec3,
        quat: CQuat,
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
        fn update(self: Pin<&mut SimSystem>) -> CTf;
        fn close(self: Pin<&mut SimSystem>);
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

    // TODO(lucasw) share with SimSystem
    let car_half_length = 2.0;
    let car_half_width = 0.9;
    let car_half_height = 0.2;

    let delta_time = 1.0 / 60.0;

    for step in 0..1900 {
        rec.set_timestamp_secs_since_epoch("view", step as f64 * delta_time as f64);

        let car_tf = sim_system.as_mut().unwrap().update();

        // need to rotate the quat for rerun
        // TODO(lucasw) based on changing the wheel sizes it appears the width and length
        // are swapped, or the quat rotation is wrong here
        let rerun_quat = rerun::external::glam::Quat::from_xyzw(
            car_tf.quat.x,
            car_tf.quat.y,
            car_tf.quat.z,
            car_tf.quat.w,
        ) * rerun::external::glam::Quat::from_euler(
            rerun::external::glam::EulerRot::XYZ,
            std::f32::consts::FRAC_PI_2,
            0.0,
            0.0, // std::f32::consts::FRAC_PI_2,
        );
        rec.log(
            "world/car",
            &rerun::Boxes3D::from_centers_and_half_sizes(
                [(car_tf.pos.x, car_tf.pos.y, car_tf.pos.z)],
                [(car_half_width, car_half_length, car_half_height)],
            )
            // .with_fill_mode(rerun::FillMode::Solid)
            .with_quaternions([rerun_quat]),
        )?;
    }

    sim_system.as_mut().unwrap().close();
    println!("done");

    Ok(())
}
