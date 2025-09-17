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

    #[derive(Clone)]
    struct CarTfs {
        body: CTf,
        wheel_fl: CTf,
        wheel_fr: CTf,
        wheel_bl: CTf,
        wheel_br: CTf,
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
            vehicle_half_size: CVec3,
        ) -> UniquePtr<SimSystem>;
        // fn init(max_num_bodies: u32) -> i64;
        fn update(self: Pin<&mut SimSystem>) -> CarTfs;
        fn close(self: Pin<&mut SimSystem>);
    }
}

fn cquat_to_rerun(quat: ffi::CQuat) -> rerun::external::glam::Quat {
    rerun::external::glam::Quat::from_xyzw(quat.x, quat.y, quat.z, quat.w)
}

// cylinder conventions in jolt and rerun differ
fn cylinder_cquat_to_rerun(quat: ffi::CQuat) -> rerun::external::glam::Quat {
    let rot = rerun::external::glam::Quat::from_euler(
        rerun::external::glam::EulerRot::XYZ,
        std::f32::consts::FRAC_PI_2,
        0.0,
        0.0,
    );

    rerun::external::glam::Quat::from_xyzw(quat.x, quat.y, quat.z, quat.w) * rot
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

    let vehicle_half_size = ffi::CVec3 {
        x: 2.0,
        y: 0.9,
        z: 0.2,
    };

    let mut sim_system = ffi::new_sim_system(8000, floor_pos.clone(), vehicle_half_size.clone());
    let floor_half_extent = 25.0;
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

    let delta_time = 1.0 / 60.0;

    for step in 0..1100 {
        rec.set_timestamp_secs_since_epoch("view", step as f64 * delta_time as f64);

        let car_tfs = sim_system.as_mut().unwrap().update();

        // need to rotate the quat for rerun
        // TODO(lucasw) based on changing the wheel sizes it appears the width and length
        // are swapped, or the quat rotation is wrong here
        let rerun_quat = cquat_to_rerun(car_tfs.body.quat);
        rec.log(
            "world/car",
            &rerun::Boxes3D::from_centers_and_half_sizes(
                [(car_tfs.body.pos.x, car_tfs.body.pos.y, car_tfs.body.pos.z)],
                [(
                    vehicle_half_size.x,
                    vehicle_half_size.y,
                    vehicle_half_size.z,
                )],
            )
            // .with_fill_mode(rerun::FillMode::Solid)
            .with_quaternions([rerun_quat]),
        )?;

        let half_wheel_width = 0.1;
        let wheel_radius = 0.3;
        let mut ind = 0;
        for wheel_tf in [
            car_tfs.wheel_fl,
            car_tfs.wheel_fr,
            car_tfs.wheel_bl,
            car_tfs.wheel_br,
        ] {
            let rerun_quat = cylinder_cquat_to_rerun(wheel_tf.quat);
            rec.log(
                format!("world/wheel{ind}"),
                &rerun::Cylinders3D::from_lengths_and_radii(
                    [half_wheel_width * 2.0],
                    [wheel_radius],
                )
                .with_centers([rerun::external::glam::vec3(
                    wheel_tf.pos.x,
                    wheel_tf.pos.y,
                    wheel_tf.pos.z,
                )])
                .with_quaternions([rerun_quat]),
            )?;
            ind += 1;
        }
    }

    sim_system.as_mut().unwrap().close();
    println!("done");

    Ok(())
}
