const NUM: usize = 100;
const NUM2: usize = NUM * NUM;

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

    #[derive(Clone)]
    struct CTerrain {
        cell_size: f32,
        num: usize,
        heights: [f32; 10000],
        //  unsupported expression, array length must be an integer literal
        // heights: [f32; NUM2],
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
            terrain: CTerrain,
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
        z: -2.0,
    };

    let vehicle_half_size = ffi::CVec3 {
        x: 2.0,
        y: 0.9,
        z: 0.2,
    };

    let cell_size = 2.5;

    use noise::{NoiseFn, Perlin};
    let perlin = Perlin::new(1);
    let mut heights: [f32; NUM2] = [0.0; NUM2];
    let sc = 16.0 / NUM as f64;
    let offset = -(NUM as f64 * cell_size * 0.5);
    let mut vertices = Vec::new();
    let mut colors = Vec::new();
    for xi in 0..NUM {
        for yi in 0..NUM {
            let x = xi as f64 * cell_size + offset;
            let y = yi as f64 * cell_size + offset;
            let z = perlin.get([xi as f64 * sc, yi as f64 * sc]);
            vertices.push([x, y, z]);
            heights[xi * NUM + yi] = z as f32;
            let r = (((x - offset) / 1.0) as u32 % 255) as u8;
            let g = ((255.0 * ((z + 1.0) / 2.0)) as u32 % 255) as u8;
            let b = (255.0 * ((z + 1.0) / 2.0)) as u8;
            colors.push(rerun::Color::from_rgb(r, g, b));
        }
    }

    let mut triangles = Vec::new();
    for xi in 0..NUM - 1 {
        for yi in 0..NUM - 1 {
            let i0 = (xi * NUM + yi) as u32;
            let i1 = ((xi + 1) * NUM + yi) as u32;
            let i2 = (xi * NUM + yi + 1) as u32;
            let i3 = ((xi + 1) * NUM + yi + 1) as u32;
            triangles.push([i0, i3, i2]);
            triangles.push([i0, i1, i3]);
        }
    }

    rec.log_static(
        "world/ground",
        &rerun::Mesh3D::new(
            // [[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
            vertices,
        )
        // .with_vertex_normals([[0.0, 0.0, 1.0]])
        // .with_vertex_colors([0x0000FFFF, 0x00FF00FF, 0xFF0000FF])
        .with_vertex_colors(colors)
        .with_triangle_indices(triangles),
    )?;

    let terrain = ffi::CTerrain {
        cell_size: cell_size as f32,
        num: NUM,
        heights,
    };

    assert_eq!(NUM * NUM, terrain.heights.len());
    assert_eq!(NUM * NUM, NUM2);

    let mut sim_system =
        ffi::new_sim_system(8000, floor_pos.clone(), vehicle_half_size.clone(), terrain);
    /*
    // TODO(lucasw) use quat in SimSystem
    let floor_quat = (0.0, 0.0, 0.0, 1.0);

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
    */

    let car_cam_trans = rerun::Vec3D::from([-5.0, 0.0, 2.0]);
    let car_cam_rot = rerun::external::glam::Quat::from_euler(
        rerun::external::glam::EulerRot::XYZ,
        0.0,
        -std::f32::consts::FRAC_PI_2,
        -std::f32::consts::FRAC_PI_2,
    );
    rec.log_static(
        "world/car/chase_cam_base",
        &rerun::Transform3D::from_translation_rotation(car_cam_trans, car_cam_rot),
    )?;

    let car_cam_trans = rerun::Vec3D::from([0.0, 0.0, 0.0]);
    let car_cam_rot = rerun::external::glam::Quat::from_euler(
        rerun::external::glam::EulerRot::XYZ,
        -0.4,
        0.0,
        0.0,
    );
    rec.log_static(
        "world/car/chase_cam_base/chase_cam",
        &rerun::Transform3D::from_translation_rotation(car_cam_trans, car_cam_rot),
    )?;

    let aspect_ratio = 1.7;
    let fov_y = 0.9;
    rec.log_static(
        "world/car/chase_cam_base/chase_cam",
        &rerun::Pinhole::from_fov_and_aspect_ratio(fov_y, aspect_ratio)
            .with_camera_xyz(rerun::components::ViewCoordinates::RUB)
            .with_image_plane_distance(0.8),
    )?;

    let delta_time = 1.0 / 60.0;

    for step in 0..4100 {
        rec.set_timestamp_secs_since_epoch("view", step as f64 * delta_time as f64);

        let car_tfs = sim_system.as_mut().unwrap().update();

        // need to rotate the quat for rerun
        // TODO(lucasw) based on changing the wheel sizes it appears the width and length
        // are swapped, or the quat rotation is wrong here
        let rerun_quat = cquat_to_rerun(car_tfs.body.quat);

        let rerun_pos =
            rerun::Vec3D::from([car_tfs.body.pos.x, car_tfs.body.pos.y, car_tfs.body.pos.z]);
        rec.log(
            "world/car",
            &rerun::Transform3D::from_translation_rotation(rerun_pos, rerun_quat),
        )?;

        rec.log(
            "world/car_box",
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
