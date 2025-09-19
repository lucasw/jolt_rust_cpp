const NUM: usize = 196;
const NUM2: usize = NUM * NUM;

const NUM_RAYS: usize = 1000;

#[cxx::bridge(namespace = "jolt_rust_cpp")]
mod ffi {
    #[derive(Clone, Debug, Default)]
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
        offset: CVec3,
        num: usize,
        heights: [f32; 38416],
        //  unsupported expression, array length must be an integer literal
        // heights: [f32; NUM2],
    }

    #[derive(Clone)]
    struct CControls {
        forward: f32,
        right: f32,
    }

    #[derive(Clone)]
    struct CRayCastConfig {
        // TODO(lucasw) hard-coded to vehicle reference frame for now
        offset: CVec3,
        directions: [CVec3; 1000],
        num_rays: usize,
    }

    unsafe extern "C++" {
        /*
        include!("jolt_rust_cpp/src/vehicle.h");
        */
        include!("jolt_rust_cpp/src/misc.h");
        type SimSystem;
        fn new_sim_system(
            max_num_bodies: u32,
            vehicle_half_size: CVec3,
            terrain: CTerrain,
        ) -> UniquePtr<SimSystem>;
        // fn init(max_num_bodies: u32) -> i64;
        fn update(self: Pin<&mut SimSystem>, control: CControls) -> CarTfs;
        fn get_rays(&self, ray_cast_config: CRayCastConfig) -> CRayCastConfig;
        fn close(self: Pin<&mut SimSystem>);
    }
}

fn ray_cast_config_default() -> ffi::CRayCastConfig {
    ffi::CRayCastConfig {
        offset: ffi::CVec3::default(),
        directions: [(); NUM_RAYS].map(|_| ffi::CVec3::default()),
        num_rays: NUM_RAYS,
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

    let ray_cast_config = {
        let max_range = 30.0;
        let mut line_strips = Vec::new();

        let mut ray_cast_config = ray_cast_config_default();
        let x0 = 0.0; // 2.0;
        let y0 = 0.0;
        let z0 = 0.0; // 5.0;
        ray_cast_config.offset = ffi::CVec3 {
            x: x0,
            y: y0,
            z: z0,
        };
        let num_rays = ray_cast_config.directions.len();
        let num_azimuth = num_rays / 10;
        let num_layers = num_rays / num_azimuth;
        let mut ind = 0;
        for layer in 0..num_layers {
            let fr_layer = layer as f32 / num_layers as f32;
            for i in 0..num_azimuth {
                let fr = i as f32 / num_azimuth as f32;
                let elevation_angle = (fr_layer - 0.5) * std::f32::consts::PI;
                let azimuth_angle = fr * 2.0 * std::f32::consts::PI;
                let dir = ffi::CVec3 {
                    x: max_range * azimuth_angle.cos() * elevation_angle.cos(),
                    y: max_range * azimuth_angle.sin() * elevation_angle.cos(),
                    z: max_range * elevation_angle.sin(),
                };
                let points = [[x0, y0, z0], [x0 + dir.x, y0 + dir.y, z0 + dir.z]];
                line_strips.push(points);
                ray_cast_config.directions[ind] = dir;
                ind += 1;
            }
        }
        println!("ray {:?}", ray_cast_config.directions[600]);

        rec.log_static(
            "world/ray_casts",
            &rerun::LineStrips3D::new(line_strips), // .with_vertex_normals([[0.0, 0.0, 1.0]])
                                                    // .with_vertex_colors([0x0000FFFF, 0x00FF00FF, 0xFF0000FF])
                                                    // .with_vertex_colors(colors)
                                                    // .with_triangle_indices(triangles),
        )?;

        ray_cast_config
    };

    let vehicle_half_size = ffi::CVec3 {
        x: 2.0,
        y: 0.9,
        z: 0.2,
    };

    let cell_size = 3.0;
    let mut yaw_lagging = 0.0;

    use noise::{NoiseFn, Perlin};
    let perlin = Perlin::new(1);
    let terrain = {
        let x0 = 0.0;
        let y0 = 0.0;
        let z0 = -0.3;

        let mut heights: [f32; NUM2] = [0.0; NUM2];
        let sc = 10.0 / NUM as f64;
        let sc2 = 30.0 / NUM as f64;
        let offset = -(NUM as f64 * cell_size * 0.5);
        let mut vertices = Vec::new();
        let mut colors = Vec::new();
        for xi in 0..NUM {
            for yi in 0..NUM {
                let x = xi as f64 * cell_size + offset;
                let y = yi as f64 * cell_size + offset;
                let z_norm = perlin.get([xi as f64 * sc, yi as f64 * sc])
                    + 0.15 * perlin.get([xi as f64 * sc2, yi as f64 * sc2]);
                let z = z_norm * 3.0;
                vertices.push([x0 + x, y0 + y, z0 + z]);
                heights[xi * NUM + yi] = z as f32;
                let r = (((x - offset) / 4.0) as u32 % 255) as u8;
                let g = ((255.0 * ((z_norm + 1.0) / 2.0)) as u32 % 255) as u8;
                let b = (255.0 * ((z_norm + 1.0) / 2.0)) as u8;
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
            offset: ffi::CVec3 {
                x: x0 as f32,
                y: y0 as f32,
                z: z0 as f32,
            },
            heights,
        };

        assert_eq!(NUM * NUM, terrain.heights.len());
        assert_eq!(NUM * NUM, NUM2);
        terrain
    };

    let mut sim_system = ffi::new_sim_system(8000, vehicle_half_size.clone(), terrain);
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
        let time_s = step as f64 * delta_time as f64;
        rec.set_timestamp_secs_since_epoch("view", time_s);

        let control = ffi::CControls {
            forward: 0.052,
            right: (perlin.get([time_s / 10.0]) / 2.0) as f32,
        };
        let car_tfs = sim_system.as_mut().unwrap().update(control);

        {
            let ray_results = sim_system
                .as_mut()
                .unwrap()
                .get_rays(ray_cast_config.clone());
            /*
               if step == 200 {
               for pos in ray_results.directions {
               println!("{pos:?}");
               }
               }
            */
            let mut points = Vec::new();
            points.push((
                ray_cast_config.offset.x,
                ray_cast_config.offset.y,
                ray_cast_config.offset.z,
            ));
            let mut radii = Vec::new();
            for pos in ray_results.directions {
                points.push((pos.x, pos.y, pos.z));
                radii.push(0.05);
                // println!("{pos:?}");
            }

            rec.log(
                "world/ray_cast_results",
                &rerun::Points3D::new(points).with_radii(radii),
            )?;
        }

        // need to rotate the quat for rerun
        // TODO(lucasw) based on changing the wheel sizes it appears the width and length
        // are swapped, or the quat rotation is wrong here
        let rerun_quat = cquat_to_rerun(car_tfs.body.quat);

        let rerun_pos =
            rerun::Vec3D::from([car_tfs.body.pos.x, car_tfs.body.pos.y, car_tfs.body.pos.z]);
        rec.log(
            "world/car0",
            &rerun::Transform3D::from_translation_rotation(rerun_pos, rerun_quat),
        )?;

        // zero out the roll and pitch
        let (_roll, _pitch, yaw) = rerun_quat.to_euler(rerun::external::glam::EulerRot::XYZ);
        // lag the new yaw
        let lag_fr = 0.98;
        yaw_lagging = yaw_lagging * lag_fr + yaw * (1.0 - lag_fr);
        let rerun_quat_yaw_only = rerun::external::glam::Quat::from_euler(
            rerun::external::glam::EulerRot::XYZ,
            0.0,
            0.0,
            yaw_lagging,
        );
        let rerun_pos =
            rerun::Vec3D::from([car_tfs.body.pos.x, car_tfs.body.pos.y, car_tfs.body.pos.z]);
        rec.log(
            "world/car",
            &rerun::Transform3D::from_translation_rotation(rerun_pos, rerun_quat_yaw_only),
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

        let half_wheel_width = 0.15;
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
