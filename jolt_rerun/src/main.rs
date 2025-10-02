use jolt_rust_cpp::*;
use noise::{NoiseFn, Perlin};
use rerun::external::glam;

fn main() -> Result<(), anyhow::Error> {
    println!("jolt_rust_cpp");

    let rec = rerun::RecordingStreamBuilder::new("jolt_rust_cpp").spawn()?;

    let (ray_cast_config, ray_cast_line_strips) = jolt_rust_cpp::make_ray_casts();

    rec.log_static(
        "world/ray_casts",
        &rerun::LineStrips3D::new(ray_cast_line_strips), // .with_vertex_normals([[0.0, 0.0, 1.0]])
                                                         // .with_vertex_colors([0x0000FFFF, 0x00FF00FF, 0xFF0000FF])
                                                         // .with_vertex_colors(colors)
                                                         // .with_triangle_indices(triangles),
    )?;

    let vehicle_half_size = ffi::CVec3 {
        x: 2.0,
        y: 0.9,
        z: 0.2,
    };

    let cell_size = 3.0;
    let mut yaw_lagging = 0.0;

    let perlin = Perlin::new(1);

    let (terrain, vertices, triangles, colors) = make_terrain(cell_size, perlin);

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
    let car_cam_rot = glam::Quat::from_euler(
        glam::EulerRot::XYZ,
        0.0,
        -std::f32::consts::FRAC_PI_2,
        -std::f32::consts::FRAC_PI_2,
    );
    rec.log_static(
        "world/car/chase_cam_base",
        &rerun::Transform3D::from_translation_rotation(car_cam_trans, car_cam_rot),
    )?;

    let car_cam_trans = rerun::Vec3D::from([0.0, 0.0, 0.0]);
    let car_cam_rot = glam::Quat::from_euler(glam::EulerRot::XYZ, -0.4, 0.0, 0.0);
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

        let car_ray_cast_config =
            transform_ray_cast_config(&ray_cast_config, &car_tfs.body.pos, &car_tfs.body.quat);
        let ray_results = sim_system.as_mut().unwrap().get_rays(car_ray_cast_config);
        {
            let mut points = Vec::new();
            points.push((
                ray_results.offset.x,
                ray_results.offset.y,
                ray_results.offset.z,
            ));
            let mut radii = Vec::new();
            for pos in ray_results.directions {
                points.push((pos.x, pos.y, pos.z));
                radii.push(0.025);
            }

            rec.log(
                "world/ray_cast_results",
                &rerun::Points3D::new(points).with_radii(radii),
            )?;
        }

        let car_pos = cvec3_to_glam(&car_tfs.body.pos);
        let car_quat = cquat_to_glam(&car_tfs.body.quat);
        rec.log(
            "world/car0",
            &rerun::Transform3D::from_translation_rotation(car_pos, car_quat),
        )?;

        // zero out the roll and pitch for the chase camera
        let (_roll, _pitch, yaw) = car_quat.to_euler(glam::EulerRot::XYZ);
        // lag the new yaw
        let lag_fr = 0.98;
        yaw_lagging = yaw_lagging * lag_fr + yaw * (1.0 - lag_fr);
        let car_quat_yaw_only = glam::Quat::from_euler(glam::EulerRot::XYZ, 0.0, 0.0, yaw_lagging);
        rec.log(
            "world/car",
            &rerun::Transform3D::from_translation_rotation(car_pos, car_quat_yaw_only),
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
            .with_quaternions([car_quat]),
        )?;

        let half_wheel_width = 0.15;
        let wheel_radius = 0.3;
        let mut ind = 0;
        for wheel in [
            car_tfs.wheel_fl,
            car_tfs.wheel_fr,
            car_tfs.wheel_bl,
            car_tfs.wheel_br,
        ] {
            let rerun_quat = cylinder_cquat_to_rerun(wheel.tf.quat);
            rec.log(
                format!("world/wheel{ind}"),
                &rerun::Cylinders3D::from_lengths_and_radii(
                    [half_wheel_width * 2.0],
                    [wheel_radius],
                )
                .with_centers([cvec3_to_glam(&wheel.tf.pos)])
                .with_quaternions([rerun_quat]),
            )?;

            rec.log(
                format!("angular_velocity/wheel{ind}"),
                &rerun::Scalars::single(wheel.angular_velocity as f64),
            )?;

            rec.log(
                format!("rotation_angle/wheel{ind}"),
                &rerun::Scalars::single(wheel.rotation_angle as f64),
            )?;

            rec.log(
                format!("steer_angle/wheel{ind}"),
                &rerun::Scalars::single(wheel.steer_angle as f64),
            )?;

            ind += 1;
        }
    }

    sim_system.as_mut().unwrap().close();
    println!("done");

    Ok(())
}
