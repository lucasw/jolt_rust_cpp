//! Lucas Walter
//! Visualize a vehicle moving around in a jolt physics environment in bevy

use jolt_rust_cpp::*;
//use rerun::external::glam;
use noise::{NoiseFn, Perlin};

use bevy::{
    asset::RenderAssetUsages,
    color::palettes::css,
    mesh::{Indices, VertexAttributeValues},
    prelude::*,
    render::{
        Render,
        RenderSet,
        // bevy_asset::RenderAssetUsages,
        render_resource::CommandEncoder,
        render_resource::PrimitiveTopology,
        render_resource::{Extent3d, TextureDimension, TextureFormat, TextureUsages},
    },
    window::{PresentMode, WindowResolution},
};
use crossbeam_channel::{Receiver, Sender};

use ffi::SimSystem;
// use cxx::UniquePtr;
/*
#[derive(Component)]
// pub struct Sim<'a> {
pub struct Sim {
    throttle: f32,
    brake: f32,
    steering: f32,
    sim_system: bevy::prelude::NonSend<'a, cxx::UniquePtr<SimSystem>>,
    // sim_system: cxx::UniquePtr<SimSystem>,
    // _non_send_marker: bevy::prelude::NonSendMarker,
}
*/

#[derive(Component)]
struct UserCamera {
    relative_transform: Transform,
}

#[derive(Component)]
struct PointCloud;

#[derive(Component)]
struct VehicleData;

// TODO(lucasw) bundle all the transforms together?
#[derive(Component)]
struct WheelTransform;

#[derive(Resource)]
struct Config {
    terrain: ffi::CTerrain,
    vertices: Vec<[f64; 3]>,
    triangles: Vec<[u32; 3]>,
    vehicle_half_size: Vec3,
}

/// This will receive asynchronously any data sent from the render world
#[derive(Resource, Deref)]
struct VehicleControlsReceiver(Receiver<ffi::CControls>);

/// This will send asynchronously any data to the main world
#[derive(Resource, Deref)]
struct VehicleControlsSender(Sender<ffi::CControls>);

/// This will receive asynchronously any data sent from the render world
#[derive(Resource, Deref)]
struct VehicleDataReceiver(Receiver<(ffi::CarTfs, ffi::CRayCastConfig)>);

/// This will send asynchronously any data to the main world
#[derive(Resource, Deref)]
struct VehicleDataSender(Sender<(ffi::CarTfs, ffi::CRayCastConfig)>);

/// Creates a colorful test pattern
fn uv_debug_texture() -> Image {
    const TEXTURE_SIZE: usize = 8;

    let mut palette: [u8; 32] = [
        255, 102, 159, 255, 255, 159, 102, 255, 236, 255, 102, 255, 121, 255, 102, 255, 102, 255,
        198, 255, 102, 198, 255, 255, 121, 102, 255, 255, 236, 102, 255, 255,
    ];

    let mut texture_data = [0; TEXTURE_SIZE * TEXTURE_SIZE * 4];
    for y in 0..TEXTURE_SIZE {
        let offset = TEXTURE_SIZE * y * 4;
        texture_data[offset..(offset + TEXTURE_SIZE * 4)].copy_from_slice(&palette);
        palette.rotate_right(4);
    }

    Image::new_fill(
        Extent3d {
            width: TEXTURE_SIZE as u32,
            height: TEXTURE_SIZE as u32,
            depth_or_array_layers: 1,
        },
        TextureDimension::D2,
        &texture_data,
        TextureFormat::Rgba8UnormSrgb,
        RenderAssetUsages::default(),
    )
}

fn sim_setup(world: &mut World) {
    println!("setup simulation");

    let config = world.get_resource::<Config>().unwrap();

    let vehicle_half_size = ffi::CVec3 {
        x: config.vehicle_half_size.x,
        y: config.vehicle_half_size.y,
        z: config.vehicle_half_size.z,
    };
    let sim_system = ffi::new_sim_system(8000, vehicle_half_size.clone(), config.terrain.clone());
    world.insert_non_send_resource(sim_system);

    let (sender, receiver) = crossbeam_channel::unbounded();
    world.insert_resource(VehicleDataSender(sender));
    world.insert_resource(VehicleDataReceiver(receiver));
    let (sender, receiver) = crossbeam_channel::unbounded();
    world.insert_resource(VehicleControlsSender(sender));
    world.insert_resource(VehicleControlsReceiver(receiver));

    println!("setup simulation done");
}

fn viz_setup(
    mut commands: Commands,
    mesh_assets: ResMut<Assets<Mesh>>,
    material_assets: ResMut<Assets<StandardMaterial>>,
    images: ResMut<Assets<Image>>,
    config: Res<Config>,
) {
    let mesh_assets = mesh_assets.into_inner();
    let material_assets = material_assets.into_inner();
    let images = images.into_inner();

    // render to texture
    let size = Extent3d {
        width: 32,
        height: 16,
        ..default()
    };
    let mut image = Image::new_fill(
        size,
        TextureDimension::D2,
        &[4, 3, 2, 0], // these bytes are repeated width * height times for bgra
        // TextureFormat::bevy_default(),
        TextureFormat::Bgra8UnormSrgb,
        RenderAssetUsages::default(),
    );
    image.texture_descriptor.usage = TextureUsages::COPY_DST
        | TextureUsages::COPY_SRC
        | TextureUsages::TEXTURE_BINDING
        | TextureUsages::RENDER_ATTACHMENT;
    let image_handle = images.add(image);
    println!("{image_handle:?}");

    // draw a bunch of sphere for the point cloud
    {
        let sphere = Sphere::new(0.1).mesh().ico(3).unwrap();
        let material = StandardMaterial {
            // Alpha channel of the color controls transparency.
            // We set it to 0.0 here, because it will be changed over time in the
            // `fade_transparency` function.
            // Note that the transparency has no effect on the objects shadow.
            base_color: Color::srgba(0.2, 0.7, 0.1, 1.0),
            // Mask sets a cutoff for transparency. Alpha values below are fully transparent,
            // alpha values above are fully opaque.
            alpha_mode: AlphaMode::Mask(0.5),
            ..default()
        };
        let (rays, _) = jolt_rust_cpp::make_ray_casts();
        for _ind in 0..rays.num_rays {
            commands.spawn((
                // TODO(lucasw) instead of clone can I instance a bunch of spheres?
                Mesh3d(mesh_assets.add(sphere.clone())),
                MeshMaterial3d(material_assets.add(material.clone())),
                Transform::from_xyz(0.0, 0.0, 0.0),
                PointCloud,
            ));
        }
    }
    /*
    commands.spawn((
        Camera3d::default(),
        Camera {
            target: image_handle.clone().into(),
            clear_color: Color::WHITE.into(),
            ..default()
        },
        Transform::from_xyz(-10.0, 0.0, 5.0).looking_at(Vec3::new(-5.0, 0.0, 3.0), Vec3::Y),
        // RenderCamera,
    ));
    */

    /*
    let _ground = commands.spawn((
        Mesh3d(mesh_assets.add(Plane3d::default().mesh().size(1200.0, 1200.0))),
        MeshMaterial3d(material_assets.add(StandardMaterial {
            base_color: Color::hsv(13.0, 0.2, 0.2),
            ..default()
        })),
        Transform::from_xyz(0.0, -10.0, 0.0),
    ));
    */

    if false {
        let ground_size = 1200.0;
        let ground_height = 0.1;

        let ground = commands.spawn((
            Mesh3d(
                mesh_assets.add(
                    Plane3d::new(
                        Vec3::new(0.0, 0.0, 1.0),
                        Vec2::new(ground_size, ground_size),
                    )
                    .mesh(),
                ),
            ),
            MeshMaterial3d(material_assets.add(StandardMaterial {
                // TODO(lucasw) add uv coordinates to make this repeat
                base_color_texture: Some(images.add(uv_debug_texture())),
                ..default()
            })),
            Transform::from_xyz(0.0, 0.0, -5.0),
        ));
        println!("spawned ground: {}", ground.id());
    }

    // mesh for terrain
    {
        // need f32 for vertices
        // TODO(lucasw) nested map?
        // let vertices_f32: Vec<[f32; 3]> = config.vertices.into_iter().map(|x| x as f32).collect();
        let mut vertices = Vec::new();
        for vertex in &config.vertices {
            // need arraytools for [f64;4] mapping
            vertices.push([vertex[0] as f32, vertex[1] as f32, vertex[2] as f32]);
        }

        // the triangles are just one long list, not a vector of 3 indices arrays
        let mut indices_flat = Vec::new();
        let mut normals = Vec::new();
        for tri in &config.triangles {
            for tri_ind in tri {
                indices_flat.push(*tri_ind);
            }
            // TODO(lucasw) calculate better normals but straight up looks decent
            normals.push([0.0, 0.0, 1.0]);
        }
        commands.spawn((
            Mesh3d(
                mesh_assets.add(
                    Mesh::new(
                        PrimitiveTopology::TriangleList,
                        RenderAssetUsages::MAIN_WORLD | RenderAssetUsages::RENDER_WORLD,
                    )
                    .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, vertices)
                    .with_inserted_indices(Indices::U32(indices_flat))
                    .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, normals),
                ),
            ),
            MeshMaterial3d(material_assets.add(StandardMaterial {
                // TODO(lucasw) add uv coordinates to make this repeat
                base_color_texture: Some(images.add(uv_debug_texture())),
                ..default()
            })),
            Transform::from_xyz(0.0, 0.0, 0.0),
        ));
    }

    let _camera = commands.spawn((
        Camera3d::default(),
        UserCamera {
            relative_transform: Transform::from_xyz(-10.0, 0.0, 5.0)
                .looking_at(Vec3::new(-5.0, 0.0, 3.0), Vec3::Z),
        },
        Transform::default(),
    ));

    commands.spawn((
        DirectionalLight {
            illuminance: 0.01,
            shadows_enabled: true,
            ..default()
        },
        Transform::IDENTITY.looking_at(Vec3::new(0.25, 0.1, -1.0), Vec3::Y),
    ));

    // Some light to see something
    commands.spawn((
        PointLight {
            intensity: 225_000_000.,
            range: 500.,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(100., 82., 40.),
    ));

    // vehicle
    commands.spawn((
        Mesh3d(mesh_assets.add(Cuboid::new(
            config.vehicle_half_size.x * 2.0,
            config.vehicle_half_size.y * 2.0,
            config.vehicle_half_size.z * 2.0,
        ))),
        MeshMaterial3d(material_assets.add(StandardMaterial {
            // TODO(lucasw) add uv coordinates to make this repeat
            base_color_texture: Some(images.add(uv_debug_texture())),
            ..default()
        })),
        VehicleData,
    ));

    // TODO(lucasw) need to handle wheels in aggregate
    let wheel_radius = 0.3;
    let wheel_half_height = 0.3;
    commands.spawn((
        Mesh3d(mesh_assets.add(Cylinder::new(wheel_radius, wheel_half_height))),
        MeshMaterial3d(material_assets.add(StandardMaterial {
            // TODO(lucasw) add uv coordinates to make this repeat
            base_color_texture: Some(images.add(uv_debug_texture())),
            ..default()
        })),
        WheelTransform,
    ));

    commands.spawn((
        Mesh3d(mesh_assets.add(Cylinder::new(wheel_radius, wheel_half_height))),
        MeshMaterial3d(material_assets.add(StandardMaterial {
            // TODO(lucasw) add uv coordinates to make this repeat
            base_color_texture: Some(images.add(uv_debug_texture())),
            ..default()
        })),
        WheelTransform,
    ));

    commands.spawn((
        Mesh3d(mesh_assets.add(Cylinder::new(wheel_radius, wheel_half_height))),
        MeshMaterial3d(material_assets.add(StandardMaterial {
            // TODO(lucasw) add uv coordinates to make this repeat
            base_color_texture: Some(images.add(uv_debug_texture())),
            ..default()
        })),
        WheelTransform,
    ));

    commands.spawn((
        Mesh3d(mesh_assets.add(Cylinder::new(wheel_radius, wheel_half_height))),
        MeshMaterial3d(material_assets.add(StandardMaterial {
            // TODO(lucasw) add uv coordinates to make this repeat
            base_color_texture: Some(images.add(uv_debug_texture())),
            ..default()
        })),
        WheelTransform,
    ));
}

/*
fn camera_update(camera: Single<&Camera, With<RenderCamera>>, images: Res<Assets<Image>>) {
    // TODO(lucasw) does this need to be PostUpdate, or it doesn't matter it'll just get the last
    // frame?
    let camera = camera.into_inner();
    let images = images.into_inner();
    // let image = camera.target.into_inner();
    match &camera.target {
        RenderTarget::Image(render_image) => {
            let Some(image) = images.get(render_image) else {
                return;
            };
            // CommandEncoder::copy_texture_to_buffer();
            println!("camera image: {:?}", image);
        }
        _ => {
            println!("should be an Image");
        }
    }
}
*/

fn user_camera_update(
    key_input: Res<ButtonInput<KeyCode>>,
    mut user_camera: Single<&mut UserCamera>,
    mut camera_global_transform: Single<&mut Transform, With<UserCamera>>,
    // vehicle_transform: Single<Transform, With<VehicleData>>,
    vehicle_transform: Single<&Transform, (With<VehicleData>, Without<UserCamera>)>,
) {
    let camera_transform = &mut user_camera.relative_transform;
    let forward = camera_transform.forward();
    let back = camera_transform.back();
    let left = camera_transform.left();
    let right = camera_transform.right();
    let up = camera_transform.up();
    let down = camera_transform.down();

    let dx = 0.02525;
    let mut changed = false;
    if key_input.pressed(KeyCode::KeyW) {
        camera_transform.translation += forward * dx;
        changed = true;
    }
    if key_input.pressed(KeyCode::KeyS) {
        camera_transform.translation += back * dx;
        changed = true;
    }

    if key_input.pressed(KeyCode::KeyA) {
        camera_transform.translation += left * dx;
        changed = true;
    }
    if key_input.pressed(KeyCode::KeyD) {
        camera_transform.translation += right * dx;
        changed = true;
    }

    if key_input.pressed(KeyCode::KeyQ) {
        camera_transform.translation += up * dx;
        changed = true;
    }
    if key_input.pressed(KeyCode::KeyZ) {
        camera_transform.translation += down * dx * 0.8;
        changed = true;
    }
    let rot = 0.012;
    if key_input.pressed(KeyCode::ArrowLeft) {
        camera_transform.rotate_z(rot * 2.0);
        changed = true;
    }
    if key_input.pressed(KeyCode::ArrowRight) {
        camera_transform.rotate_z(-rot * 2.0);
        changed = true;
    }

    if key_input.pressed(KeyCode::ArrowUp) {
        camera_transform.rotate_local_x(rot);
        changed = true;
    }
    if key_input.pressed(KeyCode::ArrowDown) {
        camera_transform.rotate_local_x(-rot);
        changed = true;
    }

    if changed {
        debug!("{:?}", camera_transform.translation);
    }

    camera_global_transform.translation =
        vehicle_transform.translation + camera_transform.translation;
    camera_global_transform.rotation = camera_transform.rotation;
}

fn vehicle_control_update(
    key_input: Res<ButtonInput<KeyCode>>,
    sender: Res<VehicleControlsSender>,
) {
    let mut controls = ffi::CControls {
        forward: 0.0,
        right: 0.0,
    };
    let mut changed = false;
    if key_input.pressed(KeyCode::KeyI) {
        controls.forward = 0.06;
        // control.forward += 0.01;
        changed = true;
    }
    if key_input.pressed(KeyCode::KeyK) {
        controls.forward = -0.04;
        // control.forward += 0.01;
        changed = true;
    }
    if key_input.pressed(KeyCode::KeyL) {
        controls.right = 0.4;
        // control.forward += 0.01;
        changed = true;
    }
    if key_input.pressed(KeyCode::KeyJ) {
        controls.right = -0.4;
        // control.forward += 0.01;
        changed = true;
    }

    // TODO(lucasw) keep track of last sent controls somewhere and blend the new ones with them
    if changed {
        let _ = sender.send(controls);
    }
}

fn cvec3_to_bevy(pos: &ffi::CVec3) -> Vec3 {
    Vec3::new(pos.x, pos.y, pos.z)
}

fn cquat_to_bevy(q: &ffi::CQuat) -> Quat {
    Quat::from_xyzw(q.x, q.y, q.z, q.w)
}

fn sim_update(world: &mut World) {
    // println!("update sim system");

    // TODO(lucasw) add another crossbeam sender receiver pair to communicate vehicle inputs
    // from the keyboard
    let mut controls = ffi::CControls {
        forward: 0.0,
        right: 0.0,
    };
    let receiver = &mut world.get_resource_mut::<VehicleControlsReceiver>().unwrap();
    while let Ok(new_controls) = receiver.try_recv() {
        controls = new_controls;
    }
    let (car_tfs, ray_results) = {
        let sim_system: &mut cxx::UniquePtr<SimSystem> = &mut world
            .get_non_send_resource_mut::<cxx::UniquePtr<SimSystem>>()
            .unwrap();
        let car_tfs = sim_system.as_mut().unwrap().update(controls);

        // TODO(lucasw) do this once and store it somewhere
        let (ray_cast_config, _ray_cast_line_strips) = jolt_rust_cpp::make_ray_casts();
        let car_ray_cast_config = jolt_rust_cpp::transform_ray_cast_config(
            &ray_cast_config,
            &car_tfs.body.pos,
            &car_tfs.body.quat,
        );

        let ray_results = sim_system.as_mut().unwrap().get_rays(car_ray_cast_config);
        (car_tfs, ray_results)
    };

    // println!("{:?}", car_tfs.body.pos);
    // TODO(lucasw) use a crossbeam sender to send the vehicle position to an
    // update that can access the vehicle transform
    let sender = &mut world.get_resource_mut::<VehicleDataSender>().unwrap();
    let _ = sender.send((car_tfs, ray_results));
}

fn vehicle_viz_update(
    receiver: Res<VehicleDataReceiver>,
    mut vehicle_transform: Single<&mut Transform, (With<VehicleData>, Without<WheelTransform>)>,
    mut wheel_transforms: Query<
        &mut Transform,
        (
            With<WheelTransform>,
            Without<VehicleData>,
            Without<PointCloud>,
        ),
    >,
    mut point_transforms: Query<
        &mut Transform,
        (
            With<PointCloud>,
            Without<VehicleData>,
            Without<WheelTransform>,
        ),
    >,
) {
    while let Ok((car_tfs, ray_results)) = receiver.try_recv() {
        vehicle_transform.translation = cvec3_to_bevy(&car_tfs.body.pos);
        vehicle_transform.rotation = cquat_to_bevy(&car_tfs.body.quat);

        let wheels_ctf = [
            car_tfs.wheel_fl.tf,
            car_tfs.wheel_fr.tf,
            car_tfs.wheel_bl.tf,
            car_tfs.wheel_br.tf,
        ];
        for (wheel_ctf, mut wheel) in std::iter::zip(wheels_ctf.iter(), &mut wheel_transforms) {
            wheel.translation = cvec3_to_bevy(&wheel_ctf.pos);
            wheel.rotation = cquat_to_bevy(&wheel_ctf.quat);
        }

        // Not sure how efficient this is
        for (ray_point, mut point_transform) in
            std::iter::zip(ray_results.directions, &mut point_transforms)
        {
            point_transform.translation = cvec3_to_bevy(&ray_point);
        }
    }
}

fn main() -> Result<(), anyhow::Error> {
    println!("jolt_bevy");

    let perlin = Perlin::new(1);
    let cell_size = 2.0;
    let (terrain, vertices, triangles, _colors) = make_terrain(cell_size, perlin);
    let config = Config {
        terrain,
        vertices,
        triangles,
        vehicle_half_size: Vec3::new(3.0, 1.0, 0.5),
    };

    App::new()
        .add_plugins((
            DefaultPlugins.set(WindowPlugin {
                primary_window: Some(Window {
                    present_mode: PresentMode::AutoNoVsync,
                    resolution: WindowResolution::new(1280, 720).with_scale_factor_override(1.0),
                    ..default()
                }),
                ..default()
            }),
            // FrameTimeDiagnosticsPlugin,
            // LogDiagnosticsPlugin::default(),
        ))
        .insert_resource(config)
        .add_systems(Startup, (sim_setup, viz_setup))
        .add_systems(
            Update,
            (
                user_camera_update,
                vehicle_control_update,
                sim_update,
                vehicle_viz_update,
            ),
        )
        .run();

    Ok(())
}
