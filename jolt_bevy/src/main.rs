//! Lucas Walter
//! Visualize a vehicle moving around in a jolt physics environment in bevy

use jolt_rust_cpp::*;
//use rerun::external::glam;
use noise::{NoiseFn, Perlin};

use bevy::{
    color::palettes::css,
    prelude::*,
    render::{
        Render, RenderSet,
        camera::RenderTarget,
        render_asset::RenderAssetUsages,
        render_resource::CommandEncoder,
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
struct UserCamera;

#[derive(Component)]
struct Vehicle;

#[derive(Resource)]
struct Config {
    vehicle_half_size: Vec3,
}

/// This will receive asynchronously any data sent from the render world
#[derive(Resource, Deref)]
struct VehicleReceiver(Receiver<ffi::CTf>);

/// This will send asynchronously any data to the main world
#[derive(Resource, Deref)]
struct VehicleSender(Sender<ffi::CTf>);

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

fn sim_setup(
    world: &mut World,
    /*
    mut commands: Commands,
    mut mesh_assets: ResMut<Assets<Mesh>>,
    mut images: ResMut<Assets<Image>>,
    mut material_assets: ResMut<Assets<StandardMaterial>>,
    */
) {
    println!("setup simulation");
    let perlin = Perlin::new(1);

    let cell_size = 1.0;
    let (terrain, _vertices, _triangles, _colors) = make_terrain(cell_size, perlin);

    let config = world.get_resource::<Config>().unwrap();

    let vehicle_half_size = ffi::CVec3 {
        x: config.vehicle_half_size.x,
        y: config.vehicle_half_size.y,
        z: config.vehicle_half_size.z,
    };
    let sim_system = ffi::new_sim_system(8000, vehicle_half_size.clone(), terrain);
    world.insert_non_send_resource(sim_system);

    let (sender, receiver) = crossbeam_channel::unbounded();
    world.insert_resource(VehicleSender(sender));
    world.insert_resource(VehicleReceiver(receiver));

    println!("setup simulation done");
}

fn setup(
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
        Transform::from_xyz(0.0, 0.0, -ground_height),
    ));
    println!("spawned ground: {}", ground.id());

    let _camera = commands.spawn((
        Camera3d::default(),
        UserCamera,
        Transform::from_xyz(-10.0, 0.0, 5.0).looking_at(Vec3::new(-5.0, 0.0, 3.0), Vec3::Z),
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
        Vehicle,
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
    mut camera_transform: Single<&mut Transform, With<UserCamera>>,
) {
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
        println!("{:?}", camera_transform.translation);
    }
}

fn cvec3_to_bevy(pos: &ffi::CVec3) -> Vec3 {
    Vec3::new(pos.x, pos.y, pos.z)
}

fn cquat_to_bevy(q: &ffi::CQuat) -> Quat {
    Quat::from_xyzw(q.x, q.y, q.z, q.w)
}

fn sim_update(world: &mut World) {
    let sim_system: &mut cxx::UniquePtr<SimSystem> = &mut world
        .get_non_send_resource_mut::<cxx::UniquePtr<SimSystem>>()
        .unwrap();
    // println!("update sim system");

    // TODO(lucasw) add another crossbeam sender receiver pair to communicate vehicle inputs
    // from the keyboard
    let control = ffi::CControls {
        forward: 0.052,
        right: 0.1,
    };
    let car_tfs = sim_system.as_mut().unwrap().update(control);
    // println!("{:?}", car_tfs.body.pos);
    // TODO(lucasw) use a crossbeam sender to send the vehicle position to an
    // update that can access the vehicle transform
    let mut sender = &mut world.get_resource_mut::<VehicleSender>().unwrap();
    let _ = sender.send(car_tfs.body);
}

fn vehicle_update(
    receiver: Res<VehicleReceiver>,
    mut vehicle_transform: Single<&mut Transform, With<Vehicle>>,
) {
    while let Ok(vehicle_tf) = receiver.try_recv() {
        vehicle_transform.translation = cvec3_to_bevy(&vehicle_tf.pos);
        vehicle_transform.rotation = cquat_to_bevy(&vehicle_tf.quat);
    }
}

fn main() -> Result<(), anyhow::Error> {
    println!("jolt_bevy");

    App::new()
        .add_plugins((
            DefaultPlugins.set(WindowPlugin {
                primary_window: Some(Window {
                    present_mode: PresentMode::AutoNoVsync,
                    resolution: WindowResolution::new(1280.0, 720.0)
                        .with_scale_factor_override(1.0),
                    ..default()
                }),
                ..default()
            }),
            // FrameTimeDiagnosticsPlugin,
            // LogDiagnosticsPlugin::default(),
        ))
        .insert_resource(Config {
            vehicle_half_size: Vec3::new(3.0, 1.0, 0.5),
        })
        .add_systems(Startup, (setup, sim_setup))
        .add_systems(Update, (user_camera_update, sim_update, vehicle_update))
        .run();

    Ok(())
}
