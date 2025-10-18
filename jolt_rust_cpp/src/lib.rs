use noise::{NoiseFn, Perlin};

// include!(concat!(env!("OUT_DIR"), "/constants.rs"));
const NUM: usize = 196;
const NUM2: usize = NUM * NUM;
const NUM_RAYS: usize = 1000;

#[cxx::bridge(namespace = "jolt_rust_cpp")]
pub mod ffi {
    #[derive(Clone, Debug, Default)]
    struct CVec3 {
        x: f32,
        y: f32,
        z: f32,
    }

    #[derive(Clone, Debug)]
    struct CQuat {
        x: f32,
        y: f32,
        z: f32,
        w: f32,
    }

    #[derive(Clone, Debug)]
    struct CTf {
        pos: CVec3,
        quat: CQuat,
        linear_vel: CVec3,
        angular_vel: CVec3,
    }

    #[derive(Clone, Debug)]
    struct Wheel {
        tf: CTf,
        angular_velocity: f32,
        rotation_angle: f32,
        steer_angle: f32,
    }

    // TODO(lucasw) store the car chassis and wheel dimensions in this also, or in a separate
    // struct?
    #[derive(Clone)]
    struct CarTfs {
        body: CTf,
        wheel_fl: Wheel,
        wheel_fr: Wheel,
        wheel_bl: Wheel,
        wheel_br: Wheel,
    }

    #[derive(Clone)]
    struct CControls {
        forward: f32,
        right: f32,
    }

    // TODO(lucasw) this doesn't work because the cxx bridge macro happens before
    // the include macro "error[cxxbridge]: unsupported item"
    // include!(concat!(env!("OUT_DIR"), "/array_structs.rs"));
    // TODO(lucasw) have to maintain these hard coded numbers manually
    #[derive(Clone)]
    struct CRayCastConfig {
        // TODO(lucasw) hard-coded to vehicle reference frame for now
        offset: CVec3,
        directions: [CVec3; 1000],
        num_rays: usize,
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

    unsafe extern "C++" {
        include!("jolt_rust_cpp/src/misc.h");
        type SimSystem;
        fn new_sim_system(
            max_num_bodies: u32,
            vehicle_position: CVec3,
            vehicle_half_size: CVec3,
            terrain: CTerrain,
        ) -> UniquePtr<SimSystem>;
        // fn init(max_num_bodies: u32) -> i64;
        fn update(self: Pin<&mut SimSystem>, control: CControls) -> CarTfs;
        fn get_rays(&self, ray_cast_config: CRayCastConfig) -> CRayCastConfig;
        fn close(self: Pin<&mut SimSystem>);
    }
}

pub fn ray_cast_config_default() -> ffi::CRayCastConfig {
    ffi::CRayCastConfig {
        offset: ffi::CVec3::default(),
        directions: [(); NUM_RAYS].map(|_| ffi::CVec3::default()),
        num_rays: NUM_RAYS,
    }
}

pub fn cvec3_to_glam(vec: &ffi::CVec3) -> glam::Vec3 {
    glam::Vec3::new(vec.x, vec.y, vec.z)
}

pub fn glam_to_cvec3(vec: &glam::Vec3) -> ffi::CVec3 {
    ffi::CVec3 {
        x: vec.x,
        y: vec.y,
        z: vec.z,
    }
}

pub fn cquat_to_glam(quat: &ffi::CQuat) -> glam::Quat {
    glam::Quat::from_xyzw(quat.x, quat.y, quat.z, quat.w)
}

// cylinder conventions in jolt and rerun differ
pub fn cylinder_cquat_to_rerun(quat: ffi::CQuat) -> glam::Quat {
    let rot = glam::Quat::from_euler(glam::EulerRot::XYZ, std::f32::consts::FRAC_PI_2, 0.0, 0.0);

    glam::Quat::from_xyzw(quat.x, quat.y, quat.z, quat.w) * rot
}

pub fn make_terrain(
    cell_size: f64,
    perlin: Perlin,
) -> (
    ffi::CTerrain,
    Vec<[f64; 3]>,
    Vec<[u32; 3]>,
    Vec<(u8, u8, u8)>,
) {
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
            colors.push((r, g, b));
            // colors.push(rerun::Color::from_rgb(r, g, b));
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
    (terrain, vertices, triangles, colors)
}

pub fn make_ray_casts() -> (ffi::CRayCastConfig, Vec<[[f32; 3]; 2]>) {
    let max_range = 30.0;
    let mut line_strips = Vec::new();

    let mut ray_cast_config = ray_cast_config_default();
    let x0 = 2.5;
    let y0 = 0.0;
    let z0 = 2.0;
    ray_cast_config.offset = ffi::CVec3 {
        x: x0,
        y: y0,
        z: z0,
    };
    let num_rays = ray_cast_config.directions.len();
    let num_azimuth = num_rays / 20;
    let num_layers = num_rays / num_azimuth;
    let mut ind = 0;
    for layer in 0..num_layers {
        let fr_layer = 0.125 + (layer as f32 / num_layers as f32) * 0.75;
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
    // println!("ray {:?}", ray_cast_config.directions[600]);

    (ray_cast_config, line_strips)
}

// rotate the ray casts into the vehicle frame
pub fn transform_ray_cast_config(
    ray_cast_config: &ffi::CRayCastConfig,
    pos: &ffi::CVec3,
    quat: &ffi::CQuat,
) -> ffi::CRayCastConfig {
    let mut car_ray_cast_config = ray_cast_config.clone();

    let scale = glam::Vec3::new(1.0, 1.0, 1.0);
    let rotation = cquat_to_glam(&quat);
    let translation = cvec3_to_glam(&pos);
    let affine = glam::f32::Affine3A::from_scale_rotation_translation(scale, rotation, translation);

    let origin = cvec3_to_glam(&ray_cast_config.offset);
    let world_origin = affine.transform_point3(origin);

    car_ray_cast_config.offset = glam_to_cvec3(&world_origin);

    for cdir in car_ray_cast_config.directions.iter_mut() {
        let dir = cvec3_to_glam(&cdir);
        let car_dir = rotation.mul_vec3(dir);
        *cdir = glam_to_cvec3(&car_dir);
    }

    car_ray_cast_config
}
