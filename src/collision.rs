use cgmath::prelude::InnerSpace;
use wgpu::util::DeviceExt;
use cgmath::Rotation;
use cgmath::Transform;

use crate::model;
use crate::model::DrawModel;
use crate::uniform;
use crate::level;

pub const EPSILON: f32 = 0.0001;

//https://wickedengine.net/2020/04/26/capsule-collision-detection/

#[derive(Debug)]
pub struct CollisionInfo {
    pub happen: bool,
    pub normal: cgmath::Vector3<f32>,
    pub penetration: f32,
    pub priority: i32,
}

pub fn sphere_mesh_one_sweep(model: &model::Model, sphere: &mut Sphere, velocity: &cgmath::Vector3<f32>, ignore_vel: bool, transform: Option<cgmath::Matrix4<f32>>) -> (bool, cgmath::Vector3<f32>) {

    let mut did_collide = false;

    let verts = model.verts.as_ref().unwrap();
    let inds = model.inds.as_ref().unwrap();

    let mut priority_time = 0;

    let mut min_penetration = f32::MAX;
    let mut resolve_normal = cgmath::Vector3::new(0.0, 1.0, 0.0);
    let mut col_happened = false;
    let mut max_priority = 0;

    for i in (0..inds.len()).step_by(3) {

        let mut p0: cgmath::Vector3<f32> = verts[inds[i] as usize].into();
        let mut p1: cgmath::Vector3<f32> = verts[inds[i + 1] as usize].into();
        let mut p2: cgmath::Vector3<f32> = verts[inds[i + 2] as usize].into();

        if transform.is_some() {
            let m = transform.unwrap();

            let mut p_temp = m * cgmath::Vector4::new(p0.x, p0.y, p0.z, 1.0);
            p0 = cgmath::Vector3::new(p_temp.x / p_temp.w, p_temp.y / p_temp.w, p_temp.z / p_temp.w);

            p_temp = m * cgmath::Vector4::new(p1.x, p1.y, p1.z, 1.0);
            p1 = cgmath::Vector3::new(p_temp.x / p_temp.w, p_temp.y / p_temp.w, p_temp.z / p_temp.w);

            p_temp = m * cgmath::Vector4::new(p2.x, p2.y, p2.z, 1.0);
            p2 = cgmath::Vector3::new(p_temp.x / p_temp.w, p_temp.y / p_temp.w, p_temp.z / p_temp.w);
        }

        let n = ((p1 - p0).cross(p2 - p0)).normalize();

        if cgmath::dot(n, *velocity) < 0.0 || ignore_vel {
            let col_info = sphere.vs_triangle(&p0, &p1, &p2);
            if col_info.happen {
                col_happened = true;
                did_collide = true;
                if (min_penetration > col_info.penetration && col_info.priority >= max_priority) || col_info.priority > max_priority /*|| (priority_time > 0 && min_penetration > col_info.penetration)*/ {
                    min_penetration = col_info.penetration;
                    resolve_normal = col_info.normal;
                    max_priority = col_info.priority;
                }
            }
        }
    }

    let mut offset_vec = cgmath::Vector3::new(0.0, 0.0, 0.0);

    if col_happened {

        offset_vec = resolve_normal * (min_penetration + EPSILON);
        sphere.pos += offset_vec;
        if max_priority > 0 {
            priority_time = 1;
        }
    }

    (col_happened, offset_vec)
}

pub fn clean_collide_triangle_mesh_sphere(model: &model::Model, sphere: &mut Sphere, velocity: &cgmath::Vector3<f32>, ignore_vel: bool, transform: Option<cgmath::Matrix4<f32>>) -> bool {

    let mut did_collide = false;

    if model.verts.is_some() {

        let verts = model.verts.as_ref().unwrap();
        let inds = model.inds.as_ref().unwrap();

        let mut priority_time = 0;

        for i in 0..4 {

            let mut min_penetration = f32::MAX;
            let mut resolve_normal = cgmath::Vector3::new(0.0, 1.0, 0.0);
            let mut col_happened = false;
            let mut max_priority = 0;

            for i in (0..inds.len()).step_by(3) {

                let mut p0: cgmath::Vector3<f32> = verts[inds[i] as usize].into();
                let mut p1: cgmath::Vector3<f32> = verts[inds[i + 1] as usize].into();
                let mut p2: cgmath::Vector3<f32> = verts[inds[i + 2] as usize].into();

                if transform.is_some() {
                    let m = transform.unwrap();

                    let mut p_temp = m * cgmath::Vector4::new(p0.x, p0.y, p0.z, 1.0);
                    p0 = cgmath::Vector3::new(p_temp.x / p_temp.w, p_temp.y / p_temp.w, p_temp.z / p_temp.w);

                    p_temp = m * cgmath::Vector4::new(p1.x, p1.y, p1.z, 1.0);
                    p1 = cgmath::Vector3::new(p_temp.x / p_temp.w, p_temp.y / p_temp.w, p_temp.z / p_temp.w);

                    p_temp = m * cgmath::Vector4::new(p2.x, p2.y, p2.z, 1.0);
                    p2 = cgmath::Vector3::new(p_temp.x / p_temp.w, p_temp.y / p_temp.w, p_temp.z / p_temp.w);
                }

                let n = ((p1 - p0).cross(p2 - p0)).normalize();

                if cgmath::dot(n, *velocity) < 0.0 || ignore_vel {
                    let col_info = sphere.vs_triangle(&p0, &p1, &p2);
                    if col_info.happen {
                        col_happened = true;
                        did_collide = true;
                        if (min_penetration > col_info.penetration && col_info.priority >= max_priority) || col_info.priority > max_priority /*|| (priority_time > 0 && min_penetration > col_info.penetration)*/ {
                            min_penetration = col_info.penetration;
                            resolve_normal = col_info.normal;
                            max_priority = col_info.priority;
                        }
                    }
                }
            }

            if col_happened {

                let offset_vec = resolve_normal * (min_penetration + EPSILON);
                sphere.pos += offset_vec;
                if max_priority > 0 {
                    priority_time = 1;
                }
            }
            else {
                break;
            }
        }
    }

    did_collide
}

pub fn clean_collide_triangle_mesh_moving(mesh: &level::CollisionMesh, capsule: &mut Capsule, velocity: &cgmath::Vector3<f32>) {

    if mesh.object.verts.is_some() {

        let verts = mesh.object.verts.as_ref().unwrap();
        let inds = mesh.object.inds.as_ref().unwrap();

        //Mesh transform
        let p_t = cgmath::Matrix4::from_translation(mesh.position);
        let p_r: cgmath::Quaternion<f32> = cgmath::Rotation3::from_angle_y(cgmath::Rad(mesh.angle));
        let p_rm: cgmath::Matrix4<f32> = p_r.into();
        let p_srt = p_t * p_rm;

        //Player to mesh space transform
        let p_temp = p_srt * cgmath::Vector4::new(0.0, 0.0, 0.0, 1.0);
        let p_final = cgmath::Vector3::new(p_temp.x / p_temp.w, p_temp.y / p_temp.w, p_temp.z / p_temp.w);

        if clean_collide_triangle_mesh(&mesh.object, capsule, &(velocity - mesh.velocity), true, Some(p_srt)) {
            capsule.change_position_vec(-p_final);
            let mid = (capsule.base + capsule.tip) / 2.0;
            let result = mid + mesh.velocity;
            let t = cgmath::Matrix4::from_translation(result);
            let r: cgmath::Quaternion<f32> = cgmath::Rotation3::from_angle_y(cgmath::Rad(mesh.speed));
            let rm: cgmath::Matrix4<f32> = r.into();
            let srt = rm * t;
            let t_temp = srt * cgmath::Vector4::new(0.0, 0.0, 0.0, 1.0);
            let t_final = cgmath::Vector3::new(t_temp.x / t_temp.w, t_temp.y / t_temp.w, t_temp.z / t_temp.w);
            capsule.set_position_vec(t_final);
            capsule.change_position_vec(p_final);
        }
    }
}

pub fn clean_collide_triangle_mesh(model: &model::Model, capsule: &mut Capsule, velocity: &cgmath::Vector3<f32>, ignore_vel: bool, transform: Option<cgmath::Matrix4<f32>>) -> bool {

    let mut did_collide = false;

    if model.verts.is_some() {

        let verts = model.verts.as_ref().unwrap();
        let inds = model.inds.as_ref().unwrap();

        let mut priority_time = 0;

        for i in 0..4 {

            let mut min_penetration = f32::MAX;
            let mut resolve_normal = cgmath::Vector3::new(0.0, 1.0, 0.0);
            let mut col_happened = false;
            let mut max_priority = 0;

            for i in (0..inds.len()).step_by(3) {

                let mut p0: cgmath::Vector3<f32> = verts[inds[i] as usize].into();
                let mut p1: cgmath::Vector3<f32> = verts[inds[i + 1] as usize].into();
                let mut p2: cgmath::Vector3<f32> = verts[inds[i + 2] as usize].into();

                if transform.is_some() {
                    let m = transform.unwrap();

                    let mut p_temp = m * cgmath::Vector4::new(p0.x, p0.y, p0.z, 1.0);
                    p0 = cgmath::Vector3::new(p_temp.x / p_temp.w, p_temp.y / p_temp.w, p_temp.z / p_temp.w);

                    p_temp = m * cgmath::Vector4::new(p1.x, p1.y, p1.z, 1.0);
                    p1 = cgmath::Vector3::new(p_temp.x / p_temp.w, p_temp.y / p_temp.w, p_temp.z / p_temp.w);

                    p_temp = m * cgmath::Vector4::new(p2.x, p2.y, p2.z, 1.0);
                    p2 = cgmath::Vector3::new(p_temp.x / p_temp.w, p_temp.y / p_temp.w, p_temp.z / p_temp.w);
                }

                let n = ((p1 - p0).cross(p2 - p0)).normalize();

                if cgmath::dot(n, *velocity) < 0.0 || ignore_vel {
                    let col_info = capsule.vs_triangle(&p0, &p1, &p2, &n);
                    if col_info.happen {
                        col_happened = true;
                        did_collide = true;
                        if (min_penetration > col_info.penetration && col_info.priority >= max_priority) || col_info.priority > max_priority /*|| (priority_time > 0 && min_penetration > col_info.penetration)*/ {
                            min_penetration = col_info.penetration;
                            resolve_normal = col_info.normal;
                            max_priority = col_info.priority;
                        }
                    }
                }
            }

            if col_happened {

                let offset_vec = resolve_normal * (min_penetration + EPSILON);
                capsule.change_position_vec(offset_vec);
                if max_priority > 0 {
                    priority_time = 1;
                }
            }
            else {
                break;
            }
        }
    }

    did_collide
}

pub fn ray_vs_level(model: &model::Model, orig: &cgmath::Vector3<f32>, dir: &cgmath::Vector3<f32>, transform: Option<cgmath::Matrix4<f32>>) {

    let mut did_collide = false;

    if model.verts.is_some() {

        let verts = model.verts.as_ref().unwrap();
        let inds = model.inds.as_ref().unwrap();

        let mut priority_time = 0;

        for i in (0..inds.len()).step_by(3) {

            let mut p0: cgmath::Vector3<f32> = verts[inds[i] as usize].into();
            let mut p1: cgmath::Vector3<f32> = verts[inds[i + 1] as usize].into();
            let mut p2: cgmath::Vector3<f32> = verts[inds[i + 2] as usize].into();

            if transform.is_some() {
                let m = transform.unwrap();
    
                let mut p_temp = m * cgmath::Vector4::new(p0.x, p0.y, p0.z, 1.0);
                p0 = cgmath::Vector3::new(p_temp.x / p_temp.w, p_temp.y / p_temp.w, p_temp.z / p_temp.w);
    
                p_temp = m * cgmath::Vector4::new(p1.x, p1.y, p1.z, 1.0);
                p1 = cgmath::Vector3::new(p_temp.x / p_temp.w, p_temp.y / p_temp.w, p_temp.z / p_temp.w);
    
                p_temp = m * cgmath::Vector4::new(p2.x, p2.y, p2.z, 1.0);
                p2 = cgmath::Vector3::new(p_temp.x / p_temp.w, p_temp.y / p_temp.w, p_temp.z / p_temp.w);
            }
    
            let n = ((p1 - p0).cross(p2 - p0)).normalize();

            let (c, t) = ray_triangle(orig, dir, &p0, &p1, &p2);

            did_collide |= c;

            if c {
                println!("{}", t);
            }
        }
    }
}

pub fn ray_triangle(orig: &cgmath::Vector3<f32>, dir: &cgmath::Vector3<f32>, v0: &cgmath::Vector3<f32>, v1: &cgmath::Vector3<f32>, v2: &cgmath::Vector3<f32>) -> (bool, f32) {

    let v0v1 = v1 - v0;
    let v0v2 = v2 - v0;

    let pvec = dir.cross(v0v2);
    let det = cgmath::dot(v0v1, pvec);

    /*if det < EPSILON {

        return (false, 1.0);
    }*/

    if det.abs() < EPSILON {
        return (false, 1.0);
    }

    let inv_det = 1.0 / det;

    let tvec = orig - v0;
    let u = cgmath::dot(tvec, pvec) * inv_det;
    if u < 0.0 || u > 1.0 {
        return (false, 1.0);
    }

    let qvec = tvec.cross(v0v1);
    let v = cgmath::dot(*dir, qvec) * inv_det;
    if v < 0.0 || u + v > 1.0 {
        return (false, 1.0);
    }

    let t = cgmath::dot(v0v2, qvec) * inv_det;

    return (true, t);
}

pub fn ray_plane(orig: &cgmath::Vector3<f32>, dir: &cgmath::Vector3<f32>, c: &cgmath::Vector3<f32>, n: &cgmath::Vector3<f32>) -> (bool, f32) {

    let denom = cgmath::dot(*n, *dir);
    if denom.abs() > EPSILON {
        let t = cgmath::dot(c - orig, *n) / denom;
        if t >= 0.0 {
            return (true, t)
        }
    }

    return (false, 0.0);
}

pub struct Sphere {
    pub pos: cgmath::Vector3<f32>,
    pub radius: f32,
}

impl Sphere {

    pub fn new(pos: cgmath::Vector3<f32>, radius: f32) -> Sphere {

        Sphere { pos, radius }
    }

    pub fn vs_sphere(&self, other: &Sphere) -> CollisionInfo {

        let penetration_test = (self.pos - other.pos).magnitude().abs();

        let penetration = (self.radius + other.radius) - penetration_test;

        let happen = penetration_test <= (self.radius + other.radius).abs();

        let normal = (self.pos - other.pos).normalize();

        CollisionInfo { happen, normal, penetration, priority: 0 }
    }

    pub fn vs_triangle(&self, p0: &cgmath::Vector3<f32>, p1: &cgmath::Vector3<f32>, p2: &cgmath::Vector3<f32>) -> CollisionInfo {

        let normal = ((p1 - p0).cross(p2 - p0)).normalize();
        let distance = cgmath::dot(self.pos - p0, normal);
        if distance < -self.radius || distance > self.radius {
            return CollisionInfo { happen: false, normal, penetration: 1.0, priority: 0 };
        }

        let point0 = self.pos - normal * distance;

        let c0 = (point0 - p0).cross(p1 - p0);
        let c1 = (point0 - p1).cross(p2 - p1);
        let c2 = (point0 - p2).cross(p0 - p2);

        let inside = cgmath::dot(c0, normal) <= 0.0 && cgmath::dot(c1, normal) <= 0.0 && cgmath::dot(c2, normal) <= 0.0;

        let sq_radius = self.radius * self.radius;

        let point1 = closest_point_on_line(&p0, &p1, &self.pos);
        let v1 = self.pos - point1;
        let sq_dist1 = cgmath::dot(v1, v1);
        let mut intersects = sq_dist1 < sq_radius;

        let point2 = closest_point_on_line(&p1, &p2, &self.pos);
        let v2 = self.pos - point2;
        let sq_dist2 = cgmath::dot(v2, v2);
        intersects |= sq_dist2 < sq_radius;

        let point3 = closest_point_on_line(&p2, &p0, &self.pos);
        let v3 = self.pos - point3;
        let sq_dist3 = cgmath::dot(v3, v3);
        intersects |= sq_dist3 < sq_radius;

        let mut out_normal = cgmath::Vector3::new(0.0, 0.0, 0.0);
        let mut penetration = 1.0;

        let mut priority = 0;

        if intersects || inside {

            let mut best_point = point0;
            
            let mut intersection_vec = cgmath::Vector3::new(0.0, 0.0, 0.0);

            if inside {
                intersection_vec = self.pos - point0;
                priority = 1;
            }
            else {

                let mut d = self.pos - point1;
                let mut sq_best_dist = cgmath::dot(d, d);
                best_point = point1;
                intersection_vec = d;

                d = self.pos - point2;
                let mut sq_dist = cgmath::dot(d, d);
                if sq_dist < sq_best_dist {
                    sq_best_dist = sq_dist;
                    best_point = point2;
                    intersection_vec = d;
                }

                d = self.pos - point3;
                sq_dist = cgmath::dot(d, d);
                if sq_dist < sq_best_dist {
                    sq_best_dist = sq_dist;
                    best_point = point3;
                    intersection_vec = d;
                }
            }

            let len = intersection_vec.magnitude();
            out_normal = intersection_vec.normalize();
            penetration = self.radius - len;
        }

        CollisionInfo { happen: intersects || inside, normal: out_normal, penetration, priority }
    }
}

#[derive(Debug, Copy, Clone)]
pub struct Capsule {
    pub base: cgmath::Vector3<f32>,
    pub tip: cgmath::Vector3<f32>,
    pub radius: f32,
}

impl Capsule {

    pub fn new(base: cgmath::Vector3<f32>, tip: cgmath::Vector3<f32>, radius: f32) -> Capsule {

        Capsule { base, tip, radius }
    }

    pub fn change_position_vec(&mut self, vec: cgmath::Vector3<f32>) {

        self.base = self.base + vec;

        self.tip = self.tip + vec;
    }

    pub fn set_position_vec(&mut self, vec: cgmath::Vector3<f32>) {

        let mid = (self.tip + self.base) / 2.0;

        self.base = vec - mid;

        self.tip = vec + mid;
    }

    pub fn change_position(&mut self, x: f32, y: f32, z: f32) {

        self.base.x += x;
        self.base.y += y;
        self.base.z += z;

        self.tip.x += x;
        self.tip.y += y;
        self.tip.z += z;
    }

    pub fn vs_capsule(&self, other: &Capsule) -> CollisionInfo {

        let a_normal = (self.tip - self.base).normalize();
        let a_line_end_offset = a_normal * self.radius;
        let a_a = self.base + a_line_end_offset;
        let a_b = self.tip - a_line_end_offset;

        let b_normal = (other.tip - other.base).normalize();
        let b_line_end_offset = b_normal * other.radius;
        let b_a = other.base + b_line_end_offset;
        let b_b = other.tip - b_line_end_offset;
        
        let v0 = b_a - a_a;
        let v1 = b_b - a_a;
        let v2 = b_a - a_b;
        let v3 = b_b - a_b;

        let d0 = cgmath::dot(v0, v0);
        let d1 = cgmath::dot(v1, v1);
        let d2 = cgmath::dot(v2, v2);
        let d3 = cgmath::dot(v3, v3);

        let mut best_a = cgmath::Vector3::new(0.0, 0.0, 0.0);
        if d2 < d0 || d2 < d1 || d3 < d0 || d3 < d1 {
            best_a = a_b;
        }
        else {
            best_a = a_a;
        }

        let best_b = closest_point_on_line(&b_a, &b_b, &best_a);

        best_a = closest_point_on_line(&a_a, &a_b, &best_b);

        let normal = (best_a - best_b).normalize();
        let len = (best_a - best_b).magnitude();
        let penetration = self.radius + other.radius - len;
        let happen = penetration > 0.0;

        CollisionInfo { happen, normal, penetration, priority: 0 }
    }

    pub fn vs_triangle(&self, p0: &cgmath::Vector3<f32>, p1: &cgmath::Vector3<f32>, p2: &cgmath::Vector3<f32>, n: &cgmath::Vector3<f32>) -> CollisionInfo {
        
        let n = ((p1 - p0).cross(p2 - p0)).normalize();

        let capsule_normal = (self.tip - self.base).normalize();

        let line_end_offset = capsule_normal * self.radius;
        let a = self.base + line_end_offset;
        let b = self.tip - line_end_offset;

        let mut reference_point = p0.clone();

        if cgmath::dot(capsule_normal, n) != 0.0 {

            let t = cgmath::dot(n, (p0 - self.base) / cgmath::dot(n, capsule_normal).abs());
            let line_plane_intersection = self.base + capsule_normal * t;

            let reference_point = closest_point_on_triangle(p0, p1, p2, &n, &line_plane_intersection);
        }

        let mut center = closest_point_on_line(&a, &b, &reference_point);
        
        let test_sphere = Sphere::new(center, self.radius);

        test_sphere.vs_triangle(p0, p1, p2)
    }

    pub fn ground_test(&mut self, model: &model::Model, gravity: f32, transform: Option<cgmath::Matrix4<f32>>) {

        let mut did_collide = false;

        let mut velocity = cgmath::Vector3::new(0.0, gravity, 0.0);

        self.change_position_vec(velocity);

        if model.verts.is_some() {

            let verts = model.verts.as_ref().unwrap();
            let inds = model.inds.as_ref().unwrap();

            let mut priority_time = 0;

            for i in 0..4 {

                let mut min_penetration = f32::MAX;
                let mut resolve_normal = cgmath::Vector3::new(0.0, 1.0, 0.0);
                let mut plane_point = cgmath::Vector3::new(0.0, 0.0, 0.0);
                let mut col_happened = false;
                let mut max_priority = 0;

                for i in (0..inds.len()).step_by(3) {

                    let mut p0: cgmath::Vector3<f32> = verts[inds[i] as usize].into();
                    let mut p1: cgmath::Vector3<f32> = verts[inds[i + 1] as usize].into();
                    let mut p2: cgmath::Vector3<f32> = verts[inds[i + 2] as usize].into();

                    if transform.is_some() {
                        let m = transform.unwrap();

                        let mut p_temp = m * cgmath::Vector4::new(p0.x, p0.y, p0.z, 1.0);
                        p0 = cgmath::Vector3::new(p_temp.x / p_temp.w, p_temp.y / p_temp.w, p_temp.z / p_temp.w);

                        p_temp = m * cgmath::Vector4::new(p1.x, p1.y, p1.z, 1.0);
                        p1 = cgmath::Vector3::new(p_temp.x / p_temp.w, p_temp.y / p_temp.w, p_temp.z / p_temp.w);

                        p_temp = m * cgmath::Vector4::new(p2.x, p2.y, p2.z, 1.0);
                        p2 = cgmath::Vector3::new(p_temp.x / p_temp.w, p_temp.y / p_temp.w, p_temp.z / p_temp.w);
                    }

                    let n = ((p1 - p0).cross(p2 - p0)).normalize();

                    if cgmath::dot(n, velocity) < 0.0 {
                        let col_info = self.vs_triangle(&p0, &p1, &p2, &n);
                        if col_info.happen {
                            col_happened = true;
                            did_collide = true;
                            if (min_penetration > col_info.penetration && col_info.priority >= max_priority) || col_info.priority > max_priority /*|| (priority_time > 0 && min_penetration > col_info.penetration)*/ {
                                min_penetration = col_info.penetration;
                                resolve_normal = col_info.normal;
                                max_priority = col_info.priority;
                                plane_point = p0;
                            }
                        }
                    }
                }

                if col_happened {

                    let offset_vec = resolve_normal * (min_penetration + EPSILON);
                    if resolve_normal.y > 0.7 {

                        let (c, t) = ray_plane(&self.tip, &velocity.normalize(), &plane_point, &resolve_normal);
                        if c {
                            println!("{:?} {} {}", velocity.normalize(), t, self.tip.y - self.base.y);
                            let intersection_point = self.tip + (velocity.normalize()) * (t - EPSILON);
                            //println!("{:?}", intersection_point);
                            let player_point = intersection_point + ((self.tip + self.base) / 2.0);
                            let offset = self.tip - self.base;
                            
                            let val = (velocity.normalize() * (t - EPSILON)) - ((self.tip - self.base));

                            //println!("p {:?}", player_point);
                            self.change_position_vec(val);
                        }
                        else {
                            println!("yo");
                            self.change_position_vec(offset_vec);
                        }
                    }
                    else {
                        println!("Woah");
                        self.change_position_vec(offset_vec);
                    }
                    if max_priority > 0 {
                        priority_time = 1;
                    }

                    velocity = cgmath::Vector3::new(0.0, 0.0, 0.0);
                }
                else {
                    break;
                }
            }
        }
    }
}

pub fn closest_point_on_triangle(p0: &cgmath::Vector3<f32>, p1: &cgmath::Vector3<f32>, p2: &cgmath::Vector3<f32>, n: &cgmath::Vector3<f32>, point:  &cgmath::Vector3<f32>) -> cgmath::Vector3<f32> {

    let c0 = (point - p0).cross(p1 - p0);
    let c1 = (point - p1).cross(p2 - p1);
    let c2 = (point - p2).cross(p0 - p2);
    let inside = cgmath::dot(c0, *n) <= 0.0 && cgmath::dot(c1, *n) <= 0.0 && cgmath::dot(c2, *n) <= 0.0;

    let mut reference_point = point.clone();

    if inside {
        return reference_point;
    }
    else {

        let point1 = closest_point_on_line(p0, p1, point);
        let v1 = point - point1;
        let mut sq_dist = cgmath::dot(v1, v1);
        let mut best_dist = sq_dist;
        reference_point = point1;

        let point2 = closest_point_on_line(p1, p2, point);
        let v2 = point - point2;
        sq_dist = cgmath::dot(v2, v2);
        if sq_dist < best_dist {
            reference_point = point2;
            best_dist = sq_dist;
        }

        let point3 = closest_point_on_line(p2, p0, point);
        let v3 = point - point3;
        sq_dist = cgmath::dot(v3, v3);
        if sq_dist < best_dist {
            reference_point = point2;
            best_dist = sq_dist;
        }
    }

    reference_point
}

pub fn closest_point_on_line(a: &cgmath::Vector3<f32>, b: &cgmath::Vector3<f32>, point: &cgmath::Vector3<f32>) -> cgmath::Vector3<f32> {

    let ab = b - a;
    let t = cgmath::dot(point - a, ab) / cgmath::dot(ab, ab);
    return (a + (t.max(0.0).min(1.0)) * ab);
}

pub struct CollisionDebug {
    sphere: model::Model,
    cylinder: model::Model,
    bind_groups: Vec<(wgpu::Buffer, wgpu::BindGroup)>,
}

impl CollisionDebug {

    pub fn new(device: &wgpu::Device, queue: &wgpu::Queue, layout: &wgpu::BindGroupLayout) -> CollisionDebug {

        let sphere = model::Model::new(&device, &queue, &layout, "sphere", false);
        let cylinder = model::Model::new(&device, &queue, &layout, "cylinder", false);
        
        let mut bind_groups: Vec<(wgpu::Buffer, wgpu::BindGroup)> = Vec::new();

        for i in 0..20 {
            bind_groups.push(update_model_matrix(device, cgmath::Matrix4::from_translation(cgmath::Vector3::new(0.0, 0.0, 0.0))));
        }
    

        CollisionDebug { sphere, cylinder, bind_groups }
    }

    pub fn update(&mut self, queue: &wgpu::Queue, device: &wgpu::Device, spheres: &Vec<Sphere>, capsules: &Vec<Capsule>) {

        for i in 0..spheres.len() {
            let t = cgmath::Matrix4::from_translation(spheres[i].pos);
            let s = cgmath::Matrix4::from_scale(spheres[i].radius);
            let ts = t * s;
            self.bind_groups[i] = update_model_matrix(device, ts);
        }

        let mut offset = spheres.len();

        for i in 0..capsules.len() {

            let capsule_normal = (capsules[i].tip - capsules[i].base).normalize();
            let line_end_offset = capsule_normal * capsules[i].radius;
            let a = capsules[i].base + line_end_offset;
            let b = capsules[i].tip - line_end_offset;

            let s = cgmath::Matrix4::from_scale(capsules[i].radius);

            let mut t = cgmath::Matrix4::from_translation(a);
            let mut ts = t * s;
            self.bind_groups[offset + (i * 3)] = update_model_matrix(device, ts);

            t = cgmath::Matrix4::from_translation(b);
            ts = t * s;
            self.bind_groups[offset + (i * 3) + 1] = update_model_matrix(device, ts);

            let r: cgmath::Quaternion<f32> = cgmath::Rotation::between_vectors(cgmath::Vector3::unit_y(), (capsules[i].tip - capsules[i].base).normalize());
            let rm: cgmath::Matrix4<f32> = r.into();
            t = cgmath::Matrix4::from_translation((capsules[i].tip + capsules[i].base) / 2.0);
            ts = t * rm;// * s;
            self.bind_groups[offset + (i * 3) + 2] = update_model_matrix(device, ts);
        }
    }

    pub fn render<'a, 'b>(&'a mut self, render_pass: &'b mut wgpu::RenderPass<'a>, spheres: &Vec<Sphere>, capsules: &Vec<Capsule>) where 'a: 'b {

        for i in 0..spheres.len() {
            render_pass.draw_model_debug(&self.sphere, &self.bind_groups[i].1);
        }

        let mut offset = spheres.len();

        for i in 0..capsules.len() {
            render_pass.draw_model_debug(&self.sphere, &self.bind_groups[offset + (i * 3)].1);
            render_pass.draw_model_debug(&self.sphere, &self.bind_groups[offset + (i * 3) + 1].1);
            render_pass.draw_model_debug(&self.cylinder, &self.bind_groups[offset + (i * 3) + 2].1);
        }
    }
}

pub fn update_model_matrix(device: &wgpu::Device, model: cgmath::Matrix4<f32>) -> (wgpu::Buffer, wgpu::BindGroup) {

    let mut joints = uniform::JointUniforms::new();

    joints.model = model.into();

    let joints_uniform_buffer = device.create_buffer_init(
        &wgpu::util::BufferInitDescriptor {
            label: Some("Joint Uniform Buffer"),
            contents: bytemuck::cast_slice(&[joints]),
            usage: wgpu::BufferUsage::UNIFORM | wgpu::BufferUsage::COPY_DST,
        }
    );

    let joints_uniform_bind_group_layout = uniform::JointUniforms::get_bind_group_layout(&device);

    let joints_uniform_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
        layout: &joints_uniform_bind_group_layout,
        entries: &[
            wgpu::BindGroupEntry {
                binding: 0,
                resource: joints_uniform_buffer.as_entire_binding(),
            }
        ],
        label: Some("joint_uniform_bind_group"),
    });

    (joints_uniform_buffer, joints_uniform_bind_group)
}