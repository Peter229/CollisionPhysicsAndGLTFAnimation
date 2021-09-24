use cgmath::prelude::InnerSpace;
use cgmath::SquareMatrix;

use crate::model;
use crate::collision;
use crate::level;

pub struct Force {
    pub dir: cgmath::Vector3<f32>,
    pub strength: f32,
}

impl Force {

    pub fn new(dir: cgmath::Vector3<f32>, strength: f32) -> Force {
        
        Force { dir, strength }
    }
}

pub struct Constraint {
    pub connection_vectors: Vec<cgmath::Vector3<f32>>,
}

impl Constraint {

    pub fn new(connection_vectors: Vec<cgmath::Vector3<f32>>) -> Constraint {
        
        Constraint { connection_vectors }
    }
}

pub struct Car {
    pub model: model::Model,
    pub pos: cgmath::Vector3<f32>,
    pub vel: cgmath::Vector3<f32>,
    pub angle: f32,
    buffer: wgpu::Buffer,
    pub bind_group: wgpu::BindGroup,
    pub col_spheres: Vec<collision::Sphere>,
    pub forces: Vec<Force>,
    pub constraints: Vec<Constraint>,
    pub mat: cgmath::Matrix4<f32>,
    pub com: cgmath::Vector3<f32>,
}

const sphere_offsets: [cgmath::Vector3<f32>; 4] = [cgmath::Vector3::new(0.37571, 0.13997, -1.4734), cgmath::Vector3::new(-0.37571, 0.13997, -1.4734), 
                                                cgmath::Vector3::new(0.37571, 0.13997, 0.0), cgmath::Vector3::new(-0.37571, 0.13997, 0.0)];
const offset: cgmath::Vector3<f32> = cgmath::Vector3::new(0.0, -0.5662, -1.411);

impl Car {

    pub fn new(device: &wgpu::Device, queue: &wgpu::Queue, layout: &wgpu::BindGroupLayout) -> Car {
        
        let model = model::Model::new(device, queue, layout, "car", false);

        let pos = cgmath::Vector3::new(0.0, 10.0, 0.0);
        let vel = cgmath::Vector3::new(0.0, 0.0, 0.0);

        let angle = 0.0;

        let mut t = cgmath::Matrix4::from_translation(pos);

        let (buffer, bind_group) = collision::update_model_matrix(device, t);

        //col sphere positions
        /* 0.4 radius
        0.37957 -0.59112 -0.60883
        -0.36373 -0.59112 -0.60883
        0.37957 -0.59112 -2.0893
        -0.36373 -0.59112 -2.0893
        */

        let mut col_spheres = Vec::new();
        col_spheres.push(collision::Sphere::new(cgmath::Vector3::new(0.37957, 2.59112, -0.60883), 0.4));
        col_spheres.push(collision::Sphere::new(cgmath::Vector3::new(-0.36373, 2.59112, -0.60883), 0.4));
        col_spheres.push(collision::Sphere::new(cgmath::Vector3::new(0.37957, 2.59112, -2.0893), 0.4));
        col_spheres.push(collision::Sphere::new(cgmath::Vector3::new(-0.36373, 2.59112, -2.0893), 0.4));

        let mut total = col_spheres[0].pos;

        for i in 1..col_spheres.len() {

            total += col_spheres[i].pos;
        }

        let com = total / (col_spheres.len() as f32);

        let mut forces = Vec::new();
        forces.push(Force::new(cgmath::Vector3::new(0.0, 0.0, 0.0), 1.0));
        forces.push(Force::new(cgmath::Vector3::new(0.0, 0.0, 0.0), 1.0));
        forces.push(Force::new(cgmath::Vector3::new(0.0, 0.0, 0.0), 1.0));
        forces.push(Force::new(cgmath::Vector3::new(0.0, 0.0, 0.0), 1.0));

        let mut constraints = Vec::new();
        for i in 0..col_spheres.len() {

            let mut con_vec = Vec::new();
            for j in 0..col_spheres.len() {

                if i != j {
                    con_vec.push(col_spheres[j].pos - col_spheres[i].pos);
                }
            }

            constraints.push(Constraint::new(con_vec));
        }

        let mat = cgmath::Matrix4::identity();

        //let com = cgmath::Vector3::new(0.0, 0.0, 0.0);

        Car { model, pos, vel, angle, buffer, bind_group, col_spheres, forces, constraints, mat, com }
    }

    pub fn update(&mut self, device: &wgpu::Device, level: &level::Level) {

        self.vel.y = -0.09;

        let forward = cgmath::Vector3::new(self.angle.sin(), 0.0, self.angle.cos()).normalize();        
        let right = forward.cross(cgmath::Vector3::unit_y()).normalize() * 0.5;
        let up = right.cross(forward);

        self.com += forward * self.vel.x;
        self.com.y += self.vel.y;

        self.col_spheres[0].pos = self.com + forward + right;
        self.col_spheres[1].pos = self.com + forward - right;
        self.col_spheres[2].pos = self.com - forward + right;
        self.col_spheres[3].pos = self.com - forward - right;

        //Move spheres, then get direction and length between each sphere and force them to be the same length away
        
        let mut min_offset = cgmath::Vector3::new(0.0, 0.0, 0.0);
        let mut full_happen = false;
        let mut min_dis = f32::MAX; 

        //Standard check
        for l in 0..4 {

            //let mut contact_manifold = Vec::new();

            min_offset = cgmath::Vector3::new(0.0, 0.0, 0.0);
            full_happen = false;
            min_dis = f32::MAX; 

            for i in 0..level.collision_meshes.len() {

                for j in 0..self.col_spheres.len() {

                    let (happened, offset_vec) = collision::sphere_mesh_one_sweep(&level.collision_meshes[i].object, &mut self.col_spheres[j], &self.vel, true, None);
                    
                    if happened {

                        self.com += offset_vec;

                        let mut t = forward + right;

                        if j == 1 {
                            t = forward - right;
                        }
                        else if j == 2 {
                            t = -forward + right;    
                        }
                        else {
                            t = -forward - right;
                        }

                        let iner = t.cross(offset_vec);

                        full_happen = true;
                    }

                }
            }
        }

        self.vel.x = 0.0;
        self.vel.y = 0.0;

        let (buffer, bind_group) = collision::update_model_matrix(device, self.mat);
        self.buffer = buffer;
        self.bind_group = bind_group;
    }

    pub fn set_center_of_mass(&mut self) {

        let mut total = self.col_spheres[0].pos;

        for i in 1..self.col_spheres.len() {

            total += self.col_spheres[i].pos;
        }

        self.com = total / (self.col_spheres.len() as f32);
    }

    pub fn update_col_spheres(&mut self) {
        for i in 0..self.col_spheres.len() {

            self.col_spheres[i].pos += (self.forces[i].dir * self.forces[i].strength);
        }
    }

    pub fn apply_constraints(&mut self) {

        let mut final_force = cgmath::Vector3::new(0.0, 0.0, 0.0);

        for i in 0..self.col_spheres.len() {

            let d = (self.com - self.col_spheres[i].pos).magnitude();
            let moment = d * self.forces[i].dir;

            self.forces[i].dir = moment;
            final_force += moment;
        }

        //for i in 0..self.forces.len() {

            //self.forces[i].dir = final_force;
        //}
    }
}

/*pub fn make_orthonormal_basis(x: &cgmath::Vector3<f32>, y: &mut cgmath::Vector3<f32>, z: &mut cgmath::Vector3<f32>) {

    z = x.cross(y);

    if z.magnitude2() == 0.0 {
        return;
    }

    y = z.cross(x);

    y.normalize();
    z.normalize();
}*/

/* OLD UPDATE
pub fn update(&mut self, device: &wgpu::Device, level: &level::Level) {

        self.vel.y = -0.09;

        let forward = cgmath::Vector3::new(self.angle.sin(), 0.0, self.angle.cos()).normalize();
        let right = forward.cross(cgmath::Vector3::unit_y()).normalize();
        let up = right.cross(forward);
        let tru_vel = forward * self.vel.x;
        //self.pos += forward * self.vel.x;
        //self.pos.y += self.vel.y;

        let mut t = cgmath::Matrix4::from_translation(self.pos + forward * self.vel.x);
        let mut r: cgmath::Quaternion<f32> = cgmath::Rotation3::from_angle_y(cgmath::Rad(self.angle));
        let mut rm: cgmath::Matrix4<f32> = r.into();
        self.mat = t * rm;

        self.apply_constraints();

        //Move spheres, then get direction and length between each sphere and force them to be the same length away

        //Gravity check
        let mut min_offset = cgmath::Vector3::new(0.0, 0.0, 0.0);
        let mut full_happen = false;
        let mut min_dis = f32::MAX; 

        /*for l in 0..4 {

            min_offset = cgmath::Vector3::new(0.0, 0.0, 0.0);
            full_happen = false;
            min_dis = f32::MAX; 

            for i in 0..level.collision_meshes.len() {

                let t_vel = cgmath::Vector3::new(0.0, self.vel.y, 0.0);

                for j in 0..self.col_spheres.len() {

                    let (happened, offset_vec) = collision::sphere_mesh_one_sweep(&level.collision_meshes[i].object, &mut self.col_spheres[j], &t_vel, true, None);
                    if happened {

                        full_happen = true;

                        let pen = offset_vec.magnitude2();
                        if pen < min_dis {
                            min_dis = pen;
                            min_offset = offset_vec;
                        }
                    }
                }

                if full_happen {
                    self.pos += min_offset;
                    
                    t = cgmath::Matrix4::from_translation(self.pos);
                    r = cgmath::Rotation3::from_angle_y(cgmath::Rad(self.angle));
                    rm = r.into();
                    self.mat = t * rm;

                    //self.update_col_spheres();
                    self.apply_constraints();
                }
            }
        }*/

        min_offset = cgmath::Vector3::new(0.0, 0.0, 0.0);
        full_happen = false;
        min_dis = f32::MAX; 

        //Standard check
        for l in 0..4 {

            min_offset = cgmath::Vector3::new(0.0, 0.0, 0.0);
            full_happen = false;
            min_dis = f32::MAX; 

            for i in 0..level.collision_meshes.len() {

                for j in 0..self.col_spheres.len() {

                    let (happened, offset_vec) = collision::sphere_mesh_one_sweep(&level.collision_meshes[i].object, &mut self.col_spheres[j], &tru_vel, true, None);
                    if happened {

                        full_happen = true;

                        let pen = offset_vec.magnitude2();
                        if pen < min_dis {
                            min_dis = pen;
                            min_offset = offset_vec;
                        }
                    }
                }

                /*if full_happen {
                    self.pos += min_offset;
                    
                    t = cgmath::Matrix4::from_translation(self.pos);
                    r = cgmath::Rotation3::from_angle_y(cgmath::Rad(self.angle));
                    rm = r.into();
                    self.mat = t * rm;

                    //self.update_col_spheres();
                    self.apply_constraints();
                }*/
            }
        }

        self.pos = forward * self.vel.x;

        self.vel.x = 0.0;

        let (buffer, bind_group) = collision::update_model_matrix(device, self.mat);
        self.buffer = buffer;
        self.bind_group = bind_group;
    }
    */

    /*pub fn update(&mut self, device: &wgpu::Device, level: &level::Level) {

        self.vel.y = -0.09;

        let forward = cgmath::Vector3::new(self.angle.sin(), 0.0, self.angle.cos()).normalize();
        let right = forward.cross(cgmath::Vector3::unit_y()).normalize();
        let up = right.cross(forward);
        let tru_vel = forward * self.vel.x;
        //self.pos += forward * self.vel.x;
        //self.pos.y += self.vel.y;

        let mut t = cgmath::Matrix4::from_translation(self.pos + forward * self.vel.x);
        let mut r: cgmath::Quaternion<f32> = cgmath::Rotation3::from_angle_y(cgmath::Rad(self.angle));
        let mut rm: cgmath::Matrix4<f32> = r.into();
        self.mat = t * rm;

        self.apply_constraints();

        //Move spheres, then get direction and length between each sphere and force them to be the same length away
        
        let mut min_offset = cgmath::Vector3::new(0.0, 0.0, 0.0);
        let mut full_happen = false;
        let mut min_dis = f32::MAX; 

        //Standard check
        for l in 0..4 {

            min_offset = cgmath::Vector3::new(0.0, 0.0, 0.0);
            full_happen = false;
            min_dis = f32::MAX; 

            for i in 0..level.collision_meshes.len() {

                for j in 0..self.col_spheres.len() {

                    let (happened, offset_vec) = collision::sphere_mesh_one_sweep(&level.collision_meshes[i].object, &mut self.col_spheres[j], &tru_vel, true, None);
                    if happened {

                        full_happen = true;

                        let pen = offset_vec.magnitude2();
                        if pen < min_dis {
                            min_dis = pen;
                            min_offset = offset_vec;
                        }
                    }
                }

                /*if full_happen {
                    self.pos += min_offset;
                    
                    t = cgmath::Matrix4::from_translation(self.pos);
                    r = cgmath::Rotation3::from_angle_y(cgmath::Rad(self.angle));
                    rm = r.into();
                    self.mat = t * rm;

                    //self.update_col_spheres();
                    self.apply_constraints();
                }*/
            }
        }

        self.pos = forward * self.vel.x;

        self.vel.x = 0.0;

        let (buffer, bind_group) = collision::update_model_matrix(device, self.mat);
        self.buffer = buffer;
        self.bind_group = bind_group;
    }*/

    /* GOOOD UPDATE

pub fn update(&mut self, device: &wgpu::Device, level: &level::Level) {

        self.vel.y = -0.09;

        let forward = cgmath::Vector3::new(self.angle.sin(), 0.0, self.angle.cos()).normalize();
        let right = forward.cross(cgmath::Vector3::unit_y()).normalize() * 0.5;
        let up = right.cross(forward);

        self.com += forward * self.vel.x;
        self.com.y += self.vel.y;

        self.col_spheres[0].pos = self.com + forward + right;
        self.col_spheres[1].pos = self.com + forward - right;
        self.col_spheres[2].pos = self.com - forward + right;
        self.col_spheres[3].pos = self.com - forward - right;

        //Move spheres, then get direction and length between each sphere and force them to be the same length away
        
        let mut min_offset = cgmath::Vector3::new(0.0, 0.0, 0.0);
        let mut full_happen = false;
        let mut min_dis = f32::MAX; 

        //Standard check
        for l in 0..4 {

            //let mut contact_manifold = Vec::new();

            min_offset = cgmath::Vector3::new(0.0, 0.0, 0.0);
            full_happen = false;
            min_dis = f32::MAX; 

            for i in 0..level.collision_meshes.len() {

                for j in 0..self.col_spheres.len() {

                    let (happened, offset_vec) = collision::sphere_mesh_one_sweep(&level.collision_meshes[i].object, &mut self.col_spheres[j], &self.vel, true, None);
                    
                    if happened {

                        self.com += offset_vec;

                        self.col_spheres[0].pos = self.com + forward + right;
                        self.col_spheres[1].pos = self.com + forward - right;
                        self.col_spheres[2].pos = self.com - forward + right;
                        self.col_spheres[3].pos = self.com - forward - right;

                    }
                }
            }
        }



        self.vel.x = 0.0;
        self.vel.y = 0.0;

        let (buffer, bind_group) = collision::update_model_matrix(device, self.mat);
        self.buffer = buffer;
        self.bind_group = bind_group;
    }*/