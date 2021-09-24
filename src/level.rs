use crate::model;

use cgmath::prelude::InnerSpace;
use cgmath::Rotation;

pub struct CollisionMesh {
    pub object: model::Model,
    pub moving: bool,
    pub position: cgmath::Vector3<f32>,
    pub velocity: cgmath::Vector3<f32>,
    pub angle: f32,
    pub speed: f32,
}

impl CollisionMesh {

    pub fn new(device: &wgpu::Device, queue: &wgpu::Queue, layout: &wgpu::BindGroupLayout, name: &str, moving: bool) -> CollisionMesh {

        let object = model::Model::new(device, queue, layout, name, true);

        let position = cgmath::Vector3::new(0.0, 4.0, 7.0);

        CollisionMesh { object, moving, position, velocity: cgmath::Vector3::new(0.0, 0.0, 0.0), angle: 0.0, speed: 0.1 }
    }
}

pub struct Level {
    pub collision_meshes: Vec<CollisionMesh>,
}

impl Level {
    pub fn new(device: &wgpu::Device, queue: &wgpu::Queue, layout: &wgpu::BindGroupLayout) -> Level {

        let mut collision_meshes: Vec<CollisionMesh> = Vec::new();
        collision_meshes.push(CollisionMesh::new(device, queue, layout, "ground", false));
        //collision_meshes.push(CollisionMesh::new(device, queue, layout, "platform", true));

        Level { collision_meshes }
    }

    pub fn update(&mut self, queue: &wgpu::Queue, ticks: u32) {

        /*self.collision_meshes[1].velocity.y = (f32::sin((ticks as f32) / 70.0) * 0.05);
        self.collision_meshes[1].velocity.x = (f32::sin((ticks as f32) / 30.0) * 0.05);
        self.collision_meshes[1].velocity.z = (f32::cos((ticks as f32) / 120.0) * 0.05);

        self.collision_meshes[1].position = self.collision_meshes[1].position + self.collision_meshes[1].velocity;
        self.collision_meshes[1].angle += self.collision_meshes[1].speed;
        self.collision_meshes[1].angle %= (2.0 * std::f32::consts::PI);*/

        for i in 0..self.collision_meshes.len() {
            if self.collision_meshes[i].moving {
                let t = cgmath::Matrix4::from_translation(self.collision_meshes[i].position);
                let r: cgmath::Quaternion<f32> = cgmath::Rotation3::from_angle_y(cgmath::Rad(self.collision_meshes[1].angle));
                let rm: cgmath::Matrix4<f32> = r.into();
                let srt = t * rm;
                self.collision_meshes[i].object.joints.model = srt.into();
                self.collision_meshes[i].object.update_model(queue);
            }
        }
    }
}