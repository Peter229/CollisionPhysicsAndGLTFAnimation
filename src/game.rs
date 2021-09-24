use crate::model;
use crate::model::DrawModel;
use crate::r_state;
use crate::collision;
use crate::player;
use crate::level;
use crate::network;
use crate::server;
use crate::car;
use crate::camera;

use std::io::{self, Read, Error, stdin, stdout, Write, ErrorKind};
use winit::event::*;
use winit::dpi::PhysicalPosition;

pub struct Game {
    level: level::Level,
    p_model: model::Model,
    col_debug: collision::CollisionDebug,
    spheres: Vec<collision::Sphere>,
    capsules: Vec<collision::Capsule>,
    player: player::Player,
    car: car::Car,
    keys: [bool; 1024],
    ticks: u32,
    network: network::Network,
}

impl Game {

    pub fn new(device: &wgpu::Device, queue: &wgpu::Queue, layout: &wgpu::BindGroupLayout) -> Game {
        
        let level = level::Level::new(device, queue, layout);

        let p_model = model::Model::new(&device, &queue, &layout, "mandemvsix", false);

        let col_debug = collision::CollisionDebug::new(device, queue, layout);

        let mut spheres: Vec<collision::Sphere> = Vec::new();

        let mut capsules: Vec<collision::Capsule> = Vec::new();
        capsules.push(collision::Capsule::new([0.0, 1.0, 0.0].into(), [0.0, 5.0, 0.0].into(), 1.0));

        let player = player::Player::new([0.0, 3.0, 0.0].into());

        let mut car = car::Car::new(device, queue, layout);
        car.update_col_spheres();

        let keys = [false; 1024];

        let network = network::Network::new();

        Game { level, p_model, col_debug, spheres, capsules, player, car, keys, ticks: 0, network }
    }

    pub fn process_keyboard(&mut self, key: VirtualKeyCode, state: ElementState) {

        let pressed = state == ElementState::Pressed;
        self.keys[key as usize] = pressed;
    }

    pub fn update(&mut self, delta: f32, queue: &wgpu::Queue, device: &wgpu::Device, camera: &mut camera::Camera) {

        self.ticks += 1;

        /*if self.keys[VirtualKeyCode::Left as usize] {
            self.player.velocity.x = -0.1;
            if !self.network.is_server {
                self.network.c_request_connect();
            }
        }
        if self.keys[VirtualKeyCode::Right as usize] {
            self.player.velocity.x = 0.1;
        }

        if self.keys[VirtualKeyCode::Up as usize] {
            self.player.velocity.z = -0.1;
        }
        if self.keys[VirtualKeyCode::Down as usize] {
            self.player.velocity.z = 0.1;
        }*/

        if self.keys[VirtualKeyCode::Space as usize] {
            self.car.vel.y = 0.5;
        }
        if self.keys[VirtualKeyCode::LShift as usize] {
            self.player.velocity.y = -0.5;
        }

        if self.keys[VirtualKeyCode::Up as usize] {
            self.player.velocity.x = -0.2;
        }
        if self.keys[VirtualKeyCode::Down as usize] {
            self.player.velocity.x = 0.2;
        }

        if self.keys[VirtualKeyCode::Right as usize] {
            self.player.velocity.z = 0.2;
        }

        if self.keys[VirtualKeyCode::Left as usize] {
            self.player.velocity.z = -0.2;
        }

        if self.keys[VirtualKeyCode::E as usize] {
            for i in 0..self.level.collision_meshes.len() {
                if self.level.collision_meshes[i].moving {
                    collision::ray_vs_level(&self.level.collision_meshes[i].object, &cgmath::Vector3::new(camera.position.x, camera.position.y, camera.position.z), &camera.forward, None);
                }
                else {
                    collision::ray_vs_level(&self.level.collision_meshes[i].object, &cgmath::Vector3::new(camera.position.x, camera.position.y, camera.position.z), &camera.forward, None);
                }
            }
        }

        /*let mut col_info = self.capsules[0].vs_capsule(&self.capsules[1]);
        if col_info.happen {
            let offset_vec = (col_info.normal * col_info.penetration);
            self.capsules[0].change_position(offset_vec.x, offset_vec.y, offset_vec.z);
        }*/

        /*let position = f32::sin((self.ticks as f32) / 30.0) * 4.0;
        let movement = cgmath::Matrix4::from_translation(cgmath::Vector3::new(0.0, position, 0.0));
        collision::clean_collide_triangle_mesh_moving(&self.platform, &mut self.capsules[0], position);
        self.platform.joints.model = movement.into();
        self.platform.update_model(queue);*/

        self.car.update(device, &self.level);

        self.level.update(queue, self.ticks);
        self.player.update(&self.level);

        self.capsules[0].base = self.player.capsule.base;
        self.capsules[0].tip = self.player.capsule.tip;

        self.p_model.update_skeleton(delta, queue);
        self.col_debug.update(queue, device, &self.car.col_spheres, &self.capsules);

        self.network.recv();
    }

    pub fn render<'a, 'b>(&'a mut self, render_pass: &'b mut wgpu::RenderPass<'a>) where 'a: 'b {

        //render_pass.draw_model(&self.p_model);

        for i in 0..self.level.collision_meshes.len() {
            render_pass.draw_model(&self.level.collision_meshes[i].object);
        }

        render_pass.draw_model_debug(&self.car.model, &self.car.bind_group);

        self.col_debug.render(render_pass, &self.car.col_spheres, &self.capsules);
    }
}