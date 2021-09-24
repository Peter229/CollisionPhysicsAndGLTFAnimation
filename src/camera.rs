#![allow(unused_variables)]
#![allow(unused_imports)]
#![allow(unused_mut)]
#![allow(unused_parens)]

use cgmath::*;
use winit::event::*;
use winit::dpi::PhysicalPosition;
use std::time::Duration;
use std::f32::consts::FRAC_PI_2;

#[rustfmt::skip]
pub const OPENGL_TO_WGPU_MATRIX: cgmath::Matrix4<f32> = cgmath::Matrix4::new(
    1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.5, 0.0,
    0.0, 0.0, 0.5, 1.0,
);

#[derive(Debug)]
pub struct Camera {
    pub position: Point3<f32>,
    pub yaw: f32,
    pub pitch: f32,
    pub view: Matrix4<f32>,
    pub forward: Vector3<f32>,
    pub right: Vector3<f32>,
}

impl Camera {
    
    pub fn new() -> Self {
        Self {
            position: Point3::new(0.0, 0.0, 1.0),
            yaw: 0.0,
            pitch: 0.0,
            view: Matrix4::look_at_dir(
                Point3::new(0.0, 0.0, 0.0),
                Vector3::new(
                    Rad(0.0).cos() * Rad(0.0).sin(),
                    Rad(0.0).sin(),
                    Rad(0.0).sin() * Rad(0.0).cos(),
                ).normalize(),
                Vector3::unit_y(),
            ),
            forward: Vector3::new(0.0, 0.0, 1.0),
            right: Vector3::new(1.0, 0.0, 0.0),
        }
    }

    pub fn update_view(&mut self) {
        let up = self.right.cross(self.forward);
        self.view = Matrix4::look_at_dir(self.position, self.forward, up);
    }
}

pub struct Projection {
    aspect: f32,
    fovy: Rad<f32>,
    znear: f32,
    zfar: f32,
}

impl Projection {
    
    pub fn new<F: Into<Rad<f32>>>(
        width: u32,
        height: u32,
        fovy: F,
        znear: f32,
        zfar: f32,
    ) -> Self {
        Self {
            aspect: width as f32 / height as f32,
            fovy: fovy.into(),
            znear,
            zfar,
        }
    }

    pub fn resize(&mut self, width: u32, height: u32) {
        self.aspect = width as f32 / height as f32;
    }

    pub fn calc_matrix(&self) -> Matrix4<f32> {
        OPENGL_TO_WGPU_MATRIX * perspective(self.fovy, self.aspect, self.znear, self.zfar)
    }
}

#[derive(Debug)]
pub struct CameraController {
    amount_left: f32,
    amount_right: f32,
    amount_forward: f32,
    amount_backward: f32,
    amount_up: f32,
    amount_down: f32,
    rotate_horizontal: f32,
    rotate_vertical: f32,
    scroll: f32,
    speed: f32,
    sensitivity: f32,
}

impl CameraController {

    pub fn new(speed: f32, sensitivity: f32) -> Self {
        Self {
            amount_left: 0.0,
            amount_right: 0.0,
            amount_forward: 0.0,
            amount_backward: 0.0,
            amount_up: 0.0,
            amount_down: 0.0,
            rotate_horizontal: 0.0,
            rotate_vertical: 0.0,
            scroll: 0.0,
            speed,
            sensitivity,
        }
    }

    pub fn process_keyboard(&mut self, key: VirtualKeyCode, state: ElementState) {
        
        let amount = if state == ElementState::Pressed { 1.0 } else { 0.0 };
        match key {
            VirtualKeyCode::W => {
                self.amount_forward = amount;
            }
            VirtualKeyCode::S => {
                self.amount_backward = amount;
            }
            VirtualKeyCode::A => {
                self.amount_left = amount;
            }
            VirtualKeyCode::D => {
                self.amount_right = amount;
            }
            VirtualKeyCode::Space => {
                self.amount_up = amount;
            }
            VirtualKeyCode::LShift => {
                self.amount_down = amount;
            }
            _ => {},
        }
    }

    pub fn process_mouse(&mut self, mouse_dx: f32, mouse_dy: f32, camera: &mut Camera) {

        let mut x_offset = mouse_dx * self.sensitivity;
        let mut y_offset = mouse_dy * self.sensitivity;
        self.rotate_horizontal += x_offset;
        self.rotate_vertical += y_offset;
        self.rotate_horizontal %= 360.0_f32;

        let max_look_up: f32 = 89.0_f32.to_radians();
        if self.rotate_vertical > max_look_up {
            self.rotate_vertical = max_look_up;
        }else if self.rotate_vertical < -max_look_up {
            self.rotate_vertical = -max_look_up;
        }

        camera.yaw = self.rotate_horizontal;
        camera.pitch = self.rotate_vertical;
    }

    pub fn process_scroll(&mut self, delta: &MouseScrollDelta) {
        /*self.scroll = match delta {
            MouseScrollDelta::LineDelta(_, scroll) => -scroll * 100.0,
            MouseScrollDelta::PixelDelta(PhysicalPosition {
                y: scroll,
                ..
            }) => -*scroll as f32,
        };*/
    }

    pub fn update_camera(&mut self, camera: &mut Camera, delta: f32) {

        camera.forward = Vector3::new(self.rotate_horizontal.cos() * self.rotate_vertical.cos(), self.rotate_vertical.sin(), self.rotate_horizontal.sin() * self.rotate_vertical.cos()).normalize();
        camera.right = camera.forward.cross(Vector3::unit_y()).normalize();
        let up = camera.right.cross(camera.forward);
        camera.position += camera.forward * (self.amount_forward - self.amount_backward) * self.speed * delta;
        camera.position += camera.right * (self.amount_right - self.amount_left) * self.speed * delta;
        camera.view = Matrix4::look_at_dir(camera.position, camera.forward, up);
    }
}