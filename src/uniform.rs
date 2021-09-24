#![allow(unused_variables)]
#![allow(unused_imports)]

use wgpu::util::DeviceExt;
use cgmath::SquareMatrix;
use cgmath::InnerSpace;
use cgmath::Rotation3;
use cgmath::Zero;

use crate::camera;

#[repr(C)]
#[derive(Debug, Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
pub struct Uniforms {
    proj: [[f32; 4]; 4],
    view: [[f32; 4]; 4],
    time: f32,
}

impl Uniforms {
    pub fn new() -> Self {
        Self {
            proj: cgmath::Matrix4::identity().into(),
            view: cgmath::Matrix4::identity().into(),
            time: 0.0,
        }
    }

    pub fn update_view_proj(&mut self, camera: &camera::Camera, projection: &camera::Projection) {
        self.proj = projection.calc_matrix().into();
        self.view =  camera.view.into();
    }

    pub fn update_time(&mut self, time: f32) {
        self.time += time;
    }
}

#[repr(C)]
#[derive(Debug, Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
pub struct JointUniforms {
    pub joints: [[[f32; 4]; 4]; 64],
    pub model: [[f32; 4]; 4],
}

impl JointUniforms {
    
    pub fn new() -> Self {
        Self {
            model: cgmath::Matrix4::identity().into(),
            joints: [cgmath::Matrix4::identity().into(); 64],
        }
    }

    pub fn get_bind_group_layout(device: &wgpu::Device) -> wgpu::BindGroupLayout {

        let uniform_bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            entries: &[
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStage::VERTEX,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
            ],
            label: Some("joint_uniform_bind_group_layout"),
        });

        uniform_bind_group_layout
    }
}