#![allow(unused_variables)]
#![allow(unused_imports)]
#![allow(unused_mut)]

use winit::{
    event::*,
    event_loop::{EventLoop, ControlFlow},
    window::{Window, WindowBuilder},
};
use futures::executor::block_on;
use wgpu::util::DeviceExt;

use cgmath::SquareMatrix;
use cgmath::prelude::VectorSpace;
use cgmath::Transform;

use crate::texture;
use crate::uniform;
use crate::camera;
use crate::r_pipeline;
use crate::model;
use crate::game;

pub struct State {
    surface: wgpu::Surface,
    device: wgpu::Device,
    queue: wgpu::Queue,
    sc_desc: wgpu::SwapChainDescriptor,
    swap_chain: wgpu::SwapChain,
    pub size: winit::dpi::PhysicalSize<u32>,
    camera: camera::Camera,
    projection: camera::Projection,
    camera_controller: camera::CameraController,
    uniforms: uniform::Uniforms,
    uniform_buffer: wgpu::Buffer,
    uniform_bind_group: wgpu::BindGroup,
    depth_texture: texture::Texture,
    pub model_pipeline: wgpu::RenderPipeline,
    ani: f32,
    game: game::Game,
    tick: f32,
}

impl State {

    pub async fn new(window: &Window) -> Self {

        let size = window.inner_size();

        let instance = wgpu::Instance::new(wgpu::BackendBit::PRIMARY);
        let surface = unsafe { instance.create_surface(window) };
        let adapter = instance.request_adapter(
            &wgpu::RequestAdapterOptions {
                power_preference: wgpu::PowerPreference::default(), //HighPerformance
                compatible_surface: Some(&surface),
            },
        ).await.unwrap();

        let (device, queue) = adapter.request_device(
            &wgpu::DeviceDescriptor {
                features: wgpu::Features::empty(),
                limits: wgpu::Limits::default(),
                label: None,
            },
            None,
        ).await.unwrap();

        //Fifo or Immediate (vsync on and off) Mailbox (double buffering)
        let sc_desc = wgpu::SwapChainDescriptor {
            usage: wgpu::TextureUsage::RENDER_ATTACHMENT,
            format: wgpu::TextureFormat::Bgra8UnormSrgb,
            width: size.width,
            height: size.height,
            present_mode: wgpu::PresentMode::Mailbox,
        };
        let swap_chain = device.create_swap_chain(&surface, &sc_desc);

        let mut camera = camera::Camera::new();
        let projection = camera::Projection::new(sc_desc.width, sc_desc.height, cgmath::Deg(90.0), 0.1, 4000.0);
        let camera_controller = camera::CameraController::new(8.0, 3.0);

        let mut uniforms = uniform::Uniforms::new();
        uniforms.update_view_proj(&camera, &projection);

        let uniform_buffer = device.create_buffer_init(
            &wgpu::util::BufferInitDescriptor {
                label: Some("Uniform Buffer"),
                contents: bytemuck::cast_slice(&[uniforms]),
                usage: wgpu::BufferUsage::UNIFORM | wgpu::BufferUsage::COPY_DST,
            }
        );

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
                }
            ],
            label: Some("uniform_bind_group_layout"),
        });

        let uniform_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            layout: &uniform_bind_group_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: uniform_buffer.as_entire_binding(),
                }
            ],
            label: Some("uniform_bind_group"),
        });

        let joints_uniform_bind_group_layout = uniform::JointUniforms::get_bind_group_layout(&device);

        let texture_bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            entries: &[
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStage::FRAGMENT,
                    ty: wgpu::BindingType::Texture {
                        multisampled: false,
                        view_dimension: wgpu::TextureViewDimension::D2,
                        sample_type: wgpu::TextureSampleType::Float { filterable: true },
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 1,
                    visibility: wgpu::ShaderStage::FRAGMENT,
                    ty: wgpu::BindingType::Sampler { 
                        comparison: false, 
                        filtering: false,
                    },
                    count: None,
                },
            ],
            label: Some("texture_bind_group_layout"),
        });

        let depth_texture = texture::Texture::create_depth_texture(&device, &sc_desc, "depth_texture");

        let model_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("Model Render Pipeline Layout"),
            bind_group_layouts: &[&uniform_bind_group_layout, &joints_uniform_bind_group_layout, &texture_bind_group_layout],
            push_constant_ranges: &[],
        });

        let model_pipeline = r_pipeline::pipeline(&device, &sc_desc, &model_layout, wgpu::include_spirv!("model.vert.spv"), wgpu::include_spirv!("model.frag.spv"), model::Vertex::desc(), wgpu::PrimitiveTopology::TriangleList);

        let game = game::Game::new(&device, &queue, &texture_bind_group_layout);

        Self {
            surface,
            device,
            queue,
            sc_desc,
            swap_chain,
            size,
            camera,
            projection,
            camera_controller,
            uniforms,
            uniform_buffer,
            uniform_bind_group,
            depth_texture,
            model_pipeline,
            ani: 0.0,
            game,
            tick: 0.0,
        }
    }

    pub fn resize(&mut self, new_size: winit::dpi::PhysicalSize<u32>) {

        self.size = new_size;
        self.sc_desc.width = new_size.width;
        self.sc_desc.height = new_size.height;
        self.projection.resize(new_size.width, new_size.height);
        self.depth_texture = texture::Texture::create_depth_texture(&self.device, &self.sc_desc, "depth_texture");
        self.swap_chain = self.device.create_swap_chain(&self.surface, &self.sc_desc);
    }

    pub fn moved(&mut self) {
        self.swap_chain = self.device.create_swap_chain(&self.surface, &self.sc_desc);
    }

    pub fn input(&mut self, event: &WindowEvent, window: &Window) {

        match event {
            WindowEvent::KeyboardInput {
                input:
                    KeyboardInput {
                        state,
                        virtual_keycode: Some(keycode),
                        ..
                    },
                ..
            } => {
                self.game.process_keyboard(*keycode, *state);
                self.camera_controller.process_keyboard(*keycode, *state);
            }
            WindowEvent::CursorMoved  { position, .. } => {
                self.camera_controller.process_mouse((position.x as f32 / self.sc_desc.width as f32) - 0.5, 0.5 - (position.y as f32 / self.sc_desc.height as f32), &mut self.camera);
                window.set_cursor_position(winit::dpi::PhysicalPosition::new(self.sc_desc.width as f32 / 2.0, self.sc_desc.height as f32 / 2.0));
            }
            WindowEvent::MouseInput { state, button, .. } => {

            }
            _ => {},
        }
    }

    pub fn update(&mut self, delta: f32) {

        self.camera_controller.update_camera(&mut self.camera, delta);
        self.uniforms.update_view_proj(&self.camera, &self.projection);
        self.uniforms.update_time(0.016);
        self.queue.write_buffer(&self.uniform_buffer, 0, bytemuck::cast_slice(&[self.uniforms]));

        self.tick += delta;

        if self.tick > 0.0166666 {
            self.game.update(delta, &self.queue, &self.device, &mut self.camera);
            self.tick = 0.0;
        }
    }

    pub fn render(&mut self) -> Result<(), wgpu::SwapChainError> {

        let frame = self.swap_chain.get_current_frame()?.output;
        let mut encoder = self.device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("Render Encoder"),
        });
        let mut offset = 0usize;
        {
            let mut render_pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: None,
                color_attachments: &[
                    wgpu::RenderPassColorAttachment {
                        view: &frame.view,
                        resolve_target: None,
                        ops: wgpu::Operations {
                            load: wgpu::LoadOp::Clear(wgpu::Color {
                                r: 0.1,
                                g: 0.2,
                                b: 0.3,
                                a: 1.0,
                            }),
                            store: true,
                        }
                    }
                ],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &self.depth_texture.view,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(1.0),
                        store: true,
                    }),
                    stencil_ops: None,
                }),
            });

            render_pass.set_pipeline(&self.model_pipeline);
            render_pass.set_bind_group(0, &self.uniform_bind_group, &[]);
            
            self.game.render(&mut render_pass);                
        }
        self.queue.submit(std::iter::once(encoder.finish()));

        Ok(())
    }
}