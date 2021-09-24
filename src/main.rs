#![allow(unused_variables)]
#![allow(unused_imports)]
#![allow(unused_mut)]

mod camera;
mod texture;
mod r_state;
mod uniform;
mod r_pipeline;
mod model;
mod game;
mod collision;
mod player;
mod level;
mod network;
mod server;
mod car;

use winit::{
    event::*,
    event_loop::{EventLoop, ControlFlow},
    window::{Window, WindowBuilder},
};

use futures::executor::block_on;
use wgpu::util::DeviceExt;
use cgmath::SquareMatrix;
use cgmath::InnerSpace;
use cgmath::Rotation3;
use cgmath::Zero;
use std::mem;
use std::time::{Instant, Duration};

fn main() {

    let mut gilrs = gilrs::Gilrs::new().unwrap();
    for (_id, gamepad) in gilrs.gamepads() {
        println!("{} is {:?}", gamepad.name(), gamepad.power_info());
    }

    env_logger::init();
    let event_loop = EventLoop::new();
    let window = WindowBuilder::new().build(&event_loop).unwrap();
    window.set_title("Smash");
    window.set_cursor_grab(true);
    window.set_cursor_visible(false);
    window.set_inner_size(winit::dpi::PhysicalSize::new(1600, 900));
    let mut state = block_on(r_state::State::new(&window));
    let mut fps: i32 = 0;
    let mut run_time = Instant::now();

    event_loop.run(move |event, _, control_flow| {
        match event {
            Event::WindowEvent {
                ref event,
                window_id,
            } if window_id == window.id() => {
                state.input(event, &window);
                match event {
                    WindowEvent::CloseRequested => *control_flow = ControlFlow::Exit,
                    WindowEvent::KeyboardInput {
                        input,
                        ..
                    } => {
                        match input {
                            KeyboardInput {
                                state: ElementState::Pressed,
                                virtual_keycode: Some(VirtualKeyCode::Escape),
                                ..
                            } => *control_flow = ControlFlow::Exit,
                            _ => {}
                        }
                    },
                    WindowEvent::Resized(physical_size) => {
                        state.resize(*physical_size);
                    }
                    WindowEvent::Moved(physical_position) => {
                        state.moved();
                    }
                    WindowEvent::ScaleFactorChanged { new_inner_size, .. } => {
                        state.resize(**new_inner_size);
                    }
                    _ => {}
                }
            }
            Event::MainEventsCleared => {
                /*fps += 1;
                if run_time.elapsed().as_millis() >= 1000 {
                    println!("fps {}", fps);
                    fps = 0;
                    run_time = Instant::now();
                }*/

                let delta_time = (((run_time.elapsed().as_nanos() as f64 / 1000.0) / 1000.0) / 1000.0) as f32;
                //println!("FPS {}", 1.0 / delta_time);
                run_time = Instant::now();
                state.update(delta_time);
                match state.render() {
                    Ok(_) => {}
                    Err(wgpu::SwapChainError::Lost) => state.resize(state.size),
                    Err(wgpu::SwapChainError::OutOfMemory) => *control_flow = ControlFlow::Exit,
                    Err(e) => eprintln!("{:?}", e),
                }
            }
            _ => {}
        }
    });
}
