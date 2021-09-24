use wgpu::util::DeviceExt;
use cgmath::SquareMatrix;
use cgmath::prelude::VectorSpace;
use cgmath::Transform;

use crate::texture;
use crate::uniform;
use crate::r_state;

#[repr(C)]
#[derive(Debug, Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
pub struct Vertex {
    pos: [f32; 3],
    normal: [f32; 3],
    tex_coord: [f32; 2],
    weight: [f32; 4],
    joint: [i32; 4],
}

impl Vertex {
    pub fn desc<'a>() -> wgpu::VertexBufferLayout<'a> {
        wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<Vertex>() as wgpu::BufferAddress,
            step_mode: wgpu::InputStepMode::Vertex,
            attributes: &[
                wgpu::VertexAttribute {
                    offset: 0,
                    shader_location: 0,
                    format: wgpu::VertexFormat::Float32x3,
                },
                wgpu::VertexAttribute {
                    offset: std::mem::size_of::<[f32; 3]>() as wgpu::BufferAddress,
                    shader_location: 1,
                    format: wgpu::VertexFormat::Float32x3,
                },
                wgpu::VertexAttribute {
                    offset: std::mem::size_of::<[f32; 6]>() as wgpu::BufferAddress,
                    shader_location: 2,
                    format: wgpu::VertexFormat::Float32x2,
                },
                wgpu::VertexAttribute {
                    offset: std::mem::size_of::<[f32; 8]>() as wgpu::BufferAddress,
                    shader_location: 3,
                    format: wgpu::VertexFormat::Float32x4,
                },
                wgpu::VertexAttribute {
                    offset: std::mem::size_of::<[f32; 12]>() as wgpu::BufferAddress,
                    shader_location: 4,
                    format: wgpu::VertexFormat::Sint32x4,
                },
            ],
        }
    }
}

pub struct Node {
    pub children: Vec<Node>,
    pub id: u32,
    pub translation: cgmath::Vector3<f32>,
    pub rotation: cgmath::Quaternion<f32>,
    pub scale: cgmath::Vector3<f32>,
    pub matrix: cgmath::Matrix4<f32>,
    pub updated_matrix: cgmath::Matrix4<f32>,
    pub translations: Vec<cgmath::Vector3<f32>>,
    pub rotations: Vec<cgmath::Quaternion<f32>>,
    pub scales: Vec<cgmath::Vector3<f32>>,
}

impl Node {

    pub fn new_root() -> Node {

        let children: Vec<Node> = Vec::new();
        let id: u32 = 0;
        let translation = cgmath::Vector3::new(0.0, 0.0, 0.0);
        let rotation = cgmath::Quaternion::new(0.0, 0.0, 0.0, 0.0);
        let scale = cgmath::Vector3::new(0.0, 0.0, 0.0);
        let matrix = cgmath::Matrix4::identity();
        let translations: Vec<cgmath::Vector3<f32>> = Vec::new();
        let rotations: Vec<cgmath::Quaternion<f32>> = Vec::new();
        let scales: Vec<cgmath::Vector3<f32>> = Vec::new();

        Node { children, id, translation, rotation, scale, matrix, updated_matrix: matrix, translations, rotations, scales }
    }

    pub fn new(node: &gltf::Node) -> Node {

        let mut children: Vec<Node> = Vec::new();

        let id = node.index() as u32;

        let (t, r, s) = node.transform().decomposed();

        let translation: cgmath::Vector3<f32> = t.into();
        let rotation: cgmath::Quaternion<f32> = r.into();
        let scale: cgmath::Vector3<f32> = s.into();

        let matrix: cgmath::Matrix4<f32> = node.transform().matrix().into();

        for child in node.children() {

            children.push(Node::new(&child));
        }

        let translations: Vec<cgmath::Vector3<f32>> = Vec::new();
        let rotations: Vec<cgmath::Quaternion<f32>> = Vec::new();
        let scales: Vec<cgmath::Vector3<f32>> = Vec::new();

        Node { children, id, translation, rotation, scale, matrix, updated_matrix: matrix, translations, rotations, scales }
    }

    pub fn treverse(&mut self) {

        for child in &mut self.children {
            child.update_matrix(self.matrix);
        }
    }

    pub fn update_matrix(&mut self, mat: cgmath::Matrix4<f32>) {

        self.updated_matrix = self.matrix * mat;

        for child in &mut self.children {
            child.update_matrix(self.updated_matrix);
        }
    }

    pub fn get_bind_pose(&mut self, joints: &mut Vec<cgmath::Matrix4<f32>>) {

        joints.push(self.matrix);
        for child in &mut self.children {
            child.get_bind_pose_child(self.matrix, joints);
        }
    }

    pub fn get_bind_pose_child(&mut self, mat: cgmath::Matrix4<f32>, joints: &mut Vec<cgmath::Matrix4<f32>>) {

        self.updated_matrix = mat * self.matrix;

        joints.push(self.updated_matrix);

        for child in &mut self.children {
            child.get_bind_pose_child(self.updated_matrix, joints);
        }
    }

    pub fn get_animation_pose(&mut self, joints: &mut Vec<cgmath::Matrix4<f32>>, frame: f32) {

        let t_s = self.translations[frame.floor() as usize].lerp(self.translations[frame.ceil() as usize], 1.0 - (frame.ceil() - frame));
        let r_s = self.rotations[frame.floor() as usize].nlerp(self.rotations[frame.ceil() as usize], 1.0 - (frame.ceil() - frame));
        let s_s = self.scales[frame.floor() as usize].lerp(self.scales[frame.ceil() as usize], 1.0 - (frame.ceil() - frame));

        let rm_s: cgmath::Matrix4<f32> = r_s.into();

        let mat_transform: cgmath::Matrix4<f32> = cgmath::Matrix4::from_translation(t_s) * rm_s * cgmath::Matrix4::from_nonuniform_scale(s_s.x, s_s.y, s_s.z);

        joints.push(mat_transform);
        for child in &mut self.children {
            child.get_animation_pose_child(mat_transform, joints, frame);
        }
    }

    pub fn get_animation_pose_child(&mut self, mat: cgmath::Matrix4<f32>, joints: &mut Vec<cgmath::Matrix4<f32>>, frame: f32) {

        let t_s = self.translations[frame.floor() as usize].lerp(self.translations[frame.ceil() as usize], 1.0 - (frame.ceil() - frame));
        let r_s = self.rotations[frame.floor() as usize].nlerp(self.rotations[frame.ceil() as usize], 1.0 - (frame.ceil() - frame));
        let s_s = self.scales[frame.floor() as usize].lerp(self.scales[frame.ceil() as usize], 1.0 - (frame.ceil() - frame));

        let rm_s: cgmath::Matrix4<f32> = r_s.into();

        let mat_transform: cgmath::Matrix4<f32> = cgmath::Matrix4::from_translation(t_s) * rm_s * cgmath::Matrix4::from_nonuniform_scale(s_s.x, s_s.y, s_s.z);

        self.updated_matrix = mat * mat_transform;

        joints.push(self.updated_matrix);

        for child in &mut self.children {
            child.get_animation_pose_child(self.updated_matrix, joints, frame);
        }
    }

    pub fn get_node(&mut self, id: u32) -> Option<&mut Node> {

        if id == self.id {
            return Some(self);
        }
        else {
            for child in &mut self.children {
                let result = child.get_node(id);
                if result.is_some() {
                    return result;
                }
            }
        }

        None
    }
}

pub struct Model {
    pub vertex_buffer: wgpu::Buffer,
    pub index_buffer: wgpu::Buffer,
    pub num: u32,
    pub inverse_bind_matrices: Vec<[[f32; 4]; 4]>,
    pub root_skeleton: Node,
    pub time_scale: usize,
    pub material: texture::Material,
    pub has_animation: bool,
    pub time: f32,
    pub joints: uniform::JointUniforms,
    pub joints_uniform_buffer: wgpu::Buffer,
    pub joints_uniform_bind_group: wgpu::BindGroup,
    pub verts: Option<Vec<[f32; 3]>>,
    pub inds: Option<Vec<u32>>,
}

impl Model {
    pub fn new(device: &wgpu::Device, queue: &wgpu::Queue, layout: &wgpu::BindGroupLayout, path: &str, load_collision: bool) -> Model {

        let mut true_path = "./res/".to_string() + path;
        let mut model_path = true_path.clone() + ".glb";
        let mut tex_path = true_path + ".png";

        let (document, buffers, images) = gltf::import(model_path).unwrap();

        let mut vertices_position: Vec<[f32; 3]> = Vec::new();
        let mut indices: Vec<u32> = Vec::new();
        let mut normals: Vec<[f32; 3]> = Vec::new();
        let mut tex_coords: Vec<[f32; 2]> = Vec::new();
        let mut weights: Vec<[f32; 4]> = Vec::new();
        let mut joints: Vec<[i32; 4]> = Vec::new();

        for mesh in document.meshes() {

            for prim in mesh.primitives() {

                let reader = prim.reader(|buffer| Some(&buffers[buffer.index()]));

                let offset = vertices_position.len() as u32;

                if let Some(iter) = reader.read_positions() {
                    for vertex_position in iter {
                        vertices_position.push(vertex_position);
                    }
                }

                if let Some(iter) = reader.read_indices() {

                    for indice in iter.into_u32() {
                        indices.push(indice + offset);
                    }
                }

                if let Some(iter) = reader.read_normals() {
                    for normal in iter {
                        normals.push(normal);
                    }
                }

                if let Some(iter) = reader.read_tex_coords(0) {
                    for tex_coord in iter.into_f32() {
                        tex_coords.push(tex_coord);
                    }
                }

                if let Some(iter) = reader.read_weights(0) {
                    for weight in iter.into_f32() {
                        weights.push(weight);
                    }
                }

                if let Some(iter) = reader.read_joints(0) {
                    for joint in iter.into_u16() {
                        joints.push([joint[0] as i32, joint[1] as i32, joint[2] as i32, joint[3] as i32]);
                    }
                }
            }
        }

        let mut vertices: Vec<Vertex> = Vec::new();

        let mut has_animation = false;

        for i in 0..vertices_position.len() {
            if weights.len() <= 0 {
                vertices.push(Vertex { pos: vertices_position[i], normal: normals[i], tex_coord: tex_coords[i], weight: [1.0, 1.0, 1.0, 1.0], joint: [0, 0, 0, 0] });
            }
            else {
                has_animation = true;
                vertices.push(Vertex { pos: vertices_position[i], normal: normals[i], tex_coord: tex_coords[i], weight: weights[i], joint: joints[i] });
            }
        }

        let vertex_buffer = device.create_buffer_init(
            &wgpu::util::BufferInitDescriptor {
                label: Some("Vertex Buffer"),
                contents: bytemuck::cast_slice(&vertices),
                usage: wgpu::BufferUsage::VERTEX,
            }
        );

        let index_buffer = device.create_buffer_init(
            &wgpu::util::BufferInitDescriptor {
                label: Some("Index Buffer"),
                contents: bytemuck::cast_slice(&indices),
                usage: wgpu::BufferUsage::INDEX,
            }
        );

        let mut inverse_bind_matrices: Vec<[[f32; 4]; 4]> = Vec::new();
        let mut root_skeleton = Node::new_root();

        for skin in document.skins() {

            let reader = skin.reader(|buffer| Some(&buffers[buffer.index()]));

            if let Some(iter) = reader.read_inverse_bind_matrices() {
                for inverse_bind_matrix in iter {
                    inverse_bind_matrices.push(inverse_bind_matrix);
                }
            }

            let first_node = skin.joints().next().unwrap();

            root_skeleton = Node::new(&first_node);
            
            root_skeleton.treverse();
        }

        let mut time_scale: usize = 0;

        for animation in document.animations() {

            for channel in animation.channels() {

                let mut node = root_skeleton.get_node(channel.target().node().index() as u32).unwrap();

                let reader = channel.reader(|buffer| Some(&buffers[buffer.index()]));

                if let Some(gltf::animation::util::ReadOutputs::Translations(iter)) = reader.read_outputs() {
                    for translation in iter {
                        node.translations.push(translation.into());
                    }
                }

                if let Some(gltf::animation::util::ReadOutputs::Rotations(iter)) = reader.read_outputs() {
                    for rotation in iter.into_f32() {
                        node.rotations.push(rotation.into());
                    }
                }

                if let Some(gltf::animation::util::ReadOutputs::Scales(iter)) = reader.read_outputs() {
                    for scale in iter {
                        node.scales.push(scale.into());
                    }
                }

                time_scale = node.scales.len();
            }
        }

        let material = texture::Material::new(device, queue, layout, &tex_path);

        let mut joints = uniform::JointUniforms::new();

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

        if load_collision {
            return Model { vertex_buffer, index_buffer, num: indices.len() as u32, inverse_bind_matrices, root_skeleton, time_scale, material, has_animation, time: 0.0, joints, joints_uniform_buffer, joints_uniform_bind_group, verts: Some(vertices_position), inds: Some(indices), };
        }

        Model { vertex_buffer, index_buffer, num: indices.len() as u32, inverse_bind_matrices, root_skeleton, time_scale, material, has_animation, time: 0.0, joints, joints_uniform_buffer, joints_uniform_bind_group, verts: None, inds: None, }
    }

    pub fn update_skeleton(&mut self, delta: f32, queue: &wgpu::Queue) {

        if self.has_animation {

            self.time += (delta * 3000.0);

            let usef = self.time_scale as f32;

            if self.time.ceil() >= usef {
                self.time = 0.0;
            }

            let mut joints_temp: Vec<cgmath::Matrix4<f32>> = Vec::new();

            self.root_skeleton.get_animation_pose(&mut joints_temp, self.time);

            for i in 0..self.inverse_bind_matrices.len() {

                let inv_mat: cgmath::Matrix4<f32> = self.inverse_bind_matrices[i].into();

                self.joints.joints[i] = (joints_temp[i] * inv_mat).into();
            }

            queue.write_buffer(&self.joints_uniform_buffer, 0, bytemuck::cast_slice(&[self.joints]));
        }
    }

    pub fn update_model(&mut self, queue: &wgpu::Queue) {
    
        queue.write_buffer(&self.joints_uniform_buffer, 0, bytemuck::cast_slice(&[self.joints]));
    }
}

pub trait DrawModel<'a, 'b>
where
    'b: 'a,
{
    fn draw_model(&mut self, model: &'b Model);
    fn draw_model_debug(&mut self, model: &'b Model, bi: &'b wgpu::BindGroup);
}
impl<'a, 'b> DrawModel<'a, 'b> for wgpu::RenderPass<'a>
where
    'b: 'a,
{

    fn draw_model(
        &mut self,
        model: &'b Model,
    ) {

        self.set_bind_group(1, &model.joints_uniform_bind_group, &[]);
        self.set_bind_group(2, &model.material.bind_group, &[]);
        self.set_vertex_buffer(0, model.vertex_buffer.slice(..));
        self.set_index_buffer(model.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
        self.draw_indexed(0..model.num, 0, 0..1);
    }

    fn draw_model_debug(
        &mut self,
        model: &'b Model,
        bi: &'b wgpu::BindGroup,
    ) {

        self.set_bind_group(1, bi, &[]);
        self.set_bind_group(2, &model.material.bind_group, &[]);
        self.set_vertex_buffer(0, model.vertex_buffer.slice(..));
        self.set_index_buffer(model.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
        self.draw_indexed(0..model.num, 0, 0..1);
    }
}