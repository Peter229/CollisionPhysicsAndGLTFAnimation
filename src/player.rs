use crate::collision;
use crate::model;
use crate::level;

pub struct Player {
    pub capsule: collision::Capsule,
    pub position: cgmath::Vector3<f32>,
    pub velocity: cgmath::Vector3<f32>,
}

impl Player {

    pub fn new(position: cgmath::Vector3<f32>) -> Player {

        let mut capsule = collision::Capsule::new(cgmath::Vector3::new(0.0, 1.0, 4.0), cgmath::Vector3::new(0.0, 5.0, 4.0), 1.0);

        capsule.base = position;
        capsule.base.y -= 2.0;

        capsule.tip = position;
        capsule.tip.y += 2.0;

        Player { capsule, position, velocity: cgmath::Vector3::new(0.0, 0.0, 0.0) }
    }

    pub fn update(&mut self, level: &level::Level) {

        self.capsule.change_position_vec(self.velocity);

        for i in 0..level.collision_meshes.len() {
            if level.collision_meshes[i].moving {
                //collision::clean_collide_triangle_mesh_moving(&level.collision_meshes[i], &mut self.capsule, &self.velocity);
            }
            else {
                //collision::clean_collide_triangle_mesh(&level.collision_meshes[i].object, &mut self.capsule, &self.velocity, false, None);
            }
            self.capsule.ground_test(&level.collision_meshes[i].object, -0.1, None);
        }

        self.velocity = cgmath::Vector3::new(0.0, self.velocity.y, 0.0);
    }
}