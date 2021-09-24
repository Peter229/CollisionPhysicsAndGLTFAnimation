#version 450

layout(location = 0) in vec3 a_position;
layout(location = 1) in vec3 a_normal;
layout(location = 2) in vec2 a_tex_coord;
layout(location = 3) in vec4 a_weight;
layout(location = 4) in ivec4 a_joint;

layout(set = 0, binding = 0)
uniform Uniforms {
    mat4 proj;
    mat4 view;
    float time;
};

layout(set = 1, binding = 0)
uniform JointsUniforms {
    mat4 joints[64];
    mat4 model;
};

layout(location = 0) out vec3 v_normal;
layout(location = 1) out vec2 v_tex_coord;

void main() {
    
    mat4 skin_mat = a_weight.x * joints[a_joint.x] +
                    a_weight.y * joints[a_joint.y] +
                    a_weight.z * joints[a_joint.z] +
                    a_weight.w * joints[a_joint.w];


    v_normal = (model * skin_mat * vec4(a_normal, 0.0)).xyz;
    v_tex_coord = a_tex_coord;

    gl_Position = proj * view * model * skin_mat * vec4(a_position, 1.0);
}