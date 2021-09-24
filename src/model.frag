#version 450

layout(location = 0) out vec4 f_color;

layout(location = 0) in vec3 v_normal;
layout(location = 1) in vec2 v_tex_coord;

layout(set = 2, binding = 0) uniform texture2D t_diffuse;
layout(set = 2, binding = 1) uniform sampler s_diffuse;

void main() {

    vec3 light_pos = vec3(2.0, 5.0, 10.0);
    vec3 light_end_pos = vec3(0.0, 0.0, 0.0);
    vec3 light_colour = vec3(0.8, 0.8, 0.8);

    vec3 norm = normalize(v_normal);
    vec3 light_dir = normalize(light_pos - light_end_pos);
    float diff = max(dot(norm, light_dir), 0.0);

    if (diff > 0.3) {
        diff = 0.87;
    }
    else if (diff >= 0.0) {
        diff = 0.1;
    }

    vec3 diffuse = diff * light_colour;

    //float specularStrength = 0.5;
    //vec3 view_dir = normalize(view_pos - FragPos);
    //vec3 reflect_dir = reflect(-light_dir, norm);  
    //float spec = pow(max(dot(view_dir, reflect_dir), 0.0), 32);
    //vec3 specular = specularStrength * spec * light_colour;  
        
    //vec3 result = (ambient + diffuse + specular) * objectColor;


    f_color = texture(sampler2D(t_diffuse, s_diffuse), v_tex_coord) * vec4(diffuse, 1.0);
}