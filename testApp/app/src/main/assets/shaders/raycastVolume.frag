#version 300 es

in vec3 frag_position; // in object space
in vec3 tex_coord;
in vec3 ray_dir;
in mat3 NormalMatrix;

uniform sampler3D uSampler_tex;
uniform sampler2D uSampler_trans;

uniform float sample_step_inverse;      // step used to advance the sampling ray
uniform float val_threshold;
uniform float brightness;
out vec4 gl_FragColor;

struct LightInfo{
    vec3 Ia;
    vec3 Id;
    vec3 Is;
};
uniform LightInfo Light;
uniform float shiness;

uniform bool u_use_color_transfer;
uniform bool u_use_ligting;

vec3 phong_illumination_model(vec3 N,vec3 curColor,vec3 curPos){
    //compute ambient
    //                  Ka  *  Ia
    vec3 ambient = curColor * Light.Ia;
    //compute diffuse
    vec3 L = normalize(vec3(0,0,-1));
    //               Kd     *    Id    *  N dot L
    vec3 diffuse = curColor * Light.Id * max(0.0,dot(N,L));
    //compute specular
    vec3 R = reflect(L,N);
    vec3 E = normalize(-curPos);
    //                    Ks          *    Is    * (R dot E)^shiness
    vec3 specular = vec3(1.0,1.0,1.0) * Light.Is * pow(max(0.0,dot(R,E)),shiness);

    return (ambient + diffuse + specular);
}


void main(void)
{
    // float intensity = texture(uSampler_tex, tex_coord).r;
    // gl_FragColor = vec4(intensity, intensity, intensity, 1.0);
  float sample_step = 1.0/sample_step_inverse;
  vec3 ray_pos = tex_coord; // the current ray position
  vec3 pos111 = vec3(1.0, 1.0, 1.0);
  vec3 pos000 = vec3(0.0, 0.0, 0.0);

  vec4 frag_color = vec4(0,0,0,0);
  vec4 color;
  float max_density = -1.0;
  do
  {
    // note:
    // - ray_dir * sample_step can be precomputed
    // - we assume the volume has a cube-like shape

    ray_pos += ray_dir * sample_step;

    // break out if ray reached the end of the cube.
    if (any(greaterThan(ray_pos,pos111)))
      break;

    if (any(lessThan(ray_pos,pos000)))
      break;

    float density = texture(uSampler_tex, ray_pos).r;
    max_density = max(max_density, density);
    if(max_density == density){
        density += val_threshold - 0.5;
        density = density * density * density;

        vec3 normal = normalize(NormalMatrix*(normalize(ray_pos)));
        vec3 sampled_color;
        if(u_use_color_transfer == true)
            sampled_color = texture(uSampler_trans, vec2(density, 1.0)).rgb;
        else
            sampled_color = vec3(density);

        if(u_use_ligting == true)
            color.rgb = phong_illumination_model(normal, sampled_color, frag_position);
        else
            color.rgb = sampled_color;
        color.a   = density * sample_step * brightness;
        frag_color.rgb = frag_color.rgb * (1.0 - color.a) + color.rgb * color.a;
    }
  }
  while(true);

  if(frag_color.r == 0.0)
    discard;
  else
    gl_FragColor = vec4(frag_color.rgb,1.0);
}