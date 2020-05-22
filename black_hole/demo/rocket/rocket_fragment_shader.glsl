/**
 * Copyright (c) 2020 Eric Bruneton
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

const float PI = 3.141592653589793;

// First few terms of the Van Der Corput sequence.
const float VAN_DER_CORPUT[32] = float[32](
  0.00000, 0.50000, 0.25000, 0.75000, 0.12500, 0.62500, 0.37500, 0.87500,
  0.06250, 0.56250, 0.31250, 0.81250, 0.18750, 0.68750, 0.43750, 0.93750,
  0.03125, 0.53125, 0.28125, 0.78125, 0.15625, 0.65625, 0.40625, 0.90625,
  0.09375, 0.59375, 0.34375, 0.84375, 0.21875, 0.71875, 0.46875, 0.96875
);

uniform vec3 camera;
uniform sampler2D base_color_texture;
uniform sampler2D occlusion_roughness_metallic_texture;
uniform sampler2D normal_map_texture;
uniform samplerCube env_map_texture;

in vec3 position;           // position, in object space.
in vec3 normal;             // geometric normal, in object space.
in vec3 tangent;            // geometric tangent, in object space.
in vec2 uv;
in float ambient_occlusion; // 0 = full occlusion, 1 = no occlusion.

layout(location = 0) out vec4 frag_color;

// The following uses the notations and equations in Appendix B of
// https://github.com/KhronosGroup/glTF/tree/master/specification/2.0

struct Surface {
  vec3 n;              // perturbed normal, in object space.
  vec3 tx;             // perturbed tangent, in object space.
  vec3 ty;             // perturbed bitangent, in object space.
  float occlusion;     // 0 = full occlusion, 1 = no occlusion.
  float alpha_sq;      // fourth power of 'roughness'.
  vec3 albedo;         // surface diffuse albedo.
  vec3 f0;             // surface reflection ratio.
};

// Returns the perturbed normal in object space.
vec3 ComputeNormal() {
  // Perturbed normal, in geometric tangent space.
  vec3 n = texture(normal_map_texture, uv).xyz * 2.0 - vec3(1.0);
  // Basis vectors of the geometric tangent space, in object space.
  vec3 ez = normalize(normal);
  vec3 ex = normalize(tangent);
  vec3 ey = cross(ez, ex);
  return normalize(n.x * ex + n.y * ey + n.z * ez);
}

// Returns the parameters of the surface.
Surface ComputeSurface() {
  Surface surface;
  surface.n = ComputeNormal();
  surface.ty = normalize(cross(surface.n, tangent));
  surface.tx = cross(surface.ty, surface.n);

  vec3 occlusion_roughness_metallic =
      texture(occlusion_roughness_metallic_texture, uv).rgb;
  surface.occlusion = occlusion_roughness_metallic.r * ambient_occlusion;
  float roughness = occlusion_roughness_metallic.g;
  float metallic = occlusion_roughness_metallic.b;
  float alpha = roughness * roughness;
  surface.alpha_sq = alpha * alpha;

  const float DIELECTRIC_F0 = 0.04;
  const vec3 METAL_ALBEDO = vec3(0.0);
  vec3 color = texture(base_color_texture, uv).rgb;
  surface.albedo = mix(color * (1.0 - DIELECTRIC_F0), METAL_ALBEDO, metallic);
  surface.f0 = mix(vec3(DIELECTRIC_F0), color, metallic);
  
  return surface;
}

// Returns the object space coordinates of a vector specified by its spherical
// coordinates in the perturbed tangent space of the surface.
vec3 GetVector(Surface surface, float cos_theta, float sin_theta, float phi) {
  float vx = sin_theta * cos(phi);
  float vy = sin_theta * sin(phi);
  float vz = cos_theta;
  return vx * surface.tx + vy * surface.ty + vz * surface.n;
}

// Surface reflection ratio F.
vec3 Fresnel(vec3 f0, float v_dot_h) {
  return f0 + (vec3(1.0) - f0) * pow(1.0 - v_dot_h, 5.0);
}

// Geometric occlusion term G divided by 4 (n.l) (n.v).
float MicroFacetVisibility(float alpha_sq, float n_dot_l, float n_dot_v) {
  float a = n_dot_l * sqrt(n_dot_v * n_dot_v * (1.0 - alpha_sq) + alpha_sq);
  float b = n_dot_v * sqrt(n_dot_l * n_dot_l * (1.0 - alpha_sq) + alpha_sq);
  return 0.5 / (a + b);
}

// Microfacet distribution D.
float MicroFacetDistribution(float alpha_sq, float n_dot_h) {
  float a = n_dot_h * n_dot_h * (alpha_sq - 1.0) + 1.0;
  return alpha_sq / (PI * a * a);
}

vec3 ImageBasedLighting(Surface surface, vec3 v) {
  // Compute the lighting integral with N_Z * N_PHI samples of the light 
  // direction l, all having the same solid angle OMEGA_SAMPLE. This is used for
  // the diffuse component, and for the specular component when the roughness is
  // "large".
  const int N_Z = 3;
  const int N_PHI = 8;
  const float OMEGA_SAMPLE = 2.0 * PI / float(N_Z * N_PHI);
  const float OMEGA_TEXEL = 4.0 * PI / (6.0 * ENV_MAP_SIZE * ENV_MAP_SIZE);
  const float LOD = 0.5 * log2(OMEGA_SAMPLE / OMEGA_TEXEL);
  float n_dot_v = clamp(dot(surface.n, v), 0.0, 1.0);
  vec3 diffuse = vec3(0.0);
  vec3 specular = vec3(0.0);
  for (int i = 0; i < N_Z; ++i) {
    float cos_theta = (float(i) + 0.5) / float(N_Z);
    float sin_theta = sqrt(1.0 - cos_theta * cos_theta);
    for (int j = 0; j < N_PHI; ++j) {
      float phi = float(j) * (2.0 * PI / float(N_PHI));
      vec3 l = GetVector(surface, cos_theta, sin_theta, phi);
      vec3 L = textureLod(env_map_texture, l, LOD).rgb;

      vec3 h = normalize(v + l);
      float n_dot_l = cos_theta;
      float n_dot_h = clamp(dot(surface.n, h), 0.0, 1.0);
      float v_dot_h = clamp(dot(v, h), 0.0, 1.0);
      vec3 F = Fresnel(surface.f0, v_dot_h);
      float V = MicroFacetVisibility(surface.alpha_sq, n_dot_l, n_dot_v);
      float D = MicroFacetDistribution(surface.alpha_sq, n_dot_h);

      // Note: the 1/pi factor is moved outside the loop, at the end.
      diffuse += surface.albedo * L * (vec3(1.0) - F) * n_dot_l;
      specular += L * F * (V * D * n_dot_l);
    }
  }
  diffuse *= OMEGA_SAMPLE / PI;
  specular *= OMEGA_SAMPLE;

  // Compute the specular component with importance sampling, when the roughness
  // is "small". References:
  // - https://blog.selfshadow.com/publications/s2013-shading-course/karis/
  //     s2013_pbs_epic_notes_v2.pdf
  // - https://placeholderart.wordpress.com/2015/07/28/implementation-notes-
  //     runtime-environment-map-filtering-for-image-based-lighting/
  // - https://github.com/KhronosGroup/glTF-IBL-Sampler/blob/master/lib/
  //     shaders/filter.frag
  const int SAMPLE_COUNT = 24;
  vec3 importance_specular = vec3(0.0);
  for (int i = 0; i < SAMPLE_COUNT; ++i) {
    float vdc = VAN_DER_CORPUT[i];
    float z_sq = (1.0 - vdc) / ((surface.alpha_sq - 1.0) * vdc + 1.0);
    float cos_theta = sqrt(z_sq);
    float sin_theta = sqrt(1.0 - z_sq);
    float phi = float(i) * (2.0 * PI / float(SAMPLE_COUNT));
    vec3 h = GetVector(surface, cos_theta, sin_theta, phi);

    vec3 l = reflect(-v, h);
    float n_dot_l = dot(surface.n, l);
    if (n_dot_l <= 0.0) continue;

    float n_dot_h = clamp(dot(surface.n, h), 0.0, 1.0);
    float v_dot_h = clamp(dot(v, h), 0.0, 1.0);
    vec3 F = Fresnel(surface.f0, v_dot_h);
    float V = MicroFacetVisibility(surface.alpha_sq, n_dot_l, n_dot_v);
    float D = MicroFacetDistribution(surface.alpha_sq, n_dot_h);

    float pdf = D * n_dot_h / (4.0 * v_dot_h);
    float omega_sample_inverse = pdf * float(SAMPLE_COUNT);
    float lod = -0.5 * log2(omega_sample_inverse * OMEGA_TEXEL);
    vec3 L = textureLod(env_map_texture, l, lod).rgb;

    importance_specular += L * F * (V * D * n_dot_l / pdf);
  }
  importance_specular *= (1.0 / float(SAMPLE_COUNT));

  return diffuse + (surface.alpha_sq < 0.0625 ? importance_specular : specular);
}

void main() {
  Surface surface = ComputeSurface();
  vec3 v = normalize(camera - position);
  frag_color = vec4(ImageBasedLighting(surface, v) * surface.occlusion, 1.0);
}
