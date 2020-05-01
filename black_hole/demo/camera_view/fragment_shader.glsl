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

// The camera position, in Schwarzschild coordinates (p^t, p^r, p^theta, p^phi).
uniform vec4 camera_position;
// The camera position, in (pseudo-)Cartesian coordinates.
uniform vec3 p;
// The camera 4-velocity, in Schwarzschild coordinates.
uniform vec4 k_s;
// The base vectors of the camera reference frame, in (pseudo-)Cartesian
// coordinates.
uniform vec3 e_tau, e_w, e_h, e_d;

uniform sampler2D ray_deflection_texture;
uniform sampler2D ray_inverse_radius_texture;

uniform samplerCube galaxy_cube_texture;
uniform samplerCube star_cube_texture;
uniform samplerCube star_cube_texture2;
uniform mat3 stars_orientation;
uniform float min_stars_lod;

uniform sampler2D black_body_texture;
uniform highp sampler3D doppler_texture;
uniform vec3 disc_params;

in vec3 view_dir;

layout(location = 0) out vec4 frag_color;

// Simple ray-tracing function to compute the intersections of a ray with the
// scene, when the space-time geometry is assumed to be flat.
float TraceRayEuclidean(float p_r, float delta, float alpha, float u_min,
                        float u_max, out float u0, out float phi0, out float t0,
                        out float u1, out float phi1, out float t1) {
  float cos_delta = cos(delta);
  float sin_delta = sin(delta);
  float tan_alpha = tan(alpha);
  float det = 1.0 - p_r * p_r * sin_delta * sin_delta;
  float deflection = det > 0.0 && cos_delta < 0.0 ? -1.0 : 0.0;
  u0 = -1.0;
  u1 = -1.0;
  float t = p_r / (sin_delta / tan_alpha - cos_delta);
  float r = length(vec2(p_r + t * cos_delta, t * sin_delta));
  if (t >= 0.0 && r * u_min <= 1.0 && r * u_max >= 1.0 &&
      (deflection == 0.0 || t < p_r)) {
    u0 = 1.0 / r;
    phi0 = alpha;
    t0 = t;
  }
  return deflection;
}

float RayTrace(float u, float u_prime, float e_square, float delta, float alpha,
               float u_min, float u_max, out float u0, out float phi0,
               out float t0, out float alpha0, out float u1, out float phi1,
               out float t1, out float alpha1) {
#if (LENSING == 1)
  return TraceRay(ray_deflection_texture, ray_inverse_radius_texture, u,
                  u_prime, e_square, delta, alpha, u_min, u_max, u0, phi0, t0,
                  alpha0, u1, phi1, t1, alpha1);
#else
  alpha0 = 1.0;
  alpha1 = 1.0;
  return TraceRayEuclidean(1.0 / u, delta, alpha, u_min, u_max, u0, phi0, t0,
                           u1, phi1, t1);
#endif
}

vec3 GalaxyColor(vec3 dir) {
  dir = stars_orientation * dir;
#if (GRID == 1)
  return texture(galaxy_cube_texture, dir).rrr;
#else
  return texture(galaxy_cube_texture, dir).rgb * 6.78494e-5;
#endif
}

vec3 StarTextureColor(vec3 dir) {
#if (GRID == 1)
  return vec3(0.8);
#else
  return texture(star_cube_texture2, dir).rgb;
#endif
}

vec3 StarTextureColor(vec3 dir, float lod, out vec2 sub_position) {
#if (GRID == 1)
  sub_position = vec2(0.0);
  return vec3(100.0);
#else
  vec3 color = textureLod(star_cube_texture, dir, lod).rgb;
  ivec2 bits = floatBitsToInt(color.rb);
  sub_position = vec2((bits >> 8) % 257) / 257.0 - vec2(0.5);
  return color;
#endif
}

vec3 StarColor(vec3 dir, float lensing_amplification_factor) {
#if (STARS == 1)
  dir = stars_orientation * dir;
  return DefaultStarColor(dir, lensing_amplification_factor, min_stars_lod);
#else
  return vec3(0.0);
#endif
}

vec3 Doppler(vec3 rgb, float doppler_factor) {
#if (DOPPLER == 1)
  return DefaultDoppler(doppler_texture, rgb, doppler_factor);
#else
  return rgb;
#endif
}

vec4 GridDiscColor(vec2 p, float t, bool top_side, float doppler_factor,
                   float temperature, sampler2D black_body_texture) {
  float p_r = length(p);
  if (p_r <= INNER_DISC_R || p_r >= OUTER_DISC_R) {
    return vec4(0.0);
  }
  const float uM = 1.0 / 6.0;
  const float dphi_dt = uM * sqrt(0.5 * uM) / (2.0 * pi);
  float p_phi = atan(p.y, p.x) - t * dphi_dt;
  float value_phi = mod(p_phi / pi * 16.0, 1.0) < 0.2 ? 0.0 : 1.0;
  float value_r = mod(p_r / 2.0, 1.0) < 0.2 ? 0.0 : 1.0;
  vec3 color = BlackBodyColor(black_body_texture, temperature * doppler_factor);
  color *= 0.2 + 0.8 * value_phi * value_r;
  return vec4(top_side ? color : color * 0.01, 1.0);
}

vec4 DiscColor(vec2 p, float t, bool top_side, float doppler_factor) {
  float density = disc_params.x;
  float opacity = disc_params.y;
  float temperature = disc_params.z;
#if (DOPPLER == 0)
  doppler_factor = 1.0;
#endif
#if (GRID == 1)
  vec4 color = GridDiscColor(p, t, top_side, doppler_factor, temperature,
                             black_body_texture);
#else
  vec4 color = DefaultDiscColor(p, t, top_side, doppler_factor, temperature,
                                black_body_texture);
#endif
  return vec4(density * color.rgb, opacity * color.a);
}

void main() {
  frag_color.rgb =
      SceneColor(camera_position, p, k_s, e_tau, e_w, e_h, e_d, view_dir);
  frag_color.a = 1.0;
}
