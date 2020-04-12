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

#ifdef VERTEX_SHADER

// -----------------------------------------------------------------------------
// Ray initialization.
// -----------------------------------------------------------------------------

// Initializes a ray from the camera position and from pixel coordinates.
// Inputs:
// - camera_position: the camera position, in Schwarzschild coordinates
//   (p^t, p^r, p^theta,p^phi).
// - camera_orientation: the Lorentz transformation specifying the camera
//   orientation and velocity.
// - camera_size: the half width, half height and focal length of the camera.
// - vertex: the normalized vertex position on screen, in [-1,1]x[-1,1].
//
// Outputs:
// - p: the camera position, in (pseudo-)Cartesian coordinates.
// - k_s: the camera 4-velocity, in Schwarzschild coordinates.
// - k: the camera velocity, in (pseudo-)Cartesian coordinates.
// - d: the view ray direction, in (pseudo-)Cartesian coordinates.
void InitRay(vec4 camera_position, mat4 camera_orientation, vec3 camera_size,
             vec2 vertex, out vec3 p, out vec4 k_s, out vec3 k, out vec3 d) {
  float r = camera_position[1];
  float cos_theta = cos(camera_position[2]);
  float sin_theta = sin(camera_position[2]);
  float cos_phi = cos(camera_position[3]);
  float sin_phi = sin(camera_position[3]);

  float u = 1.0 / r;
  float v = sqrt(1.0 - u);
  vec4 ur = vec4(0.0, sin_theta * cos_phi, sin_theta * sin_phi, cos_theta);

  vec4 e_t = vec4(1.0 / v, 0.0, 0.0, 0.0);
  vec4 e_r = v * ur;
  vec4 e_theta = 
      vec4(0.0, cos_theta * cos_phi, cos_theta * sin_phi, -sin_theta);
  vec4 e_phi = vec4(0.0, -sin_phi, cos_phi, 0.0);

  mat4 L = camera_orientation;
  vec4 e_tau = 
      L[0][0] * e_t + L[1][0] * e_r + L[2][0] * e_theta + L[3][0] * e_phi;
  vec4 e_w = 
      L[0][1] * e_t + L[1][1] * e_r + L[2][1] * e_theta + L[3][1] * e_phi;
  vec4 e_h = 
      L[0][2] * e_t + L[1][2] * e_r + L[2][2] * e_theta + L[3][2] * e_phi;
  vec4 e_d = 
      L[0][3] * e_t + L[1][3] * e_r + L[2][3] * e_theta + L[3][3] * e_phi;

  vec3 q = vec3(vertex * camera_size.xy, -camera_size.z);

  p = r * ur.yzw;
  k_s = vec4(L[0][0] / v, v * L[1][0], u * L[2][0], u / sin_theta * L[3][0]);
  k = e_tau.yzw;
  d = (q.x * e_w + q.y * e_h + q.z * e_d).yzw;
}

#else

// -----------------------------------------------------------------------------
// Shading.
// -----------------------------------------------------------------------------

// Returns the color of a black body of the given temperature. The 1D texture
// should contain this color at texture coord log(T / 100) / 6.
vec3 BlackBodyColor(sampler2D black_body_texture, float temperature) {
  float tex_u = (1.0 / 6.0) * log(temperature * (1.0 / 100.0));
  return texture(black_body_texture, vec2(tex_u, 0.5)).rgb;
}

// Abstract functions, which must be implemented by the user:
// - ray tracing function (see the default implementation in functions.glsl).
Angle RayTrace(Real p_r, Angle delta, Angle alpha, Real u_min, Real u_max, 
               out Real u0, out Angle phi0, out Real t0, 
               out Real u1, out Angle phi1, out Real t1);
// - Doppler function (see the default implementation below).
vec3 Doppler(vec3 rgb, float doppler_factor);
// - average color of the extended light sources (e.g. nebulae and galaxies) in
//   the footprint of the pixel in direction 'dir'.
vec3 GalaxyColor(vec3 dir);
// - average color of the punctual light sources (i.e. stars) in the footprint
//   of the pixel in direction 'dir'.
vec3 StarTextureColor(vec3 dir);
// - *sum* (in the footprint of the pixel in direction 'dir') of the colors of
//   the punctual light sources in the texel at 'lod' corresponding to 'dir', 
//   and sub-texel position (in [-0.5,0.5]^2).
vec3 StarTextureColor(vec3 dir, float lod, out vec2 sub_position);
// - color of the stars in the footprint of the pixel in direction 'dir', times
//   the given gravitational lensing amplification factor.
vec3 StarColor(vec3 dir, float lensing_amplification_factor);
// - color and opacity of the accretion disc at 'p', and at time 't', for the
//   top or bottom side of the disc, and with the given Doppler factor.
vec4 DiscColor(vec2 p, float t, bool top_side, float doppler_factor);

// Default implementation for 'Doppler'.
// Returns the given color when shifted by the given Doppler factor. The 3D
// texture should contain this color at texture coord (r, 2*g, d) where r, g is
// the rg chromaticity and d = atan(log(doppler_factor) / 0.21) / 3 + 0.5.
vec3 DefaultDoppler(highp sampler3D doppler_texture, vec3 rgb, 
    float doppler_factor) {
  float sum = rgb.r + rgb.g + rgb.b;
  if (sum == 0.0) {
    return vec3(0.0);
  }
  vec3 tex_coord;
  tex_coord.x = rgb.r / sum;
  tex_coord.y = 2.0 * rgb.g / sum;
  tex_coord.z = (1.0 / 3.0) * atan((1.0 / 0.21) * log(doppler_factor)) + 0.5;
  return sum * texture(doppler_texture, tex_coord).rgb;
}

// Default implementation for 'StarColor', which uses the two 'StarTextureColor'
// functions above, and assumes they are based on a cube map. The following
// constants must be provided by the user:
// - const float STARS_CUBE_MAP_SIZE = ...;
// - const float MAX_FOOTPRINT_SIZE = ...;
// - const float MAX_FOOTPRINT_LOD = ...;
// They define the size in pixels of the cube map, the maximum with and height
// of the footprint to consider around 'dir' (so the maximum number of texels
// used will be the square of this number), and the maximum LOD for which
// 'StarTextureColor(dir, lod, sub_position)' must be used (for larger LODs,
// 'StarTextureColor(dir)' is used instead).
vec3 DefaultStarColor(vec3 dir, float lensing_amplification_factor, 
    float min_lod) {
  // Compute the partial derivatives of dir (continuous across cube edges).
  vec3 dx_dir = dFdx(dir);
  vec3 dy_dir = dFdy(dir);

  // Swap the coordinates depending on the cube face, to always get the maximum
  // absolute value of the 'dir' components in the z coordinate.
  vec3 abs_dir = abs(dir);
  float max_abs_dir_comp = max(abs_dir.x, max(abs_dir.y, abs_dir.z));
  if (max_abs_dir_comp == abs_dir.x) {
    dir = dir.zyx;
    dx_dir = dx_dir.zyx;
    dy_dir = dy_dir.zyx;
  } else if (max_abs_dir_comp == abs_dir.y) {
    dir = dir.xzy;
    dx_dir = dx_dir.xzy;
    dy_dir = dy_dir.xzy;
  }

  // Compute the cube face texture coordinates uv and their derivatives dx_uv
  // and dy_uv (using an analytic formula instead of dFdx and dFdy, to avoid
  // discontinuities at cube edges - uv is not continuous here).
  float inv_dir_z = 1.0 / dir.z;
  vec2 uv = dir.xy * inv_dir_z;
  vec2 dx_uv = (dx_dir.xy - uv * dx_dir.z) * inv_dir_z;
  vec2 dy_uv = (dy_dir.xy - uv * dy_dir.z) * inv_dir_z;

  // Compute the LOD level to use to fetch the stars in the footprint of 'dir'.
  vec2 d_uv = max(abs(dx_uv + dy_uv), abs(dx_uv - dy_uv));
  vec2 fwidth = (0.5 * STARS_CUBE_MAP_SIZE / MAX_FOOTPRINT_SIZE) * d_uv;
  float lod = max(ceil(max(log2(fwidth.x), log2(fwidth.y))), min_lod);
  float lod_width = (0.5 * STARS_CUBE_MAP_SIZE) / pow(2.0, lod);
  if (lod > MAX_FOOTPRINT_LOD) {
    return StarTextureColor(dir);
  }

  // Fetch, filter and accumulate the colors of the stars in the texels in the
  // footprint of 'dir' at 'lod'.
  mat2 to_screen_pixel_coords = inverse(mat2(dx_uv, dy_uv));
  ivec2 ij0 = ivec2(floor((uv - d_uv) * lod_width));
  ivec2 ij1 = ivec2(floor((uv + d_uv) * lod_width));
  vec3 color_sum = vec3(0.0);
  for (int j = ij0.y; j <= ij1.y; ++j) {
    for (int i = ij0.x; i <= ij1.x; ++i) {
      vec2 texel_uv = (vec2(i, j) + vec2(0.5)) / lod_width;
      vec3 texel_dir = vec3(texel_uv * dir.z, dir.z);
      if (max_abs_dir_comp == abs_dir.x) { texel_dir = texel_dir.zyx; } 
      else if (max_abs_dir_comp == abs_dir.y) { texel_dir = texel_dir.xzy; }
      vec2 delta_uv;
      vec3 star_color = StarTextureColor(texel_dir, lod, delta_uv);
      vec2 star_uv = uv - texel_uv + delta_uv / lod_width;
      vec2 star_pixel_coords = to_screen_pixel_coords * star_uv;
      vec2 overlap = max(vec2(1.0) - abs(star_pixel_coords), 0.0);
      color_sum += star_color * overlap.x * overlap.y;
    }
  }
  return color_sum * lensing_amplification_factor;
}

// Default implementation for 'DiscColor'.
// The following constants must be provided by the user:
// - const float INNER_DISC_R = ...;
// - const float OUTER_DISC_R = ...;
// - const int NUM_DISC_PARTICLES = ...;
// - const vec4 DISC_PARTICLE_PARAMS[NUM_DISC_PARTICLES] = ...;
// They define the inner and outer radius of the disc, the number of particles
// used to compute its density, and the orbital parameters for each particle
// (inverse max and min radius, initial azimuth angle, precession 'ratio').
vec4 DefaultDiscColor(vec2 p, float p_t, bool top_side, float doppler_factor,
    float disc_temperature, sampler2D black_body_texture) {
  float p_r = length(p);
  float p_phi = atan(p.y, p.x);

  float density = 2.0;
  for (int i = 0; i < NUM_DISC_PARTICLES; ++i) {
    vec4 params = DISC_PARTICLE_PARAMS[i];
    float u1 = params.x;
    float u2 = params.y;
    float phi0 = params.z;
    float dtheta_dphi = params.w;
    float u_avg = (u1 + u2) * 0.5;
    float dphi_dt = u_avg * sqrt(0.5 * u_avg);
    float dphi = dphi_dt * p_t;
    float phi = mod(p_phi - dphi, 2.0 * pi);
    float s = sin(dtheta_dphi * (phi + dphi) + phi0);
    float r = 1.0 / (u1 + (u2 - u1) * s * s);
    vec2 d = vec2((phi - pi) / pi, r - p_r);
    density += smoothstep(1.0, 0.0, length(d));
  }

  const float r_max = 49.0 / 12.0;
  const float temperature_profile_max = 
      pow((1.0 - sqrt(3.0 / r_max)) / (r_max * r_max * r_max), 0.25);
  float temperature_profile = 
      pow((1.0 - sqrt(3.0 / p_r)) / (p_r * p_r * p_r), 0.25);
  float temperature = 
      disc_temperature * temperature_profile * (1.0 / temperature_profile_max);

  vec3 color = density * 
      BlackBodyColor(black_body_texture, temperature * doppler_factor);
  float alpha = smoothstep(INNER_DISC_R, INNER_DISC_R * 1.2, p_r) * 
      smoothstep(OUTER_DISC_R, OUTER_DISC_R / 1.2, p_r);
  return vec4(color * alpha, alpha);
}

// Finds the intersection of the given view ray with the scene, computes the
// emitted light at these intersection points, computes the corresponding
// received light, and composites and returns the final pixel color. 
//
// Inputs:
// - camera_position: the camera position, in Schwarzschild coordinates
//     (p^t, p^r, p^theta,p^phi).
// - p: the camera position, in (pseudo-)Cartesian coordinates.
// - k_s: the camera 4-velocity, in Schwarzschild coordinates.
// - k: the camera velocity, in (pseudo-)Cartesian coordinates.
// - d: the view ray direction, in (pseudo-)Cartesian coordinates.
vec3 SceneColor(vec4 camera_position, vec3 p, vec4 k_s, vec3 k, vec3 d) {
  vec3 e_x_prime = normalize(p);
  vec3 e_z_prime = normalize(cross(e_x_prime, d));
  vec3 e_y_prime = normalize(cross(e_z_prime, e_x_prime));

  const vec3 e_z = vec3(0.0, 0.0, 1.0);
  vec3 t = normalize(cross(e_z, e_z_prime));
  if (dot(t, e_y_prime) < 0.0) {
    t = -t;
  }

  float alpha = acos(dot(e_x_prime, t));
  float delta = acos(dot(e_x_prime, normalize(d)));

  float u0, phi0, t0, u1, phi1, t1;
  float deflection = RayTrace(camera_position[1], delta, alpha,
      1.0 / OUTER_DISC_R, 1.0 / INNER_DISC_R, u0, phi0, t0, u1, phi1, t1);

  float u = 1.0 / camera_position[1];
  float u_prime = -u / tan(delta);
  float e = -sqrt(u_prime * u_prime + u * u * (1.0 - u));

  vec4 l = vec4(e / (1.0 - u), -u_prime, 0.0, u * u);
  float g_k_l_receiver = k_s.x * l.x * (1.0 - u) -
      k_s.y * l.y / (1.0 - u) - u * dot(k, e_y_prime) * l.w / (u * u);

  vec3 color = vec3(0.0, 0.0, 0.0);
  if (deflection >= 0.0) {
    float g_k_l_source = e;
    float doppler_factor = g_k_l_receiver / g_k_l_source;

    float delta_prime = delta + deflection;
    vec3 d_prime = cos(delta_prime) * e_x_prime + sin(delta_prime) * e_y_prime;

    float lensing_amplification_factor = abs(sin(delta) / sin(delta_prime) /
        (1.0 + fwidth(deflection) / fwidth(delta)));
    // Clamp the result (otherwise potentially infinite) and use a hack to avoid
    // an artefact in the direction opposite to the black hole center.
    lensing_amplification_factor = 
        delta < 0.01 ? 1.0 : min(lensing_amplification_factor, 1e6);

    color += GalaxyColor(d_prime);
    color += StarColor(d_prime, lensing_amplification_factor);
    color = Doppler(color, doppler_factor);
  }
  if (u1 >= 0.0) {
    float g_k_l_source = e * sqrt(2.0 / (2.0 - 3.0 * u1)) - 
        u1 * sqrt(u1 / (2.0 - 3.0 * u1)) * dot(e_z, e_z_prime);
    float doppler_factor = g_k_l_receiver / g_k_l_source;
    bool top_side = 
        (mod(abs(phi1 - alpha), 2.0 * pi) < 1e-3) == (e_x_prime.z > 0.0);

    vec3 i1 = (e_x_prime * cos(phi1) + e_y_prime * sin(phi1)) / u1;
    vec4 disc_color =
        DiscColor(i1.xy, camera_position[0] - t1, top_side, doppler_factor);
    color = color * (1.0 - disc_color.a) + disc_color.rgb;
  }
  if (u0 >= 0.0) {
    float g_k_l_source = e * sqrt(2.0 / (2.0 - 3.0 * u0)) - 
        u0 * sqrt(u0 / (2.0 - 3.0 * u0)) * dot(e_z, e_z_prime);
    float doppler_factor = g_k_l_receiver / g_k_l_source;
    bool top_side = 
        (mod(abs(phi0 - alpha), 2.0 * pi) < 1e-3) == (e_x_prime.z > 0.0);
    
    vec3 i0 = (e_x_prime * cos(phi0) + e_y_prime * sin(phi0)) / u0;
    vec4 disc_color =
        DiscColor(i0.xy, camera_position[0] - t0, top_side, doppler_factor);
    color = color * (1.0 - disc_color.a) + disc_color.rgb;
  }
  return color;
}

#endif
