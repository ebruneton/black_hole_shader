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

/*<h2>black_hole/model.glsl</h2>

<p>This file provides a main <code>SceneColor</code> GLSL function, which
computes the color of a fragment for a scene made of a black hole with an
accretion disc, plus background stars and extended light sources (e.g. nebulae).
This main function uses some abstract auxiliary functions, declared below. These
functions must be provided by the user. A default implementation is provided for
most of them, but other implementations can be used, for instance to render the
scene in a non-realistic way for educational purposes.
*/

// Abstract functions, which must be implemented by the user:
// - ray tracing function (see the default implementation in functions.glsl).
Angle RayTrace(Real u, Real u_dot, Real e_square, Angle delta, Angle alpha,
               Real u_ic, Real u_oc, out Real u0, out Angle phi0, out Real t0,
               out Real alpha0, out Real u1, out Angle phi1, out Real t1,
               out Real alpha1);
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
// - noise function used in the default accretion disc shading function 
//   'DefaultDiscColor()'.
float Noise(vec2 uv);
// - color and opacity of the accretion disc at 'p', and at time 't', for the
//   top or bottom side of the disc, and with the given Doppler factor.
vec4 DiscColor(vec2 p, float t, bool top_side, float doppler_factor);

/*
<p>This function provides a default implementation for <code>Doppler()</code>.
It is designed to work with the 3D texture precomputed with
<code>color_maps/doppler_texture_generator.cc</code>.
*/

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

/*
<p>This function provides a default implementation for <code>StarColor()</code>,
based on the custom texture filtering algorithm described in <a
href="https://arxiv.org/abs/2010.08735">Section 3.3.2</a> of our model.
*/

// Returns the light emitted by the stars in the pixel footprint around 'dir',
// times the given gravitational lensing amplification factor.
// This implementation uses the two 'StarTextureColor' functions above, and
// assumes that they are based on a cube map. The following constants must be
// provided by the user:
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
      if (max_abs_dir_comp == abs_dir.x) {
        texel_dir = texel_dir.zyx;
      } else if (max_abs_dir_comp == abs_dir.y) {
        texel_dir = texel_dir.xzy;
      }
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

/*
<p>This function provides the light emitted by a black body, by using the 1D
texture precomputed with
<code>color_maps/black_body_texture_generator.cc</code>.
*/

// Returns the light emitted by a black body at the given temperature. The 1D
// texture should contain this color at texture coord log(T / 100) / 6.
vec3 BlackBodyColor(sampler2D black_body_texture, float temperature) {
  float tex_u = (1.0 / 6.0) * log(temperature * (1.0 / 100.0));
  return texture(black_body_texture, vec2(tex_u, 0.5)).rgb;
}

/*
<p>This function provides a default implementation for <code>DiscColor()</code>,
based on the accretion disc model described in
<a href="https://arxiv.org/abs/2010.08735">Section 3.3.3</a> and 
<a href="https://arxiv.org/abs/2010.08735">Appendix B</a> of our model.
*/

// Returns the light emitted by the accretion disc at 'p', at time 'p_t', 
// shifted by the given Doppler factor. The 1D texture should contain the light
// emitted by a black body at temperature T at texture coord log(T / 100) / 6.
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

  float density = 0.0;
  for (int i = 0; i < NUM_DISC_PARTICLES; ++i) {
    vec4 params = DISC_PARTICLE_PARAMS[i];
    float u1 = params.x;
    float u2 = params.y;
    float phi0 = params.z;
    float dtheta_dphi = params.w;
    float u_avg = (u1 + u2) * 0.5;
    float dphi_dt = u_avg * sqrt(0.5 * u_avg);
    float phi = dphi_dt * p_t + phi0;
    float a = mod(p_phi - phi, 2.0 * pi);
    float s = sin(dtheta_dphi * (a + phi));
    float r = 1.0 / (u1 + (u2 - u1) * s * s);
    vec2 d = vec2(a - pi, r - p_r) * vec2(1.0 / pi, 0.5);
    float noise = Noise(d * vec2(p_r / OUTER_DISC_R, 1.0));
    density += smoothstep(1.0, 0.0, length(d)) * noise;
  }

  const float r_max = 49.0 / 12.0;
  const float temperature_profile_max =
      pow((1.0 - sqrt(3.0 / r_max)) / (r_max * r_max * r_max), 0.25);
  float temperature_profile =
      pow((1.0 - sqrt(3.0 / p_r)) / (p_r * p_r * p_r), 0.25);
  float temperature =
      disc_temperature * temperature_profile * (1.0 / temperature_profile_max);

  vec3 color = max(density, 0.0) *
      BlackBodyColor(black_body_texture, temperature * doppler_factor);
  float alpha = smoothstep(INNER_DISC_R, INNER_DISC_R * 1.2, p_r) *
      smoothstep(OUTER_DISC_R, OUTER_DISC_R / 1.2, p_r);
  return vec4(color * alpha, alpha);
}

/*
<p>Finally, the main <code>SceneColor</code> function:
<ul>
<li>initializes the light beam parameters as described in 
<a href="https://arxiv.org/abs/2010.08735">Section 3.2</a> of our model,</li>
<li>calls the <code>RayTrace()</code> function to get the light ray deflection
and the accretion disc intersections,</li>
<li>computes the light emitted by the stars and the accretion disc with
<code>StarColor()</code>, <code>GalaxyColor()</code> and
<code>DiscColor()</code>,
<li>computes the corresponding received light by computing the gravitational
lensing effect and the Doppler and beaming effects as described in 
<a href="https://arxiv.org/abs/2010.08735">Section 3.4</a> of our model,</li>
<li>returns the composited star and disc colors.</li>
</ul>
*/

// Finds the intersection of the given view ray with the scene, computes the
// emitted light at these intersection points, computes the corresponding
// received light, and composites and returns the final pixel color.
//
// Inputs:
// - camera_position: the camera position, in Schwarzschild coordinates
//     (p^t, p^r, p^theta, p^phi).
// - p: the camera position, in (pseudo-)Cartesian coordinates.
// - k_s: the camera 4-velocity, in Schwarzschild coordinates.
// - e_tau, e_w, e_h, e_d: the base vectors of the camera reference frame, in
//     (pseudo-)Cartesian coordinates.
// - view_dir: the view ray direction, in the camera reference frame.
vec3 SceneColor(vec4 camera_position, vec3 p, vec4 k_s, vec3 e_tau, vec3 e_w,
                vec3 e_h, vec3 e_d, vec3 view_dir) {
  vec3 q = normalize(view_dir);
  vec3 d = -e_tau + q.x * e_w + q.y * e_h + q.z * e_d;

  vec3 e_x_prime = normalize(p);
  vec3 e_z_prime = normalize(cross(e_x_prime, d));
  vec3 e_y_prime = normalize(cross(e_z_prime, e_x_prime));

  const vec3 e_z = vec3(0.0, 0.0, 1.0);
  vec3 t = normalize(cross(e_z, e_z_prime));
  if (dot(t, e_y_prime) < 0.0) {
    t = -t;
  }

  float alpha = acos(clamp(dot(e_x_prime, t), -1.0, 1.0));
  float delta = acos(clamp(dot(e_x_prime, normalize(d)), -1.0, 1.0));

  float u = 1.0 / camera_position[1];
  float u_dot = -u / tan(delta);
  float e_square = u_dot * u_dot + u * u * (1.0 - u);
  float e = -sqrt(e_square);

  const float U_IC = 1.0 / INNER_DISC_R;
  const float U_OC = 1.0 / OUTER_DISC_R;
  float u0, phi0, t0, alpha0, u1, phi1, t1, alpha1;
  float deflection = RayTrace(u, u_dot, e_square, delta, alpha, U_IC, U_OC,
                              u0, phi0, t0, alpha0, u1, phi1, t1, alpha1);

  vec4 l = vec4(e / (1.0 - u), -u_dot, 0.0, u * u);
  float g_k_l_receiver = k_s.x * l.x * (1.0 - u) - k_s.y * l.y / (1.0 - u) -
                         u * dot(e_tau, e_y_prime) * l.w / (u * u);

  float delta_prime = delta + max(deflection, 0.0);
  vec3 d_prime = cos(delta_prime) * e_x_prime + sin(delta_prime) * e_y_prime;

  vec3 color = vec3(0.0, 0.0, 0.0);
  if (deflection >= 0.0) {
    float g_k_l_source = e;
    float doppler_factor = g_k_l_receiver / g_k_l_source;

    // The solid angle (times 4pi) of the pixel.
    float omega = length(cross(dFdx(q), dFdy(q)));
    // The solid angle (times 4pi) of the deflected light beam.
    float omega_prime = length(cross(dFdx(d_prime), dFdy(d_prime)));

    float lensing_amplification_factor = omega / omega_prime;
    // Clamp the result (otherwise potentially infinite).
    lensing_amplification_factor = min(lensing_amplification_factor, 1e6);

    // The galaxy texture contains the radiant intensity of stars, per unit area
    // on the celestial sphere, i.e. radiance values (using omega0 as area unit,
    // with omega0 = 4pi * the solid angle of the center texel of a cube face).
    // The stars texture contains radiant intensities. To convert the total
    // intensity inside a pixel to a radiance, this intensity must be divided by
    // the pixel area on the celestial sphere. Expressed in the units used for
    // the galaxy texture, this area is omega / omega0 (where, since the galaxy
    // texture is a 2048x2048 cubemap, omega0 is 1 / 1024^2).
    float pixel_area = max(omega * (1024.0 * 1024.0), 1.0);

    color += GalaxyColor(d_prime);
    color += StarColor(d_prime, lensing_amplification_factor / pixel_area);
    color = Doppler(color, doppler_factor);
  }
  if (u1 >= 0.0 && alpha1 > 0.0) {
    float g_k_l_source = e * sqrt(2.0 / (2.0 - 3.0 * u1)) -
                         u1 * sqrt(u1 / (2.0 - 3.0 * u1)) * dot(e_z, e_z_prime);
    float doppler_factor = g_k_l_receiver / g_k_l_source;
    bool top_side =
        (mod(abs(phi1 - alpha), 2.0 * pi) < 1e-3) == (e_x_prime.z > 0.0);

    vec3 i1 = (e_x_prime * cos(phi1) + e_y_prime * sin(phi1)) / u1;
    vec4 disc_color =
        DiscColor(i1.xy, camera_position[0] - t1, top_side, doppler_factor);
    color = color * (1.0 - disc_color.a) + alpha1 * disc_color.rgb;
  }
  if (u0 >= 0.0 && alpha0 > 0.0) {
    float g_k_l_source = e * sqrt(2.0 / (2.0 - 3.0 * u0)) -
                         u0 * sqrt(u0 / (2.0 - 3.0 * u0)) * dot(e_z, e_z_prime);
    float doppler_factor = g_k_l_receiver / g_k_l_source;
    bool top_side =
        (mod(abs(phi0 - alpha), 2.0 * pi) < 1e-3) == (e_x_prime.z > 0.0);

    vec3 i0 = (e_x_prime * cos(phi0) + e_y_prime * sin(phi0)) / u0;
    vec4 disc_color =
        DiscColor(i0.xy, camera_position[0] - t0, top_side, doppler_factor);
    color = color * (1.0 - disc_color.a) + alpha0 * disc_color.rgb;
  }
  return color;
}
