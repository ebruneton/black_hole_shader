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

// -----------------------------------------------------------------------------
// Lookup function for D(e, u), the ray deflection texture.
// -----------------------------------------------------------------------------

const Real kMu = 4.0 / 27.0;

Real GetRayDeflectionTextureUFromEsquare(const Real e_square) {
  if (e_square < kMu) {
    return 0.5 - sqrt(-log(1.0 - e_square / kMu) * (1.0 / 50.0));
  } else {
    return 0.5 + sqrt(-log(1.0 - kMu / e_square) * (1.0 / 50.0));
  }
}

Real GetUapsisFromEsquare(const Real e_square) {
  Real x = (2.0 / kMu) * e_square - 1.0;
  return 1.0 / 3.0 + (2.0 / 3.0) * sin(asin(x) * (1.0 / 3.0));
}

Real GetRayDeflectionTextureVFromEsquareAndU(const Real e_square,
                                             const Real u) {
  if (e_square > kMu) {
    Real x = u < 2.0 / 3.0 ? -sqrt(2.0 / 3.0 - u) : sqrt(u - 2.0 / 3.0);
    return (x + sqrt(2.0 / 3.0)) / (sqrt(1.0 / 3.0) + sqrt(2.0 / 3.0));
  } else {
    return 1.0 - sqrt(max(1.0 - u / GetUapsisFromEsquare(e_square), 0.0));
  }
}

Real GetTextureCoordFromUnitRange(const Real x, const int texture_size) {
  return 0.5 / Real(texture_size) + x * (1.0 - 1.0 / Real(texture_size));
}

TimedAngle LookupRayDeflection(IN(RayDeflectionTexture) ray_deflection_texture,
                               const Real e_square, const Real u,
                               OUT(TimedAngle) deflection_apsis) {
  Real tex_u = GetTextureCoordFromUnitRange(
      GetRayDeflectionTextureUFromEsquare(e_square),
      RAY_DEFLECTION_TEXTURE_WIDTH);
  Real tex_v = GetTextureCoordFromUnitRange(
      GetRayDeflectionTextureVFromEsquareAndU(e_square, u),
      RAY_DEFLECTION_TEXTURE_HEIGHT);
  Real tex_v_apsis =
      GetTextureCoordFromUnitRange(1.0, RAY_DEFLECTION_TEXTURE_HEIGHT);
  deflection_apsis =
      TimedAngle(texture(ray_deflection_texture, vec2(tex_u, tex_v_apsis)));
  return TimedAngle(texture(ray_deflection_texture, vec2(tex_u, tex_v)));
}

// -----------------------------------------------------------------------------
// Lookup function for U(e, phi), the ray inverse radius texture.
// -----------------------------------------------------------------------------

Angle GetPhiUpFromEsquare(const Real e_square) {
  return (e_square + 1.0) / (2.0 * e_square * sqrt(e_square) + 1.0 / 3.0) * rad;
}

Real GetRayInverseRadiusTextureUFromEsquare(const Real e_square) {
  return 1.0 / (6.0 * e_square + 1.0);
}

TimedInverseDistance LookupRayInverseRadius(IN(RayInverseRadiusTexture)
                                                ray_inverse_radius_texture,
                                            const Real e_square,
                                            const Angle phi) {
  Real tex_u = GetTextureCoordFromUnitRange(
      GetRayInverseRadiusTextureUFromEsquare(e_square),
      RAY_INVERSE_RADIUS_TEXTURE_WIDTH);
  Real tex_v = GetTextureCoordFromUnitRange(phi / GetPhiUpFromEsquare(e_square),
                                            RAY_INVERSE_RADIUS_TEXTURE_HEIGHT);
  return TimedInverseDistance(
      texture(ray_inverse_radius_texture, vec2(tex_u, tex_v)));
}

// -----------------------------------------------------------------------------
// Main "ray-tracing" function, using the above lookup functions to get the ray
// deflection and the ray intersections with the accretion disc in constant time
// (and at most 4 texture lookups).
// -----------------------------------------------------------------------------

Angle TraceRay(IN(RayDeflectionTexture) ray_deflection_texture,
               IN(RayInverseRadiusTexture) ray_inverse_radius_texture,
               const Real p_r, const Angle delta, const Angle alpha,
               const Real u_min, const Real u_max, OUT(Real) u0,
               OUT(Angle) phi0, OUT(Real) t0, OUT(Real) u1, OUT(Angle) phi1,
               OUT(Real) t1) {
  // Compute the ray deflection.
  Real u = 1.0 / p_r;
  Real u_prime = -u / tan(delta);
  Real e_square = u_prime * u_prime + u * u * (1.0 - u);
  u0 = -1.0;
  u1 = -1.0;
  if (e_square < kMu && u > 2.0 / 3.0) {
    return -1.0 * rad;
  }
  TimedAngle deflection_apsis;
  TimedAngle deflection = LookupRayDeflection(ray_deflection_texture, e_square,
                                              u, deflection_apsis);
  Angle ray_deflection = deflection.x;
  if (u_prime > 0.0) {
    ray_deflection =
        e_square > kMu ? -1.0 * rad : 2.0 * deflection_apsis.x - ray_deflection;
  }
  // Compute the accretion disc intersections.
  Real s = sign(u_prime);
  Angle phi = deflection.x + (s == 1.0 ? pi - delta : delta) + s * alpha;
  Angle phi_apsis = deflection_apsis.x + pi / 2.0;
  phi0 = mod(phi, pi);
  TimedInverseDistance ui =
      LookupRayInverseRadius(ray_inverse_radius_texture, e_square, phi0);
  if (phi0 < phi_apsis && ui.x >= u_min && ui.x <= u_max &&
      (sign(ui.x - u) == s)) {
    u0 = ui.x;
    phi0 = alpha + phi - phi0;
    t0 = s * (ui.y - deflection.y);
  }
  if (e_square < kMu && s == 1.0) {
    phi = 2.0 * phi_apsis - phi;
    phi1 = mod(phi, pi);
    ui = LookupRayInverseRadius(ray_inverse_radius_texture, e_square, phi1);
    if (phi1 < phi_apsis && ui.x >= u_min && ui.x <= u_max) {
      if (u0 == -1.0) {
        u0 = ui.x;
        phi0 = alpha + phi - phi1;
        t0 = 2.0 * deflection_apsis.y - ui.y - deflection.y;
      } else {
        u1 = ui.x;
        phi1 = alpha + phi - phi1;
        t1 = 2.0 * deflection_apsis.y - ui.y - deflection.y;
      }
    }
  }
  return ray_deflection;
}
