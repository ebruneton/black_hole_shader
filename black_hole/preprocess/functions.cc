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

#include "black_hole/preprocess/functions.h"

#include <algorithm>
#include "math/vector.h"

namespace black_hole {
namespace preprocess {

// Include the source code of the glsl functions, seen here as C++ code.

#define IN(x) const x&
#define OUT(x) x&

using dimensional::Angle;
using dimensional::BinaryFunction;
using dimensional::pi;
using dimensional::rad;
using dimensional::vec2;
using std::max;
using std::min;

Real abs(const Real x) { return x < 0.0 ? -x : x; }
Real fwidth(const Real x) { return 0.0; }

#include "black_hole/functions.glsl"

// The function used to generate the ray deflection texture D(e,u).

Real GetEsquareFromRayDeflectionTextureU(const Real tex_u) {
  const Real x = 1.0 - exp(-50.0 * (tex_u - 0.5) * (tex_u - 0.5));
  return tex_u < 0.5 ? kMu * x : kMu / x;
}

void ComputeRayDeflectionTexture(RayDeflectionTexture* texture) {
  for (int i = 0; i < RAY_DEFLECTION_TEXTURE_WIDTH; ++i) {
    const Real e_square = GetEsquareFromRayDeflectionTextureU(
        i / (RAY_DEFLECTION_TEXTURE_WIDTH - 1.0));
    const Real e = sqrt(e_square);
    Real u = 0;
    Real u_prime = e;
    Angle phi = 0 * rad;
    Real t = 0;
    constexpr Real dphi = 1e-5;

    Angle previous_deflection = 0 * rad;
    Real previous_t = 0;
    Real previous_j = 0;
    texture->Set(i, 0, TimedAngle(0 * rad, 0));
    while (true) {
      if (u >= 1 || u_prime < 0.0) {
        texture->Set(i, RAY_DEFLECTION_TEXTURE_HEIGHT - 1,
                     TimedAngle(previous_deflection, previous_t));
        break;
      }

      const Angle deflection = phi - atan2(u, u_prime);
      const Real j = GetRayDeflectionTextureVFromEsquareAndU(e_square, u) *
                     (RAY_DEFLECTION_TEXTURE_HEIGHT - 1);
      const int k0 = static_cast<int>(ceil(previous_j()));
      const int k1 = static_cast<int>(ceil(j()));
      for (int k = k0; k < k1; ++k) {
        const Real lerp = (k - previous_j) / (j - previous_j);
        const Angle lerp_deflection =
            previous_deflection * (1.0 - lerp) + deflection * lerp;
        const Real lerp_t = previous_t * (1.0 - lerp) + t * lerp;
        texture->Set(i, k, TimedAngle(lerp_deflection, lerp_t));
      }

      u_prime = u_prime + (1.5 * u * u - u) * dphi;
      u = u + u_prime * dphi;
      phi = phi + dphi * rad;
      if (u > 1e-2) {
        t = t + e / (u * u * (1.0 - u)) * dphi;
      }
      previous_deflection = deflection;
      previous_t = t;
      previous_j = j;
    }
  }
}

// The function used to generate the ray inverse radius texture U(e,phi).

Real GetEsquareFromRayInverseRadiusTextureU(const Real tex_u) {
  return (1.0 / tex_u - 1.0) / 6.0;
}

void ComputeRayInverseRadiusTexture(RayInverseRadiusTexture* texture) {
  for (int i = 0; i < RAY_INVERSE_RADIUS_TEXTURE_WIDTH; ++i) {
    const Real e_square =
        GetEsquareFromRayInverseRadiusTextureU(dimensional::clamp(
            i / (RAY_INVERSE_RADIUS_TEXTURE_WIDTH - 1.0), 0.001, 0.999));
    const Real e = sqrt(e_square);
    const Angle phi_up = GetPhiUpFromEsquare(e_square);
    Real u = 0;
    Real u_prime = e;
    Angle phi = 0 * rad;
    Real t = 0;
    constexpr Real dphi = 1e-5;

    Real previous_u = 0;
    Real previous_t = 0;
    Real previous_j = 0;
    texture->Set(i, 0, TimedInverseDistance(0, 0));
    while (true) {
      assert(u < 1.0);
      const Real j = (phi / phi_up) * (RAY_INVERSE_RADIUS_TEXTURE_HEIGHT - 1);
      const int k0 = static_cast<int>(ceil(previous_j()));
      const int k1 = std::min(static_cast<int>(ceil(j())),
                              RAY_INVERSE_RADIUS_TEXTURE_HEIGHT);
      for (int k = k0; k < k1; ++k) {
        const Real lerp = (k - previous_j) / (j - previous_j);
        const Real lerp_u = previous_u * (1.0 - lerp) + u * lerp;
        const Real lerp_t = previous_t * (1.0 - lerp) + t * lerp;
        texture->Set(i, k, TimedInverseDistance(lerp_u, lerp_t));
      }
      if (k1 == RAY_INVERSE_RADIUS_TEXTURE_HEIGHT) {
        break;
      }
      previous_u = u;
      previous_t = t;
      previous_j = j;

      u_prime = u_prime + (1.5 * u * u - u) * dphi;
      u = u + u_prime * dphi;
      phi = phi + dphi * rad;
      if (u > 1e-2) {
        t = t + e / (u * u * (1.0 - u)) * dphi;
      }
    }
  }
}

}  // namespace preprocess
}  // namespace black_hole
