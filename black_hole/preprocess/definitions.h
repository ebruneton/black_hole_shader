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

#ifndef BLACK_HOLE_PREPROCESS_DEFINITIONS_H_
#define BLACK_HOLE_PREPROCESS_DEFINITIONS_H_

#include "math/angle.h"
#include "math/binary_function.h"
#include "math/scalar.h"

namespace black_hole {
namespace preprocess {

// Provides the C++ equivalent of 'definitions.glsl', used to compile the main
// functions of our black hole shader with a C++ compiler (both to reuse them in
// order to precompute the textures they need, and for testing them).

constexpr int RAY_DEFLECTION_TEXTURE_WIDTH = 128;
constexpr int RAY_DEFLECTION_TEXTURE_HEIGHT = 64;

constexpr int RAY_INVERSE_RADIUS_TEXTURE_WIDTH = 64;
constexpr int RAY_INVERSE_RADIUS_TEXTURE_HEIGHT = 32;

template <typename T1, typename T2>
struct Tuple2 {
  T1 x;
  T2 y;
  inline constexpr Tuple2() {}
  inline constexpr Tuple2(const T1 x, const T2 y) : x(x), y(y) {}

  inline Tuple2 operator+(const Tuple2& rhs) const {
    return Tuple2(x + rhs.x, y + rhs.y);
  }
  inline Tuple2& operator+=(const Tuple2& rhs) {
    x += rhs.x;
    y += rhs.y;
    return *this;
  }
  inline Tuple2 operator*(const double rhs) const {
    return Tuple2(x * rhs, y * rhs);
  }
};

typedef dimensional::Scalar<0, 0, 0, 0, 0> Real;

typedef Tuple2<dimensional::Angle, Real> TimedAngle;

typedef Tuple2<Real, Real> TimedInverseDistance;

typedef dimensional::BinaryFunction<RAY_DEFLECTION_TEXTURE_WIDTH,
                                    RAY_DEFLECTION_TEXTURE_HEIGHT, TimedAngle>
    RayDeflectionTexture;

typedef dimensional::BinaryFunction<RAY_INVERSE_RADIUS_TEXTURE_WIDTH,
                                    RAY_INVERSE_RADIUS_TEXTURE_HEIGHT,
                                    TimedInverseDistance>
    RayInverseRadiusTexture;

}  // namespace preprocess
}  // namespace black_hole

#endif  // BLACK_HOLE_PREPROCESS_DEFINITIONS_H_
