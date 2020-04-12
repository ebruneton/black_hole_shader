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

#ifndef GAIA_SKY_MAP_CUBE_MAP_H_
#define GAIA_SKY_MAP_CUBE_MAP_H_

#include "math/vector.h"

// Provides a class to store and edit a cube map.

namespace gaia_sky_map {

// With and height of each cube map texture, in pixels.
constexpr int kCubeMapSize = 2048;

// Converts a 3D direction vector into a cube map texel coordinate.
void ConvertDirectionToCubeMapFaceXy(const dimensional::vec3& dir, int* face,
                                     int* x, int* y);

// Converts a cube map texel coordinate into a 3D direction vector.
dimensional::vec3 ConvertCubeMapFaceXyToDirection(int face, int x, int y);

// The texture elements (texels) of a face of a cube map.
template <typename T>
class CubeMapFace {
 public:
  const T& texel(const int x, const int y) const {
    return texels_[x + y * kCubeMapSize];
  }
  T& texel(const int x, const int y) { return texels_[x + y * kCubeMapSize]; }

 private:
  T texels_[kCubeMapSize * kCubeMapSize];
};

// The texture elements (texels) of a cube map.
template <typename T>
struct CubeMap {
 public:
  const T& texel(const int face, const int x, const int y) const {
    return faces_[face].texel(x, y);
  }

  T& texel(const int face, const int x, const int y) {
    return faces_[face].texel(x, y);
  }

  const T& texel(const dimensional::vec3& dir) const {
    int face, x, y;
    ConvertDirectionToCubeMapFaceXy(dir, &face, &x, &y);
    return texel(face, x, y);
  }

  T& texel(const dimensional::vec3& dir) {
    int face, x, y;
    ConvertDirectionToCubeMapFaceXy(dir, &face, &x, &y);
    return texel(face, x, y);
  }

 private:
  CubeMapFace<T> faces_[6];
};

}  // namespace gaia_sky_map

#endif  // GAIA_SKY_MAP_CUBE_MAP_H_
