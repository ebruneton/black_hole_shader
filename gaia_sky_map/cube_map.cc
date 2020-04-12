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

#include "gaia_sky_map/cube_map.h"

#include <algorithm>
#include <cmath>

using dimensional::vec3;

namespace gaia_sky_map {

void ConvertDirectionToCubeMapFaceXy(const vec3& dir, int* face, int* x,
                                     int* y) {
  const double abs_x = fabs(dir.x());
  const double abs_y = fabs(dir.y());
  const double abs_z = fabs(dir.z());
  double max_axis, u, v;
  if (abs_x >= abs_y && abs_x >= abs_z) {
    max_axis = abs_x;
    if (dir.x() > 0.0) {
      u = -dir.z();
      v = -dir.y();
      *face = 0;
    } else {
      max_axis = abs_x;
      u = dir.z();
      v = -dir.y();
      *face = 1;
    }
  } else if (abs_y >= abs_x && abs_y >= abs_z) {
    max_axis = abs_y;
    if (dir.y() > 0.0) {
      u = dir.x();
      v = dir.z();
      *face = 2;
    } else {
      u = dir.x();
      v = -dir.z();
      *face = 3;
    }
  } else {
    max_axis = abs_z;
    if (dir.z() > 0.0) {
      u = dir.x();
      v = -dir.y();
      *face = 4;
    } else {
      u = -dir.x();
      v = -dir.y();
      *face = 5;
    }
  }
  const auto to_texel_coord = [max_axis](const double u) {
    const double s = 0.5 * (u / max_axis + 1.0);
    const int i = static_cast<int>(floor(s * kCubeMapSize));
    return std::max(0, std::min(kCubeMapSize - 1, i));
  };
  *x = to_texel_coord(u);
  *y = to_texel_coord(v);
}

vec3 ConvertCubeMapFaceXyToDirection(const int face, const int x, const int y) {
  const double s = (x + 0.5) / kCubeMapSize;
  const double t = (y + 0.5) / kCubeMapSize;
  const double u = 2.0 * s - 1.0;
  const double v = 2.0 * t - 1.0;
  switch (face) {
    case 0:
      return vec3(1.0, -v, -u);
    case 1:
      return vec3(-1.0, -v, u);
    case 2:
      return vec3(u, 1.0, v);
    case 3:
      return vec3(u, -1.0, -v);
    case 4:
      return vec3(u, -v, 1.0);
    default:
      return vec3(-u, -v, -1.0);
  }
}

}  // namespace gaia_sky_map
