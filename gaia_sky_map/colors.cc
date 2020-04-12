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

#include "gaia_sky_map/colors.h"

#include <cassert>
#include <cmath>
#include <vector>

namespace gaia_sky_map {

namespace {

constexpr int kGLambdaMin = 300;
constexpr int kGLambdaMax = 1098;

#include "gaia_sky_map/colors.inc"

double GaiaDr2GpassbandTableValue(const double wavelength) {
  if (wavelength <= kGLambdaMin || wavelength >= kGLambdaMax) {
    return 0.0;
  }
  double u = wavelength - kGLambdaMin;
  const int row = static_cast<int>(std::floor(u));
  assert(row >= 0 && row + 1 < 799);
  assert(GAIA_DR2_G_PASSBAND[2 * row] <= wavelength &&
         GAIA_DR2_G_PASSBAND[2 * (row + 1)] >= wavelength);
  u -= row;
  return GAIA_DR2_G_PASSBAND[2 * row + 1] * (1.0 - u) +
         GAIA_DR2_G_PASSBAND[2 * (row + 1) + 1] * u;
}

Xyzg ComputeXyzgFromTemperature(const double temperature) {
  double g = 0.0;
  constexpr double dlambda = 1e-9;
  for (int lambda = kGLambdaMin; lambda < kGLambdaMax; ++lambda) {
    const double l = lambda * 1e-9;
    // The black body spectrum as a function of wavelength,
    // see https://en.wikipedia.org/wiki/Planck%27s_law#The_law.
    constexpr double c = 299792458.0;
    constexpr double h = 6.62607015e-34;
    constexpr double kB = 1.380649e-23;
    const double spectral_radiance =
        2.0 * h * c * c /
        (pow(l, 5.0) * (exp(h * c / (l * kB * temperature)) - 1.0));
    g += GaiaDr2GpassbandTableValue(lambda) * spectral_radiance;
  }
  const Xyz xyz = color_maps::ComputeXyzFromTemperature(temperature);
  return Xyzg(xyz[0], xyz[1], xyz[2], g * dlambda);
}

}  // namespace

const Xyzg& XyzgFromTemperature(const double temperature) {
  constexpr int t_min = 100;
  constexpr int t_max = 40000;
  static const std::vector<Xyzg> values = [] {
    std::vector<Xyzg> result;
    for (int temperature = t_min; temperature <= t_max; ++temperature) {
      result.push_back(ComputeXyzgFromTemperature(temperature));
    }
    return result;
  }();
  const int t = static_cast<int>(std::floor(temperature));
  return values[std::max(t_min, std::min(t_max, t)) - t_min];
}

Xyz ToXyz(const Xyzg& xyzg) { return Xyz(xyzg.x(), xyzg.y(), xyzg.z()); }

}  // namespace gaia_sky_map
