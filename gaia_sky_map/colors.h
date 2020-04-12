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

#ifndef GAIA_SKY_MAP_COLORS_H_
#define GAIA_SKY_MAP_COLORS_H_

#include "color_maps/colors.h"

// Provides very basic utilities to represent colors in XYZ + Gaia DR2 G band.

namespace gaia_sky_map {

using LinearSrgb = color_maps::LinearSrgb;
using Srgb = color_maps::Srgb;
using Xyz = color_maps::Xyz;

// CIE XYZ color, plus Gaia DR2 G passband data.
class Xyzg {
 public:
  Xyzg() {
    channels_[0] = 0;
    channels_[1] = 0;
    channels_[2] = 0;
    channels_[3] = 0;
  }

  Xyzg(const double x, const double y, const double z, const double g) {
    channels_[0] = x;
    channels_[1] = y;
    channels_[2] = z;
    channels_[3] = g;
  }

  double x() const { return channels_[0]; }
  double y() const { return channels_[1]; }
  double z() const { return channels_[2]; }
  double g() const { return channels_[3]; }

  inline Xyzg operator+(const Xyzg& rhs) const {
    return Xyzg(
        channels_[0] + rhs.channels_[0], channels_[1] + rhs.channels_[1],
        channels_[2] + rhs.channels_[2], channels_[3] + rhs.channels_[3]);
  }

  inline Xyzg operator-(const Xyzg& rhs) const {
    return Xyzg(
        channels_[0] - rhs.channels_[0], channels_[1] - rhs.channels_[1],
        channels_[2] - rhs.channels_[2], channels_[3] - rhs.channels_[3]);
  }

  inline Xyzg operator*(const double rhs) const {
    return Xyzg(channels_[0] * rhs, channels_[1] * rhs, channels_[2] * rhs,
                channels_[3] * rhs);
  }

  inline Xyzg operator/(const double rhs) const {
    return Xyzg(channels_[0] / rhs, channels_[1] / rhs, channels_[2] / rhs,
                channels_[3] / rhs);
  }

 private:
  double channels_[4];
};

// Returns the Xyzg color corresponding to a black body at (the integer part of)
// the given temperature in Kelvins (using a cache for efficiency).
const Xyzg& XyzgFromTemperature(double temperature);

Xyz ToXyz(const Xyzg& xyzg);

}  // namespace gaia_sky_map

#endif  // GAIA_SKY_MAP_COLORS_H_
