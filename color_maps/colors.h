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

#ifndef COLOR_MAPS_COLORS_H_
#define COLOR_MAPS_COLORS_H_

// Provides very basic utility classes to represent and manipulate colors in
// XYZ, linear sRGB, and sRGB color spaces. Also provides utility functions to
// convert colors from one color space to another.

namespace color_maps {

enum ColorSpace { XYZ, LINEAR_SRGB, SRGB };

template <ColorSpace S>
class Color {
 public:
  Color() {
    channels_[0] = 0;
    channels_[1] = 0;
    channels_[2] = 0;
  }

  Color(const double c0, const double c1, const double c2) {
    channels_[0] = c0;
    channels_[1] = c1;
    channels_[2] = c2;
  }

  ColorSpace color_space() const { return S; }

  inline double operator[](int channel) const { return channels_[channel]; }

  inline Color operator+(const Color& rhs) const {
    return Color(channels_[0] + rhs.channels_[0],
                 channels_[1] + rhs.channels_[1],
                 channels_[2] + rhs.channels_[2]);
  }

  inline Color operator-(const Color& rhs) const {
    return Color(channels_[0] - rhs.channels_[0],
                 channels_[1] - rhs.channels_[1],
                 channels_[2] - rhs.channels_[2]);
  }

  inline Color operator*(const double rhs) const {
    return Color(channels_[0] * rhs, channels_[1] * rhs, channels_[2] * rhs);
  }

  inline Color operator/(const double rhs) const {
    return Color(channels_[0] / rhs, channels_[1] / rhs, channels_[2] / rhs);
  }

 private:
  double channels_[3];
};

typedef Color<XYZ> Xyz;
typedef Color<LINEAR_SRGB> LinearSrgb;
typedef Color<SRGB> Srgb;

LinearSrgb ToLinearSrgb(const Xyz& xyz);

// Converts the given XYZ color to sRGB, clamped to the sRGB gammut (if the
// given color is outside the sRGB triangle gammut in the CIE xy diagram, it is
// clamped to the triangle boundary - along the line segment joining it to the
// white point - before being converted to sRGB).
LinearSrgb ToClampedLinearSrgb(const Xyz& xyz);

Xyz ToXyz(const LinearSrgb& linear_srgb);

Srgb ToSrgb(const LinearSrgb& linear_srgb);

LinearSrgb ToLinearSrgb(const Srgb& srgb);

// Returns the value of the CIE color matching functions (X=1, Y=2, Z=3) at the
// given wavelength.
double CieColorMatchingFunctionTableValue(double wavelength, int column);

// Computes the Xyz color corresponding to a black body at the given temperature
// in Kelvins.
Xyz ComputeXyzFromTemperature(double temperature);

}  // namespace color_maps

#endif  // COLOR_MAPS_COLORS_H_
