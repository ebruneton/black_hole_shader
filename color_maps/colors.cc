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

#include "color_maps/colors.h"

#include <cassert>
#include <cmath>
#include <vector>

namespace color_maps {

namespace {

constexpr int kLambdaMin = 360;
constexpr int kLambdaMax = 830;

#include "color_maps/colors.inc"

// Returns the CIE xy chromaticity corresponding to the given linear sRGB color.
Xyz XyFromLinearSrgb(const double r, const double g, const double b) {
  Xyz result = ToXyz(LinearSrgb(r, g, b));
  return result / (result[0] + result[1] + result[2]);
}

// Clamps the given CIE xy value to the line segment AB, if it is on the other
// side of this line segment compared to W (by computing the intersection of the
// AB segment and of the segment joining W and the given point). All this is
// done in the 2D CIE xy space.
void MaybeClamp(const Xyz& pW, const Xyz& pA, const Xyz& pB, double* x,
                double* y) {
  const double dx = *x - pW[0];
  const double dy = *y - pW[1];
  const double ex = pB[0] - pA[0];
  const double ey = pB[1] - pA[1];
  const double fx = pA[0] - pW[0];
  const double fy = pA[1] - pW[1];
  const double det = dx * ey - dy * ex;
  const double t = (fx * ey - fy * ex) / det;
  const double s = (fx * dy - fy * dx) / det;
  if (t > 0.0 && t < 1.0 && s >= 0.0 && s <= 1.0) {
    *x = pW[0] + t * dx;
    *y = pW[1] + t * dy;
  }
}

}  // namespace

LinearSrgb ToLinearSrgb(const Xyz& xyz) {
  // The conversion matrix from CIE XYZ to linear sRGB color spaces.
  // Values from https://en.wikipedia.org/wiki/SRGB.
  constexpr double matrix[9] = {+3.2406, -1.5372, -0.4986, -0.9689, +1.8758,
                                +0.0415, +0.0557, -0.2040, +1.0570};
  return LinearSrgb(
      matrix[0] * xyz[0] + matrix[1] * xyz[1] + matrix[2] * xyz[2],
      matrix[3] * xyz[0] + matrix[4] * xyz[1] + matrix[5] * xyz[2],
      matrix[6] * xyz[0] + matrix[7] * xyz[1] + matrix[8] * xyz[2]);
}

LinearSrgb ToClampedLinearSrgb(const Xyz& xyz) {
  const double Y = xyz[1];
  double x = xyz[0] / (xyz[0] + xyz[1] + xyz[2]);
  double y = xyz[1] / (xyz[0] + xyz[1] + xyz[2]);
  static const Xyz pR = XyFromLinearSrgb(1.0, 0.0, 0.0);
  static const Xyz pG = XyFromLinearSrgb(0.0, 1.0, 0.0);
  static const Xyz pB = XyFromLinearSrgb(0.0, 0.0, 1.0);
  static const Xyz pW = XyFromLinearSrgb(1.0, 1.0, 1.0);
  MaybeClamp(pW, pR, pG, &x, &y);
  MaybeClamp(pW, pG, pB, &x, &y);
  MaybeClamp(pW, pB, pR, &x, &y);
  const Xyz clamped(x / y * Y, Y, (1.0 - x - y) / y * Y);
  return ToLinearSrgb(clamped);
}

Xyz ToXyz(const LinearSrgb& linear_srgb) {
  // The conversion matrix from linear sRGB to CIE XYZ color spaces.
  // Values from https://en.wikipedia.org/wiki/SRGB.
  constexpr double matrix[9] = {0.4124, 0.3576, 0.1805, 0.2126, 0.7152,
                                0.0722, 0.0193, 0.1192, 0.9505};
  return Xyz(matrix[0] * linear_srgb[0] + matrix[1] * linear_srgb[1] +
                 matrix[2] * linear_srgb[2],
             matrix[3] * linear_srgb[0] + matrix[4] * linear_srgb[1] +
                 matrix[5] * linear_srgb[2],
             matrix[6] * linear_srgb[0] + matrix[7] * linear_srgb[1] +
                 matrix[8] * linear_srgb[2]);
}

Srgb ToSrgb(const LinearSrgb& linear_srgb) {
  // The sRGB gamma correction function.
  // Definition from https://en.wikipedia.org/wiki/SRGB.
  const auto from_linear = [](const double c) {
    if (c <= 0.0031308) {
      return c * 12.92;
    } else {
      constexpr double a = 0.055;
      return pow(c, 1.0 / 2.4) * (1 + a) - a;
    }
  };
  return Srgb(from_linear(linear_srgb[0]), from_linear(linear_srgb[1]),
              from_linear(linear_srgb[2]));
}

LinearSrgb ToLinearSrgb(const Srgb& srgb) {
  // The sRGB inverse gamma correction function.
  // Definition from https://en.wikipedia.org/wiki/SRGB.
  const auto to_linear = [](const double c) {
    if (c <= 0.04045) {
      return c / 12.92;
    } else {
      constexpr double a = 0.055;
      return pow((c + a) / (1 + a), 2.4);
    }
  };
  return LinearSrgb(to_linear(srgb[0]), to_linear(srgb[1]), to_linear(srgb[2]));
}

double CieColorMatchingFunctionTableValue(const double wavelength,
                                          const int column) {
  if (wavelength <= kLambdaMin || wavelength >= kLambdaMax) {
    return 0.0;
  }
  double u = (wavelength - kLambdaMin) / 5.0;
  int row = static_cast<int>(std::floor(u));
  assert(row >= 0 && row + 1 < 95);
  assert(CIE_2_DEG_COLOR_MATCHING_FUNCTIONS[4 * row] <= wavelength &&
         CIE_2_DEG_COLOR_MATCHING_FUNCTIONS[4 * (row + 1)] >= wavelength);
  u -= row;
  return CIE_2_DEG_COLOR_MATCHING_FUNCTIONS[4 * row + column] * (1.0 - u) +
         CIE_2_DEG_COLOR_MATCHING_FUNCTIONS[4 * (row + 1) + column] * u;
}

Xyz ComputeXyzFromTemperature(const double temperature) {
  Xyz xyz;
  constexpr double dlambda = 1e-9;
  for (int lambda = kLambdaMin; lambda < kLambdaMax; ++lambda) {
    const double l = lambda * 1e-9;
    // The black body spectrum as a function of wavelength,
    // see https://en.wikipedia.org/wiki/Planck%27s_law#The_law.
    constexpr double c = 299792458.0;
    constexpr double h = 6.62607015e-34;
    constexpr double kB = 1.380649e-23;
    const double spectral_radiance =
        2.0 * h * c * c /
        (pow(l, 5.0) * (exp(h * c / (l * kB * temperature)) - 1.0));
    xyz = xyz + Xyz(CieColorMatchingFunctionTableValue(lambda, 1),
                    CieColorMatchingFunctionTableValue(lambda, 2),
                    CieColorMatchingFunctionTableValue(lambda, 3)) *
                    spectral_radiance;
  }
  return xyz * dlambda;
}

}  // namespace color_maps
