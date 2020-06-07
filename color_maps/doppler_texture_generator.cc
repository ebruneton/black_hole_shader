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

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <vector>

#include "color_maps/colors.h"

namespace color_maps {
namespace {

// Size of the 3D doppler texture.
constexpr int kDopplerTextureWidth = 64;
constexpr int kDopplerTextureHeight = 32;
constexpr int kDopplerTextureDepth = 64;
constexpr int kDopplerTextureSize =
    kDopplerTextureWidth * kDopplerTextureHeight * kDopplerTextureDepth;

// Minimum and maximum wavelengths for the CIE color matching functions.
constexpr double kLambdaMin = 360;
constexpr double kLambdaMax = 830;

// Minimum and maximum supported black body temperatures, in Kelvins.
constexpr double kTmin = 500;
constexpr double kTmax = 35000;

// Maximum Doppler intensity factor. Without clamping, large Doppler factors
// cause numerical precision errors, which in turn cause flicker effects.
constexpr double kMaxDopplerIntensityFactor = 1000;

// The speed of light, the Planck constant and the Boltzmann constant.
constexpr double c = 299792458.0;
constexpr double h = 6.62607015e-34;
constexpr double kB = 1.380649e-23;

// Two fixed absorption bands 1 and 2, used to modify a black body spectrum in
// order to get a given chromaticity.
double Band1(const double lambda) {
  constexpr double lambda0 = 455.0;
  return exp(-0.0006 * (lambda - lambda0) * (lambda - lambda0));
}

double Band2(const double lambda) {
  constexpr double lambda0 = 535.0;
  return exp(-0.0004 * (lambda - lambda0) * (lambda - lambda0));
}

// Returns whether the given absorption coefficients for the above absorption
// bands are valid (the total absorption at any given wavelength must be less
// than 100%).
bool ValidAbsorption(const double alpha, const double beta) {
  for (int lambda = kLambdaMin; lambda < kLambdaMax; ++lambda) {
    if (alpha * Band1(lambda) + beta * Band2(lambda) >= 1.0) {
      return false;
    }
  }
  return true;
}

// A spectrum, modeled as a black body spectrum B with some absorption in the
// absorption bands 1 and 2, and scaled so that the sum of the components of the
// corresponding XYZ color is 1: S(lambda)=B_temperature(lambda) * (1 - alpha *
// Band1(lambda) - beta * Band2(lambda)) * scale. The absorption coefficients
// can be negative, in which case they correspond to emission coefficients
// instead.
struct Spectrum {
  // The spectrum parameters (see above).
  double temperature, alpha, beta, scale;
  // The corresponding CIE xy chromaticity.
  double x, y;

  // The XYZ colors corresponding to B_temperature(lambda) * Band1(lambda) and
  // B_temperature(lambda) * Band2(lambda). Must be initialized with
  // ComputeBandColors().
  Xyz a, b;

  Spectrum() {}

  // Creates a pure black-body spectrum.
  explicit Spectrum(const double temperature)
      : temperature(temperature), alpha(0.0), beta(0.0) {
    const Xyz xyz = ComputeXyzFromTemperature(temperature);
    scale = 1.0 / (xyz[0] + xyz[1] + xyz[2]);
    x = xyz[0] * scale;
    y = xyz[1] * scale;
  }

  // Creates a black-body spectrum with some absorption in the absorption bands
  // 1 and 2.
  Spectrum(const double temperature, const double alpha, const double beta)
      : temperature(temperature), alpha(alpha), beta(beta), scale(1.0) {
    const Xyz xyz = ComputeColor(1.0);
    scale = 1.0 / (xyz[0] + xyz[1] + xyz[2]);
    x = xyz[0] * scale;
    y = xyz[1] * scale;
  }

  // Computes the XYZ colors corresponding to B_temperature(lambda) *
  // Band1(lambda) and B_temperature(lambda) * Band2(lambda).
  void ComputeBandColors() {
    Spectrum alpha = *this;
    Spectrum beta = *this;
    alpha.alpha = 1.0;
    beta.beta = 1.0;
    const Xyz xyz = ComputeColor(1.0);
    a = xyz - alpha.ComputeColor(1.0);
    b = xyz - beta.ComputeColor(1.0);
  }

  // Computes the distance between two spectrums, as the distance between their
  // r,g chromaticity (i.e. r/(r+g+b), g/(r+g+b) in linear sRGB space).
  double Distance(const Spectrum& other) const {
    LinearSrgb srgb = ToLinearSrgb(Xyz(x, y, 1.0 - x - y));
    LinearSrgb other_srgb =
        ToLinearSrgb(Xyz(other.x, other.y, 1.0 - other.x - other.y));
    srgb = srgb / (srgb[0] + srgb[1] + srgb[2]);
    other_srgb = other_srgb / (other_srgb[0] + other_srgb[1] + other_srgb[2]);

    const double dr = srgb[0] - other_srgb[0];
    const double dg = srgb[1] - other_srgb[1];
    return sqrt(dr * dr + dg * dg);
  }

  // Computes the value of this spectrum at the given wavelength in nanometer.
  double ComputeValue(const double lambda) const {
    const double l = lambda * 1e-9;
    // The black body spectrum as a function of wavelength, see
    // https://en.wikipedia.org/wiki/Planck%27s_law#The_law.
    const double black_body_spectrum =
        (2.0 * h * c * c) /
        (pow(l, 5.0) * (exp(h * c / (l * kB * temperature)) - 1.0));
    return black_body_spectrum *
           (1.0 - alpha * Band1(lambda) - beta * Band2(lambda)) * scale;
  }

  // Computes the XYZ color corresponding to this spectrum, modified with the
  // Doppler factor D = received frequency / emitted frequency. D < 1 is a red
  // shift, and D > 1 a blue shift. The received intensity is
  // I_received(lambda) dlambda = D^5 I_emitted(D * lambda) * dlambda. It is
  // smaller than the emitted intensity for a red shift, and larger for a blue
  // shift.
  Xyz ComputeColor(const double D) const {
    Xyz xyz;
    for (int lambda = kLambdaMin; lambda < kLambdaMax; ++lambda) {
      const double value = ComputeValue(D * lambda);
      const double x_match = CieColorMatchingFunctionTableValue(lambda, 1);
      const double y_match = CieColorMatchingFunctionTableValue(lambda, 2);
      const double z_match = CieColorMatchingFunctionTableValue(lambda, 3);
      xyz = xyz + Xyz(x_match, y_match, z_match) * value;
    }
    constexpr double dlambda = 1e-9;
    return xyz * dlambda * pow(D, 5.0);
  }
};

// Computes black body spectrums for the intermediate temperatures between the
// temperatures of two given black body spectrums.
void ComputeBlackBodySpectrums(const Spectrum& start, const Spectrum& end,
                               std::vector<Spectrum>* black_body_spectrums) {
  if (start.Distance(end) < 1.0 / kDopplerTextureWidth) {
    black_body_spectrums->push_back(end);
  } else {
    Spectrum middle((start.temperature + end.temperature) / 2.0f);
    ComputeBlackBodySpectrums(start, middle, black_body_spectrums);
    ComputeBlackBodySpectrums(middle, end, black_body_spectrums);
  }
}

// Returns the spectrum in 'spectrums' whose CIE xy chromaticity is the nearest
// to the given one.
const Spectrum& FindNearestSpectrum(const std::vector<Spectrum>& spectrums,
                                    const double x, const double y) {
  int min_index = 0;
  double min_dist = 1.0;
  for (unsigned int i = 0; i < spectrums.size(); ++i) {
    const Spectrum& s = spectrums[i];
    double dist = sqrt((x - s.x) * (x - s.x) + (y - s.y) * (y - s.y));
    if (dist < min_dist) {
      min_index = i;
      min_dist = dist;
    }
  }
  return spectrums[min_index];
}

// Returns true if there exists a Spectrum, with the same black body temperature
// as 'spectrum' but with different absorption coefficients in the absorption
// bands 1 & 2, whose CIE xy chromaticity is equal to the given one. If so,
// returns it in 'result'.
bool FindSpectrum(const Spectrum& spectrum, const double x, const double y,
                  Spectrum* result) {
  const Xyz& a = spectrum.a;
  const Xyz& b = spectrum.b;
  const double m00 = x * (a[0] + a[1] + a[2]) - a[0];
  const double m01 = x * (b[0] + b[1] + b[2]) - b[0];
  const double m10 = y * (a[0] + a[1] + a[2]) - a[1];
  const double m11 = y * (b[0] + b[1] + b[2]) - b[1];
  const double c0 = x - spectrum.x;
  const double c1 = y - spectrum.y;
  const double det = m00 * m11 - m10 * m01;
  const double alpha = (c0 * m11 - c1 * m01) / det;
  const double beta = (c1 * m00 - c0 * m10) / det;
  if (alpha <= 1.0 && beta <= 1.0 && ValidAbsorption(alpha, beta)) {
    *result = Spectrum(spectrum.temperature, alpha, beta);
    return true;
  }
  return false;
}

// Computes and saves a 3D texture containing the sRGB color corresponding to
// a given chromaticity, shifted by a given Doppler factor. A 'Spectrum' (see
// above) is found from each chromaticity, shifted by the Doppler factor (and
// amplified by the corresponding relativistic beaming effect, see
// https://en.wikipedia.org/wiki/Relativistic_beaming), and converted back to
// linear sRGB.
void ComputeDopplerTexture(const std::string& file_name) {
  const Spectrum tmin_spectrum(kTmin);
  const Spectrum tmax_spectrum(kTmax);
  std::vector<Spectrum> black_body_spectrums;
  black_body_spectrums.push_back(tmin_spectrum);
  ComputeBlackBodySpectrums(tmin_spectrum, tmax_spectrum,
                            &black_body_spectrums);
  for (unsigned int i = 0; i < black_body_spectrums.size(); ++i) {
    black_body_spectrums[i].ComputeBandColors();
  }

  std::unique_ptr<float[]> texture(new float[kDopplerTextureSize * 3]);
  for (int i = 0; i < kDopplerTextureSize; ++i) {
    texture[i] = 0.0;
  }
  for (int j = 0; j < kDopplerTextureHeight; ++j) {
    for (int i = 0; i < kDopplerTextureWidth; ++i) {
      double r = (i + 0.5) / kDopplerTextureWidth;
      double g = (j + 0.5) / kDopplerTextureWidth;
      if (g > 1.0 - r) {
        r = 1.0 - (j + 0.5) / kDopplerTextureWidth;
        g = 1.0 - (i + 0.5) / kDopplerTextureWidth;
      }

      const Xyz xyz = ToXyz(LinearSrgb(r, g, 1.0 - r - g));
      const double x = xyz[0] / (xyz[0] + xyz[1] + xyz[2]);
      const double y = xyz[1] / (xyz[0] + xyz[1] + xyz[2]);

      Spectrum s;
      if (!FindSpectrum(FindNearestSpectrum(black_body_spectrums, x, y), x, y,
                        &s)) {
        continue;
      }

      for (int k = 0; k < kDopplerTextureDepth; ++k) {
        const double D =
            exp(0.21 * tan(3.0 * (k / (kDopplerTextureDepth - 1.0) - 0.5)));
        const Xyz xyz = s.ComputeColor(D);
        LinearSrgb linear_srgb = ToClampedLinearSrgb(xyz);
        const float sum = linear_srgb[0] + linear_srgb[1] + linear_srgb[2];
        if (sum > kMaxDopplerIntensityFactor) {
          linear_srgb = linear_srgb / sum * kMaxDopplerIntensityFactor;
        }
        const int offset = i + j * kDopplerTextureWidth +
                           k * kDopplerTextureWidth * kDopplerTextureHeight;
        texture[3 * offset] = std::max(linear_srgb[0], 0.0);
        texture[3 * offset + 1] = std::max(linear_srgb[1], 0.0);
        texture[3 * offset + 2] = std::max(linear_srgb[2], 0.0);
      }
    }
  }

  std::ofstream output_stream(file_name,
                              std::ofstream::out | std::ofstream::binary);
  output_stream.write((const char*)texture.get(),
                      (kDopplerTextureSize * 3) * sizeof(float));
  output_stream.close();
}

}  // namespace
}  // namespace color_maps

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: doppler_texture_generator <output file>"
              << std::endl;
    return -1;
  }
  color_maps::ComputeDopplerTexture(argv[1]);
}
