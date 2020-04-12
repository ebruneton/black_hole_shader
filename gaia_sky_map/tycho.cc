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

#include "gaia_sky_map/tycho.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>

using dimensional::vec3;

namespace gaia_sky_map {

namespace {

constexpr char kTychoMainFileName[] = "tyc2.dat";
constexpr char kTychoSupplementaryFileName[] = "suppl_1.dat";

// Converts coordinates in the International Celestial Reference System into
// cartesian galactic coordinates, using the matrix from
// http://gea.esac.esa.int/archive/documentation/GDR2/Data_processing/
// chap_cu3ast/sec_cu3ast_intro/ssec_cu3ast_intro_tansforms.html#SSS1
vec3 RaDeclDegreeToGalacticXyz(const double ra, const double dec) {
  const double alpha = ra / 180.0 * M_PI;
  const double delta = dec / 180.0 * M_PI;
  const double x0 = cos(delta) * cos(alpha);
  const double y0 = cos(delta) * sin(alpha);
  const double z0 = sin(delta);
  return vec3(-0.0548755604162154 * x0 - 0.8734370902348850 * y0 -
                  0.4838350155487132 * z0,
              0.4941094278755837 * x0 - 0.4448296299600112 * y0 +
                  0.7469822444972189 * z0,
              -0.8676661490190047 * x0 - 0.1980763734312015 * y0 +
                  0.4559837761750669 * z0);
}

// Return the Gaia DR2 flux in the G passband, from the Gaia DR2 magnitude in
// this passband. Relation fitted from many phot_g_mean_mag and phot_g_mean_flux
// value pairs.
double GfluxFromGmag(const double g_mag) {
  return 1.885e10 * pow(10, -0.4 * g_mag);
}

double TemperatureFromBtVt(const double bt, const double vt) {
  // B-V, from Readme of Tycho data (clamped to a safe interval).
  const double bv = std::max(-0.5, std::min(2.0, 0.850 * (bt - vt)));
  // Temperature from B-V, using table 5 from "Transformations from Theoretical
  // Hertzsprung-Russell Diagrams to Color-Magnitude Diagrams: Effective
  // Temperatures, B-V Colors, and Bolometric Corrections"
  // (http://articles.adsabs.harvard.edu/pdf/1996ApJ...469..355F).
  constexpr double a = 3.979145;
  constexpr double b = -0.654499;
  constexpr double c = 1.740690;
  constexpr double d = -4.608815;
  constexpr double e = 6.792600;
  constexpr double f = -5.396910;
  constexpr double g = 2.192970;
  constexpr double h = -0.359496;
  const double log_temperature =
      ((((((h * bv + g) * bv + f) * bv + e) * bv + d) * bv + c) * bv + b) * bv +
      a;
  return pow(10.0, log_temperature);
}

void InitTychoCubeMap(CubeMap<Xyz>* tycho_cube_map) {
  for (int face = 0; face < 6; ++face) {
    for (int j = 0; j < kCubeMapSize; ++j) {
      for (int i = 0; i < kCubeMapSize; ++i) {
        tycho_cube_map->texel(face, i, j) = Xyz();
      }
    }
  }
}

void ProcessTychoData(const double ra, const double dec, const double bt,
                      const double vt, CubeMap<Xyz>* tycho_cube_map) {
  // Computes the magnitude in the Gaia DR2 G passband, from the magnitudes in
  // the Tycho Bt and Vt passbands, using the relation from Table 4 in "Gaia
  // broad band photometry" (https://arxiv.org/pdf/1008.0815.pdf).
  const double bv = bt - vt;
  const double g_mag =
      vt - 0.0260 - 0.1767 * bv - 0.2980 * bv * bv + 0.1393 * bv * bv * bv;
  if (g_mag > 7.0) {
    return;
  }

  const vec3 dir = RaDeclDegreeToGalacticXyz(ra, dec);
  Xyz& xyz = tycho_cube_map->texel(dir);

  const double g_flux = GfluxFromGmag(g_mag);
  const Xyzg xyzg = XyzgFromTemperature(TemperatureFromBtVt(bt, vt));
  // TODO(me): use ToXyz(xyzg) / xyzg.g() * g_flux instead?
  xyz = xyz + ToXyz(xyzg) / xyzg.y() * g_flux;
}

int ProcessMainTychoData(const std::string& input_dir,
                         CubeMap<Xyz>* tycho_cube_map) {
  std::cout << "Reading Tycho2 main data... ";
  std::ifstream in(input_dir + kTychoMainFileName);
  std::string str;
  int num_stars = 0;
  while (std::getline(in, str)) {
    try {
      const double ra = std::stod(str.substr(152, 12), nullptr);
      const double dec = std::stod(str.substr(165, 12), nullptr);
      const bool has_bt = str.substr(110, 6) != "      ";
      const bool has_vt = str.substr(123, 6) != "      ";
      const double bt = std::stod(str.substr(has_bt ? 110 : 123, 6), nullptr);
      const double vt = std::stod(str.substr(has_vt ? 123 : 110, 6), nullptr);
      ProcessTychoData(ra, dec, bt, vt, tycho_cube_map);
      num_stars++;
    } catch (std::exception e) {
      std::cerr << "Error! " << str << std::endl;
      continue;
    }
  }
  in.close();
  std::cout << "OK." << std::endl;
  return num_stars;
}

int ProcessSupplementaryTychoData(const std::string& input_dir,
                                  CubeMap<Xyz>* tycho_cube_map) {
  std::cout << "Reading Tycho2 supplementary data... ";
  std::ifstream in(input_dir + kTychoSupplementaryFileName);
  std::string str;
  int num_stars = 0;
  while (std::getline(in, str)) {
    try {
      const double ra = std::stod(str.substr(15, 12), nullptr);
      const double dec = std::stod(str.substr(28, 12), nullptr);
      const bool has_bt = str.substr(83, 6) != "      ";
      const bool has_vt = str.substr(96, 6) != "      ";
      const double bt = std::stod(str.substr(has_bt ? 83 : 96, 6), nullptr);
      const double vt = std::stod(str.substr(has_vt ? 96 : 83, 6), nullptr);
      ProcessTychoData(ra, dec, bt, vt, tycho_cube_map);
      num_stars++;
    } catch (std::exception e) {
      std::cerr << "Error! " << str << std::endl;
      continue;
    }
  }
  in.close();
  std::cout << "OK." << std::endl;
  return num_stars;
}

}  // namespace

int ReadTycho2Data(const std::string& input_dir, CubeMap<Xyz>* tycho_cube_map) {
  InitTychoCubeMap(tycho_cube_map);
  return ProcessMainTychoData(input_dir, tycho_cube_map) +
         ProcessSupplementaryTychoData(input_dir, tycho_cube_map);
}

}  // namespace gaia_sky_map
