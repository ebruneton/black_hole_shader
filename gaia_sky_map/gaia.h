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

#ifndef GAIA_SKY_MAP_GAIA_H_
#define GAIA_SKY_MAP_GAIA_H_

#include <string>

#include "gaia_sky_map/colors.h"
#include "gaia_sky_map/cube_map.h"

namespace gaia_sky_map {

// A texel of a Gaia DR2 cube map.
struct GaiaTexel {
  // Sum of the fluxes in the CIE X,Y,Z and Gaia DR2 G band, of the stars with
  // an effective temperature and a magnitude > 7.
  Xyzg xyzg_sum;

  // Number of stars with an effective temperature and a magnitude > 7.
  unsigned int star_count;

  // Sum of the effective temperature of the stars with an effective
  // temperature and a magnitude > 7.
  float temperature_sum;

  // Sum of the square of the effective temperature of the stars with an
  // effective temperature and a magnitude > 7.
  float temperature_square_sum;

  // Sum of the fluxes in the Gaia DR2 G band, of the stars without an effective
  // temperature, but with a magnitude > 7.
  double additional_g_sum;

  // Number of stars without an effective temperature, but with a magnitude > 7.
  unsigned additional_star_count;
};

// A Gaia DR2 cube map.
typedef CubeMap<GaiaTexel> GaiaCubeMap;

// Downloads the Gaia DR2 main table and converts it into a GaiaCubeMap.
// If the download has already been done and cached on disk, read the cached
// data from disk instead.
bool DownloadOrReadGaiaDr2Data(const std::string& output_dir,
                               GaiaCubeMap* gaia_cube_map);

}  // namespace gaia_sky_map

#endif  // GAIA_SKY_MAP_GAIA_H_
