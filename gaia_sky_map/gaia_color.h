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

#ifndef GAIA_SKY_MAP_GAIA_COLOR_H_
#define GAIA_SKY_MAP_GAIA_COLOR_H_

#include <memory>
#include <string>

#include "gaia_sky_map/colors.h"

namespace gaia_sky_map {

class GaiaColorMap {
 public:
  GaiaColorMap(int width, int height);

  int width() const { return width_; }
  int height() const { return height_; }
  const Xyz& pixel(const int x, const int y) const {
    return pixels_[x + y * width_];
  }
  Xyz& pixel(const int x, const int y) { return pixels_[x + y * width_]; }

 private:
  int width_;
  int height_;
  std::unique_ptr<Xyz[]> pixels_;
};

// Reads the "Gaia DR2 sky in color" map in PNG format, cleans it and converts
// it to the CIE XYZ color space.
GaiaColorMap ReadGaiaColorMap(const std::string& input_dir);

}  // namespace gaia_sky_map

#endif  // GAIA_SKY_MAP_GAIA_COLOR_H_
