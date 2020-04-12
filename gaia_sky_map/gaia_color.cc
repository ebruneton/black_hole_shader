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

#include "gaia_sky_map/gaia_color.h"

#include <algorithm>
#include <random>

#include "png++/png.hpp"

namespace gaia_sky_map {

namespace {

constexpr char kGaiaColorMapFileName[] = "gaia.png";

// Replaces the black and greenish pixels with neighboring ones.
void CleanImage(png::image<png::rgba_pixel>* image) {
  std::cout << "Cleaning image... ";
  std::mt19937_64 generator(1234);
  std::uniform_int_distribution<> distribution(-10, 10);

  // Fix a greenish patch by replacing it with a neighboring one.
  const int width = image->get_width();
  const int height = image->get_height();
  const int x0 = static_cast<int>(2890.0 / 8000.0 * width);
  const int y0 = static_cast<int>(2650.0 / 8000.0 * width);
  const int x1 = static_cast<int>(3050.0 / 8000.0 * width);
  const int y1 = static_cast<int>(2810.0 / 8000.0 * width);
  const int dx = static_cast<int>(-300.0 / 8000.0 * width);
  const int dy = static_cast<int>(200.0 / 8000.0 * width);
  for (int y = y0; y < y1; ++y) {
    for (int x = x0; x < x1; ++x) {
      (*image)[y][x] = (*image)[y + dy][x + dx];
    }
  }

  const auto is_bad_pixel = [](const png::rgba_pixel& p) {
    if (p.alpha == 255) {
      if (p.red == 0 && p.green == 0 && p.blue == 0) {
        return true;
      }
      if (p.green > 1.05 * std::max(p.red, p.blue)) {
        return true;
      }
    }
    return false;
  };

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      png::rgba_pixel p = (*image)[y][x];
      while (is_bad_pixel(p)) {
        int dx = distribution(generator);
        int dy = distribution(generator);
        if (x + dx >= 0 && x + dx < width && y + dy >= 0 && y + dy < height) {
          png::rgba_pixel q = (*image)[y + dy][x + dx];
          if (q.alpha == 255) {
            p = q;
          }
        }
      }
      (*image)[y][x] = p;
    }
  }

  std::cout << "OK." << std::endl;
}

GaiaColorMap ConvertImageToXyz(const png::image<png::rgba_pixel>& image) {
  std::cout << "Converting image to XYZ... ";
  GaiaColorMap color_map(image.get_width(), image.get_height());

  const int width = image.get_width();
  const int height = image.get_height();
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const png::rgba_pixel p = image[y][x];
      const Srgb srgb = Srgb(p.red / 255.0, p.green / 255.0, p.blue / 255.0);
      color_map.pixel(x, y) = ToXyz(ToLinearSrgb(srgb));
    }
  }

  std::cout << "OK." << std::endl;
  return color_map;
}

}  // namespace

GaiaColorMap::GaiaColorMap(const int width, const int height)
    : width_(width), height_(height), pixels_(new Xyz[width * height]) {}

GaiaColorMap ReadGaiaColorMap(const std::string& input_dir) {
  std::cout << "Reading Gaia color map... ";
  png::image<png::rgba_pixel> image(input_dir + kGaiaColorMapFileName);
  std::cout << "OK." << std::endl;

  CleanImage(&image);
  return ConvertImageToXyz(image);
}

}  // namespace gaia_sky_map
