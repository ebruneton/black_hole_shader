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

#include "color_maps/colors.h"

namespace color_maps {
namespace {

// Computes and saves a 1D texture containing the linear sRGB color
// corresponding to a black body at a given temperature. Note that this is a
// color and not a chromaticity: the color is not normalized to a constant
// luminosity. It is much larger for high temperatures than for low
// temperatures.
void ComputeBlackBodyColorTexture(const std::string& file_name) {
  constexpr int N = 128;
  std::unique_ptr<float[]> texture(new float[N * 3]);
  for (int i = 0; i < N; ++i) {
    const double temperature = 100.0 * exp(6.0 * (i + 0.5) / N);
    const Xyz xyz = ComputeXyzFromTemperature(temperature);
    const LinearSrgb linear_srgb = ToClampedLinearSrgb(xyz);
    texture[3 * i] = std::max(linear_srgb[0], 0.0);
    texture[3 * i + 1] = std::max(linear_srgb[1], 0.0);
    texture[3 * i + 2] = std::max(linear_srgb[2], 0.0);
  }
  std::ofstream output_stream(file_name,
                              std::ofstream::out | std::ofstream::binary);
  output_stream.write((const char*)texture.get(), (N * 3) * sizeof(float));
  output_stream.close();
}

}  // namespace
}  // namespace color_maps

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: black_body_texture_generator <output file>"
              << std::endl;
    return -1;
  }
  color_maps::ComputeBlackBodyColorTexture(argv[1]);
}
