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

#include <fstream>
#include <iostream>
#include "black_hole/preprocess/functions.h"
#include "math/binary_function.h"

namespace black_hole {
namespace preprocess {
namespace {

using dimensional::BinaryFunction;

template <unsigned int NX, unsigned int NY, class T1, class T2>
void SaveTexture(const BinaryFunction<NX, NY, Tuple2<T1, T2>>& texture,
                 const std::string& file_name) {
  std::unique_ptr<float[]> pixels(new float[2 * NX * NY + 2]);
  pixels[0] = static_cast<float>(NX);
  pixels[1] = static_cast<float>(NY);
  for (unsigned int y = 0; y < NY; ++y) {
    for (unsigned int x = 0; x < NX; ++x) {
      const auto& texel = texture.Get(x, y);
      const int offset = 2 * (x + y * NX) + 2;
      pixels[offset] = static_cast<float>(texel.x.to(T1::Unit()));
      pixels[offset + 1] = static_cast<float>(texel.y.to(T2::Unit()));
    }
  }
  std::ofstream output_stream(file_name,
                              std::ofstream::out | std::ofstream::binary);
  output_stream.write((const char*)pixels.get(),
                      (2 * NX * NY + 2) * sizeof(float));
  output_stream.close();
}

void Main(const std::string& output_dir) {
  RayDeflectionTexture ray_deflection_texture;
  ComputeRayDeflectionTexture(&ray_deflection_texture);
  SaveTexture(ray_deflection_texture, output_dir + "deflection.dat");

  RayInverseRadiusTexture ray_inverse_radius_texture;
  ComputeRayInverseRadiusTexture(&ray_inverse_radius_texture);
  SaveTexture(ray_inverse_radius_texture, output_dir + "inverse_radius.dat");
}

}  // namespace
}  // namespace preprocess
}  // namespace black_hole

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: preprocess <output folder>" << std::endl;
    return -1;
  }
  black_hole::preprocess::Main(argv[1]);
  return 0;
}
