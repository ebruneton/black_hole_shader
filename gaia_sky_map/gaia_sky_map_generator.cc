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
#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <vector>

#include "gaia_sky_map/cube_map.h"
#include "gaia_sky_map/gaia.h"
#include "gaia_sky_map/gaia_color.h"
#include "gaia_sky_map/tycho.h"
#include "rgb9_e5/rgb9_e5.h"

using dimensional::vec3;

namespace gaia_sky_map {
namespace {

unsigned int ToRgb9E5(const Xyz& xyz) {
  const LinearSrgb linear_srgb = ToLinearSrgb(xyz);
  return to_rgb9e5(linear_srgb[0], linear_srgb[1], linear_srgb[2]);
}

// Computes a "galaxy" cube map (which will eventually contain only the
// 'area' light sources) by combining the Gaia color and cube maps.
void TransferColors(const GaiaColorMap& gaia_color_map,
                    const GaiaCubeMap& gaia_cube_map,
                    CubeMap<Xyz>* galaxy_cube_map) {
  std::cout << "Transfering Gaia colors... ";
  const int width = gaia_color_map.width();
  const int height = gaia_color_map.height();
  for (int face = 0; face < 6; ++face) {
    for (int y = 0; y < kCubeMapSize; ++y) {
      for (int x = 0; x < kCubeMapSize; ++x) {
        const vec3 dir = ConvertCubeMapFaceXyToDirection(face, x, y);
        // Convert from galactic coordinates to Gaia color map coordinates
        // (using Hammer projection).
        const double phi = M_PI / 2 - acos(dir.z() / length(dir)());
        const double lambda = -atan2(dir.y(), dir.x());  // Note l flipped!
        const double ga = sqrt(1.0 + cos(phi) * cos(lambda / 2.0));
        const double gx = cos(phi) * sin(lambda / 2.0) / ga;
        const double gy = sin(phi) / ga;
        const int ix = static_cast<int>(width / 2 + gx * width / 2);
        const int iy = static_cast<int>(height / 2 - gy * height / 2);
        // Combine the Gaia color map chromaticity with Gaia cube map intensity.
        const Xyz& color =
            gaia_color_map.pixel(std::max(0, std::min(ix, width - 1)),
                                 std::max(0, std::min(iy, height - 1)));
        const double luminance =
            gaia_cube_map.texel(face, x, y).xyzg_sum.y() +
            gaia_cube_map.texel(face, x, y).additional_g_sum;
        if (color[1] > 0.0) {
          galaxy_cube_map->texel(face, x, y) = color / color[1] * luminance;
        } else {
          galaxy_cube_map->texel(face, x, y) = Xyz();
        }
      }
    }
  }
  std::cout << "OK." << std::endl;
}

// Removes the outliers from the galaxy cube map and transfers them to the stars
// cube map (which will enventually contain only the punctual light sources).
void FilterStars(CubeMap<Xyz>* galaxy_cube_map, CubeMap<Xyz>* star_cube_map) {
  std::cout << "Filtering stars... ";
  std::unique_ptr<CubeMapFace<double>> filter(new CubeMapFace<double>);
  for (int face = 0; face < 6; ++face) {
    for (int j = 0; j < kCubeMapSize; ++j) {
      for (int i = 0; i < kCubeMapSize; ++i) {
        double y_sum = 0.0;
        int count = 0;
        for (int dj = -1; dj <= 1; ++dj) {
          for (int di = -1; di <= 1; ++di) {
            if ((di != 0 || dj != 0) &&
                (i + di >= 0 && i + di < kCubeMapSize) &&
                (j + dj >= 0 && j + dj < kCubeMapSize)) {
              y_sum += galaxy_cube_map->texel(face, i + di, j + dj)[1];
              count += 1;
            }
          }
        }
        const double y = galaxy_cube_map->texel(face, i, j)[1];
        const double y_avg = y_sum / count;
        if (y > 2.0 * y_avg) {
          filter->texel(i, j) = (y - y_avg) / y;
        } else {
          filter->texel(i, j) = 0.0;
        }
      }
    }

    for (int j = 0; j < kCubeMapSize; ++j) {
      for (int i = 0; i < kCubeMapSize; ++i) {
        const Xyz xyz = galaxy_cube_map->texel(face, i, j);
        const double lerp = filter->texel(i, j);
        galaxy_cube_map->texel(face, i, j) = xyz * (1.0 - lerp);
        star_cube_map->texel(face, i, j) =
            star_cube_map->texel(face, i, j) + xyz * lerp;
      }
    }
  }
  std::cout << "OK." << std::endl;
}

// Removes more outliers from the galaxy cube map.
void FilterMoreStars(CubeMap<Xyz>* galaxy_cube_map, CubeMap<Xyz>* star_cube_map,
                     double* max_galaxy_value, double* max_star_value) {
  std::cout << "Filtering more stars... ";
  double blur_weight[9][9];
  for (int i = -4; i <= 4; ++i) {
    for (int j = -4; j <= 4; ++j) {
      blur_weight[i + 4][j + 4] = exp(-0.4 * (i * i + j * j));
    }
  }

  std::unique_ptr<CubeMapFace<Xyz>> src(new CubeMapFace<Xyz>);
  *max_galaxy_value = 0.0;
  *max_star_value = 0.0;
  for (int face = 0; face < 6; ++face) {
    for (int j = 0; j < kCubeMapSize; ++j) {
      for (int i = 0; i < kCubeMapSize; ++i) {
        src->texel(i, j) = galaxy_cube_map->texel(face, i, j);
      }
    }
    for (int j = 0; j < kCubeMapSize; ++j) {
      for (int i = 0; i < kCubeMapSize; ++i) {
        double y_sum = 0.0;
        double y_square_sum = 0.0;
        double weight_sum = 0.0;
        Xyz xyz_weighted_sum;
        int count = 0;
        for (int dj = -4; dj <= 4; ++dj) {
          for (int di = -4; di <= 4; ++di) {
            if (i + di >= 0 && i + di < kCubeMapSize && j + dj >= 0 &&
                j + dj < kCubeMapSize) {
              const Xyz& xyz = src->texel(i + di, j + dj);
              double weight = blur_weight[di + 4][dj + 4];

              y_sum += xyz[1];
              y_square_sum += xyz[1] * xyz[1];
              weight_sum += weight;
              xyz_weighted_sum = xyz_weighted_sum + xyz * weight;
              count += 1;
            }
          }
        }
        const double src_y = src->texel(i, j)[1];
        double y_avg = y_sum / count;
        double y_sigma = sqrt(y_square_sum / count - y_avg * y_avg);
        Xyz& galaxy_texel = galaxy_cube_map->texel(face, i, j);
        Xyz& star_texel = star_cube_map->texel(face, i, j);
        if (src_y > y_avg + 2.0 * y_sigma) {
          galaxy_texel = xyz_weighted_sum / weight_sum;
        }
        *max_galaxy_value = std::max(*max_galaxy_value, galaxy_texel[0]);
        *max_galaxy_value = std::max(*max_galaxy_value, galaxy_texel[1]);
        *max_galaxy_value = std::max(*max_galaxy_value, galaxy_texel[2]);
        *max_star_value = std::max(*max_star_value, star_texel[0]);
        *max_star_value = std::max(*max_star_value, star_texel[1]);
        *max_star_value = std::max(*max_star_value, star_texel[2]);
      }
    }
  }
  std::cout << "OK. (galaxy max = " << *max_galaxy_value
            << ", star max = " << *max_star_value << ")" << std::endl;
}

// The final cube map is mip-mapped and split in tiles. For instance, at mipmap
// level 0, a 2048x2048 face of the cube map is split in 8x8 tiles of 256x256
// pixels each. Each tile contains the galaxy cube map data, followed by the
// star cube map data, and thus contains 2x256x256 pixels. Levels 4 to 11 are
// saved together in a single file per face: this file contains the 128x128
// (single) tile of level 4, followed by the 64x64 (single) tile of level 5,
// etc.
constexpr int kTileSize = 256;

// Computes the data of the (level,tile_i,tile_j) tile in the mipmap pyramid.
std::vector<unsigned int> ComputeMipMapTile(
    const int face, const int level, const int tile_i, const int tile_j,
    const CubeMap<Xyz>& galaxy_cube_map, const CubeMap<Xyz>& star_cube_map,
    const CubeMapFace<double>& texel_area, const double galaxy_scale,
    const double star_scale) {
  const int filter_width = 1 << level;
  const int level_size = kCubeMapSize / filter_width;
  const int tile_size = std::min(kTileSize, level_size);
  std::vector<unsigned int> pixels(2 * tile_size * tile_size);
  for (int j = 0; j < tile_size; ++j) {
    const int j0 = (tile_j * tile_size + j) * filter_width;
    for (int i = 0; i < tile_size; ++i) {
      const int i0 = (tile_i * tile_size + i) * filter_width;
      Xyz galaxy_xyz_sum;
      Xyz star_xyz_sum;
      double area = 0.0;
      for (int dj = 0; dj < filter_width; ++dj) {
        for (int di = 0; di < filter_width; ++di) {
          galaxy_xyz_sum =
              galaxy_xyz_sum + galaxy_cube_map.texel(face, i0 + di, j0 + dj);
          star_xyz_sum =
              star_xyz_sum + star_cube_map.texel(face, i0 + di, j0 + dj);
          area += texel_area.texel(i0 + di, j0 + dj);
        }
      }
      galaxy_xyz_sum = galaxy_xyz_sum / area * galaxy_scale;
      star_xyz_sum = star_xyz_sum * star_scale;
      if (level > 4) {
        star_xyz_sum = star_xyz_sum / area;
      }
      pixels[i + j * tile_size] = ToRgb9E5(galaxy_xyz_sum);
      pixels[i + j * tile_size + tile_size * tile_size] =
          ToRgb9E5(star_xyz_sum);
    }
  }
  return pixels;
}

// Computes and saves in a single file the data of the (face, level, tile_i,
// tile_j) tile in the mipmap pyramid (plus all the tiles at the coarser levels
// if the tile size at 'level' is strictly less than kTileSize). Returns true if
// the function needs to be called again with 'level + 1' to compute and save
// more mipmap levels.
bool SaveMipMap(const std::string& output_dir, const int face, const int level,
                const CubeMap<Xyz>& galaxy_cube_map,
                const CubeMap<Xyz>& star_cube_map,
                const CubeMapFace<double>& texel_area,
                const double galaxy_scale, const double star_scale) {
  const std::string prefixes[6] = {"pos-x-", "neg-x-", "pos-y-",
                                   "neg-y-", "pos-z-", "neg-z-"};
  const int level_size = kCubeMapSize >> level;
  const int num_tiles = level_size / std::min(kTileSize, level_size);
  for (int tile_j = 0; tile_j < num_tiles; ++tile_j) {
    for (int tile_i = 0; tile_i < num_tiles; ++tile_i) {
      std::vector<unsigned int> file_data;
      file_data.reserve(kTileSize * kTileSize);
      int current_level = level;
      int current_level_size = level_size;
      while (current_level_size > 0) {
        std::vector<unsigned int> tile_data = ComputeMipMapTile(
            face, current_level, tile_i, tile_j, galaxy_cube_map, star_cube_map,
            texel_area, galaxy_scale, star_scale);
        file_data.insert(file_data.end(), tile_data.begin(), tile_data.end());
        if (level_size < kTileSize) {
          current_level += 1;
          current_level_size /= 2;
        } else {
          break;
        }
      }
      const std::string file_name =
          output_dir + prefixes[face] + std::to_string(level) + "-" +
          std::to_string(tile_i) + "-" + std::to_string(tile_j) + ".dat";
      std::ofstream output_stream(file_name,
                                  std::ofstream::out | std::ofstream::binary);
      output_stream.write((const char*)file_data.data(),
                          file_data.size() * sizeof(unsigned int));
      output_stream.close();
    }
  }
  return level_size >= kTileSize;
}

void Main(const std::string& base_dir) {
  std::cout << std::unitbuf;

  std::unique_ptr<CubeMap<Xyz>> galaxy_cube_map(new CubeMap<Xyz>);
  {
    std::unique_ptr<GaiaCubeMap> gaia_cube_map(new GaiaCubeMap);
    if (!DownloadOrReadGaiaDr2Data(base_dir, gaia_cube_map.get())) {
      return;
    }
    GaiaColorMap gaia_color_map = ReadGaiaColorMap(base_dir);
    TransferColors(gaia_color_map, *gaia_cube_map, galaxy_cube_map.get());
  }
  std::unique_ptr<CubeMap<Xyz>> star_cube_map(new CubeMap<Xyz>);
  ReadTycho2Data(base_dir, star_cube_map.get());

  double max_galaxy_value;
  double max_star_value;
  FilterStars(galaxy_cube_map.get(), star_cube_map.get());
  FilterMoreStars(galaxy_cube_map.get(), star_cube_map.get(), &max_galaxy_value,
                  &max_star_value);

  const double kMaxRgb9e5Value = 6.5e4;
  const double galaxy_scale = kMaxRgb9e5Value / max_galaxy_value;
  const double star_scale = kMaxRgb9e5Value / (2.0 * max_star_value);
  std::cout << "Galaxy scale: " << galaxy_scale << std::endl;
  std::cout << "Star scale: " << star_scale << std::endl;
  std::cout << "Scale ratio: " << star_scale / galaxy_scale << std::endl;

  std::cout << "Saving mipmap... ";
  std::unique_ptr<CubeMapFace<double>> texel_area(new CubeMapFace<double>);
  for (int j = 0; j < kCubeMapSize; ++j) {
    const double v = 2.0 * (j + 0.5) / kCubeMapSize - 1.0;
    for (int i = 0; i < kCubeMapSize; ++i) {
      const double u = 2.0 * (i + 0.5) / kCubeMapSize - 1.0;
      const double theta = atan(sqrt(u * u + v * v));
      const double c = cos(theta);
      texel_area->texel(i, j) = c * c * c;
    }
  }
  for (int face = 0; face < 6; ++face) {
    for (int level = 0; (1 << level) <= kCubeMapSize; ++level) {
      bool has_next_level =
          SaveMipMap(base_dir, face, level, *galaxy_cube_map, *star_cube_map,
                     *texel_area, galaxy_scale, star_scale);
      if (!has_next_level) {
        break;
      }
    }
  }
  std::cout << "OK." << std::endl;
}

}  // namespace
}  // namespace gaia_sky_map

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: gaia_sky_map_generator <output folder>" << std::endl;
    return -1;
  }
  gaia_sky_map::Main(argv[1]);
  return 0;
}
