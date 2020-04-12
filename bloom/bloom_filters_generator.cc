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
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iostream>
#include <memory>
#include <vector>

#include "nnls.h"

namespace bloom {
namespace {

// Provides a tool to generate the bloom filter coefficients for the bloom
// shader effect. The bloom shader effect with unknown filter coefficients is
// applied to an image with a single white pixel, yielding an image whose pixels
// are expressed in terms of the unknown coefficients. A non-negative least
// square fitting is then used to find the best coefficient values such that
// this image best matches the desired bloom filter impulse response.

const double DOWNSAMPLE[4][4] = {
    {1.0 / 81.0, 3.0 / 81.0, 3.0 / 81.0, 1.0 / 81.0},
    {3.0 / 81.0, 9.0 / 81.0, 9.0 / 81.0, 3.0 / 81.0},
    {3.0 / 81.0, 9.0 / 81.0, 9.0 / 81.0, 3.0 / 81.0},
    {1.0 / 81.0, 3.0 / 81.0, 3.0 / 81.0, 1.0 / 81.0}};

const double UPSAMPLE[4][4]{{1.0 / 16.0, 3.0 / 16.0, 3.0 / 16.0, 9.0 / 16.0},
                            {3.0 / 16.0, 1.0 / 16.0, 9.0 / 16.0, 3.0 / 16.0},
                            {3.0 / 16.0, 9.0 / 16.0, 1.0 / 16.0, 3.0 / 16.0},
                            {9.0 / 16.0, 3.0 / 16.0, 3.0 / 16.0, 1.0 / 16.0}};

int Mod(const int x, const int m) {
  int result = x % m;
  return result < 0 ? result + m : result;
}

template <int N>
class Vec {
 public:
  double x[N];

  Vec() {}
  explicit Vec(const double value) {
    for (int i = 0; i < N; ++i) x[i] = value;
  }
  Vec(const int index, const double value) {
    for (int i = 0; i < N; ++i) x[i] = i == index ? value : 0.0;
  }

  Vec<N> operator*(const double value) const {
    Vec<N> result;
    for (int i = 0; i < N; ++i) result.x[i] = x[i] * value;
    return result;
  }

  Vec<N> operator+(const Vec<N>& other) const {
    Vec<N> result;
    for (int i = 0; i < N; ++i) result.x[i] = x[i] + other.x[i];
    return result;
  }

  void operator+=(const Vec<N>& other) {
    for (int i = 0; i < N; ++i) x[i] += other.x[i];
  }
};

template <typename T>
class ImageT {
 public:
  ImageT() {}

  ImageT(const int size, const int border) : size_(size), border_(border) {
    assert(border >= 0);
    pixels_ =
        std::unique_ptr<T[]>(new T[(size + 2 * border) * (size + 2 * border)]);
  }

  int size() const { return size_; }

  const T& Get(const int x, const int y) const {
    assert(x >= -border_ && x < size_ + border_);
    assert(y >= -border_ && y < size_ + border_);
    return pixels_[x + border_ + (y + border_) * (size_ + 2 * border_)];
  }

  void Set(const int x, const int y, const T& value) {
    assert(x >= -border_ && x < size_ + border_);
    assert(y >= -border_ && y < size_ + border_);
    pixels_[x + border_ + (y + border_) * (size_ + 2 * border_)] = value;
  }

  template <typename U>
  ImageT<U> Convolution(const ImageT<U>& kernel) const {
    ImageT<U> result(size_, border_);
    int k_size = kernel.size() / 2;
    for (int y = -border_; y < size_ + border_; ++y) {
      for (int x = -border_; x < size_ + border_; ++x) {
        U value(0.0);
        for (int dy = -k_size; dy <= k_size; ++dy) {
          for (int dx = -k_size; dx < k_size; ++dx) {
            if (x + dx >= -border_ && x + dx < size_ + border_ &&
                y + dy >= -border_ && y + dy < size_ + border_) {
              value +=
                  kernel.Get(dx + k_size, dy + k_size) * Get(x + dx, y + dy);
            }
          }
        }
        result.Set(x, y, value);
      }
    }
    return result;
  }

  ImageT<T> Add(const ImageT<T>& other) const {
    assert(size_ == other.size_);
    assert(border_ >= other.border_);
    ImageT<T> result(size_, border_);
    for (int y = -border_; y < size_ + border_; ++y) {
      for (int x = -border_; x < size_ + border_; ++x) {
        if (x >= -other.border_ && x < size_ + other.border_ &&
            y >= -other.border_ && y < size_ + other.border_) {
          result.Set(x, y, Get(x, y) + other.Get(x, y));
        } else {
          result.Set(x, y, Get(x, y));
        }
      }
    }
    return result;
  }

  ImageT<T> Downsample() const {
    const int half_size = size_ / 2;
    const int half_border = border_ / 2 - 1;
    ImageT<T> result(half_size, half_border);
    for (int y = -half_border; y < half_size + half_border; ++y) {
      for (int x = -half_border; x < half_size + half_border; ++x) {
        T value(0);
        for (int dy = 0; dy < 4; ++dy) {
          for (int dx = 0; dx < 4; ++dx) {
            value += Get(2 * x + dx - 1, 2 * y + dy - 1) * DOWNSAMPLE[dx][dy];
          }
        }
        result.Set(x, y, value);
      }
    }
    return result;
  }

  ImageT<T> Upsample() const {
    const int double_size = size_ * 2;
    const int double_border = border_ - 1;
    ImageT<T> result(double_size, double_border);
    for (int y = -double_border; y < double_size + double_border; ++y) {
      for (int x = -double_border; x < double_size + double_border; ++x) {
        int sx = x <= 0 ? -(2 - x) / 2 : (x - 1) / 2;
        int sy = y <= 0 ? -(2 - y) / 2 : (y - 1) / 2;
        const T& c0 = Get(sx, sy);
        const T& c1 = Get(sx + 1, sy);
        const T& c2 = Get(sx, sy + 1);
        const T& c3 = Get(sx + 1, sy + 1);
        int w = Mod(x, 2) + 2 * Mod(y, 2);
        assert(w >= 0 && w < 4);
        T value = c0 * UPSAMPLE[w][0] + c1 * UPSAMPLE[w][1] +
                  c2 * UPSAMPLE[w][2] + c3 * UPSAMPLE[w][3];
        result.Set(x, y, value);
      }
    }
    return result;
  }

 private:
  int size_;
  int border_;
  std::unique_ptr<T[]> pixels_;
};

typedef ImageT<double> Image;

constexpr int MAX_LEVELS = 9;
constexpr int GetNumLevelsImpl(const int h) {
  return h <= 2 ? 0 : GetNumLevelsImpl(h / 2 + h % 2) + 1;
}
constexpr int GetNumLevels(const int h) {
  return GetNumLevelsImpl(h) < MAX_LEVELS ? GetNumLevelsImpl(h) : MAX_LEVELS;
}

template <int height>
bool ComputeFilters(std::ofstream& output_stream) {
  std::cout << "Computing filters for height = " << height << std::endl;
  // The bloom filter function we want to approximate with a hierarchy of small
  // filter kernels, in an "downsample/filter/upsample" pyramid.
  constexpr double r0 = height / 20.0;
  const auto filter = [](double x, double y) {
    const double r = sqrt(x * x + y * y);
    // Curve f1(theta) from "Physically-Based Glare Effects for Digital Images",
    // Spencer et al. 1995 (http://mathinfo.univ-reims.fr/IMG/pdf/SPENCER.pdf).
    return pow(0.02 / (r / r0 + 0.02), 3.0);
  };

  // Compute the number of levels of the "downsample/filter/upsample" pyramid
  // and initialize the image size for the following computations.
  constexpr int kNumLevels = GetNumLevels(height);
  const int size = 1024;
  int border = 2;
  for (int i = 1; i < kNumLevels; ++i) {
    border = 2 * (border + 1);
  }

  // Create an image with the bloom filter function values, scaled to have an
  // integral equal to 1. This is the target image we want to approximate.
  double integral = 0.0;
  for (int y = -border; y < size + border; ++y) {
    for (int x = -border; x < size + border; ++x) {
      integral += filter(x - size / 2, y - size / 2);
    }
  }
  Image target(size, border);
  for (int y = -border; y < size + border; ++y) {
    for (int x = -border; x < size + border; ++x) {
      target.Set(x, y, filter(x - size / 2, y - size / 2) / integral);
    }
  }

  // Set up images for the unknown filter kernels we want to compute. Each pixel
  // is a linear combination sum_i (c_i * x_i) of the unknown variables x_i,
  // represented by the vector of its coefficients c_i.
  constexpr int kernel_size = 2;
  constexpr int kernel_unknowns = ((kernel_size + 1) * (kernel_size + 2)) / 2;
  constexpr int total_unknowns = kNumLevels * kernel_unknowns;
  typedef ImageT<Vec<total_unknowns>> VecImage;
  VecImage filter_kernels[kNumLevels];
  for (int i = 0; i < kNumLevels; ++i) {
    filter_kernels[i] = VecImage(2 * kernel_size + 1, 0);
    const int offset = i * kernel_unknowns;
    const auto idx = [](int x, int y) {
      x = x < 0 ? -x : x;
      y = y < 0 ? -y : y;
      if (x < y) std::swap(x, y);
      return (x * (x + 1)) / 2 + y;
    };
    for (int y = -kernel_size; y <= kernel_size; ++y) {
      for (int x = -kernel_size; x <= kernel_size; ++x) {
        Vec<total_unknowns> value(0.0);
        for (int index = idx(x, y); index < kernel_unknowns; ++index) {
          value.x[index + offset] = 1.0;
        }
        filter_kernels[i].Set(x + kernel_size, y + kernel_size, value);
      }
    }
  }

  // Create an image with a single white pixel in the center, and apply the
  // bloom shader on it: downsample...
  Image img_mipmap[kNumLevels];
  img_mipmap[0] = Image(size, border);
  for (int y = -border; y < size + border; ++y) {
    const double dy = y - size / 2;
    for (int x = -border; x < size + border; ++x) {
      const double dx = x - size / 2;
      img_mipmap[0].Set(x, y, dx == 0 && dy == 0 ? 1.0 : 0.0);
    }
  }
  for (int i = 1; i < kNumLevels; ++i) {
    img_mipmap[i] = img_mipmap[i - 1].Downsample();
  }
  // filter...
  VecImage conv_img_mipmap[kNumLevels];
  for (int i = 0; i < kNumLevels; ++i) {
    conv_img_mipmap[i] = img_mipmap[i].Convolution(filter_kernels[i]);
  }
  // and upsample. The result is an image where each pixel is new a linear
  // combination sum_i (c'_i * x_i) of the unknown variables x_i, represented by
  // the vector of its coefficients c'_i.
  for (int i = kNumLevels - 2; i >= 0; --i) {
    conv_img_mipmap[i] =
        conv_img_mipmap[i].Add(conv_img_mipmap[i + 1].Upsample());
  }

  // We must now find the best x_i values such that 'conv_img_mipmap[0]' best
  // matches the 'target' image. For this we use a non-negative least square
  // fitting algorithm. We first set up the linear system A*x=B to solve:
  int expand_size = kernel_size;
  for (int i = 1; i < kNumLevels; ++i) {
    expand_size = 2 * (expand_size + 1);
  }
  const int first_rows = (2 * kernel_size + 1) * (2 * kernel_size + 1);
  const int rows = first_rows + expand_size - (kernel_size + 1) + 1;
  const int cols = total_unknowns;
  Eigen::MatrixXf A = Eigen::MatrixXf(rows, cols);
  Eigen::VectorXf B = Eigen::VectorXf(rows);
  for (int row = 0; row < rows; ++row) {
    for (int col = 0; col < cols; ++col) {
      A(row, col) = 0.0;
    }
    B(row) = 0.0;
  }
  Vec<cols> sum(0.0);
  for (int x = -expand_size; x <= expand_size; ++x) {
    for (int y = -expand_size; y <= expand_size; ++y) {
      if (x >= -kernel_size && x <= kernel_size && y >= -kernel_size &&
          y <= kernel_size) {
        const int row =
            (x + kernel_size) + (y + kernel_size) * (2 * kernel_size + 1);
        for (int col = 0; col < cols; ++col) {
          A(row, col) =
              conv_img_mipmap[0].Get(size / 2 + x, size / 2 + y).x[col];
        }
        B(row) += filter(x, y) / integral;
      } else {
        const int r = static_cast<int>(floor(sqrt(x * x + y * y)));
        if (r < expand_size) {
          const double weight = r * r;
          const int row = first_rows + (r - (kernel_size + 1));
          for (int col = 0; col < cols; ++col) {
            A(row, col) +=
                conv_img_mipmap[0].Get(size / 2 + x, size / 2 + y).x[col] *
                weight;
          }
          B(row) += filter(x, y) / integral * weight;
        }
      }
      sum += conv_img_mipmap[0].Get(size / 2 + x, size / 2 + y);
    }
  }
  // Add a row to make sure the integral of 'conv_img_mipmap[0]' is 1, i.e. to
  // make sure that the overall bloom shader effect is energy preserving.
  for (int col = 0; col < cols; ++col) {
    A(rows - 1, col) = sum.x[col];
  }
  B(rows - 1) = 1.0;

  // The non-negative least square fitting step:
  Eigen::NNLS<Eigen::MatrixXf> nnls(A, 10000, 1e-6);
  if (!nnls.solve(B)) {
    std::cerr << "Solver failed!" << std::endl;
    return false;
  }
  Eigen::VectorXf X = nnls.x();

  // Output the result kernel coefficients:
  output_stream << "  " << height << "," << std::endl;
  for (int i = 0; i < kNumLevels; ++i) {
    const int offset = i * kernel_unknowns;
    output_stream << (i == 0 ? "  [[" : "   [");
    for (int j = 0; j < kernel_unknowns; ++j) {
      double value = 0.0;
      for (int k = j; k < kernel_unknowns; ++k) {
        value += X(k + offset);
      }
      output_stream << value << (j == kernel_unknowns - 1 ? "]" : ",");
    }
    output_stream << (i == kNumLevels - 1 ? "]," : ",") << std::endl;
  }
  return true;
}

}  // namespace
}  // namespace bloom

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: bloom_filters_generator <output file>" << std::endl;
    return -1;
  }
  std::ofstream output_stream(argv[1], std::ofstream::out);
  output_stream << "const BLOOM_FILTERS = [" << std::endl;
  assert(bloom::ComputeFilters<400>(output_stream));
  if (!bloom::ComputeFilters<600>(output_stream)) return -1;
  if (!bloom::ComputeFilters<800>(output_stream)) return -1;
  if (!bloom::ComputeFilters<1000>(output_stream)) return -1;
  if (!bloom::ComputeFilters<1200>(output_stream)) return -1;
  if (!bloom::ComputeFilters<1400>(output_stream)) return -1;
  if (!bloom::ComputeFilters<1600>(output_stream)) return -1;
  output_stream << "];" << std::endl;
  output_stream.close();
  return 0;
}
