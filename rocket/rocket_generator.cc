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

#include "math/vector.h"
#include "png++/png.hpp"

namespace rocket {
namespace {

using dimensional::vec3;

constexpr double kEpsilon = 1e-6;

constexpr int NUM_CIRCUMFERENCE_SAMPLES = 32;
constexpr int NUM_VERTEX_COMPONENTS = 13;
constexpr int TEXTURE_SIZE = 512;

vec3 cross(const vec3& p, const vec3& q) {
  const double x = p.y() * q.z() - p.z() * q.y();
  const double y = p.z() * q.x() - p.x() * q.z();
  const double z = p.x() * q.y() - p.y() * q.x();
  return vec3(x, y, z);
}

class Triangle {
 public:
  Triangle(const vec3& a, const vec3& b, const vec3& c) {
    a_ = a;
    ab_ = b - a;
    ac_ = c - a;
  }

  bool RayHit(const vec3& origin, const vec3& dir) const {
    const vec3 p = cross(dir, ac_);
    const double det = dot(ab_, p)();
    if (std::abs(det) < kEpsilon) {
      return false;
    }
    const vec3 t = origin - a_;
    double u = dot(t, p)() / det;
    if (u < 0.0 || u > 1.0) {
      return false;
    }
    const vec3 q = cross(t, ab_);
    const double v = dot(dir, q)() / det;
    if (v < 0.0 || u + v > 1.0) {
      return false;
    }
    return dot(ac_, q) / det > 0.0;
  }

 private:
  vec3 a_;
  vec3 ab_;
  vec3 ac_;
};

class Mesh {
 public:
  Mesh() {}

  void AddTriangle(const vec3& a, const vec3& b, const vec3& c) {
    triangles_.emplace_back(a, b, c);
  }

  bool RayHit(const vec3& origin, const vec3& dir) const {
    for (const Triangle& triangle : triangles_) {
      if (triangle.RayHit(origin, dir)) {
        return true;
      }
    }
    return false;
  }

 private:
  std::vector<Triangle> triangles_;
};

double Theta(const int u_index) {
  return (2.0 * M_PI * u_index) / NUM_CIRCUMFERENCE_SAMPLES;
}

class ProfilePoint {
 public:
  ProfilePoint(const double z, const double r)
      : z_(z), r_(r), v_(0.0), ao_(1.0) {}

  double z() const { return z_; }
  double r() const { return r_; }
  double v() const { return v_; }
  double ao() const { return ao_; }

  void SetZ(const double z) { z_ = z; }
  void SetR(const double r) { r_ = r; }
  void SetV(const double v) { v_ = v; }
  void SetAo(const double ao) { ao_ = ao; }

 private:
  double z_;
  double r_;
  double v_;
  double ao_;
};

class Profile {
 public:
  Profile(const std::vector<ProfilePoint>& points) : points_(points) {}

  int size() const { return points_.size(); }
  const ProfilePoint& point(const int index) const { return points_[index]; }
  ProfilePoint& point(const int index) { return points_[index]; }

  double GetLength() const {
    double length = 0.0;
    for (int i = 0; i < size() - 1; ++i) {
      const ProfilePoint& p = point(i);
      const ProfilePoint& q = point(i + 1);
      const double dz = p.z() - q.z();
      const double dr = p.r() - q.r();
      length += sqrt(dz * dz + dr * dr);
    }
    return length;
  }

  vec3 GetPoint(const int u_index, const int v_index) const {
    const double theta = Theta(u_index);
    const ProfilePoint& p = point(v_index);
    return vec3(p.r() * cos(theta), p.r() * sin(theta), p.z());
  }

  vec3 GetNormal(const int u_index, const int v_index) const {
    const double theta = Theta(u_index);
    const vec3 n = GetNormal(v_index);
    return vec3(n.x() * cos(theta), n.x() * sin(theta), n.z());
  }

  vec3 GetTangent(const int u_index, const int v_index) const {
    const double theta = Theta(u_index);
    return vec3(-sin(theta), cos(theta), 0.0);
  }

  void Resize(const float dz, const float scale) {
    for (int i = 0; i < size(); ++i) {
      ProfilePoint& p = point(i);
      p.SetZ((p.z() + dz) * scale);
      p.SetR(p.r() * scale);
    }
  }

  void ComputeTexCoords(const double v0, const double length_scale) {
    double v = v0;
    point(0).SetV(v);
    for (int v_index = 1; v_index < size(); ++v_index) {
      const ProfilePoint& p = point(v_index - 1);
      ProfilePoint& q = point(v_index);
      const double dz = p.z() - q.z();
      const double dr = p.r() - q.r();
      v += sqrt(dz * dz + dr * dr) * length_scale;
      q.SetV(v);
    }
  }

  void ComputeAmbientOcclusion(const Mesh& mesh) {
    for (int v_index = 0; v_index < size(); ++v_index) {
      const vec3 p = GetPoint(0, v_index);
      const vec3 n = GetNormal(0, v_index);
      const vec3 t1 = GetTangent(0, v_index);
      const vec3 t2 = cross(n, t1);
      const int NPHI = 16;
      const int NZ = 16;
      int num_visible_dirs = 0;
      for (int i = 0; i < NPHI; ++i) {
        const double phi = (i + 0.5) * M_PI / NPHI;
        for (int j = 0; j < NZ; ++j) {
          const double z = (j + 0.5) / NZ;
          const double r = sqrt(1.0 - z * z);
          const vec3 dir = (t1 * cos(phi) + t2 * sin(phi)) * r + n * z;
          if (!mesh.RayHit(p + n * kEpsilon, dir)) {
            ++num_visible_dirs;
          }
        }
      }
      point(v_index).SetAo(static_cast<double>(num_visible_dirs) / (NZ * NPHI));
    }
  }

  static void Save(const std::vector<Profile>& profiles,
                   const std::string& filename) {
    std::vector<int32_t> header;
    std::vector<float> vertex_buffer;
    std::vector<int32_t> index_buffer;
    for (const Profile& profile : profiles) {
      profile.AppendToBuffers(&index_buffer, &vertex_buffer);
    }
    header.push_back(vertex_buffer.size());
    header.push_back(index_buffer.size());

    std::ofstream output_stream(filename,
                                std::ofstream::out | std::ofstream::binary);
    output_stream.write((const char*)header.data(),
                        header.size() * sizeof(int32_t));
    output_stream.write((const char*)vertex_buffer.data(),
                        vertex_buffer.size() * sizeof(float));
    output_stream.write((const char*)index_buffer.data(),
                        index_buffer.size() * sizeof(int32_t));
    output_stream.close();
  }

 private:
  vec3 GetNormal(const int v_index) const {
    vec3 n(0.0, 0.0, 0.0);
    vec3 p(point(v_index).r(), 0.0, point(v_index).z());
    if (v_index + 1 < size()) {
      vec3 q(point(v_index + 1).r(), 0.0, point(v_index + 1).z());
      vec3 n_pq(q.z() - p.z(), 0.0, p.x() - q.x());
      n += n_pq;
    }
    if (v_index > 0) {
      vec3 q(point(v_index - 1).r(), 0.0, point(v_index - 1).z());
      vec3 n_qp(p.z() - q.z(), 0.0, q.x() - p.x());
      n += n_qp;
    }
    return normalize(n);
  }

  void AppendToBuffers(std::vector<int32_t>* index_buffer,
                       std::vector<float>* vertex_buffer) const {
    const int start_index = vertex_buffer->size() / NUM_VERTEX_COMPONENTS;
    for (int v = 0; v < size() - 1; ++v) {
      for (int u = 0; u < NUM_CIRCUMFERENCE_SAMPLES; ++u) {
        const int index0 =
            start_index + u + v * (NUM_CIRCUMFERENCE_SAMPLES + 1);
        const int index1 = index0 + 1;
        const int index2 = index0 + 1 + (NUM_CIRCUMFERENCE_SAMPLES + 1);
        const int index3 = index0 + (NUM_CIRCUMFERENCE_SAMPLES + 1);
        index_buffer->push_back(index0);
        index_buffer->push_back(index1);
        index_buffer->push_back(index2);
        index_buffer->push_back(index2);
        index_buffer->push_back(index3);
        index_buffer->push_back(index0);
      }
    }
    for (int v = 0; v < size(); ++v) {
      for (int u = 0; u <= NUM_CIRCUMFERENCE_SAMPLES; ++u) {
        const vec3 p = GetPoint(u, v);
        const vec3 n = GetNormal(u, v);
        const vec3 t = GetTangent(u, v);
        const float tex_u = static_cast<float>(u) / NUM_CIRCUMFERENCE_SAMPLES;
        const float tex_v = point(v).v();
        vertex_buffer->push_back(p.x());
        vertex_buffer->push_back(p.y());
        vertex_buffer->push_back(p.z());
        vertex_buffer->push_back(n.x());
        vertex_buffer->push_back(n.y());
        vertex_buffer->push_back(n.z());
        vertex_buffer->push_back(t.x());
        vertex_buffer->push_back(t.y());
        vertex_buffer->push_back(t.z());
        vertex_buffer->push_back(1.0);
        vertex_buffer->push_back(tex_u);
        vertex_buffer->push_back(1.0 - tex_v);
        vertex_buffer->push_back(point(v).ao());
      }
    }
  }

  std::vector<ProfilePoint> points_;
};

Mesh BuildMesh(const std::vector<Profile>& profiles) {
  Mesh m;
  for (const Profile& profile : profiles) {
    for (int v = 0; v < profile.size() - 1; ++v) {
      for (int u = 0; u < NUM_CIRCUMFERENCE_SAMPLES; ++u) {
        vec3 p0 = profile.GetPoint(u, v);
        vec3 p1 = profile.GetPoint(u + 1, v);
        vec3 p2 = profile.GetPoint(u + 1, v + 1);
        vec3 p3 = profile.GetPoint(u, v + 1);
        m.AddTriangle(p0, p1, p2);
        m.AddTriangle(p2, p3, p0);
      }
    }
  }
  return m;
}

void ComputeRocketModel(const std::string& output_dir) {
  std::vector<Profile> profiles;
  profiles.push_back(Profile({{180, 0}, {180, 23}}));
  profiles.push_back(Profile({{180, 23},
                              {160, 32},
                              {128, 47},
                              {96, 60},
                              {64, 71},
                              {32, 79},
                              {0, 84}}));
  profiles.push_back(Profile({{0, 84},
                              {32, 79},
                              {64, 71},
                              {96, 60},
                              {128, 47},
                              {160, 32},
                              {180, 23}}));
  profiles.push_back(Profile({{180, 23}, {185, 29}}));
  profiles.push_back(
      Profile({{185, 29}, {185, 72}, {188, 84}, {197, 93}, {209, 96}}));
  profiles.push_back(Profile({{209, 96}, {209, 138}}));
  profiles.push_back(Profile({{209, 138}, {532, 138}}));
  profiles.push_back(Profile({{532, 138}, {610, 90}, {688, 42}}));
  profiles.push_back(Profile({{688, 42}, {688, 0}}));
  for (Profile& profile : profiles) {
    profile.Resize(-344, 1.0 / 138);
  }

  std::vector<double> lengths;
  double total_length = 0.0;
  for (Profile& profile : profiles) {
    const double length = profile.GetLength();
    lengths.push_back(length);
    total_length += length;
  }

  std::vector<int> texture_sizes;
  int total_texture_size = 0;
  for (unsigned int i = 0; i < lengths.size(); ++i) {
    const int texture_size = std::max(
        static_cast<int>(round(lengths[i] / total_length * TEXTURE_SIZE)), 2);
    texture_sizes.push_back(texture_size);
    total_texture_size += texture_size;
  }

  int i = 0;
  while (total_texture_size != TEXTURE_SIZE) {
    if (texture_sizes[i] <= TEXTURE_SIZE / 20) {
      i = (i + 1) % texture_sizes.size();
      continue;
    }
    const int increment = total_texture_size > TEXTURE_SIZE ? -1 : 1;
    texture_sizes[i] += increment;
    total_texture_size += increment;
  }

  int texture_offset = 0;
  for (unsigned int i = 0; i < lengths.size(); ++i) {
    const double v0 = (texture_offset + 0.5) / TEXTURE_SIZE;
    const double v1 = (texture_offset + texture_sizes[i] - 0.5) / TEXTURE_SIZE;
    profiles[i].ComputeTexCoords(v0, (v1 - v0) / lengths[i]);
    texture_offset += texture_sizes[i];
  }

  Mesh mesh = BuildMesh(profiles);
  for (Profile& profile : profiles) {
    profile.ComputeAmbientOcclusion(mesh);
  }

  Profile::Save(profiles, output_dir + "rocket.dat");
}

void ComputeRocketTextures(const std::string& input_dir,
                           const std::string& output_dir) {
  png::image<png::rgba_pixel> color(input_dir + "color.png");
  png::image<png::rgba_pixel> height(input_dir + "height.png");
  png::image<png::rgba_pixel> occlusion(input_dir + "occlusion.png");
  png::image<png::rgba_pixel> roughness(input_dir + "roughness.png");
  png::image<png::rgba_pixel> metallic(input_dir + "metallic.png");

  const int w = height.get_width();
  const int h = height.get_height();
  png::image<png::rgba_pixel> base_color_map(w, h);
  png::image<png::rgba_pixel> occlusion_roughness_metallic_map(w, h);
  png::image<png::rgba_pixel> normal_map(w, h);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const double slope_x =
          (height[y][(x + 1) % w].red - height[y][(x + w - 1) % w].red) / 64.0;
      const double slope_y = (height[std::max(y - 1, 0)][x].red -
                              height[std::min(y + 1, h - 1)][x].red) /
                             64.0;
      const vec3 normal = normalize(vec3(-slope_x, -slope_y, 1.0));

      base_color_map[y][x].red = color[y][x].red;
      base_color_map[y][x].green = color[y][x].green;
      base_color_map[y][x].blue = color[y][x].blue;
      base_color_map[y][x].alpha = 255;

      occlusion_roughness_metallic_map[y][x].red = occlusion[y][x].red;
      occlusion_roughness_metallic_map[y][x].green = roughness[y][x].red;
      occlusion_roughness_metallic_map[y][x].blue = metallic[y][x].red;
      occlusion_roughness_metallic_map[y][x].alpha = 255;

      normal_map[y][x].red =
          static_cast<int>(round((normal.x() * 0.5 + 0.5) * 255.0));
      normal_map[y][x].green =
          static_cast<int>(round((normal.y() * 0.5 + 0.5) * 255.0));
      normal_map[y][x].blue =
          static_cast<int>(round((normal.z() * 0.5 + 0.5) * 255.0));
      normal_map[y][x].alpha = 255;
    }
  }
  base_color_map.write(output_dir + "rocket_base_color.png");
  occlusion_roughness_metallic_map.write(
      output_dir + "rocket_occlusion_roughness_metallic.png");
  normal_map.write(output_dir + "rocket_normal.png");
}

}  // namespace
}  // namespace rocket

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "Usage: rocket_generator <input directory> <output directory>"
              << std::endl;
    return -1;
  }
  rocket::ComputeRocketModel(argv[2]);
  rocket::ComputeRocketTextures(argv[1], argv[2]);
}
