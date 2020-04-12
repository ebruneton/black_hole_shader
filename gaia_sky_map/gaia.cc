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

#include "gaia_sky_map/gaia.h"

#define CSV_IO_NO_THREAD

#include <signal.h>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <regex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>

#include <condition_variable>
#include <curlpp/Easy.hpp>
#include <curlpp/Exception.hpp>
#include <curlpp/Options.hpp>
#include <curlpp/cURLpp.hpp>
#include "csv/csv.h"
#include "gzip/decompress.hpp"

using dimensional::vec3;

namespace gaia_sky_map {

namespace {

constexpr int kNumSourceFiles = 61234;
constexpr char kGaiaIndexFileName[] = "gaia.html";
constexpr char kCacheFileName[] = "gaia.dat";

struct Status {
  bool processed[kNumSourceFiles];
};

void InitGaiaCubeMap(GaiaCubeMap* gaia_cube_map, Status* status) {
  GaiaTexel empty_gaia_texel;
  empty_gaia_texel.xyzg_sum = Xyzg();
  empty_gaia_texel.star_count = 0;
  empty_gaia_texel.temperature_sum = 0;
  empty_gaia_texel.temperature_square_sum = 0;
  empty_gaia_texel.additional_g_sum = 0;
  empty_gaia_texel.additional_star_count = 0;

  for (int face = 0; face < 6; ++face) {
    for (int y = 0; y < kCubeMapSize; ++y) {
      for (int x = 0; x < kCubeMapSize; ++x) {
        gaia_cube_map->texel(face, x, y) = empty_gaia_texel;
      }
    }
  }
  for (int i = 0; i < kNumSourceFiles; ++i) {
    status->processed[i] = false;
  }
}

bool IsDone(const Status* status) {
  for (int i = 0; i < kNumSourceFiles; ++i) {
    if (!status->processed[i]) {
      return false;
    }
  }
  return true;
}

void SaveGaiaCubeMap(const std::string& output_dir,
                     const GaiaCubeMap* gaia_cube_map, const Status* status) {
  std::cout << "Writing Gaia cache..." << std::endl;
  std::ofstream output_stream(output_dir + kCacheFileName,
                              std::ofstream::out | std::ofstream::binary);
  output_stream.write(reinterpret_cast<const char*>(gaia_cube_map),
                      sizeof(GaiaCubeMap));
  output_stream.write(reinterpret_cast<const char*>(status), sizeof(Status));
  output_stream.close();
}

std::string DownloadGaiaSourceFile(const std::string& url) {
  constexpr char kGaiaSourceBaseUrl[] =
      "http://cdn.gea.esac.esa.int/Gaia/gdr2/gaia_source/csv/";

  std::stringstream out;
  curlpp::Cleanup cleaner;
  curlpp::Easy request;
  request.setOpt(new curlpp::options::WriteStream(&out));
  request.setOpt(new curlpp::options::Url(kGaiaSourceBaseUrl + url));
  request.setOpt(new curlpp::options::Timeout(3 * 60));
  request.setOpt(new curlpp::options::NoSignal(true));
  request.perform();

  const std::string compressed = out.str();
  const std::string decompressed =
      gzip::decompress(compressed.data(), compressed.size());
  return decompressed;
}

void ProcessGaiaSourceFile(const std::string& url,
                           const std::string& decompressed,
                           GaiaCubeMap* gaia_cube_map) {
  std::stringstream in_string(decompressed);
  io::CSVReader<5> in(url, in_string);
  in.read_header(io::ignore_extra_column, "l", "b", "phot_g_mean_flux",
                 "phot_g_mean_mag", "teff_val");

  double l, b, g_flux, g_mag, t_eff;
  while (in.read_row(l, b, g_flux, g_mag, t_eff)) {
    const double l_rad = l / 180.0 * M_PI;
    const double b_rad = b / 180.0 * M_PI;
    const vec3 dir(cos(b_rad) * cos(l_rad), cos(b_rad) * sin(l_rad),
                   sin(b_rad));
    GaiaTexel& gaia_texel = gaia_cube_map->texel(dir);

    // NOTE: g_mag = 0 corresponds to Vega's magnitude.
    if (g_mag >= 7.0) {
      if (t_eff > 0.0) {
        const Xyzg& xyzg = XyzgFromTemperature(t_eff);
        gaia_texel.xyzg_sum = gaia_texel.xyzg_sum + xyzg / xyzg.g() * g_flux;
        gaia_texel.star_count += 1;
        gaia_texel.temperature_sum += t_eff;
        gaia_texel.temperature_square_sum += t_eff * t_eff;
      } else {
        gaia_texel.additional_g_sum += g_flux;
        gaia_texel.additional_star_count += 1;
      }
    }
  }
}

std::queue<std::pair<int, std::string>> process_queue;
std::mutex process_queue_mutex;
std::mutex cube_map_mutex;
int thread_process[3];

volatile bool stop_download = false;

void SignalCallbackHandler(const int signum) {
  std::cerr << "Interrupt signal " << signum << std::endl;
  if (signum == 2) {
    stop_download = true;
  }
}

void DownloadAndProcessGaiaData(const int thread_id,
                                const std::string& output_dir,
                                GaiaCubeMap* gaia_cube_map, Status* status) {
  while (!stop_download) {
    int url_index = 0;
    std::string url;
    {
      std::lock_guard<std::mutex> lock(process_queue_mutex);
      if (process_queue.empty()) {
        return;
      }
      url_index = process_queue.front().first;
      url = process_queue.front().second;
      process_queue.pop();
    }

    thread_process[thread_id] = url_index;
    std::string data;
    bool ok = false;
    try {
      data = DownloadGaiaSourceFile(url);
      ok = true;
    } catch (std::logic_error& e) {
      std::cerr << "Error downloading " << url_index << " " << url << std::endl;
      std::cerr << e.what() << std::endl;
    } catch (std::runtime_error& e) {
      std::cerr << "Error downloading " << url_index << " " << url << std::endl;
      std::cerr << e.what() << std::endl;
    }

    if (ok) {
      std::lock_guard<std::mutex> lock(cube_map_mutex);
      ProcessGaiaSourceFile(url, data, gaia_cube_map);
      status->processed[url_index] = true;

      std::cout << "Thread status: " << thread_process[0] << "/, "
                << thread_process[1] << "/, " << thread_process[2]
                << "/ (total= " << kNumSourceFiles << ")" << std::endl;
      if (url_index % 500 == 0) {
        SaveGaiaCubeMap(output_dir, gaia_cube_map, status);
      }
    }
  }
}

}  // namespace

bool DownloadOrReadGaiaDr2Data(const std::string& output_dir,
                               GaiaCubeMap* gaia_cube_map) {
  std::unique_ptr<Status> status(new Status());

  std::ifstream input_stream(output_dir + kCacheFileName,
                             std::ifstream::in | std::ifstream::binary);
  if (input_stream) {
    std::cout << "Reading Gaia cache... ";
    input_stream.read(reinterpret_cast<char*>(gaia_cube_map),
                      sizeof(GaiaCubeMap));
    input_stream.read(reinterpret_cast<char*>(status.get()), sizeof(Status));
    input_stream.close();
    std::cout << "OK." << std::endl;
  } else {
    InitGaiaCubeMap(gaia_cube_map, status.get());
  }

  if (IsDone(status.get())) {
    return true;
  }

  signal(SIGINT, SignalCallbackHandler);

  std::cout << "Downloading and processing the Gaia DR2 data (550GB)..."
            << std::endl;
  std::cout << "This will take several hours, but can be interrupted"
            << std::endl;
  std::cout << "with CTRL+C and restarted later without loss of data."
            << std::endl;
  const std::regex url_exp("GaiaSource_[^\\.]*\\.csv\\.gz");
  std::ifstream in(output_dir + kGaiaIndexFileName);
  int url_index = 0;
  std::string line;
  while (std::getline(in, line)) {
    std::smatch smatch;
    std::regex_search(line, smatch, url_exp);
    if (smatch.size() == 0) {
      continue;
    }
    const std::string url = smatch[0];
    if (!status->processed[url_index]) {
      process_queue.push(std::make_pair(url_index, url));
    }
    url_index++;
  }

  std::thread t1(DownloadAndProcessGaiaData, 0, output_dir, gaia_cube_map,
                 status.get());
  std::thread t2(DownloadAndProcessGaiaData, 1, output_dir, gaia_cube_map,
                 status.get());
  std::thread t3(DownloadAndProcessGaiaData, 2, output_dir, gaia_cube_map,
                 status.get());
  t1.join();
  t2.join();
  t3.join();

  SaveGaiaCubeMap(output_dir, gaia_cube_map, status.get());
  return IsDone(status.get());
}

}  // namespace gaia_sky_map
