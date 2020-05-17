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

uniform vec3 camera;

uniform vec3 intensity;
uniform vec3 k_r;
uniform vec3 k_z;

in vec3 position;

layout(location = 0) out vec4 frag_color;

void main() {
  frag_color = vec4(0.0);

  vec3 dir = normalize(position - camera);
  float a = dot(dir.xy, dir.xy);
  float b = dot(camera.xy, dir.xy);
  float c = dot(camera.xy, camera.xy);
  float discriminant = b * b - a * (c - RADIUS * RADIUS);
  if (discriminant >= 0.0) {
    float t_min = max((-b - sqrt(discriminant)) / a, 0.0);
    float t_max = (-b + sqrt(discriminant)) / a;
    if (dir.z > 0.0) {
      t_min = max(t_min, (Z_MIN - camera.z) / dir.z);
      t_max = min(t_max, (Z_MAX - camera.z) / dir.z);
    } else {
      t_min = max(t_min, (Z_MAX - camera.z) / dir.z);
      t_max = min(t_max, (Z_MIN - camera.z) / dir.z);
    }
    if (t_min < t_max) {
      const int N = 16;
      vec3 emitted = vec3(0.0);
      float dt = (t_max - t_min) / float(N);
      float t = t_min + 0.5 * dt;
      // TODO(me): use analytic indefinite integral instead?
      for (int i = 0; i < N; ++i) {
        float r2 = ((a * t + 2.0 * b) * t) + c;
        float z = camera.z + t * dir.z;
        emitted += exp(- k_r * r2 - k_z * (Z_MAX - z));
        t += dt;
      }
      frag_color = vec4(intensity * emitted * dt, 0.0);
    }
  }
}
