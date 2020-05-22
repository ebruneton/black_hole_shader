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

// Provides a bloom shader effect by mipmapping an input image, filtering each
// mipmap with a small kernel, and upsampling and adding the filtered images.

const MAX_LEVELS = 9;
const MAX_FLOAT16 = '6.55e4';

const VERTEX_SHADER =
  `#version 300 es
  layout(location=0) in vec4 vertex;
  void main() { gl_Position = vertex; }`;

const DOWNSAMPLE_SHADER =
  `#version 300 es
  precision highp float;
  const vec4 WEIGHTS = vec4(1.0, 3.0, 3.0, 1.0) / 8.0;
  uniform sampler2D source;
  uniform vec2 source_delta_uv;
  layout(location=0) out vec4 frag_color;
  void main() { 
    vec2 ij = floor(gl_FragCoord.xy);
    vec2 source_ij = ij * 2.0 - vec2(1.5);
    vec2 source_uv = source_ij * source_delta_uv;
    vec3 color = vec3(0.0);
    for (int i = 0; i < 4; ++i) {
      float wi = WEIGHTS[i];
      for (int j = 0; j < 4; ++j) {
        float wj = WEIGHTS[j];
        vec2 delta_uv = vec2(i, j) * source_delta_uv;
        color += wi * wj * texture(source, source_uv + delta_uv).rgb;
      }
    }
    frag_color = vec4(min(color, ${MAX_FLOAT16}), 1.0);
  }`;

const BLOOM_SHADER =
  `#version 300 es
  precision highp float;
  uniform sampler2D source;
  uniform vec2 source_delta_uv;
  uniform vec3 source_samples_uvw[SIZE];
  layout(location=0) out vec4 frag_color;
  void main() { 
    vec2 source_uv = (gl_FragCoord.xy + vec2(1.0)) * source_delta_uv;
    vec3 color = vec3(0.0);
    for (int i = 0; i < SIZE; ++i) {
      vec3 uvw = source_samples_uvw[i];
      color += uvw.z * texture(source, source_uv + uvw.xy).rgb;
    }
    frag_color = vec4(min(color, ${MAX_FLOAT16}), 1.0);
  }`;

const UPSAMPLE_SHADER =
  `#version 300 es
  precision highp float;
  const vec4 WEIGHTS[4] = vec4[4] (
    vec4(1.0, 3.0, 3.0, 9.0) / 16.0,
    vec4(3.0, 1.0, 9.0, 3.0) / 16.0,
    vec4(3.0, 9.0, 1.0, 3.0) / 16.0,
    vec4(9.0, 3.0, 3.0, 1.0) / 16.0
  );
  uniform sampler2D source;
  uniform vec2 source_delta_uv;
  layout(location=0) out vec4 frag_color;
  void main() {
    vec2 ij = floor(gl_FragCoord.xy);
    vec2 source_ij = floor((ij - vec2(1.0)) * 0.5) + vec2(0.5);
    vec2 source_uv = source_ij * source_delta_uv;
    vec3 c0 = texture(source, source_uv).rgb;
    vec3 c1 = texture(source, source_uv + vec2(source_delta_uv.x, 0.0)).rgb;
    vec3 c2 = texture(source, source_uv + vec2(0.0, source_delta_uv.y)).rgb;
    vec3 c3 = texture(source, source_uv + source_delta_uv).rgb;
    vec4 weight = WEIGHTS[int(mod(ij.x, 2.0) + 2.0 * mod(ij.y, 2.0))];
    vec3 color = weight.x * c0 + weight.y * c1 + weight.z * c2 + weight.w * c3;
    frag_color = vec4(min(color, ${MAX_FLOAT16}), 1.0);
  }`;

const RENDER_SHADER =
  `#version 300 es
  precision highp float;
  uniform sampler2D source;
  uniform vec2 source_delta_uv;
  uniform vec3 source_samples_uvw[SIZE];
  uniform sampler2D bloom;
  uniform vec2 bloom_delta_uv;
  uniform float intensity;
  uniform float exposure;
  uniform bool high_contrast;
  layout(location=0) out vec4 frag_color;

  vec3 toneMap(vec3 color) {
    return pow(vec3(1.0) - exp(-color), vec3(1.0 / 2.2));
  }
  
  // ACES tone map, see
  // https://knarkowicz.wordpress.com/2016/01/06/aces-filmic-tone-mapping-curve/
  vec3 toneMapACES(vec3 color) {
    const float A = 2.51;
    const float B = 0.03;
    const float C = 2.43;
    const float D = 0.59;
    const float E = 0.14;
    color = (color * (A * color + B)) / (color * (C * color + D) + E);
    return pow(color, vec3(1.0 / 2.2));
  }

  void main() {
    vec2 source_uv = (gl_FragCoord.xy + vec2(1.0)) * source_delta_uv;
    vec3 color = texture(bloom, 0.5 * gl_FragCoord.xy * bloom_delta_uv).rgb;
    for (int i = 0; i < SIZE; ++i) {
      vec3 uvw = source_samples_uvw[i];
      color += uvw.z * texture(source, source_uv + uvw.xy).rgb;
    }
    color = mix(texture(source, source_uv).rgb, color, intensity) * exposure;
    color = min(color, 10.0);
    if (high_contrast) {
      color = toneMapACES(color);
    } else {
      color = toneMap(color);
    }
    frag_color = vec4(color, 1.0);
  }`;

const createShader = function(gl, type, source) {
  const shader = gl.createShader(type);
  gl.shaderSource(shader, source);
  gl.compileShader(shader);
  return shader;
};

const createTexture = function(gl, textureUnit, target) {
  const texture = gl.createTexture();
  gl.activeTexture(textureUnit);
  gl.bindTexture(target, texture);
  gl.texParameteri(target, gl.TEXTURE_MIN_FILTER, gl.NEAREST);
  gl.texParameteri(target, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
  gl.texParameteri(target, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
  gl.texParameteri(target, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
  return texture;
};

// Usage: create an instance for the desired viewport size, and draw your scene
// between a call to begin() and a call to end(). Use resize() when the viewport
// size changes.
class Bloom {

  constructor(gl, width, height) {
    this.gl = gl;
    this.width = width;
    this.height = height;
    gl.getExtension('OES_texture_float_linear');
    gl.getExtension('EXT_color_buffer_float');
    gl.getExtension('EXT_float_blend');

    this.vertexBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, this.vertexBuffer);
    gl.bufferData(gl.ARRAY_BUFFER,
       new Float32Array([-1, -1, +1, -1, -1, +1, +1, +1]), gl.STATIC_DRAW);

    const vertexShader = 
        createShader(gl, gl.VERTEX_SHADER, VERTEX_SHADER);

    this.downsampleProgram = gl.createProgram();
    gl.attachShader(this.downsampleProgram, vertexShader);
    gl.attachShader(this.downsampleProgram, 
        createShader(gl, gl.FRAGMENT_SHADER, DOWNSAMPLE_SHADER));
    gl.linkProgram(this.downsampleProgram);
    gl.useProgram(this.downsampleProgram);
    gl.uniform1i(gl.getUniformLocation(this.downsampleProgram, 'source'), 0);
    this.downsampleProgram.sourceDeltaUvUniform = 
        gl.getUniformLocation(this.downsampleProgram, 'source_delta_uv');

    this.bloomProgram = gl.createProgram();
    gl.attachShader(this.bloomProgram, vertexShader);
    gl.attachShader(this.bloomProgram, 
        createShader(gl, gl.FRAGMENT_SHADER, 
            BLOOM_SHADER.replace(/SIZE/g, 25)));
    gl.linkProgram(this.bloomProgram);
    gl.useProgram(this.bloomProgram);
    gl.uniform1i(gl.getUniformLocation(this.bloomProgram, 'source'), 0);
    this.bloomProgram.sourceDeltaUvUniform = 
        gl.getUniformLocation(this.bloomProgram, 'source_delta_uv');

    this.upsampleProgram = gl.createProgram();
    gl.attachShader(this.upsampleProgram, vertexShader);
    gl.attachShader(this.upsampleProgram, 
        createShader(gl, gl.FRAGMENT_SHADER, UPSAMPLE_SHADER));
    gl.linkProgram(this.upsampleProgram);
    gl.useProgram(this.upsampleProgram);
    gl.uniform1i(gl.getUniformLocation(this.upsampleProgram, 'source'), 0);
    this.upsampleProgram.sourceDeltaUvUniform = 
        gl.getUniformLocation(this.upsampleProgram, 'source_delta_uv');

    this.renderProgram = gl.createProgram();
    gl.attachShader(this.renderProgram, vertexShader);
    gl.attachShader(this.renderProgram, 
        createShader(gl, gl.FRAGMENT_SHADER, 
            RENDER_SHADER.replace(/SIZE/g, 25)));
    gl.linkProgram(this.renderProgram);
    gl.useProgram(this.renderProgram);
    gl.uniform1i(gl.getUniformLocation(this.renderProgram, 'source'), 0);
    gl.uniform1i(gl.getUniformLocation(this.renderProgram, 'bloom'), 1);
    this.renderProgram.intensityUniform = 
        gl.getUniformLocation(this.renderProgram, 'intensity');
    this.renderProgram.exposureUniform = 
        gl.getUniformLocation(this.renderProgram, 'exposure');
    this.renderProgram.highContrastUniform = 
        gl.getUniformLocation(this.renderProgram, 'high_contrast');
    this.renderProgram.sourceDeltaUvUniform = 
        gl.getUniformLocation(this.renderProgram, 'source_delta_uv');
    this.renderProgram.bloomDeltaUvUniform = 
        gl.getUniformLocation(this.renderProgram, 'bloom_delta_uv'); 

    this.numLevels = 0;
    this.mipmapTextures = [];
    this.filterTextures = [];
    this.bloomFilters = [];
    for (let i = 0; i < MAX_LEVELS; ++i) {
      const mipmapTexture = createTexture(gl, gl.TEXTURE0, gl.TEXTURE_2D);
      this.mipmapTextures.push(mipmapTexture);
      if (i > 0) {
        const filterTexture = createTexture(gl, gl.TEXTURE0, gl.TEXTURE_2D);
        if (i == 1) {
          gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
          gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
        }
        this.filterTextures.push(filterTexture);
      } else {
        this.filterTextures.push(null);
      }
    }

    this.mipmapFbos = [];
    this.filterFbos = [];
    this.depthBuffer = undefined;
    for (let i = 0; i < MAX_LEVELS; ++i) {
      const mipmapFbo = gl.createFramebuffer();
      this.mipmapFbos.push(mipmapFbo);
      gl.bindFramebuffer(gl.FRAMEBUFFER, mipmapFbo);
      gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0, 
          gl.TEXTURE_2D, this.mipmapTextures[i], 0);
      if (i > 0) {
        const filterFbo = gl.createFramebuffer();
        this.filterFbos.push(filterFbo);
        gl.bindFramebuffer(gl.FRAMEBUFFER, filterFbo);
        gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0, 
            gl.TEXTURE_2D, this.filterTextures[i], 0);
      } else {
        this.depthBuffer = gl.createRenderbuffer();
        gl.bindRenderbuffer(gl.RENDERBUFFER, this.depthBuffer);
        gl.renderbufferStorage(gl.RENDERBUFFER, gl.DEPTH_COMPONENT16,
            this.mipmapTextures[0].width, this.mipmapTextures[0].height);
        gl.framebufferRenderbuffer(gl.FRAMEBUFFER, gl.DEPTH_ATTACHMENT,
            gl.RENDERBUFFER, this.depthBuffer);
        this.filterFbos.push(null);
      }
    }
    gl.bindFramebuffer(gl.FRAMEBUFFER, null);

    this.resize(width, height);
  }

  resize(width, height) {
    this.width = width;
    this.height = height;

    const gl = this.gl;
    gl.activeTexture(gl.TEXTURE0);
    let level = 0;
    let w = width;
    let h = height;
    while (h > 2 && level < MAX_LEVELS) {
      gl.bindTexture(gl.TEXTURE_2D, this.mipmapTextures[level]);
      gl.texImage2D(
          gl.TEXTURE_2D, 0, gl.RGBA16F, w + 2, h + 2, 0, gl.RGBA, gl.FLOAT, 
          null);
      this.mipmapTextures[level].width = w + 2;
      this.mipmapTextures[level].height = h + 2;
      if (level > 0) {
        gl.bindTexture(gl.TEXTURE_2D, this.filterTextures[level]);
        gl.texImage2D(
           gl.TEXTURE_2D, 0, gl.RGBA16F, w, h, 0, gl.RGBA, gl.FLOAT, null);
        this.filterTextures[level].width = w;
        this.filterTextures[level].height = h;
      } else {
        gl.bindRenderbuffer(gl.RENDERBUFFER, this.depthBuffer);
        gl.renderbufferStorage(gl.RENDERBUFFER, gl.DEPTH_COMPONENT16,
            this.mipmapTextures[0].width, this.mipmapTextures[0].height);
      }
      level += 1;
      w = Math.ceil(w / 2);
      h = Math.ceil(h / 2);
    }
    this.numLevels = level;

    let nearest_size_index = 0;
    let nearest_size = BLOOM_FILTERS[nearest_size_index];
    for (let i = 2; i < BLOOM_FILTERS.length; i += 2) {
      const size = BLOOM_FILTERS[i]
      if (Math.abs(BLOOM_FILTERS[i] - height) < 
          Math.abs(nearest_size - height)) {
        nearest_size_index = i;
        nearest_size = BLOOM_FILTERS[i];
      }
    }

    const filters = BLOOM_FILTERS[nearest_size_index + 1];
    for (let i = 0; i < this.numLevels; ++i) {
      const bloomFilter = [];
      const width = this.mipmapTextures[i].width;
      const height = this.mipmapTextures[i].height;
      for (let y = -2; y <= 2; ++y) {
        const iy = Math.abs(y);
        for (let x = -2; x <= 2; ++x) {
          const ix = Math.abs(x);
          const index = 
              ix < iy ? (iy * (iy + 1)) / 2 + ix : (ix * (ix + 1)) / 2 + iy;
          const w = filters[i][index];
          bloomFilter.push([x / width, y / height, w]);
        }
      }
      this.bloomFilters.push(bloomFilter);
    }
  }

  begin() {
    const gl = this.gl;
    gl.bindFramebuffer(gl.FRAMEBUFFER, this.mipmapFbos[0]);
    gl.viewport(1, 1, this.mipmapTextures[0].width - 2, 
        this.mipmapTextures[0].height - 2);
  }

  end(intensity, exposure, highContrast) {
    const gl = this.gl;
    gl.activeTexture(gl.TEXTURE0);

    let program = this.downsampleProgram;
    gl.useProgram(program);
    for (let level = 1; level < this.numLevels; ++level) {
      const targetTexture = this.mipmapTextures[level];
      gl.bindFramebuffer(gl.FRAMEBUFFER, this.mipmapFbos[level]);
      gl.viewport(1, 1, targetTexture.width - 2, targetTexture.height - 2);
      gl.bindTexture(gl.TEXTURE_2D, this.mipmapTextures[level - 1]);
      gl.uniform2f(program.sourceDeltaUvUniform, 
          1.0 / this.mipmapTextures[level - 1].width,
          1.0 / this.mipmapTextures[level - 1].height);
      this.drawQuad(program);
    }

    program = this.bloomProgram;
    gl.useProgram(program);
    for (let level = 1; level < this.numLevels; ++level) {
      const targetTexture = this.filterTextures[level];
      gl.bindFramebuffer(gl.FRAMEBUFFER, this.filterFbos[level]);
      gl.viewport(0, 0, targetTexture.width, targetTexture.height);
      gl.bindTexture(gl.TEXTURE_2D, this.mipmapTextures[level]);
      gl.uniform2f(program.sourceDeltaUvUniform, 
          1.0 / this.mipmapTextures[level].width,
          1.0 / this.mipmapTextures[level].height);
      for (let i = 0; i < 25; ++i) {
        gl.uniform3f(gl.getUniformLocation(program, `source_samples_uvw[${i}]`),
            this.bloomFilters[level][i][0],
            this.bloomFilters[level][i][1], 
            this.bloomFilters[level][i][2]);
      }
      this.drawQuad(program);
    }

    program = this.upsampleProgram;
    gl.activeTexture(gl.TEXTURE0);
    gl.enable(gl.BLEND);
    gl.blendEquation(gl.FUNC_ADD);
    gl.blendFunc(gl.ONE, gl.ONE);
    gl.useProgram(program);
    for (let level = this.numLevels - 2; level >= 1; --level) {
      const targetTexture = this.filterTextures[level];
      gl.bindFramebuffer(gl.FRAMEBUFFER, this.filterFbos[level]);
      gl.viewport(0, 0, targetTexture.width, targetTexture.height);
      gl.bindTexture(gl.TEXTURE_2D, this.filterTextures[level + 1]);
      gl.uniform2f(program.sourceDeltaUvUniform, 
          1.0 / this.filterTextures[level + 1].width,
          1.0 / this.filterTextures[level + 1].height);
      this.drawQuad(program);
    }
    gl.disable(gl.BLEND);

    gl.bindFramebuffer(gl.FRAMEBUFFER, null);
    gl.viewport(0, 0, this.width, this.height);

    program = this.renderProgram;
    gl.useProgram(program);
    gl.activeTexture(gl.TEXTURE0);
    gl.bindTexture(gl.TEXTURE_2D, this.mipmapTextures[0]);
    gl.activeTexture(gl.TEXTURE1);
    gl.bindTexture(gl.TEXTURE_2D, this.filterTextures[1]);
    gl.uniform2f(program.sourceDeltaUvUniform, 
        1.0 / this.mipmapTextures[0].width, 
        1.0 / this.mipmapTextures[0].height);
    gl.uniform2f(program.bloomDeltaUvUniform, 
        1.0 / this.filterTextures[1].width,
        1.0 / this.filterTextures[1].height);
    if (this.numLevels > 0) {
      for (let i = 0; i < 25; ++i) {
        gl.uniform3f(gl.getUniformLocation(program, `source_samples_uvw[${i}]`),
            this.bloomFilters[0][i][0],
            this.bloomFilters[0][i][1], 
            this.bloomFilters[0][i][2]);
      }
    }
    gl.uniform1f(program.intensityUniform, intensity);
    gl.uniform1f(program.exposureUniform, exposure);
    gl.uniform1i(program.highContrastUniform, highContrast ? 1 : 0);
    this.drawQuad(program);
  }

  drawQuad(program) {
    const gl = this.gl;
    const vertexAttrib = gl.getAttribLocation(program, 'vertex');
    gl.bindBuffer(gl.ARRAY_BUFFER, this.vertexBuffer);
    gl.vertexAttribPointer(
        vertexAttrib,
        /*numComponents=*/ 2,
        /*type=*/ this.gl.FLOAT,
        /*normalize=*/ false,
        /*stride=*/ 0,
        /*offset=*/ 0);
    gl.enableVertexAttribArray(vertexAttrib);
    gl.drawArrays(gl.TRIANGLE_STRIP, /*offset=*/ 0, /*vertexCount=*/ 4);
  }
}
