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

(function() {

const generateDiscParticleParams = function() {
  const rMin = 3.0;
  const rMax = 12.0;
  const computeDthetaDphi = function(u1, u2, u3) {
    const k2 = (u2 - u1) / (u3 - u1);
    const N = 100000;
    let K = 0.0;
    for (let i = 0; i < N; ++i) {
      const dy = 1.0 / N;
      const y = (i + 0.5) / N;
      K += dy / Math.sqrt((1 - y * y) * (1 - k2 * y * y));
    }
    return Math.PI * Math.sqrt(u3 - u1) / (4 * K);
  };

  let ringParams = '';
  let numRings = 0;
  for (let r1 = rMin; r1 < rMax; r1 += 0.2) {
    const e = 0.1 * Math.random();
    const r2 = r1 * (1.0 + e) / (1.0 - e);
    const u1 = 1 / r2;
    const u2 = 1 / r1;
    const u3 = 1 - u1 - u2;
    const phi0 = 2 * Math.PI * Math.random();
    const dThetaDphi = computeDthetaDphi(u1, u2, u3);

    const x = u1.toPrecision(3);
    const y = u2.toPrecision(3);
    const z = phi0.toPrecision(3);
    const w = dThetaDphi.toPrecision(3);
    ringParams += `${numRings == 0 ? '' : ',\n'}vec4(${x}, ${y}, ${z}, ${w})`;
    numRings += 1;
  }

  return `
      const float INNER_DISC_R = ${rMin.toPrecision(3)};
      const float OUTER_DISC_R = ${rMax.toPrecision(3)};
      const int NUM_DISC_PARTICLES = ${numRings};
      const vec4 DISC_PARTICLE_PARAMS[${numRings}] = vec4[${numRings}] (
        ${ringParams}
      );`;
}

const createShader = function(gl, type, source) {
  const shader = gl.createShader(type);
  gl.shaderSource(shader, source);
  gl.compileShader(shader);
  return shader;
};

class ShaderManager {
  constructor(model, textureManager, gl) {
    this.model = model;
    this.textureManager = textureManager;
    this.gl = gl;

    this.programs = {};
    this.program = null;
  }

  getProgram() {
    const options =
        `#define LENSING ${this.model.lensing.getValue() ? 1 : 0}
        #define DOPPLER ${this.model.doppler.getValue() ? 1 : 0}
        #define GRID ${this.model.grid.getValue() ? 1 : 0}
        #define STARS ${this.model.stars.getValue() ? 1 : 0}`;
    this.program = this.programs[options];
    if (this.program ||
        !this.textureManager.rayDeflectionTexture ||
        !this.textureManager.rayInverseRadiusTexture) {
      return this.program;
    }

    const header = 
        `#version 300 es
        precision highp float;
        #define IN(x) const in x
        #define OUT(x) out x
        ${options}
        const float pi = ${Math.PI};
        const float rad = 1.0;
        const int RAY_DEFLECTION_TEXTURE_WIDTH = 
            ${this.textureManager.rayDeflectionTexture.width};
        const int RAY_DEFLECTION_TEXTURE_HEIGHT =
            ${this.textureManager.rayDeflectionTexture.height};
        const int RAY_INVERSE_RADIUS_TEXTURE_WIDTH = 
            ${this.textureManager.rayInverseRadiusTexture.width};
        const int RAY_INVERSE_RADIUS_TEXTURE_HEIGHT = 
            ${this.textureManager.rayInverseRadiusTexture.height};
        const float STARS_CUBE_MAP_SIZE =
            float(${this.model.grid.getValue() ? 128 : 2048});
        const float MAX_FOOTPRINT_SIZE = float(4);
        const float MAX_FOOTPRINT_LOD =
            float(${this.model.grid.getValue() ? 0 : 4});\n`;

    const gl = this.gl;
    const vertexShader = createShader(
        gl, 
        gl.VERTEX_SHADER,
        `#version 300 es
        precision highp float;
        ${document.querySelector("#vertex_shader").innerHTML}`);

    const fragmentShader = createShader(
        gl,
        gl.FRAGMENT_SHADER,
        `${header}
        ${generateDiscParticleParams()} 
        ${document.querySelector("#black_hole_shader").innerHTML}
        ${document.querySelector("#fragment_shader").innerHTML}`);

    const program = gl.createProgram();
    gl.attachShader(program, vertexShader);
    gl.attachShader(program, fragmentShader);
    gl.linkProgram(program);
    
    program.vertexAttrib = gl.getAttribLocation(program, 'vertex');
    program.cameraSize =
        gl.getUniformLocation(program, 'camera_size');
    program.cameraPosition =
        gl.getUniformLocation(program, 'camera_position');
    program.p =
        gl.getUniformLocation(program, 'p');
    program.kS =
        gl.getUniformLocation(program, 'k_s');
    program.eTau =
        gl.getUniformLocation(program, 'e_tau');
    program.eW =
        gl.getUniformLocation(program, 'e_w');
    program.eH =
        gl.getUniformLocation(program, 'e_h');
    program.eD =
        gl.getUniformLocation(program, 'e_d');
    program.rayDeflectionTexture = 
        gl.getUniformLocation(program, 'ray_deflection_texture');
    program.rayInverseRadiusTexture = 
        gl.getUniformLocation(program, 'ray_inverse_radius_texture');
    program.galaxyCubeTexture = 
        gl.getUniformLocation(program, 'galaxy_cube_texture');
    program.starCubeTexture = 
        gl.getUniformLocation(program, 'star_cube_texture');
    program.starCubeTexture2 = 
        gl.getUniformLocation(program, 'star_cube_texture2');
    program.starsOrientation =
        gl.getUniformLocation(program, 'stars_orientation');
    program.minStarsLod = 
        gl.getUniformLocation(program, 'min_stars_lod');
    program.blackBodyTexture = 
        gl.getUniformLocation(program, 'black_body_texture');
    program.dopplerTexture =
        gl.getUniformLocation(program, 'doppler_texture');
    program.discParams =
        gl.getUniformLocation(program, 'disc_params');

    this.programs[options] = program;
    this.program = program;
    return program;
  }
}

BlackHoleShaderDemoApp.ShaderManager = ShaderManager;
})();
