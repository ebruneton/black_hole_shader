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

const NEAR_PLANE = 0.1;
const FAR_PLANE = 100;

const ENV_MAP_LEVELS = 7;
const ENV_MAP_SIZE = 1 << (ENV_MAP_LEVELS - 1);

const EXHAUST_RADIUS = 0.514;
const EXHAUST_Z_MIN = -20;
const EXHAUST_Z_MAX = -2.1;

const createShader = function(gl, type, source) {
  const shader = gl.createShader(type);
  gl.shaderSource(shader, source);
  gl.compileShader(shader);
  return shader;
};

const loadRocketMesh = function(rocketDataUrl, callback) {
  const xhr = new XMLHttpRequest();
  xhr.open('GET', rocketDataUrl);
  xhr.responseType = 'arraybuffer';
  xhr.onload = (event) => {
    const data = new DataView(xhr.response);
    const numVertexFloats = data.getUint32(0, true);
    const numIndices = data.getUint32(Uint32Array.BYTES_PER_ELEMENT, true);

    let offset = 2 * Uint32Array.BYTES_PER_ELEMENT;
    const vertices = new Float32Array(numVertexFloats);
    for (let i = 0; i < numVertexFloats; ++i) {
      vertices[i] =
          data.getFloat32(i * Float32Array.BYTES_PER_ELEMENT + offset, true);
    }
 
    offset += numVertexFloats * Float32Array.BYTES_PER_ELEMENT;
    const indices = new Uint32Array(numIndices);
    for (let i = 0; i < numIndices; ++i) {
      indices[i] =
          data.getUint32(i * Uint32Array.BYTES_PER_ELEMENT + offset, true);
    }
    callback(vertices, indices);
  };
  xhr.send();
};

const loadRocketTexture = function(gl, textureUrl) {
  const glExt = gl.getExtension('EXT_texture_filter_anisotropic');
  const texture = gl.createTexture();
  gl.activeTexture(gl.TEXTURE0);
  gl.bindTexture(gl.TEXTURE_2D, texture);
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.REPEAT);
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER,
                   gl.LINEAR_MIPMAP_LINEAR);
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
  gl.texParameterf(gl.TEXTURE_2D, glExt.TEXTURE_MAX_ANISOTROPY_EXT, 
                   gl.getParameter(glExt.MAX_TEXTURE_MAX_ANISOTROPY_EXT));
  const image = new Image();
  image.addEventListener('load', function() {
    gl.bindTexture(gl.TEXTURE_2D, texture);
    gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGB, gl.RGB, gl.UNSIGNED_BYTE, image);
    gl.generateMipmap(gl.TEXTURE_2D);
  });
  image.src = textureUrl;
  return texture;
}

class RocketManager {
  constructor(model, gl) {
    this.model = model;
    this.gl = gl;
    this.rocketProgram = undefined;
    this.exhaustProgram = undefined;
    this.envMapTexture = undefined;
    this.envMapFbo = undefined;
    this.rocketVertexBuffer = undefined;
    this.rocketIndexBuffer = undefined;
    this.exhaustVertexBuffer = undefined;
    this.exhaustIndexBuffer = undefined;
    this.baseColorTexture = loadRocketTexture(gl, 'rocket_base_color.png');
    this.occlusionRoughnessMetallicTexture =
        loadRocketTexture(gl, 'rocket_occlusion_roughness_metallic.png');
    this.normalMapTexture = loadRocketTexture(gl, 'rocket_normal.png');

    this.createRocketProgram(gl);
    this.createExhaustProgram(gl);
    this.createEnvMap(gl);
    loadRocketMesh('rocket.dat', 
        (vertices, indices) => this.createRocketBuffers(vertices, indices));
    this.createExhaustBuffers(gl);
  }

  createEnvMap(gl) {
    this.envMapTexture = gl.createTexture();
    gl.activeTexture(gl.TEXTURE0);
    gl.bindTexture(gl.TEXTURE_CUBE_MAP, this.envMapTexture);
    gl.bindTexture(gl.TEXTURE_CUBE_MAP, this.envMapTexture);
    gl.texStorage2D(gl.TEXTURE_CUBE_MAP, ENV_MAP_LEVELS, gl.RGBA16F,
        ENV_MAP_SIZE, ENV_MAP_SIZE);
    gl.texParameteri(gl.TEXTURE_CUBE_MAP, gl.TEXTURE_MIN_FILTER,
                     gl.LINEAR_MIPMAP_LINEAR);
    gl.texParameteri(gl.TEXTURE_CUBE_MAP, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
    const glExt = gl.getExtension('EXT_texture_filter_anisotropic');
    gl.texParameterf(gl.TEXTURE_CUBE_MAP, glExt.TEXTURE_MAX_ANISOTROPY_EXT, 
                     gl.getParameter(glExt.MAX_TEXTURE_MAX_ANISOTROPY_EXT));

    this.envMapFbo = gl.createFramebuffer();
    gl.bindFramebuffer(gl.FRAMEBUFFER, this.envMapFbo);
    gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0,   
        gl.TEXTURE_CUBE_MAP_POSITIVE_X, this.envMapTexture, 0);
    gl.bindFramebuffer(gl.FRAMEBUFFER, null);
  }

  createRocketProgram(gl) {
    const vertexShader = createShader(
        gl, 
        gl.VERTEX_SHADER,
        `#version 300 es
        precision highp float;
        ${document.querySelector("#rocket_vertex_shader").innerHTML}`);
    const fragmentShader = createShader(
        gl,
        gl.FRAGMENT_SHADER,
        `#version 300 es
        precision highp float;
        const float ENV_MAP_SIZE = float(${ENV_MAP_SIZE});
        ${document.querySelector("#rocket_fragment_shader").innerHTML}`);
    const program = gl.createProgram();
    gl.attachShader(program, vertexShader);
    gl.attachShader(program, fragmentShader);
    gl.linkProgram(program);    

    program.positionAttrib =
        gl.getAttribLocation(program, 'position_attribute');
    program.normalAttrib =
        gl.getAttribLocation(program, 'normal_attribute');
    program.tangentAttrib =
        gl.getAttribLocation(program, 'tangent_attribute');
    program.uvAttrib =
        gl.getAttribLocation(program, 'uv_attribute');
    program.ambientOcclusionAttrib =
        gl.getAttribLocation(program, 'ambient_occlusion_attribute');
    program.modelViewProjMatrix =
        gl.getUniformLocation(program, 'model_view_proj_matrix');
    program.camera = 
        gl.getUniformLocation(program, 'camera');
    program.baseColorTexture =
        gl.getUniformLocation(program, 'base_color_texture');
    program.occlusionRoughnessMetallicTexture =
        gl.getUniformLocation(program, 'occlusion_roughness_metallic_texture');
    program.normalMapTexture =
        gl.getUniformLocation(program, 'normal_map_texture');
    program.envMapTexture =
        gl.getUniformLocation(program, 'env_map_texture');
    this.rocketProgram = program;
  }

  createExhaustProgram(gl) {
    const vertexShader = createShader(
        gl, 
        gl.VERTEX_SHADER,
        `#version 300 es
        precision highp float;
        ${document.querySelector("#exhaust_vertex_shader").innerHTML}`);
    const fragmentShader = createShader(
        gl,
        gl.FRAGMENT_SHADER,
        `#version 300 es
        precision highp float;
        const float RADIUS = float(${EXHAUST_RADIUS});
        const float Z_MIN = float(${EXHAUST_Z_MIN});
        const float Z_MAX = float(${EXHAUST_Z_MAX});
        ${document.querySelector("#exhaust_fragment_shader").innerHTML}`);
    const program = gl.createProgram();
    gl.attachShader(program, vertexShader);
    gl.attachShader(program, fragmentShader);
    gl.linkProgram(program);    

    program.positionAttrib =
        gl.getAttribLocation(program, 'position_attribute');
    program.modelViewProjMatrix =
        gl.getUniformLocation(program, 'model_view_proj_matrix');
    program.camera = gl.getUniformLocation(program, 'camera');
    program.intensity = gl.getUniformLocation(program, 'intensity');
    program.kZ = gl.getUniformLocation(program, 'k_z');
    program.kR = gl.getUniformLocation(program, 'k_r');
    this.exhaustProgram = program;
  }

  createRocketBuffers(vertices, indices) {
    const gl = this.gl;

    this.rocketVertexBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, this.rocketVertexBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, vertices, gl.STATIC_DRAW);

    this.rocketIndexBuffer = gl.createBuffer();
    this.rocketIndexBuffer.size = indices.length;
    gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, this.rocketIndexBuffer);
    gl.bufferData(gl.ELEMENT_ARRAY_BUFFER, indices, gl.STATIC_DRAW);    
  }

  createExhaustBuffers(gl) {
    const NUM_CIRCUMFERENCE_SAMPLES = 32;

    const vertices = new Float32Array(6 * (NUM_CIRCUMFERENCE_SAMPLES + 1));
    for (let i = 0; i <= NUM_CIRCUMFERENCE_SAMPLES; ++i) {
      const r = i == 0 ? 0 : EXHAUST_RADIUS;
      const alpha = (2 * Math.PI * i) / NUM_CIRCUMFERENCE_SAMPLES;
      vertices[6 * i] = r * Math.cos(alpha); 
      vertices[6 * i + 1] = r * Math.sin(alpha); 
      vertices[6 * i + 2] = EXHAUST_Z_MIN;
      vertices[6 * i + 3] = r * Math.cos(alpha); 
      vertices[6 * i + 4] = r * Math.sin(alpha); 
      vertices[6 * i + 5] = EXHAUST_Z_MAX;     
    }

    this.exhaustVertexBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, this.exhaustVertexBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, vertices, gl.STATIC_DRAW);

    const indices = new Uint32Array(12 * NUM_CIRCUMFERENCE_SAMPLES);
    for (let i = 1; i <= NUM_CIRCUMFERENCE_SAMPLES; ++i) {
      const j = (i % NUM_CIRCUMFERENCE_SAMPLES) + 1;
      indices[12 * i - 12] = 0;
      indices[12 * i - 11] = 2 * j;
      indices[12 * i - 10] = 2 * i;
      indices[12 * i - 9] = 2 * i;
      indices[12 * i - 8] = 2 * j;
      indices[12 * i - 7] = 2 * j + 1;
      indices[12 * i - 6] = 2 * j + 1;
      indices[12 * i - 5] = 2 * i + 1;
      indices[12 * i - 4] = 2 * i;
      indices[12 * i - 3] = 1;
      indices[12 * i - 2] = 2 * i + 1;
      indices[12 * i - 1] = 2 * j + 1;
    }    
    this.exhaustIndexBuffer = gl.createBuffer();
    this.exhaustIndexBuffer.size = indices.length;
    gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, this.exhaustIndexBuffer);
    gl.bufferData(gl.ELEMENT_ARRAY_BUFFER, indices, gl.STATIC_DRAW);    
  }

  renderEnvMap(program, quadVertexBuffer) {
    const gl = this.gl;
    const model = this.model;

    const currentViewport = gl.getParameter(gl.VIEWPORT);
    const currentFbo = gl.getParameter(gl.FRAMEBUFFER_BINDING);
    gl.bindFramebuffer(gl.FRAMEBUFFER, this.envMapFbo);
    gl.viewport(0, 0, ENV_MAP_SIZE, ENV_MAP_SIZE);

    gl.useProgram(program);
    gl.uniform3f(program.cameraSize, 
        ENV_MAP_SIZE / 2, ENV_MAP_SIZE / 2, ENV_MAP_SIZE / 2);
    gl.uniform3f(program.eTau,
        model.rocketTau[1], model.rocketTau[2], model.rocketTau[3]);

    gl.bindBuffer(gl.ARRAY_BUFFER, quadVertexBuffer);
    gl.vertexAttribPointer(program.vertexAttrib, 2, gl.FLOAT, false, 0, 0);
    gl.enableVertexAttribArray(program.vertexAttrib);

    gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0,   
        gl.TEXTURE_CUBE_MAP_POSITIVE_X, this.envMapTexture, 0);
    gl.uniform3f(program.eW,
        -model.rocketD[1], -model.rocketD[2], -model.rocketD[3]);
    gl.uniform3f(program.eH,
        -model.rocketH[1], -model.rocketH[2], -model.rocketH[3]);
    gl.uniform3f(program.eD,
        -model.rocketW[1], -model.rocketW[2], -model.rocketW[3]);
    gl.drawArrays(gl.TRIANGLE_STRIP, 0, 4);

    gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0,   
        gl.TEXTURE_CUBE_MAP_NEGATIVE_X, this.envMapTexture, 0);
    gl.uniform3f(program.eW,
        model.rocketD[1], model.rocketD[2], model.rocketD[3]);
    gl.uniform3f(program.eH,
        -model.rocketH[1], -model.rocketH[2], -model.rocketH[3]);
    gl.uniform3f(program.eD,
        model.rocketW[1], model.rocketW[2], model.rocketW[3]);
    gl.drawArrays(gl.TRIANGLE_STRIP, 0, 4);

    gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0,   
        gl.TEXTURE_CUBE_MAP_POSITIVE_Y, this.envMapTexture, 0);
    gl.uniform3f(program.eW,
        model.rocketW[1], model.rocketW[2], model.rocketW[3]);
    gl.uniform3f(program.eH,
        model.rocketD[1], model.rocketD[2], model.rocketD[3]);
    gl.uniform3f(program.eD,
        -model.rocketH[1], -model.rocketH[2], -model.rocketH[3]);
    gl.drawArrays(gl.TRIANGLE_STRIP, 0, 4);

    gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0,   
        gl.TEXTURE_CUBE_MAP_NEGATIVE_Y, this.envMapTexture, 0);
    gl.uniform3f(program.eW,
        model.rocketW[1], model.rocketW[2], model.rocketW[3]);
    gl.uniform3f(program.eH,
        -model.rocketD[1], -model.rocketD[2], -model.rocketD[3]);
    gl.uniform3f(program.eD,
        model.rocketH[1], model.rocketH[2], model.rocketH[3]);
    gl.drawArrays(gl.TRIANGLE_STRIP, 0, 4);

    gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0,   
        gl.TEXTURE_CUBE_MAP_POSITIVE_Z, this.envMapTexture, 0);
    gl.uniform3f(program.eW,
        model.rocketW[1], model.rocketW[2], model.rocketW[3]);
    gl.uniform3f(program.eH,
        -model.rocketH[1], -model.rocketH[2], -model.rocketH[3]);
    gl.uniform3f(program.eD,
        -model.rocketD[1], -model.rocketD[2], -model.rocketD[3]);
    gl.drawArrays(gl.TRIANGLE_STRIP, 0, 4);

    gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0,   
        gl.TEXTURE_CUBE_MAP_NEGATIVE_Z, this.envMapTexture, 0);
    gl.uniform3f(program.eW,
        -model.rocketW[1], -model.rocketW[2], -model.rocketW[3]);
    gl.uniform3f(program.eH,
        -model.rocketH[1], -model.rocketH[2], -model.rocketH[3]);
    gl.uniform3f(program.eD,
        model.rocketD[1], model.rocketD[2], model.rocketD[3]);
    gl.drawArrays(gl.TRIANGLE_STRIP, 0, 4);

    gl.disableVertexAttribArray(program.vertexAttrib);
    gl.bindFramebuffer(gl.FRAMEBUFFER, currentFbo);
    gl.viewport(currentViewport[0], currentViewport[1], currentViewport[2],
        currentViewport[3]);
  }

  drawRocket() {
    if (!this.rocketVertexBuffer) return;

    const gl = this.gl;
    gl.clear(gl.DEPTH_BUFFER_BIT);
    gl.enable(gl.DEPTH_TEST);
    gl.enable(gl.CULL_FACE);

    gl.activeTexture(gl.TEXTURE0);
    gl.bindTexture(gl.TEXTURE_2D, this.baseColorTexture);
    gl.activeTexture(gl.TEXTURE1);
    gl.bindTexture(gl.TEXTURE_2D, this.occlusionRoughnessMetallicTexture);
    gl.activeTexture(gl.TEXTURE2);
    gl.bindTexture(gl.TEXTURE_2D, this.normalMapTexture);
    gl.activeTexture(gl.TEXTURE3);
    gl.bindTexture(gl.TEXTURE_CUBE_MAP, this.envMapTexture);
    gl.generateMipmap(gl.TEXTURE_CUBE_MAP);

    const program = this.rocketProgram;
    gl.useProgram(program);
    gl.uniform1i(program.baseColorTexture, 0);
    gl.uniform1i(program.occlusionRoughnessMetallicTexture, 1);
    gl.uniform1i(program.normalMapTexture, 2);
    gl.uniform1i(program.envMapTexture, 3);
    this.setCameraUniforms(program);

    gl.bindBuffer(gl.ARRAY_BUFFER, this.rocketVertexBuffer);
    gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, this.rocketIndexBuffer);
    const stride = (3 + 3 + 4 + 1 + 2) * 4;
    gl.vertexAttribPointer(
        program.positionAttrib, 3, gl.FLOAT, false, stride, 0);
    gl.vertexAttribPointer(
        program.normalAttrib, 3, gl.FLOAT, false, stride, 3 * 4);
    gl.vertexAttribPointer(
        program.tangentAttrib, 4, gl.FLOAT, false, stride, 6 * 4);
    gl.vertexAttribPointer(
        program.uvAttrib, 2, gl.FLOAT, false, stride, 10 * 4);
    gl.vertexAttribPointer(
        program.ambientOcclusionAttrib, 1, gl.FLOAT, false, stride, 12 * 4);
    gl.enableVertexAttribArray(program.positionAttrib);
    gl.enableVertexAttribArray(program.normalAttrib);
    gl.enableVertexAttribArray(program.tangentAttrib);
    gl.enableVertexAttribArray(program.uvAttrib);
    gl.enableVertexAttribArray(program.ambientOcclusionAttrib);

    gl.drawElements(
        gl.TRIANGLES, this.rocketIndexBuffer.size, gl.UNSIGNED_INT, 0);

    gl.disableVertexAttribArray(program.positionAttrib);
    gl.disableVertexAttribArray(program.normalAttrib);
    gl.disableVertexAttribArray(program.tangentAttrib);
    gl.disableVertexAttribArray(program.uvAttrib);
    gl.disableVertexAttribArray(program.ambientOcclusionAttrib);
    gl.disable(gl.DEPTH_TEST);
    gl.disable(gl.CULL_FACE);
  }

  drawExhaust(time, gForce) {
    if (!this.rocketVertexBuffer) return;

    const gl = this.gl;
    gl.enable(gl.DEPTH_TEST);
    gl.enable(gl.BLEND);
    gl.blendEquation(gl.FUNC_ADD);
    gl.blendFunc(gl.ONE, gl.ONE);
    gl.enable(gl.CULL_FACE);

    const program = this.exhaustProgram;
    gl.useProgram(program);

    const intensity = 0.1 * Math.pow(gForce, 0.75);
    gl.uniform3f(program.intensity, 
        46 / 255 * intensity, 176 / 255 * intensity, intensity);

    time *= 100;
    const kR1 = 6.75 + 0.5 * Math.cos(time);
    const kR2 = 5.75 + 0.5 * Math.cos((time + 1) / Math.sqrt(2));
    const kR3 = 4.75 + 0.5 * Math.cos((time + 2) / Math.sqrt(3));
    const R2 = EXHAUST_RADIUS * EXHAUST_RADIUS;
    gl.uniform3f(program.kR, kR1 / R2, kR2 / R2, kR3 / R2);

    const kZ1 = 27 + 2 * Math.cos((time + 1) / Math.sqrt(2));
    const kZ2 = 23 + 2 * Math.cos((time + 2) / Math.sqrt(3));
    const kZ3 = 19 + 2 * Math.cos(time);
    const DZ = EXHAUST_Z_MAX - EXHAUST_Z_MIN;
    gl.uniform3f(program.kZ, kZ1 / DZ, kZ2 / DZ, kZ3 / DZ);
    this.setCameraUniforms(program);

    gl.bindBuffer(gl.ARRAY_BUFFER, this.exhaustVertexBuffer);
    gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, this.exhaustIndexBuffer);
    gl.vertexAttribPointer(
        program.positionAttrib, 3, gl.FLOAT, false, 0, 0);
    gl.enableVertexAttribArray(program.positionAttrib);

    gl.cullFace(gl.BACK);
    gl.drawElements(
        gl.TRIANGLES, this.exhaustIndexBuffer.size, gl.UNSIGNED_INT, 0);

    gl.cullFace(gl.FRONT);
    gl.drawElements(
        gl.TRIANGLES, this.exhaustIndexBuffer.size, gl.UNSIGNED_INT, 0);

    gl.disableVertexAttribArray(program.positionAttrib);
    gl.disable(gl.DEPTH_TEST);
    gl.disable(gl.BLEND);
    gl.disable(gl.CULL_FACE);
    gl.cullFace(gl.BACK);
  }

  setCameraUniforms(program) {
    const yaw = this.model.cameraYaw.getValue() + this.model.cameraYawOffset -
        this.model.rocketYaw;
    const cameraDist = this.model.rocketDistance.getValue() / 2;
    const offsetDist = 0.4 * cameraDist;
    const tx = -offsetDist * Math.sin(this.model.rocketYaw);
    const tz = offsetDist * Math.cos(this.model.rocketYaw);
    const cy = Math.cos(yaw);
    const sy = Math.sin(yaw);
    const cp = Math.cos(this.model.cameraPitch.getValue());
    const sp = Math.sin(this.model.cameraPitch.getValue());
    const modelViewMatrix = [
      [      cy,  0,      -sy,                    cy * tx      - sy * tz],
      [-sy * sp, cp, -cy * sp,             - sy * sp * tx - cy * sp * tz],
      [ sy * cp, sp,  cy * cp, -cameraDist + sy * cp * tx + cy * cp * tz],
      [       0,  0,        0,                                         1]
    ];

    const f = 1 / Math.tan(this.model.fovY / 2);
    const a = document.body.clientWidth / document.body.clientHeight;
    const b = -(FAR_PLANE + NEAR_PLANE) / (FAR_PLANE - NEAR_PLANE);
    const c = -2 * FAR_PLANE * NEAR_PLANE / (FAR_PLANE - NEAR_PLANE);
    const projMatrix = [
      [f / a, 0,  0, 0],
      [    0, f,  0, 0],
      [    0, 0,  b, c],
      [    0, 0, -1, 0]
    ];

    const modelViewProjMatrix = 
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    for (let i = 0; i < 4; ++i) {
      for (let j = 0; j < 4; ++j) {
        for (let k = 0; k < 4; ++k) {
          modelViewProjMatrix[i + 4 * j] +=
              projMatrix[i][k] * modelViewMatrix[k][j];
        }
      }
    }
    this.gl.uniformMatrix4fv(program.modelViewProjMatrix, false,
        modelViewProjMatrix);

    const camera = [0, 0, 0, 1];
    for (let i = 0; i < 3; ++i) {
      for (let j = 0; j < 3; ++j) {
        camera[i] -= modelViewMatrix[j][i] * modelViewMatrix[j][3];
      }
    }
    this.gl.uniform3f(program.camera, camera[0], camera[1], camera[2]);
  }
}

BlackHoleShaderDemoApp.RocketManager = RocketManager;
})();
