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

(function(model, Bloom, TextureManager, ShaderManager) {

const createQuadVertexBuffer = function(gl) {
  const vertexBuffer = gl.createBuffer();
  gl.bindBuffer(gl.ARRAY_BUFFER, vertexBuffer);
  gl.bufferData(gl.ARRAY_BUFFER,
       new Float32Array([-1, -1, +1, -1, -1, +1, +1, +1]), gl.STATIC_DRAW);
  return vertexBuffer;
};

class CameraView {
  constructor(model, rootElement) {
    this.model = model;
    this.rootElement = rootElement;
    this.devicePixelRatio = this.getDevicePixelRatio();
    this.canvas = rootElement.querySelector('#camera_view');
    this.canvas.style.width = `${rootElement.clientWidth}px`;
    this.canvas.style.height = `${rootElement.clientHeight}px`;
    this.canvas.width = rootElement.clientWidth * this.devicePixelRatio;
    this.canvas.height = rootElement.clientHeight * this.devicePixelRatio;
    this.errorPanel = rootElement.querySelector('#cv_error_panel');
    this.errorPanelShown = false;

    this.gl = this.canvas.getContext('webgl2');
    if (!this.initGl()) {
      return;
    }

    this.vertexBuffer = createQuadVertexBuffer(this.gl);
    this.textureManager = new TextureManager(rootElement, this.gl);
    this.shaderManager = new ShaderManager(model, this.textureManager, this.gl);
    this.bloom = new Bloom(this.gl, this.canvas.width, this.canvas.height);

    this.lastTauSeconds = Date.now() / 1000.0;
    this.lastFrameTime = undefined;
    this.numFrames = 0;

    this.drag = false;
    this.previousMouseX = undefined;
    this.previousMouseY = undefined;
    this.hidden = false;

    window.addEventListener('mousedown', (e) => this.onMouseDown(e));
    window.addEventListener('mousemove', (e) => this.onMouseMove(e));
    window.addEventListener('mouseup', (e) => this.onMouseUp(e));
    window.addEventListener('resize', (e) => this.onResize(e));
    document.addEventListener('visibilitychange', (e) => {
      this.hidden = document.hidden;
      if (!this.hidden) {
        this.lastFrameTime = undefined;
      }
    });

    requestAnimationFrame(() => this.onRender());
  }

  initGl() {
    if (!this.gl ||
        !this.gl.getExtension('OES_texture_float_linear') ||
        !this.gl.getExtension('EXT_texture_filter_anisotropic') ||
        !this.gl.getExtension('EXT_color_buffer_float') ||
        !this.gl.getExtension('EXT_float_blend')) {
      this.errorPanel.innerHTML = 'Unfortunately your browser doesn\'t ' + 
          'support WebGL 2 or the WebGL 2 extensions required for this demo.';
      this.errorPanel.classList.toggle('cv-hidden');
      return false;
    }
    this.errorPanel.addEventListener('click', () => {
      this.errorPanel.classList.toggle('cv-hidden');
    });
    return true;
  }

  onRender() {
    if (this.hidden) {
      return;
    }
    const program = this.shaderManager.getProgram();
    if (!program) {
      requestAnimationFrame(() => this.onRender());
      return;
    }
    if (this.devicePixelRatio != this.getDevicePixelRatio()) {
      this.onResize();      
    }

    const tanFovY = Math.tan(this.model.fovY / 2);
    const focalLength = this.canvas.height / (2 * tanFovY);

    const gl = this.gl; 
    gl.activeTexture(gl.TEXTURE0);
    gl.bindTexture(gl.TEXTURE_2D, this.textureManager.rayDeflectionTexture);      

    gl.activeTexture(gl.TEXTURE1);
    gl.bindTexture(gl.TEXTURE_2D, this.textureManager.rayInverseRadiusTexture);

    gl.activeTexture(gl.TEXTURE2);
    if (this.model.grid.getValue()) {
      gl.bindTexture(gl.TEXTURE_CUBE_MAP, this.textureManager.gridTexture);
    } else {
      gl.bindTexture(gl.TEXTURE_CUBE_MAP, this.textureManager.galaxyTexture);
    }
    const minLod = this.model.grid.getValue() ?
        0 : this.textureManager.getMinLoadedStarTextureLod();
    gl.texParameteri(gl.TEXTURE_CUBE_MAP, gl.TEXTURE_MIN_LOD, minLod);

    gl.activeTexture(gl.TEXTURE3);
    gl.bindTexture(gl.TEXTURE_CUBE_MAP, this.textureManager.starTexture);

    gl.activeTexture(gl.TEXTURE4);
    gl.bindTexture(gl.TEXTURE_CUBE_MAP, this.textureManager.starTexture2);

    gl.activeTexture(gl.TEXTURE5);
    gl.bindTexture(gl.TEXTURE_2D, this.textureManager.blackbodyTexture);

    gl.activeTexture(gl.TEXTURE6);
    gl.bindTexture(gl.TEXTURE_3D, this.textureManager.dopplerTexture);

    gl.activeTexture(gl.TEXTURE7);
    gl.bindTexture(gl.TEXTURE_2D, this.textureManager.noiseTexture);

    gl.useProgram(program);
    gl.uniform3f(program.cameraSize, 
        this.canvas.width / 2, this.canvas.height / 2, focalLength);
    gl.uniform4f(program.cameraPosition, 
        this.model.t, this.model.r, this.model.worldTheta, this.model.worldPhi);
    gl.uniform3f(program.p, this.model.p[0], this.model.p[1], this.model.p[2]);
    gl.uniform4f(program.kS, 
        this.model.kS[0], this.model.kS[1], this.model.kS[2], this.model.kS[3]);
    gl.uniform3f(program.eTau,
        this.model.eTau[1], this.model.eTau[2], this.model.eTau[3]);
    gl.uniform3f(program.eW,
        this.model.eW[1], this.model.eW[2], this.model.eW[3]);
    gl.uniform3f(program.eH,
        this.model.eH[1], this.model.eH[2], this.model.eH[3]);
    gl.uniform3f(program.eD,
        this.model.eD[1], this.model.eD[2], this.model.eD[3]);
    gl.uniform1i(program.rayDeflectionTexture, 0);
    gl.uniform1i(program.rayInverseRadiusTexture, 1); 
    gl.uniform1i(program.galaxyCubeTexture, 2);
    gl.uniform1i(program.starCubeTexture, 3);
    gl.uniform1i(program.starCubeTexture2, 4);
    gl.uniformMatrix3fv(program.starsOrientation, false, 
        this.model.starsMatrix);
    gl.uniform1f(program.minStarsLod, minLod);
    gl.uniform1i(program.blackBodyTexture, 5);
    gl.uniform1i(program.dopplerTexture, 6);
    gl.uniform1i(program.noiseTexture, 7);
    gl.uniform3f(program.discParams, 
        this.model.discDensity.getValue(),
        this.model.discOpacity.getValue(), 
        this.model.discTemperature.getValue());

    this.bloom.begin();

    gl.bindBuffer(gl.ARRAY_BUFFER, this.vertexBuffer);
    gl.vertexAttribPointer(program.vertexAttrib, 2, gl.FLOAT, false, 0, 0);
    gl.enableVertexAttribArray(program.vertexAttrib);
    gl.drawArrays(gl.TRIANGLE_STRIP, 0, 4);

    this.bloom.end(this.model.bloom.getValue(), this.model.exposure.getValue(),
        this.model.highContrast.getValue());

    const tauSeconds = Date.now() / 1000.0;
    const dTauSeconds = tauSeconds - this.lastTauSeconds;
    this.lastTauSeconds = tauSeconds;
    this.model.updateOrbit(dTauSeconds);

    requestAnimationFrame(() => this.onRender());
    this.checkFrameRate();
  }

  checkFrameRate() {
    this.numFrames += 1;
    const time = Date.now();
    if (!this.lastFrameTime) {
      this.lastFrameTime = time;
      this.numFrames = 0;
    }
    if (time > this.lastFrameTime + 1000) {
      if (this.numFrames <= 10 && this.model.stars.getValue() && 
          !this.errorPanelShown) {
        this.model.stars.setValue(false);
        this.errorPanel.innerHTML = 'Stars have been automatically disabled ' +
            'to improve performance. You can re-enable them from the left ' +
            'hand side panel.';
        this.errorPanel.classList.toggle('cv-hidden');
        this.errorPanel.classList.toggle('cv-warning');
        this.errorPanelShown = true;
      }
      this.lastFrameTime = time;
      this.numFrames = 0;
    }
  }

  onMouseDown(event) {
    this.previousMouseX = event.screenX;
    this.previousMouseY = event.screenY;
    this.drag = (event.target.tagName != 'INPUT') && !event.ctrlKey;
  }

  onMouseMove(event) {
    const mouseX = event.screenX;
    const mouseY = event.screenY;
    if (this.drag) {
      const kScale = 500;
      let yaw = this.model.cameraYaw.getValue();
      let pitch = this.model.cameraPitch.getValue();
      yaw += (this.previousMouseX - mouseX) / kScale;
      pitch -= (this.previousMouseY - mouseY) / kScale;
      this.model.cameraYaw.setValue(
          yaw - 2 * Math.PI * Math.floor(yaw / (2 * Math.PI)));
      this.model.cameraPitch.setValue(pitch);
    }
    this.previousMouseX = mouseX;
    this.previousMouseY = mouseY;
  }

  onMouseUp(event) {
    this.drag = false;
  }

  onResize(event) {
    const rootElement = this.rootElement;
    this.devicePixelRatio = this.getDevicePixelRatio();
    this.canvas.style.width = `${rootElement.clientWidth}px`;
    this.canvas.style.height = `${rootElement.clientHeight}px`;
    this.canvas.width = rootElement.clientWidth * this.devicePixelRatio;
    this.canvas.height = rootElement.clientHeight * this.devicePixelRatio;
    this.bloom.resize(this.canvas.width, this.canvas.height);
  }

  getDevicePixelRatio() {
    return this.model.highDefinition.getValue() ? window.devicePixelRatio : 1;
  }
}

window.addEventListener('DOMContentLoaded', () => {
  new CameraView(model, document.body);
});
})(BlackHoleShaderDemoApp.model,
    BlackHoleShaderDemoApp.Bloom,
    BlackHoleShaderDemoApp.TextureManager,
    BlackHoleShaderDemoApp.ShaderManager);
