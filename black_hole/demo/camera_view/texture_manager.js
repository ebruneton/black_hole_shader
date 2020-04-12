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

const cubeMapTargets = function(gl) {
  return [
      gl.TEXTURE_CUBE_MAP_POSITIVE_X,
      gl.TEXTURE_CUBE_MAP_NEGATIVE_X,
      gl.TEXTURE_CUBE_MAP_POSITIVE_Y,
      gl.TEXTURE_CUBE_MAP_NEGATIVE_Y,
      gl.TEXTURE_CUBE_MAP_POSITIVE_Z,
      gl.TEXTURE_CUBE_MAP_NEGATIVE_Z];
};

const createTexture = function(gl, target) {
  const texture = gl.createTexture();
  gl.activeTexture(gl.TEXTURE0);
  gl.bindTexture(target, texture);
  gl.texParameteri(target, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
  gl.texParameteri(target, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
  gl.texParameteri(target, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
  gl.texParameteri(target, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
  return texture;
};

const loadTextureData = function(textureDataUrl, callback) {
  const xhr = new XMLHttpRequest();
  xhr.open('GET', textureDataUrl);
  xhr.responseType = 'arraybuffer';
  xhr.onload = (event) => {
    const data = new DataView(xhr.response);
    const array =
        new Float32Array(data.byteLength / Float32Array.BYTES_PER_ELEMENT);
    for (let i = 0; i < array.length; ++i) {
      array[i] = data.getFloat32(i * Float32Array.BYTES_PER_ELEMENT, true);
    }
    callback(array);
  };
  xhr.send();
};

const loadIntTextureData = function(textureDataUrl, callback) {
  const xhr = new XMLHttpRequest();
  xhr.open('GET', textureDataUrl);
  xhr.responseType = 'arraybuffer';
  xhr.onload = (event) => {
    const data = new DataView(xhr.response);
    const array =
        new Uint32Array(data.byteLength / Uint32Array.BYTES_PER_ELEMENT);
    for (let i = 0; i < array.length; ++i) {
      array[i] = data.getUint32(i * Uint32Array.BYTES_PER_ELEMENT, true);
    }
    callback(array);
  };
  xhr.send();
};

class TextureManager {
  constructor(rootElement, gl) {
    this.loadingPanel = rootElement.querySelector('#cv_loading_panel');
    this.loadingBar = rootElement.querySelector('#cv_loading_bar');
    this.gl = gl;

    this.rayDeflectionTexture = null;
    this.rayInverseRadiusTexture = null;
    this.blackbodyTexture = null;
    this.dopplerTexture = null;
    this.gridTexture = null;

    this.galaxyTexture = null;
    this.starTexture = null;
    this.starTexture2 = null;
    this.tilesQueue = [];
    this.numTilesLoaded = 0;
    this.numTilesLoadedPerLevel = [0, 0, 0, 0, 0];
    this.numPendingRequests = 0;

    const ext = gl.getExtension('EXT_texture_filter_anisotropic');
    this.loadTextures(ext);
    this.loadStarTextures(ext);

    document.body.addEventListener('keypress', (e) => this.onKeyPress(e)); 
  }

  loadTextures(ext) {
    const gl = this.gl;

    loadTextureData('deflection.dat', (data) => {
      this.rayDeflectionTexture = createTexture(gl, gl.TEXTURE_2D);
      this.rayDeflectionTexture.width = data[0];
      this.rayDeflectionTexture.height = data[1];
      gl.texImage2D(gl.TEXTURE_2D, 0, gl.RG32F, data[0], data[1], 0, 
                    gl.RG, gl.FLOAT, data.slice(2));
    });

    loadTextureData('inverse_radius.dat', (data) => {
      this.rayInverseRadiusTexture = createTexture(gl, gl.TEXTURE_2D);
      this.rayInverseRadiusTexture.width = data[0];
      this.rayInverseRadiusTexture.height = data[1];
      gl.texImage2D(gl.TEXTURE_2D, 0, gl.RG32F, data[0], data[1], 0,
                    gl.RG, gl.FLOAT, data.slice(2));
    });

    this.dopplerTexture = createTexture(gl, gl.TEXTURE_3D);
    gl.texParameteri(gl.TEXTURE_3D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
    gl.texParameteri(gl.TEXTURE_3D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
    gl.texParameteri(gl.TEXTURE_3D, gl.TEXTURE_WRAP_R, gl.CLAMP_TO_EDGE);
    loadTextureData('doppler.dat', (data) => {
      gl.activeTexture(gl.TEXTURE0);
      gl.bindTexture(gl.TEXTURE_3D, this.dopplerTexture);
      gl.texImage3D(gl.TEXTURE_3D, 0, gl.RGB32F, 64, 32, 64, 0, 
                    gl.RGB, gl.FLOAT, data);
    });

    this.blackbodyTexture = createTexture(gl, gl.TEXTURE_2D);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
    loadTextureData('black_body.dat', (data) => {
      gl.activeTexture(gl.TEXTURE0);
      gl.bindTexture(gl.TEXTURE_2D, this.blackbodyTexture);
      gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGB32F, 128, 1, 0,
                    gl.RGB, gl.FLOAT, data);
    });

    this.gridTexture = createTexture(gl, gl.TEXTURE_CUBE_MAP);
    gl.texStorage2D(gl.TEXTURE_CUBE_MAP, 10, gl.R8, 512, 512);
    gl.texParameteri(gl.TEXTURE_CUBE_MAP, gl.TEXTURE_MIN_FILTER, 
                     gl.LINEAR_MIPMAP_LINEAR);
    gl.texParameteri(gl.TEXTURE_CUBE_MAP, gl.TEXTURE_MAG_FILTER, gl.LINEAR);   
    gl.texParameterf(gl.TEXTURE_CUBE_MAP, ext.TEXTURE_MAX_ANISOTROPY_EXT, 
                     gl.getParameter(ext.MAX_TEXTURE_MAX_ANISOTROPY_EXT));
    const gridData = new Uint8Array(512 * 512);
    for (let j = 0; j < 512; ++j) {
      const jmod = (j + 2) % 32;
      for (let i = 0; i < 512; ++i) {
        const imod = (i + 2) % 32;
        gridData[i + j * 512] = (imod < 4 || jmod < 4) ? 255 : 0;
      }
    }
    for (let target of cubeMapTargets(gl)) {
      gl.texSubImage2D(target, 0, 0, 0, 512, 512, 
                       gl.RED, gl.UNSIGNED_BYTE, gridData, 0);
    }
    gl.generateMipmap(gl.TEXTURE_CUBE_MAP);
  }

  loadStarTextures(glExt) {
    const gl = this.gl;

    this.galaxyTexture = createTexture(gl, gl.TEXTURE_CUBE_MAP);
    gl.bindTexture(gl.TEXTURE_CUBE_MAP, this.galaxyTexture);
    gl.texStorage2D(gl.TEXTURE_CUBE_MAP, 12, gl.RGB9_E5, 2048, 2048);
    gl.texParameteri(gl.TEXTURE_CUBE_MAP, gl.TEXTURE_MIN_FILTER,
                     gl.LINEAR_MIPMAP_LINEAR);
    gl.texParameteri(gl.TEXTURE_CUBE_MAP, gl.TEXTURE_MAG_FILTER, gl.LINEAR);   
    gl.texParameterf(gl.TEXTURE_CUBE_MAP, glExt.TEXTURE_MAX_ANISOTROPY_EXT, 
                     gl.getParameter(glExt.MAX_TEXTURE_MAX_ANISOTROPY_EXT));

    this.starTexture = createTexture(gl, gl.TEXTURE_CUBE_MAP);
    gl.bindTexture(gl.TEXTURE_CUBE_MAP, this.starTexture);
    gl.texStorage2D(gl.TEXTURE_CUBE_MAP, 5, gl.RGB9_E5, 2048, 2048);
    gl.texParameteri(gl.TEXTURE_CUBE_MAP, gl.TEXTURE_MIN_FILTER, 
                     gl.NEAREST_MIPMAP_NEAREST);
    gl.texParameteri(gl.TEXTURE_CUBE_MAP, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
    gl.texParameteri(gl.TEXTURE_CUBE_MAP, gl.TEXTURE_MAX_LOD, 4);
    gl.texParameteri(gl.TEXTURE_CUBE_MAP, gl.TEXTURE_MAX_LEVEL, 4);

    this.starTexture2 = createTexture(gl, gl.TEXTURE_CUBE_MAP);
    gl.bindTexture(gl.TEXTURE_CUBE_MAP, this.starTexture2);
    gl.texStorage2D(gl.TEXTURE_CUBE_MAP, 7, gl.RGB9_E5, 64, 64);
    gl.texParameteri(gl.TEXTURE_CUBE_MAP, gl.TEXTURE_MIN_FILTER, 
                     gl.NEAREST_MIPMAP_LINEAR);
    gl.texParameteri(gl.TEXTURE_CUBE_MAP, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
    gl.texParameterf(gl.TEXTURE_CUBE_MAP, glExt.TEXTURE_MAX_ANISOTROPY_EXT, 
                     gl.getParameter(glExt.MAX_TEXTURE_MAX_ANISOTROPY_EXT));

    const base = '../../gaia_sky_map';
    const prefixes = ['pos-x', 'neg-x', 'pos-y', 'neg-y', 'pos-z', 'neg-z'];
    const targets = cubeMapTargets(gl);
    for (let l = 0; l <= 4; ++l) {
      for (let i = 0; i < 6; ++i) {
        const size = 2048 / (1 << l);
        const tileSize = Math.min(256, size);
        const numTiles = size / tileSize;
        for (let tj = 0; tj < numTiles; ++tj) {
          for (let ti = 0; ti < numTiles; ++ti) {
            const target = targets[i];
            const url = `${base}/${prefixes[i]}-${l}-${ti}-${tj}.dat`;
            this.tilesQueue.push({l, ti, tj, i, target, url});
          }
        }
      }
    }
    this.updateLoadingBar();
    this.loadStarTextureTiles();
  }

  loadStarTextureTiles() {
    while (this.tilesQueue.length > 0 && this.numPendingRequests < 6) {
      const tile = this.tilesQueue.pop();
      this.loadStarTextureTile(
          tile.l, tile.ti, tile.tj, tile.i, tile.target, tile.url);
    }
  }

  loadStarTextureTile(l, ti, tj, i, target, url) {
    const gl = this.gl;
    const size = 2048 / (1 << l);
    loadIntTextureData(url, (data) => {
      gl.activeTexture(gl.TEXTURE0);
      let start = 0;
      let level = l;
      let tileSize = Math.min(256, size);
      while (start < data.length) {
        gl.bindTexture(gl.TEXTURE_CUBE_MAP, this.galaxyTexture);
        gl.texSubImage2D(target, level, ti * tileSize, tj * tileSize, 
            tileSize, tileSize, gl.RGB, gl.UNSIGNED_INT_5_9_9_9_REV, 
            data.subarray(start, start + tileSize * tileSize), 0);
        start += tileSize * tileSize;
        if (level <= 4) {
          gl.bindTexture(gl.TEXTURE_CUBE_MAP, this.starTexture);
          gl.texSubImage2D(target, level, ti * tileSize, tj * tileSize, 
              tileSize, tileSize, gl.RGB, gl.UNSIGNED_INT_5_9_9_9_REV,
              data.subarray(start, start + tileSize * tileSize), 0);
        } else {
          gl.bindTexture(gl.TEXTURE_CUBE_MAP, this.starTexture2);
          gl.texSubImage2D(target, level - 5, ti * tileSize, tj * tileSize, 
              tileSize, tileSize, gl.RGB, gl.UNSIGNED_INT_5_9_9_9_REV, 
              data.subarray(start, start + tileSize * tileSize), 0);
        }
        start += tileSize * tileSize;
        level += 1;
        tileSize /= 2;
      }
      this.numTilesLoaded += 1;
      if (l <= 4) {
        this.numTilesLoadedPerLevel[l] += 1;
      }
      this.numPendingRequests -= 1;
      this.updateLoadingBar();
      this.loadStarTextureTiles();
    });
    this.numPendingRequests += 1;
  }

  updateLoadingBar() {
    this.loadingBar.style.width = `${this.numTilesLoaded / 516 * 100}%`;
    if (this.numTilesLoaded == 516) {
      this.loadingPanel.classList.toggle('cv-loaded');
    }
  } 

  getMinLoadedStarTextureLod() {
    if (this.numTilesLoadedPerLevel[0] == 384) {
      return 0;
    } else if (this.numTilesLoadedPerLevel[1] == 96) {
      return 1;
    } else if (this.numTilesLoadedPerLevel[2] == 24) {
      return 2;
    } else if (this.numTilesLoadedPerLevel[3] == 6) {
      return 3;
    }
    return 4;
  }

  onKeyPress(event) {
    if (event.key == ' ') {
      this.loadingPanel.classList.toggle('cv-hidden');
    }
  }
}

BlackHoleShaderDemoApp.TextureManager = TextureManager;
})();
