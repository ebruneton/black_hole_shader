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

(function(model, State){

class BoolParam {
  constructor(name, model) {
    this.name = name;
    this.model = model;
  }
  read(searchParams) {
    const value = searchParams.get(this.name);
    if (this.model.getDefaultValue()) {
      this.model.setValue(value != '0');
    } else {
      this.model.setValue(value == '1');
    }
  }
  write(searchParams) {
    if (this.model.getValue() == this.model.getDefaultValue()) {
      searchParams.delete(this.name);
    } else {
      searchParams.set(this.name, this.model.getValue() ? '1' : '0');
    }
  }
}

class IntParam {
  constructor(name, model) {
    this.name = name;
    this.model = model;
  }
  read(searchParams) {
    const index = parseInt(searchParams.get(this.name));
    if (index >= 0) {
      this.model.setIndex(index);
    }
  }
  write(searchParams) {
    if (this.model.getIndex() == this.model.getDefaultIndex()) {
      searchParams.delete(this.name);
    } else {
      searchParams.set(this.name, this.model.getIndex());
    }
  }
}

class UrlParams {
  constructor(model) {
    this.model = model;

    this.params = [];
    this.params.push(new IntParam('cy', model.cameraYaw));
    this.params.push(new IntParam('cp', model.cameraPitch));
    this.params.push(new IntParam('ce', model.exposure));
    this.params.push(new IntParam('cb', model.bloom));
    this.params.push(new IntParam('or', model.startRadius));
    this.params.push(new IntParam('od', model.startDirection));
    this.params.push(new IntParam('os', model.startSpeed));
    this.params.push(new IntParam('oi', model.orbitInclination));
    this.params.push(new BoolParam('pl', model.lensing));
    this.params.push(new BoolParam('pd', model.doppler));
    this.params.push(new BoolParam('sr', model.rocket));
    this.params.push(new BoolParam('sg', model.grid));
    this.params.push(new IntParam('bhm', model.blackHoleMass));
    this.params.push(new IntParam('dd', model.discDensity));
    this.params.push(new IntParam('do', model.discOpacity));
    this.params.push(new IntParam('dt', model.discTemperature));
    this.params.push(new IntParam('sfy', model.starsYaw));
    this.params.push(new IntParam('sfp', model.starsPitch));
    this.params.push(new IntParam('sfr', model.starsRoll));
    this.params.push(new BoolParam('sfe', model.stars));
    this.lastState = undefined;

    this.timeout = null;
    this.readUrlParams();
    this.model.addListener(this);
  }

  onSettingsChange() {
    if (this.timeout) {
      clearTimeout(this.timeout);
    }
    this.timeout = setTimeout(() => this.writeUrlParams(), 500);
  }

  onOrbitChange() {
    if (this.model.state == this.lastState) {
      return;
    }
    this.lastState = this.model.state;
    if (this.model.state != State.PLAYED) {
      this.writeUrlParams();
    }
  }

  readUrlParams() {
    const searchParams = new URLSearchParams(window.location.search);
    for (let param of this.params) {
      param.read(searchParams);
    }
    const r = parseFloat(searchParams.get('r'));
    const dr = parseFloat(searchParams.get('dr'));
    const phi = parseFloat(searchParams.get('phi'));
    if (!isNaN(r) && !isNaN(dr) && !isNaN(phi) && r > 1) {
      this.model.r = r;
      this.model.drOverDtau = dr;
      this.model.phi = phi;
      this.model.state = State.PAUSED;
    }
  }

  writeUrlParams() {
    const url = new URL(window.location.toString());
    const searchParams = new URLSearchParams(url.search);
    for (let param of this.params) {
      param.write(searchParams);
    }
    if (model.state == State.PAUSED) {
      searchParams.set('r', this.model.r);
      searchParams.set('dr', this.model.drOverDtau);
      searchParams.set('phi', this.model.phi);
    } else {
      searchParams.delete('r');
      searchParams.delete('dr');
      searchParams.delete('phi');
    }
    url.search = searchParams.toString();
    window.history.replaceState(null, null, url.toString());
    this.timeout = null;
  }
}

window.addEventListener('DOMContentLoaded', () => new UrlParams(model));
})(BlackHoleShaderDemoApp.model, BlackHoleShaderDemoApp.State);
