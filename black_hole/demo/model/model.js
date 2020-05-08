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

BlackHoleShaderDemoApp = {};
(function() {

// The speed of light.
const C = 299792458;

// The gravitational constant.
const G = 6.6743e-11;

// The mass of the Sun.
const SOLAR_MASS = 1.98847e30;

class BooleanValue {
  constructor(model, defaultValue) {
    this.model = model;
    this.value = defaultValue;
    this.defaultValue = defaultValue;
  }
  getDefaultValue() { return this.defaultValue; }
  getValue() { return this.value; }
  setValue(value) {
    this.value = !!value;
    this.model.notifyListeners();
  }
}

class QuantizedValue {
  constructor(model, f, defaultIndex, size = 1000) {
    this.model = model;
    this.values = Array.from({length: size + 1}, (value, i) => f(i / size));
    this.index = defaultIndex;
    this.defaultIndex = defaultIndex;
  }
  getSize() { return this.values.length; }
  getDefaultIndex() { return this.defaultIndex; }
  getIndex() { return this.index; }
  getValue() { return this.values[this.index]; }
  setIndex(index) { 
    this.index = Math.max(0, Math.min(index, this.values.length - 1)); 
    this.model.notifyListeners();
  }
  setValue(value) {
    let i0 = 0;
    let i1 = this.values.length - 1;
    if (value <= this.values[i0]) {
      this.index = i0;
    } else if (value >= this.values[i1]) {
      this.index = i1;
    } else {
      while (i1 > i0 + 1) {
        const i = Math.floor((i0 + i1) / 2);
        if (value < this.values[i]) {
          i1 = i;
        } else {
          i0 = i;
        }
      }
      this.index = value - this.values[i0] < this.values[i1] - value ? i0 : i1;
    }
    this.model.notifyListeners();
  }
}

const State = {
  STOPPED: 'STOPPED',
  PLAYING: 'PLAYING',
  PAUSED: 'PAUSED'
};

const safeSqrt = function(x) {
  return Math.sqrt(Math.max(x, 0));
};

const matrixProduct = function(a, b) {
  const c = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]];
  for (let i = 0; i < 4; ++i) {
    for (let j = 0; j < 4; ++j) {
      for (let k = 0; k < 4; ++k) {
        c[i][j] += a[i][k] * b[k][j];
      }
    }
  }
  return c;
};

const vectorMatrixProduct = function(v, m) {
  const c = [0, 0, 0, 0];
  for (let i = 0; i < 4; ++i) {
    for (let j = 0; j < 4; ++j) {
      c[i] += v[j] * m[j][i];
    }
  }
  return c;
};

class Model {
  constructor() {
    this.cameraYaw = 
        new QuantizedValue(this, (x) => 2 * Math.PI * x, 0, 36000);
    this.cameraPitch =
        new QuantizedValue(this, (x) => Math.PI * (x - 0.5), 9000, 18000);
    this.exposure = 
        new QuantizedValue(this, (x) => Math.pow(10, 3 * x - 3), 500);
    this.bloom = 
        new QuantizedValue(this, (x) => x, 500);
    this.highDefinition =
        new BooleanValue(this, false);
    this.startRadius = 
        new QuantizedValue(this, (x) => Math.max(1 + 39 * x * x, 1.01), 940);
    this.startDirection = 
        new QuantizedValue(this, (x) => Math.PI * (x - 0.5), 1800, 1800);
    this.startSpeed = 
        new QuantizedValue(this, (x) => Math.min(x * x, 0.99), 347);
    this.orbitInclination = 
        new QuantizedValue(this, (x) => Math.PI * (x - 0.5), 970, 1799);
    this.lensing =
        new BooleanValue(this, true);
    this.doppler =
        new BooleanValue(this, true);
    this.rocket =
        new BooleanValue(this, true);
    this.grid =
        new BooleanValue(this, false);
    this.blackHoleMass = 
        new QuantizedValue(this, (x) => 10 * Math.pow(10, 6 * x), 330);
    this.discDensity = 
        new QuantizedValue(this, (x) => 100 * Math.pow(x, 10), 500);
    this.discOpacity =
        new QuantizedValue(this, (x) => x, 300);
    this.discTemperature = 
        new QuantizedValue(this, (x) => 1000 * Math.pow(10, x), 350);
    this.starsYaw = 
        new QuantizedValue(this, (x) => 2 * Math.PI * (x - 0.5), 1800, 3600);
    this.starsPitch = 
        new QuantizedValue(this, (x) => Math.PI * (x - 0.5), 900, 1800);
    this.starsRoll = 
        new QuantizedValue(this, (x) => 2 * Math.PI * (x - 0.5), 1800, 3600);
    this.stars =
        new BooleanValue(this, true);
    this.starsMatrix = undefined;

    // The current state of the camera (i.e. the observer) motion.
    this.state = State.STOPPED;
    // The constant of motion dt / dTau = e / (1 - u) of the camera.
    this.e = undefined;
    // The constant of motion dphi / dTau = l u^2 of the camera.
    this.l = undefined;
    // The current value of the Schwarzschild's t coordinate of the camera.
    this.t = 0;
    // The current value of the Schwarzschild's r coordinate of the camera.
    this.r = undefined;
    // The current value of the derivative of r with respect to proper time.
    this.drOverDtau = undefined;
    // The current value of the Schwarzschild's theta coordinate of the camera.
    this.worldTheta = undefined;
    // The current value of the Schwarzschild's phi coordinate of the camera.
    this.worldPhi = undefined;
    // The current value of the Schwarzschild's phi coordinate of the camera (in
    // rotated coordinates such that its orbit is in the equatorial plane).
    this.phi = undefined;
    // The Lorentz transformation matrix specifying the current camera 
    // orientation and velocity.
    this.lorentz = undefined;
    // The camera position, in (pseudo-)Cartesian coordinates.
    this.p = undefined;
    // The camera 4-velocity, in Schwarzschild coordinates.
    this.kS = undefined;
    // The camera's vertical field of view.
    this.fovY = 50 / 180 * Math.PI;
    // The base vectors of the camera reference frame, in (pseudo-)Cartesian
    // coordinates.
    this.eTau = undefined;
    this.eW = undefined;
    this.eH = undefined;
    this.eD = undefined;

    this.blackHoleRadiusMeters = undefined;
    this.speedMetersPerSecond = undefined;
    this.gForce = undefined;
    this.localElapsedTimeSeconds = 0;
    this.globalElapsedTimeSeconds = 0;

    this.updateDerivedValues();
    this.listeners = [];
  }

  addListener(listener) {
    this.listeners.push(listener);
  }

  setState(state) {
    if (state != this.state) {
      this.state = state;
      if (state == State.PLAYING) {
        this.t = 0;
        this.localElapsedTimeSeconds = 0;
        this.globalElapsedTimeSeconds = 0;
      }
      this.notifyListeners(false);
    }
  }

  updateOrbit(dTauSeconds) {
    const M = this.blackHoleMass.getValue() * SOLAR_MASS;
    const dTauOverDtauSeconds = C * C * C / ( 2 * G * M);
    const dTau = dTauOverDtauSeconds * dTauSeconds;

    let u = 1 / this.r;
    const e = this.e;
    const l = this.l;
    const dtOverDtau = 
        this.state == State.PLAYING ? e / (1 - u) : 1 / Math.sqrt(1 - u);
    this.t += dtOverDtau * dTau;
    this.localElapsedTimeSeconds += dTauSeconds;
    this.globalElapsedTimeSeconds += dtOverDtau * dTauSeconds;

    if (this.state == State.PLAYING) {
      const n = 1000;
      const dTauN = dTau / n;
      for (let i = 0; i < n; ++i) {
        u = 1 / this.r;
        const d2rOverDtau2 = u * u * (l * l * (2 - 3 * u) * u - 1) / 2;
        this.drOverDtau += d2rOverDtau2 * dTauN;
        this.r += this.drOverDtau * dTauN;
        this.phi += l * u * u * dTauN;
        if (this.r <= 1.0 || this.r > 100.0) {
          this.setState(State.STOPPED);
          return;
        }
      }
    }
    this.notifyListeners(false);
  }

  notifyListeners(settingsChanged = true) {
    this.updateDerivedValues();
    for (let listener of this.listeners) {
      if (settingsChanged) {
        listener.onSettingsChange();
      }
      listener.onOrbitChange();
    }
  }

  updateDerivedValues() {
    this.updateStarsMatrix();
    this.updateCameraCoordinates();
    this.updateCameraLorentzTransform();
    this.updateCameraReferenceFrame();
    this.updateOrbitInfo();
  }

  updateStarsMatrix() {
    const cy = Math.cos(this.starsYaw.getValue() + Math.PI);
    const sy = Math.sin(this.starsYaw.getValue() + Math.PI);
    const cp = Math.cos(this.starsPitch.getValue());
    const sp = Math.sin(this.starsPitch.getValue());
    const cr = Math.cos(this.starsRoll.getValue());
    const sr = Math.sin(this.starsRoll.getValue());
    this.starsMatrix = [
                     cp * cy,                cp * sy,     -sp,
      sr * sp * cy - cr * sy, sr * sp * sy + cr * cy, sr * cp,
      cr * sp * cy + sr * sy, cr * sp * sy - sr * cy, cr * cp
    ];
  }

  updateCameraCoordinates() {
    const r0 = this.startRadius.getValue();
    const delta = this.startDirection.getValue();
    const v = this.startSpeed.getValue();

    // Compute the constants of motion from the initial conditions r0, delta, v.
    const u0 = 1 / r0;
    const cotDelta = 1 / Math.tan(delta);
    const e2 = (1 - u0) / (1 - v * v);
    const l2 = (e2 - 1 + u0) / (u0 * u0 * (1 - u0 + cotDelta * cotDelta));
    const e = safeSqrt(e2);
    const l = delta == 0 ? 0 : (delta > 0 ? 1 : -1) * safeSqrt(l2);
    this.e = e;
    this.l = l;

    // Update the Schwarzschild coordinates of the camera.
    if (this.state == State.STOPPED) {
      this.r = r0;
      this.drOverDtau = -safeSqrt(e2 - (1 - u0) - l * l * u0 * u0 * (1 - u0));
      this.phi = 0;
    }
    const ci = Math.cos(this.orbitInclination.getValue());
    const si = Math.sin(this.orbitInclination.getValue());
    const cphi = Math.cos(this.phi);
    const sphi = Math.sin(this.phi);
    this.worldTheta = Math.acos(cphi * si);
    this.worldPhi = Math.atan2(sphi, cphi * ci);
  }

  updateCameraLorentzTransform() {
    // Compute the 4-velocity vector of the camera (in rotated Schwarzschild
    // coordinates such that its orbit is in the equatorial plane).
    const e = this.e;
    const l = this.l;
    const u = 1 / this.r;
    let k;
    if (this.state == State.PLAYING) {
      k = [e / (1 - u), this.drOverDtau, 0, l * u * u];
    } else {
      k = [1 / Math.sqrt(1 - u), 0, 0, 0];
    }

    // Compute the rotation matrix from the global Schwarzschild reference frame
    // e_t, e_r, e_theta, e_phi to the rotated Schwarzschild reference frame
    // such that the camera orbit is in the equatorial plane.
    const ct = Math.cos(this.worldTheta);
    const st = Math.sin(this.worldTheta);
    const cp = Math.cos(this.worldPhi);
    const sp = Math.sin(this.worldPhi);
    const ci = Math.cos(this.orbitInclination.getValue());
    const si = Math.sin(this.orbitInclination.getValue());
    const ca = si * ct * cp + st * ci;
    const sa = si * sp;
    const orbitRot = [
      [1, 0,  0,   0],
      [0, 1,  0,   0],
      [0, 0, ca, -sa],
      [0, 0, sa,  ca]];

    // Compute the 4-velocity of the camera in the *rotated* reference frame of 
    // a STATIC observer at this location.
    const k_s = 
        [k[0] * Math.sqrt(1 - u), k[1] / Math.sqrt(1 - u), k[2] / u, k[3] / u];
    // Compute the speed vector of the camera in the *rotated* reference frame
    // of a STATIC observer at this location.
    const v = [k_s[1] / k_s[0], k_s[2] / k_s[0], k_s[3] / k_s[0]];
    // Compute the corresponding Lorentz factor gamma and the corresponding
    // Lorentz boost matrix (for a rotated static observer).
    const v2 = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
    const gamma = 1 / Math.sqrt(1 - v2);
    const gv = v2 == 0 ? 0 : (gamma - 1) / v2;
    const boost = [
        [     gamma,       gamma*v[0],       gamma*v[1],       gamma*v[2]],
        [gamma*v[0], 1 + gv*v[0]*v[0],     gv*v[0]*v[1],     gv*v[0]*v[2]],
        [gamma*v[1],     gv*v[1]*v[0], 1 + gv*v[1]*v[1],     gv*v[1]*v[2]],
        [gamma*v[2],     gv*v[2]*v[0],     gv*v[2]*v[1], 1 + gv*v[2]*v[2]]];

    // Compute the rotation matrix of the camera, in its local reference frame.
    const cosY = Math.cos(this.cameraYaw.getValue());
    const sinY = Math.sin(this.cameraYaw.getValue());
    const cosP = Math.cos(this.cameraPitch.getValue());
    const sinP = Math.sin(this.cameraPitch.getValue());
    const cameraRot = [
        [1,            0,     0,            0],
        [0,        -sinY,     0,         cosY],
        [0, -cosY * sinP, -cosP, -sinY * sinP],
        [0,  cosY * cosP, -sinP,  sinY * cosP]];

    // The final Lorentz transform is the product of the 3 above matrices.
    this.lorentz = matrixProduct(cameraRot, matrixProduct(boost, orbitRot));
  }

  updateCameraReferenceFrame() {
    const r = this.r;
    const cos_theta = Math.cos(this.worldTheta);
    const sin_theta = Math.sin(this.worldTheta);
    const cos_phi = Math.cos(this.worldPhi);
    const sin_phi = Math.sin(this.worldPhi);

    const u = 1 / r;
    const v = Math.sqrt(1 - u);
    const ur = [sin_theta * cos_phi, sin_theta * sin_phi, cos_theta];

    const e_t = [1 / v, 0, 0, 0];
    const e_r = [0, v * ur[0], v * ur[1], v * ur[2]];
    const e_theta = [0, cos_theta * cos_phi, cos_theta * sin_phi, -sin_theta];
    const e_phi = [0, -sin_phi, cos_phi, 0];

    const L = this.lorentz;
    const e_static = [e_t, e_r, e_theta, e_phi];
    this.eTau = vectorMatrixProduct(L[0], e_static);
    this.eW = vectorMatrixProduct(L[1], e_static);
    this.eH = vectorMatrixProduct(L[2], e_static);
    this.eD = vectorMatrixProduct(L[3], e_static);

    this.p = [r * ur[0], r * ur[1], r * ur[2]];
    this.kS = [L[0][0] / v, v * L[0][1], u * L[0][2], u / sin_theta * L[0][3]];
  }

  updateOrbitInfo() {
    const M = this.blackHoleMass.getValue() * SOLAR_MASS;
    this.blackHoleRadiusMeters = 2 * G  * M / (C * C);
    const e = this.e;
    const u = 1 / this.r;
    if (this.state == State.PLAYING) {
      this.speedMetersPerSecond = Math.sqrt(1 - (1 - u) / (e * e)) * C;
      this.gForce = 0;
      this.timeDilationFactor = e / (1 - u);
    } else {
      const rMeters = this.r * this.blackHoleRadiusMeters;
      this.speedMetersPerSecond = 0;
      this.gForce = G * M / (rMeters * rMeters * Math.sqrt(1 - u));
      this.timeDilationFactor = 1 / Math.sqrt(1 - u);
    }
  }
}

BlackHoleShaderDemoApp.State = State;
BlackHoleShaderDemoApp.model = new Model();
})();
