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

(function(model, State) {

class Checkbox {
  constructor(rootElement, name, model) {
    this.checkbox = rootElement.querySelector(`#sp_${name}`);
    this.checkbox.addEventListener('input', () => {
      model.setValue(this.checkbox.checked);
    });
    this.model = model;
  }
  update() {
    this.checkbox.checked = this.model.getValue();
  }
}

class Slider {
  constructor(rootElement, name, model) {
    this.slider = rootElement.querySelector(`#sp_${name}`);
    this.value = rootElement.querySelector(`#sp_${name}_value`);
    this.model = model;

    this.slider.min = 0;
    this.slider.max = model.getSize();
    this.slider.addEventListener('input', () => {
      model.setIndex(parseInt(this.slider.value));
    });
    this.slider.previousElementSibling.addEventListener('click', () => {
      model.setIndex(model.getIndex() - 1);
    });
    this.slider.nextElementSibling.addEventListener('click', () => {
      model.setIndex(model.getIndex() + 1);
    });
  }
  getValue() { return this.model.getValue(); }
  setValue(value) { this.model.setValue(value); }
  enable(enabled) {
    this.slider.disabled = !enabled;
    this.slider.previousElementSibling.disabled = !enabled;
    this.slider.nextElementSibling.disabled = !enabled;
  }  
  update(format) {
    this.slider.value = this.model.getIndex();
    this.value.innerText = format(this.model.getValue());
  }
}

const quaternionFromAxisAngle = function(x, y, z, theta) {
  const ct = Math.cos(theta * 0.5);
  const st = Math.sin(theta * 0.5);
  return [ct, st * x, st * y, st * z];
};

const quaternionFromEulerAngles = function(yaw, pitch, roll) {
  const cy = Math.cos(yaw * 0.5);
  const sy = Math.sin(yaw * 0.5);
  const cp = Math.cos(pitch * 0.5);
  const sp = Math.sin(pitch * 0.5);
  const cr = Math.cos(roll * 0.5);
  const sr = Math.sin(roll * 0.5);
  return [
    cy * cp * cr + sy * sp * sr,
    cy * cp * sr - sy * sp * cr,
    sy * cp * sr + cy * sp * cr,
    sy * cp * cr - cy * sp * sr];
};

const quaternionProduct = function(q1, q2) {
  return [
    q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3],
    q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2],
    q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1],
    q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0]
  ];
};

const quaternionToEulerAngles = function(qw, qx, qy, qz) {
  const sinr_cosp = 2 * (qw * qx + qy * qz);
  const cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
  const roll = Math.atan2(sinr_cosp, cosr_cosp);
  const sinp = 2 * (qw * qy - qz * qx);
  let pitch;
  if (Math.abs(sinp) >= 1) {
    pitch = sinp > 0 ? Math.PI / 2 : -Math.PI / 2;
  } else {
    pitch = Math.asin(sinp);
  }
  const siny_cosp = 2 * (qw * qz + qx * qy);
  const cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
  const yaw = Math.atan2(siny_cosp, cosy_cosp);
  return [yaw, pitch, roll];
}

class SettingsPanel {
  constructor(rootElement, model) {
    this.rootElement = rootElement;
    this.model = model;
    this.model.addListener(this);

    this.exposure =
        new Slider(rootElement, 'exposure', model.exposure);
    this.bloom = 
        new Slider(rootElement, 'bloom', model.bloom);
    this.startRadius = 
        new Slider(rootElement, 'start_radius', model.startRadius);
    this.startDirection = 
        new Slider(rootElement, 'start_direction', model.startDirection);
    this.startSpeed = 
        new Slider(rootElement, 'start_speed', model.startSpeed);
    this.orbitInclination = 
        new Slider(rootElement, 'orbit_inclination', model.orbitInclination);
    this.lensing =
        new Checkbox(rootElement, 'lensing', model.lensing);
    this.doppler = 
        new Checkbox(rootElement, 'doppler', model.doppler);
    /* TODO: add the 'rocket' setting. */
    this.grid = 
        new Checkbox(rootElement, 'grid', model.grid);
    this.blackHoleMass = 
        new Slider(rootElement, 'black_hole_mass', model.blackHoleMass);
    this.discDensity =
        new Slider(rootElement, 'disc_density', model.discDensity);
    this.discOpacity =
        new Slider(rootElement, 'disc_opacity', model.discOpacity);
    this.discTemperature =
        new Slider(rootElement, 'disc_temperature', model.discTemperature);
    this.starsYaw =
        new Slider(rootElement, 'stars_yaw', model.starsYaw);
    this.starsPitch =
        new Slider(rootElement, 'stars_pitch', model.starsPitch);
    this.starsRoll =
        new Slider(rootElement, 'stars_roll', model.starsRoll);
    this.stars = 
        new Checkbox(rootElement, 'stars', model.stars);
    this.previousMouseX = undefined;
    this.previousMouseY = undefined;
    this.drag = false;

    this.onSettingsChange();
    this.onOrbitChange();
    this.toggleVisibility();

    this.rootElement.querySelector('#sp_play').addEventListener(
        'click', (e) => this.model.setState(State.PLAYING));
    this.rootElement.querySelector('#sp_pause').addEventListener(
        'click', (e) => this.model.setState(State.PAUSED));
    this.rootElement.querySelector('#sp_stop').addEventListener(
        'click', (e) => this.model.setState(State.STOPPED));
    document.body.addEventListener('keypress', (e) => this.onKeyPress(e));
    document.body.addEventListener('wheel', (e) => this.onMouseWheel(e));
    window.addEventListener('mousedown', (e) => this.onMouseDown(e));
    window.addEventListener('mousemove', (e) => this.onMouseMove(e));
    window.addEventListener('mouseup', (e) => this.onMouseUp(e));
  }

  onSettingsChange() {
    this.exposure.update((v) => `${(Math.log2(v * 1000)).toPrecision(3)}`);
    this.bloom.update((v) => `${(v * 100).toFixed(0)}%`);
    this.startRadius.update((v) => `${v.toPrecision(3)}`);
    this.startDirection.update((v) => `${(v * 180 / Math.PI).toFixed(1)}°`);
    this.startSpeed.update((v) => `${v.toPrecision(3)}`);
    this.orbitInclination.update((v) => `${(v * 180 / Math.PI).toFixed(1)}°`);
    this.lensing.update();
    this.doppler.update();
    this.grid.update();
    this.blackHoleMass.update((v) => `${v.toExponential(2)}`);
    this.discDensity.update((v) => `${v.toExponential(2)}`);
    this.discOpacity.update((v) => `${(v * 100).toFixed(1)}%`);
    this.discTemperature.update((v) => `${v.toFixed(0)}K`);
    this.starsYaw.update((v) => `${(v * 180 / Math.PI).toFixed(1)}°`);
    this.starsPitch.update((v) => `${(v * 180 / Math.PI).toFixed(1)}°`);
    this.starsRoll.update((v) => `${(v * 180 / Math.PI).toFixed(1)}°`);
    this.stars.update();
  }

  onOrbitChange() {
    const playing = this.model.state == State.PLAYING;
    this.rootElement.classList.toggle('sp-playing', playing);

    const stopped = this.model.state == State.STOPPED;
    this.startRadius.enable(stopped);
    this.startDirection.enable(stopped);
    this.startSpeed.enable(stopped);
  }

  onKeyPress(event) {
    const key = event.key;
    if (key == '+') {
      this.exposure.setValue(this.exposure.getValue() * 1.1);
    } else if (key == '-') {
      this.exposure.setValue(this.exposure.getValue() / 1.1);
    } else if (key == 'p') {
      this.model.setState(
          this.model.state == State.PLAYING ? State.PAUSED : State.PLAYING); 
    } else if (key == ' ') {
      this.toggleVisibility();
    }
  }

  onMouseWheel(event) {
    if (this.model.state == State.STOPPED) {
      if (event.deltaY > 0) {
        this.startRadius.setValue(this.startRadius.getValue() * 1.05);
      } else {
        this.startRadius.setValue(this.startRadius.getValue() / 1.05);
      }
    }
  }

  onMouseDown(event) {
    this.previousMouseX = event.screenX;
    this.previousMouseY = event.screenY;
    this.drag = (event.target.tagName != 'INPUT') && event.ctrlKey;
  }

  onMouseMove(event) {
    const kScale = 500;
    const mouseX = event.screenX;
    const mouseY = event.screenY;
    if (this.drag) {
      const x = (this.previousMouseX - mouseX) / kScale;
      const y = (this.previousMouseY - mouseY) / kScale;
      const qx = quaternionFromAxisAngle(0, 0, 1, -x);
      const qy = quaternionFromAxisAngle(0, 1, 0, y);
      let q = quaternionFromEulerAngles(this.model.starsYaw.getValue(), 
          this.model.starsPitch.getValue(), this.model.starsRoll.getValue());    
      q = quaternionProduct(q, qx);
      q = quaternionProduct(q, qy);
      const euler = quaternionToEulerAngles(q[0], q[1], q[2], q[3]);
      this.model.starsYaw.setValue(euler[0]);
      this.model.starsPitch.setValue(euler[1]);
      this.model.starsRoll.setValue(euler[2]);
    }
    this.previousMouseX = mouseX;
    this.previousMouseY = mouseY;
  }

  onMouseUp(event) {
    this.drag = false;
  }

  toggleVisibility() {
    this.rootElement.classList.toggle('sp-hidden');
  }
}

window.addEventListener('DOMContentLoaded', () => {
  new SettingsPanel(document.body.querySelector('#settings_panel'), model);
});
})(BlackHoleShaderDemoApp.model, BlackHoleShaderDemoApp.State);
