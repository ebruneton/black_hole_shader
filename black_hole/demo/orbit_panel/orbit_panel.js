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

(function(model) {

class Context3d {
  constructor(canvas) {
    this.canvas = canvas;
    this.context = canvas.getContext('2d');

    const r = 120;
    const theta = 70 * Math.PI / 180;
    const phi = -135 * Math.PI / 180;
    const fovY = 30 * Math.PI / 180;

    this.ex = [-Math.sin(phi), Math.cos(phi), 0];
    this.ey = [-Math.cos(theta) * Math.cos(phi),
               -Math.cos(theta) * Math.sin(phi),
                Math.sin(theta)];
    this.ez = [Math.sin(theta) * Math.cos(phi),
               Math.sin(theta) * Math.sin(phi),
               Math.cos(theta)];
    this.camera = [r * this.ez[0], r * this.ez[1], r * this.ez[2]];

    this.width = canvas.width;
    this.height = canvas.height;
    this.focal = this.height / (2 * Math.tan(fovY / 2));
    this.nearPlane = -0.1;
    this.lastCameraPt = [];
  }

  moveTo(x, y, z) {
    this.pathTo(x, y, z, true);
  }

  lineTo(x, y, z) {
    this.pathTo(x, y, z, false);
  }

  pathTo(x, y, z, move) {
    const cameraPt = this.toCameraPt([x, y, z]);
    const screenPt = this.toScreenPt(cameraPt);
    if (move) {
      this.context.moveTo(screenPt[0], screenPt[1]);
    } else {
      if (cameraPt[2] < this.nearPlane) {
        if (this.lastCameraPt[2] > this.nearPlane) {
          this.clipTo(cameraPt, true);
        }
        this.context.lineTo(screenPt[0], screenPt[1]);
      } else if (this.lastCameraPt[2] < this.nearPlane) {
        this.clipTo(cameraPt, false);
      }
    }
    this.lastCameraPt = cameraPt;
  }

  clipTo(cameraPt, move) {
    const t = (this.nearPlane - this.lastCameraPt[2]) / 
        (cameraPt[2] - this.lastCameraPt[2]);
    const nearPlanePt = [
        this.lastCameraPt[0] + t * (cameraPt[0] - this.lastCameraPt[0]),
        this.lastCameraPt[1] + t * (cameraPt[1] - this.lastCameraPt[1]),
        this.nearPlane];
    const screenPt = this.toScreenPt(nearPlanePt);
    if (move) {
      this.context.moveTo(screenPt[0], screenPt[1]);
    } else {
      this.context.lineTo(screenPt[0], screenPt[1]);
    }
  }

  toCameraPt(worldPt) {
    const q = [worldPt[0] - this.camera[0],
               worldPt[1] - this.camera[1],
               worldPt[2] - this.camera[2]];
    return [q[0] * this.ex[0] + q[1] * this.ex[1] + q[2] * this.ex[2],
            q[0] * this.ey[0] + q[1] * this.ey[1] + q[2] * this.ey[2],
            q[0] * this.ez[0] + q[1] * this.ez[1] + q[2] * this.ez[2]];
  }

  toScreenPt(cameraPt) {
    return [this.width * 0.5 - 0.5 * this.focal * cameraPt[0] / cameraPt[2],
            this.height * 0.5 + 0.5 * this.focal * cameraPt[1] / cameraPt[2]];
  }
}

const safeSqrt = function(x) {
  return Math.sqrt(Math.max(x, 0));
};

class OrbitPanel {
  constructor(rootElement, model) {
    this.rootElement = rootElement;   
    this.model = model;
    this.model.addListener(this);

    this.blackHoleRadius = rootElement.querySelector('#op_black_hole_radius');
    this.radius = rootElement.querySelector('#op_radius');
    this.speed = rootElement.querySelector('#op_speed');
    this.localTime = rootElement.querySelector('#op_local_time');
    this.globalTime = rootElement.querySelector('#op_global_time');
    this.timeDilation = rootElement.querySelector('#op_time_dilation');
    this.dot = rootElement.querySelector('#op_dot');
    this.numberFormat = 
        new Intl.NumberFormat('en-US', {maximumFractionDigits : 1});

    this.lastStartRadius = undefined;
    this.lastStartDirection = undefined;
    this.lastStartSpeed = undefined;
    this.lastOrbitInclination = undefined;

    this.canvas = rootElement.querySelector('#canvas');
    this.context = this.canvas.getContext('2d');
    this.context3d = new Context3d(this.canvas);

    this.onSettingsChange();
    this.onOrbitChange();
    this.toggleVisibility();

    document.body.addEventListener('keypress', (e) => this.onKeyPress(e));
  }

  onSettingsChange() {
    if (this.lastStartRadius == this.model.startRadius.getValue() &&
        this.lastStartDirection == this.model.startDirection.getValue() &&
        this.lastStartSpeed == this.model.startSpeed.getValue() &&
        this.lastOrbitInclination == this.model.orbitInclination.getValue()) {
      return;
    }
    this.lastStartRadius = this.model.startRadius.getValue();
    this.lastStartDirection = this.model.startDirection.getValue();
    this.lastStartSpeed = this.model.startSpeed.getValue();
    this.lastOrbitInclination = this.model.orbitInclination.getValue();

    this.context.clearRect(0, 0, this.canvas.width, this.canvas.height);
    this.context.strokeStyle = '#AAA';
    this.context.beginPath();
    this.drawGrid(100, 12, this.model.orbitInclination.getValue());
    this.context.stroke();    
    this.drawAxes();
    this.drawDisc();
    this.drawOrbit();
  }

  onOrbitChange() {
    const model = this.model;
    const radiusMeters = model.r * model.blackHoleRadiusMeters;
    this.blackHoleRadius.innerText =
        `${this.numberFormat.format(model.blackHoleRadiusMeters / 1000)}km`;
    this.radius.innerText = 
        `${this.numberFormat.format(radiusMeters / 1000)}km`;
    this.speed.innerText = 
        `${this.numberFormat.format(model.speedMetersPerSecond / 1000)}km/s`;
    this.localTime.innerText = 
        `${model.localElapsedTimeSeconds.toFixed(2)}s`;
    this.globalTime.innerText = 
        `${model.globalElapsedTimeSeconds.toFixed(2)}s`;
    this.timeDilation.innerText = 
        `${model.timeDilationFactor.toFixed(3)}`;

    const ci = Math.cos(this.model.orbitInclination.getValue());
    const si = Math.sin(this.model.orbitInclination.getValue());
    const worldPt = [ci * model.r * Math.cos(model.phi),
                          model.r * Math.sin(model.phi),
                     si * model.r * Math.cos(model.phi)];
    const screenPt = 
        this.context3d.toScreenPt(this.context3d.toCameraPt(worldPt));
    this.dot.style.left = `${screenPt[0]}px`;
    this.dot.style.top = `${screenPt[1]}px`;
  }

  drawGrid(halfSize, steps, orbitInclination) {
    const ci = Math.cos(orbitInclination);
    const si = Math.sin(orbitInclination);
    for (let i = 0; i <= steps; ++i) {
      const step = -halfSize + 2 * (i / steps) * halfSize;
      this.context3d.moveTo(-ci * halfSize, step, -si * halfSize);
      this.context3d.lineTo(ci * halfSize, step, si * halfSize);
      this.context3d.moveTo(ci * step, -halfSize, si * step);
      this.context3d.lineTo(ci * step, halfSize, si * step);
    }
  }

  drawAxes() {
    const context = this.context;
    const context3d = this.context3d;
    context.strokeStyle = '#F00';
    context.beginPath();
    context3d.moveTo(0, 0, 0);
    context3d.lineTo(40, 0, 0);
    context.stroke(); 
    context.strokeStyle = '#0F0';
    context.beginPath();
    context3d.moveTo(0, 0, 0);
    context3d.lineTo(0, 40, 0);
    context.stroke(); 
    context.strokeStyle = '#00F';
    context.beginPath();
    context3d.moveTo(0, 0, 0);
    context3d.lineTo(0, 0, 40);
    context.stroke(); 
  }

  drawDisc() {
    const context = this.context;
    context.strokeStyle = '#FF0';
    context.lineWidth = 2;
    context.beginPath();
    this.drawCircle(3);
    this.drawCircle(12);
    context.stroke();
    context.lineWidth = 1;
  }

  drawCircle(radius) {
    this.context3d.moveTo(radius, 0, 0);
    for (let i = 1; i <= 64; ++i) {
      const a = 2 * Math.PI * i / 64;
      this.context3d.lineTo(radius * Math.cos(a), radius * Math.sin(a), 0);
    }
  }

  drawOrbit() {
    const context = this.context;
    context.lineWidth = 2;
    context.strokeStyle = '#FFF';
    context.shadowOffsetX = 1;
    context.shadowOffsetY = 1;
    context.shadowBlur = 2;
    context.shadowColor = '#000';
    context.beginPath();
    const ci = Math.cos(this.model.orbitInclination.getValue());
    const si = Math.sin(this.model.orbitInclination.getValue());
    const e = this.model.e;
    const l = this.model.l;
    let r = this.model.startRadius.getValue();
    let u = 1 / r;
    let drOverDtau = -safeSqrt(e * e - (1 - u) - l * l * u * u * (1 - u));
    let phi = 0;

    let i = 0;
    let dashes = false;
    this.context3d.moveTo(ci * r, 0, si * r);
    while (Math.abs(phi) < 6 * Math.PI) {
      u = 1 / r;
      // Adaptive integration step size, chosen such that the proper distance
      // ds^2 = dr^2 / (1 - u) + r^2 dPhi^2 = dTau^2 (e^2 / (1 - u) - 1) is
      // constant at each step.
      const dTau = 1e-2 / Math.sqrt(e * e / (1 - u) - 0.99);
      const d2rOverDtau2 = u * u * (l * l * u * (2 - 3 * u) - 1) / 2;
      drOverDtau += d2rOverDtau2 * dTau;
      r += drOverDtau * dTau;
      phi += l * u * u * dTau;
      if (r > 100.0 || r <= 1.0) {
        break;
      }
      if ((++i) % 100 == 0) {
        const x = r * Math.cos(phi);
        const y = r * Math.sin(phi);
        this.context3d.lineTo(ci * x, y, si * x);
        if (Math.abs(phi) > 5 * Math.PI && !dashes) {
          context.stroke(); 
          context.beginPath();
          this.context3d.moveTo(ci * x, y, si * x);
          context.setLineDash([5, 5]);
          dashes = true;
        }
      }
    }
    context.stroke(); 
    context.lineWidth = 1;
    context.setLineDash([]);
    context.shadowOffsetX = 0;
    context.shadowOffsetY = 0;
    context.shadowBlur = 0;
  }

  onKeyPress(event) {
    if (event.key == ' ') {
      this.toggleVisibility();
    }
  }

  toggleVisibility() {
    this.rootElement.classList.toggle('op-hidden');
  }
}

window.addEventListener('DOMContentLoaded', () => {
  new OrbitPanel(document.body.querySelector('#orbit_panel'), model);
});
})(BlackHoleShaderDemoApp.model);
