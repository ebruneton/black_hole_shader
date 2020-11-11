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

/*<h2>black_hole/functions.glsl</h2>

<p>This GLSL file contains the core functions that implement our
<a href="https://arxiv.org/abs/2010.08735">black hole model</a>. It provides
functions to compute the deflection and the accretion disc intersections of a
light ray, in constant time, by using precomputed textures.

<p><i><b>Note</b>: the notations used below are defined in the description of
our <a href="https://arxiv.org/abs/2010.08735">black hole model</a></i>. The
code uses the same notations (e.g. <code>e_square</code> refer to $e^2$,
<code>kMu</code> to $\mu$, etc).

<a name="ray_deflection"><h3>Ray deflection</h3>

<p>As described in <a href="https://arxiv.org/abs/2010.08735">Section 3.3.2</a>,
our model uses a precomputed table $\mathbb{D}(e,u)$ for the ray deflection
$\Delta$. In order to store this table in a texture, we need a
<a href="https://en.wikipedia.org/wiki/Map_(mathematics)">mapping</a> from the
$(e,u)$ parameters to texture coordinates in $[0,1]\times[0,1]$. And for this we
need to solve several issues.

<p>The first issue is that the deflection at the apsis $\mathbb{D}(e,u_a(e))$
diverges for $e^2 \rightarrow \mu^-$, and likewise for the deflection
$\mathbb{D}(e,1)$ when the ray is captured by the black hole, when
$e^2 \rightarrow \mu^+$ (see <a href="#fig1">Fig. 1</a>). This is because, as we
approach the limit where the ray either falls or doesn't fall into the black
hole, it makes more and more turns around it before being captured or not.

<a name="fig1">
<svg width="635" height="260" xmlns="http://www.w3.org/2000/svg">
  <style>
    path { fill:none; }
    text { font-size:16px; font-family:Sans }
  </style>
  <path d="M17 225V35m0 190h276" fill="none" stroke="#000"/>
  <path d="M53 225V32" fill="none" stroke="#888" stroke-dasharray="5,2"/>
  <text x="20" y="32">Δ</text>
  <text x="47" y="245">√μ</text>
  <text x="297" y="245">e</text>
  <path d="M52 32c-1 93 1 169-35 193M55 32c3 76 5 139 39 167 31 24 91 24 200 24"
        stroke="#f00"/>
  <path d="M335 225V35m0 190h276" stroke="#000"/>
  <text x="338" y="32">Δ</text>
  <text x="331" y="245">0</text>
  <text x="470" y="245">0.5</text>
  <text x="614" y="245">1</text>
  <path d="M334 138c44 57 102 86 143 86 24 0 84-59 133-181" stroke="#f00"/>
</svg>
<div class="caption"><b>Fig. 1</b>. <i>Left</i>: plot of $\mathbb{D}(e,u_a(e))$
and $\mathbb{D}(e,1)$, showing the discontinuity and the divergence at
$e^2=\mu$. <i>Right</i>: same plot, but as a function of the texture coordinate
given by <code>GetRayDeflectionTextureUFromEsquare</code>, showing that the
discontinuity and the large slopes are removed.</div>

<p><a name="deflection_mapping_u">This divergence is an issue because it yields
large slope values near the discontinuity. Thus, if $e$ was mapped to a $[0,1]$
texture coordinate with a simple scaling factor, we would get a very small
number of texels near the discontinuity, and thus an important loss of
precision. Also the discontinuity itself would lead to invalid texture
interpolations. To solve this, we use a more complex texture coordinate mapping
for $e$, defined in <a href="https://arxiv.org/abs/2010.08735">Appendix A</a> of
our model and implemented by the function below. As shown in
<a href="#fig1">Fig. 1</a>, this ad-hoc mapping removes the discontinuity and
the large slopes, thus solving the sampling density and precision issues
mentioned above.
*/

const Real kMu = 4.0 / 27.0;

Real GetRayDeflectionTextureUFromEsquare(const Real e_square) {
  if (e_square < kMu) {
    return 0.5 - sqrt(-log(1.0 - e_square / kMu) * (1.0 / 50.0));
  } else {
    return 0.5 + sqrt(-log(1.0 - kMu / e_square) * (1.0 / 50.0));
  }
}

/*
<p>A second issue to map the $(e,u)$ parameters to texture coordinates is that,
for rays which don't fall into the black hole (i.e. for $e^2<\mu$), the
$\mathbb{D}(e,u)$ table is defined only for values of $u$ between $0$ and
$u_a(e) \le 1$ (the value of $u$ at the apsis). We could store dummy values in
our texture for $u_a(e) \lt u \le 1$, but this would waste texture samples and
decrease the sampling density (and thus the precision of the end results) for
$e \rightarrow 0$, where $u_a(e)\rightarrow 0$. Instead, we use a mapping such
that $u=u_a(e)$ is mapped to the texture coordinate $1$.

<p>However, a simple mapping from $u$ to $u/u_a(e)$ is not sufficient. Indeed,
near the apsis $u$ does not vary much while the deflection $\Delta$ changes
rapidly. This yields large slopes for $\mathbb{D}(e,u)$ when
$u \rightarrow u_a(e)$ (see <a href="#fig2">Fig. 2</a>). Likewise, rays which
turn around the black hole many times before being captured stay near the
<a href="https://en.wikipedia.org/wiki/Photon_sphere">photon sphere</a> $r=3/2$
while the deflection changes rapidly. This yields large slopes when
$u \rightarrow 2/3$, as shown in <a href="#fig2">Fig. 2</a>.

<a name="fig2">
<svg width="635" height="260" xmlns="http://www.w3.org/2000/svg">
  <style>
    path { fill:none }
    text { font-size:16px; font-family:Sans }
  </style>
  <path d="M17 225V35m0 190h276" stroke="#000"/>
  <text x="20" y="32">Δ</text>
  <path d="M335 225V35m0 190h276" stroke="#000"/>
  <text x="338" y="32">Δ</text>
  <text x="331" y="245">0</text>
  <text x="470" y="245">0.5</text>
  <text x="614" y="245">1</text>
  <text x="13" y="245">0</text>
  <text x="187" y="245">2/3</text>
  <text x="293" y="245">1</text>
  <path d="M17 225c89 0 122-1 150-14 30-13 30-36 33-70 3-37-5-76 92-96"
        stroke="#f00"/>
  <path d="M17 225c55-1 93 0 123-6s48-17 49-39" stroke="#00f"/>
  <path d="M200 225V35" stroke="#888" stroke-dasharray="5,2"/>
  <path d="M335 225c145 0 186-12 275-47" stroke="#00f"/>
  <path d="M334 225c73 1 93-10 121-31 27-21 29-58 42-58 9 0 14-27 31-48
           14-17 39-36 81-52" stroke="#f00"/>
</svg>
<div class="caption"><b>Fig. 2</b>. <i>Left</i>: plot of the deflection $\Delta$
as a function of $u$, for 2 values of $e$. <i>Right</i>: same plot, as a
function of the texture coordinate given by
<code>GetRayDeflectionTextureVFromEsquareAndU</code>.</div>

<p><a name="deflection_mapping_v">To avoid these large slopes (which cause
precision issues as explained above), while remapping $u_a(e)$ to $1$, we use
for $u$ the texture coordinate mapping defined in
<a href="https://arxiv.org/abs/2010.08735">Appendix A</a> of our model and
implemented by the function below (where <code>GetUapsisFromEsquare</code>
computes $u_a$ with <a href="https://arxiv.org/abs/2010.08735">Eq. 9</a>). As
shown in <a href="#fig2">Fig. 2</a>, this ad-hoc mapping removes the large
slopes and unused parameter intervals, thus solving the sampling density and
precision issues mentioned above.
*/

Real GetUapsisFromEsquare(const Real e_square) {
  Real x = (2.0 / kMu) * e_square - 1.0;
  return 1.0 / 3.0 + (2.0 / 3.0) * sin(asin(x) * (1.0 / 3.0));
}

Real GetRayDeflectionTextureVFromEsquareAndU(const Real e_square,
                                             const Real u) {
  if (e_square > kMu) {
    Real x = u < 2.0 / 3.0 ? -sqrt(2.0 / 3.0 - u) : sqrt(u - 2.0 / 3.0);
    return (sqrt(2.0 / 3.0) + x) / (sqrt(2.0 / 3.0) + sqrt(1.0 / 3.0));
  } else {
    return 1.0 - sqrt(max(1.0 - u / GetUapsisFromEsquare(e_square), 0.0));
  }
}

/*
<p>With this we can finally implement a function to do a lookup in a precomputed
deflection texture, for some given $(e,u)$ parameters (for convenience we also
do a lookup for the deflection at the apsis):
*/

Real GetTextureCoordFromUnitRange(const Real x, const int texture_size) {
  return 0.5 / Real(texture_size) + x * (1.0 - 1.0 / Real(texture_size));
}

TimedAngle LookupRayDeflection(IN(RayDeflectionTexture) ray_deflection_texture,
                               const Real e_square, const Real u,
                               OUT(TimedAngle) deflection_apsis) {
  Real tex_u = GetTextureCoordFromUnitRange(
      GetRayDeflectionTextureUFromEsquare(e_square),
      RAY_DEFLECTION_TEXTURE_WIDTH);
  Real tex_v = GetTextureCoordFromUnitRange(
      GetRayDeflectionTextureVFromEsquareAndU(e_square, u),
      RAY_DEFLECTION_TEXTURE_HEIGHT);
  Real tex_v_apsis =
      GetTextureCoordFromUnitRange(1.0, RAY_DEFLECTION_TEXTURE_HEIGHT);
  deflection_apsis =
      TimedAngle(texture(ray_deflection_texture, vec2(tex_u, tex_v_apsis)));
  return TimedAngle(texture(ray_deflection_texture, vec2(tex_u, tex_v)));
}

/*
<p>where <code>GetTextureCoordFromUnitRange</code> is used to map $[0,1]$ to the
$[0.5/n,1-0.5/n]$ interval for a texture of size $n$, in texels, because texture
values are stored at texel centers.

<a name="ray_inverse_radius"><h3>Ray inverse radius</h3>

<p>As described in <a href="https://arxiv.org/abs/2010.08735">Section 3.3.3</a>, 
our model uses a precomputed table $\mathbb{U}(e,\varphi)$ for the ray inverse
radius $u$. In order to store this table in a texture, we need a mapping from
the $(e,\varphi)$ parameters, in $\mathbb{R}^+ \times [0,\pi]$, to texture
coordinates in $[0,1] \times [0,1]$. And for this we need to solve several
issues.

<p><a name="inverse_radius_mapping">The first issue is that, for a small impact
parameter $b=1/e \rightarrow 0$, light rays fall directly into the black hole
with $y'=r\sin\varphi \approx b$, yielding $u \approx e\sin\varphi$, i.e. a 
large slope $e \rightarrow \infty$ for $\mathbb{U}(\cdot,\varphi)$. To solve
this, we use the fact that we don't need $\mathbb{U}(e,\varphi)$ for $\varphi
\gt \varphi_{1/3}$, the value of $\varphi$ such that $u=1/3$. And the above relations show that $\varphi_{1/3}(e) \rightarrow 1/3e$ for $e \rightarrow 
\infty$. We can thus use a texture coordinate mapping for $\varphi$ such that 
$\varphi_{1/3}(e)$ is mapped to $1$ to solve the large slope issues.
Unfortunately, $\varphi_{1/3}(e)$ does not have an analytical expression. So we
use instead an upper bound $\varphi_{ub}(e) \ge \varphi_{1/3}(e)$, as simple and
as close as possible to $\varphi_{1/3}(e)$, defined in
<a href="https://arxiv.org/abs/2010.08735">Appendix A</a> of our model and
implemented by the following function:
*/

Angle GetPhiUbFromEsquare(const Real e_square) {
  return (1.0 + e_square) / (1.0 / 3.0 + 2.0 * e_square * sqrt(e_square)) * rad;
}

/*
<p>A plot of this function is shown in <a href="#fig3">Fig. 3</a>. Note that it
is actually an upper bound of $\min(\varphi_a(e), \varphi_{1/3}(e))$ because,
for light rays which do not fall into the blackhole, $\mathbb{U}(e,\varphi)$ for
$\varphi$ values larger than the apsis angle $\varphi_a$ can be deduced by
symmetry.

<a name="fig3">
<svg width="635" height="260" xmlns="http://www.w3.org/2000/svg">
  <style>
    path { fill:none }
    text { font-size:16px; font-family:Sans }
  </style>
  <path d="M17 225V35m0 190h594" stroke="#000"/>
  <path d="M93 225V35" stroke="#888" stroke-dasharray="5,2"/>
  <text x="20" y="32">φ</text>
  <text x="85" y="245">√μ</text>
  <text x="607" y="245">e</text>
  <path d="M71 117c0 102 106 96 540 102" stroke="#080"/>
  <path d="M87 88c-1 119 91 120 524 129" stroke="#0f0"/>
  <path d="M17 143c76-19 72-58 75-108" stroke="#00f"/>
  <path d="M17 68h28c54 1 67 70 160 109 88 36 298 37 406 39" stroke="#f00"/>
</svg>
<div class="caption"><b>Fig. 3</b>. Plot of $\varphi_{ub}(e)$, in red, showing
that it is an upper bound of the minimum of $\varphi_a(e)$, in blue, and of
$\varphi_{1/3}(e)$, in dark green (actually of $\varphi_{1/2}(e)$, in green,
to be conservative).</div>

<a name="inverse_radius_mapping_u"><p>The second issue to map $(e,\varphi)$ to
texture coordinates is that $e$ is unbounded and that
$\mathbb{U}(e,\varphi_{ub})$ varies rapidly for small values of $e$, as shown in
<a href="#fig4">Fig. 4</a>.

<a name="fig4">
<svg width="635" height="260" xmlns="http://www.w3.org/2000/svg">
  <style>
    path { fill:none }
    text { font-size:16px; font-family:Sans }
  </style>
  <path d="M17 225V35m0 190h276" stroke="#000"/>
  <text x="20" y="32">u</text>
  <text x="13" y="245">0</text>
  <text x="293" y="245">e</text>
  <path d="M17 225c8-22 6-163 13-163 6 0 5 50 37 63 32 12 58 9 205 13"
        stroke="#f00"/>
  <path d="M335 225V35m0 190h276" stroke="#000"/>
  <text x="338" y="32">u</text>
  <text x="331" y="245">0</text>
  <text x="614" y="245">1</text>
  <path d="M335 136c16-27 42-74 77-74s55 45 83 78c28 32 47 62 116 84"
        stroke="#f00"/>
</svg>
<div class="caption"><b>Fig. 4</b>. <i>Left</i>: plot of
$\mathbb{U}(e,\varphi_{ub}(e))$ as a function of $e$. <i>Right</i>: same plot,
 as a function of the texture coordinate given by
<code>GetRayInverseRadiusTextureUFromEsquare</code>.</div>

<p>To get more samples in this area while remapping the unbounded range to
$[0,1]$ we use the ad-hoc mapping function for $e$ defined in
<a href="https://arxiv.org/abs/2010.08735">Appendix A</a> of our model and
implemented as follows (see <a href="#fig4">Fig. 4</a>):
*/

Real GetRayInverseRadiusTextureUFromEsquare(const Real e_square) {
  return 1.0 / (1.0 + 6.0 * e_square);
}

/*
<p>With this we can finally implement a function to do a lookup in a precomputed
ray inverse radius texture, for some given $(e,\varphi)$ parameters:
*/

TimedInverseDistance LookupRayInverseRadius(IN(RayInverseRadiusTexture)
                                                ray_inverse_radius_texture,
                                            const Real e_square,
                                            const Angle phi) {
  Real tex_u = GetTextureCoordFromUnitRange(
      GetRayInverseRadiusTextureUFromEsquare(e_square),
      RAY_INVERSE_RADIUS_TEXTURE_WIDTH);
  Real tex_v = GetTextureCoordFromUnitRange(phi / GetPhiUbFromEsquare(e_square),
                                            RAY_INVERSE_RADIUS_TEXTURE_HEIGHT);
  return TimedInverseDistance(
      texture(ray_inverse_radius_texture, vec2(tex_u, tex_v)));
}

/*
<h3>Precomputed ray-tracing function</h3>

<p>We can finally implement the <span 
style="font-variant:small-caps;">TraceRay</span> procedure in
<a href="https://arxiv.org/abs/2010.08735">Algorithm 1</a> of our black hole
model, which computes the ray deflection and accretion disc intersections in
constant time, by using the above texture lookup functions.
*/

// Anti-aliased pulse function. See
// https://renderman.pixar.com/resources/RenderMan_20/basicAntialiasing.html.
Real FilteredPulse(Real edge0, Real edge1, Real x, Real fw) {
  fw = max(fw, 1e-6);
  Real x0 = x - fw * 0.5;
  Real x1 = x0 + fw;
  return max(0.0, (min(x1, edge1) - max(x0, edge0)) / fw);
}

Angle TraceRay(IN(RayDeflectionTexture) ray_deflection_texture,
               IN(RayInverseRadiusTexture) ray_inverse_radius_texture,
               const Real u, const Real u_dot, const Real e_square,
               const Angle delta, const Angle alpha, const Real u_ic,
               const Real u_oc, OUT(Real) u0, OUT(Angle) phi0, OUT(Real) t0,
               OUT(Real) alpha0, OUT(Real) u1, OUT(Angle) phi1, OUT(Real) t1,
               OUT(Real) alpha1) {
  // Compute the ray deflection.
  u0 = -1.0;
  u1 = -1.0;
  if (e_square < kMu && u > 2.0 / 3.0) {
    return -1.0 * rad;
  }
  TimedAngle deflection_apsis;
  TimedAngle deflection = LookupRayDeflection(ray_deflection_texture, e_square,
                                              u, deflection_apsis);
  Angle ray_deflection = deflection.x;
  if (u_dot > 0.0) {
    ray_deflection =
        e_square < kMu ? 2.0 * deflection_apsis.x - ray_deflection : -1.0 * rad;
  }
  // Compute the accretion disc intersections.
  Real s = sign(u_dot);
  Angle phi = deflection.x + (s == 1.0 ? pi - delta : delta) + s * alpha;
  Angle phi_apsis = deflection_apsis.x + pi / 2.0;
  phi0 = mod(phi, pi);
  TimedInverseDistance ui0 =
      LookupRayInverseRadius(ray_inverse_radius_texture, e_square, phi0);
  if (phi0 < phi_apsis) {
    Real side = s * (ui0.x - u);
    if (side > 1e-3 || (side > -1e-3 && alpha < delta)) {
      u0 = ui0.x;
      phi0 = alpha + phi - phi0;
      t0 = s * (ui0.y - deflection.y);
    }
  }
  phi = 2.0 * phi_apsis - phi;
  phi1 = mod(phi, pi);
  TimedInverseDistance ui1 =
      LookupRayInverseRadius(ray_inverse_radius_texture, e_square, phi1);
  if (e_square < kMu && s == 1.0 && phi1 < phi_apsis) {
    u1 = ui1.x;
    phi1 = alpha + phi - phi1;
    t1 = 2.0 * deflection_apsis.y - ui1.y - deflection.y;
  }
  // Compute the anti-aliasing opacity values.
  Real fw0 = min(fwidth(ui0.x), fwidth(u0 == -1.0 ? u1 : u0));
  Real fw1 = min(fwidth(ui1.x), fwidth(u1 == -1.0 ? u0 : u1));
  alpha0 = FilteredPulse(u_oc, u_ic, u0, fw0);
  alpha1 = FilteredPulse(u_oc, u_ic, u1, fw1);
  if (s == 1.0 && abs(e_square - kMu) < min(fwidth(e_square), kMu)) {
    if (alpha0 < 0.99) u0 = 2.0 / (1.0 / u_ic + 1.0 / u_oc);
    if (alpha1 < 0.99) u1 = 2.0 / (1.0 / u_ic + 1.0 / u_oc);
  }
  return ray_deflection;
}

Angle TraceRay(IN(RayDeflectionTexture) ray_deflection_texture,
               IN(RayInverseRadiusTexture) ray_inverse_radius_texture,
               const Real p_r, const Angle delta, const Angle alpha,
               const Real u_ic, const Real u_oc, OUT(Real) u0,
               OUT(Angle) phi0, OUT(Real) t0, OUT(Real) alpha0, OUT(Real) u1,
               OUT(Angle) phi1, OUT(Real) t1, OUT(Real) alpha1) {
  Real u = 1.0 / p_r;
  Real u_dot = -u / tan(delta);
  Real e_square = u_dot * u_dot + u * u * (1.0 - u);
  return TraceRay(ray_deflection_texture, ray_inverse_radius_texture, u,
                  u_dot, e_square, delta, alpha, u_ic, u_oc, u0, phi0, t0,
                  alpha0, u1, phi1, t1, alpha1);
}
