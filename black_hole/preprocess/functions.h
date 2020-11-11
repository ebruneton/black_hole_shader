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

/*<h2>black_hole/preprocess/functions.h</h2>

<p>This file declares the preprocessing functions that generate the precomputed
textures of our <a href="https://arxiv.org/abs/2010.08735">black hole model</a>.
It also declares the <a href="../functions.glsl.html">GLSL functions</a> that
implement this model, and in particular the ones using the precomputed textures.
The goal is to be able to check, in
<a href="functions_test.cc.html">unit tests</a>, that ray deflection and
accretion disc intersections, obtained via the precomputed textures, are close
to reference values computed via direct numerical integration.

<p><i><b>Note</b>: the code uses the same notations as those defined in our
<a href="https://arxiv.org/abs/2010.08735">black hole model</a></i>. For
example, <code>p_r</code> and <code>delta</code> refer to $p^r$ and $\delta$,
the origin and initial direction of a light ray, <code>alpha</code> refers to
the orientation $\alpha$ of the accretion disc, <code>u_ic</code> and
<code>u_oc</code> are the inverse of its inner and outer radius, etc.
*/

#ifndef BLACK_HOLE_PREPROCESS_FUNCTIONS_H_
#define BLACK_HOLE_PREPROCESS_FUNCTIONS_H_

#include "black_hole/preprocess/definitions.h"

namespace black_hole {
namespace preprocess {

// Declares a C++ header for the functions in 'functions.glsl', and for some
// additional functions used to precompute the textures required by the glsl
// functions.

Real GetUapsisFromEsquare(Real e_square);

// Ray deflection texture: mapping, lookup & precomputations.

Real GetRayDeflectionTextureUFromEsquare(Real e_square);

Real GetRayDeflectionTextureVFromEsquareAndU(Real e_square, Real u);

Real GetTextureCoordFromUnitRange(Real x, int texture_size);

Real GetEsquareFromRayDeflectionTextureU(Real tex_u);

TimedAngle LookupRayDeflection(const RayDeflectionTexture &texture,
                               Real e_square, Real u,
                               TimedAngle &deflection_apsis);

void ComputeRayDeflectionTexture(RayDeflectionTexture *texture);

// Ray inverse radius texture: mapping, lookup & precomputations.

dimensional::Angle GetPhiUpFromEsquare(Real e_square);

Real GetRayInverseRadiusTextureUFromEsquare(Real e_square);

Real GetEsquareFromRayInverseRadiusTextureU(Real tex_u);

TimedInverseDistance LookupRayInverseRadius(
    const RayInverseRadiusTexture &texture, Real e_square,
    dimensional::Angle phi);

void ComputeRayInverseRadiusTexture(RayInverseRadiusTexture *texture);

// Ray "tracing" using the above precomputed textures. Returns the ray
// deflection, or -1 if the ray falls into the black hole. The accretion disc
// intersections, if any, are output in (u0, phi0, t0) and (u1, phi1, t1), with
// ti the light propagation time between the intersection and the camera (must
// be subtracted from the camera time to get the retarded time). ui is set to -1
// if there is no intersection. alpha0 and alpha1 are anti-aliasing opacity
// values.

dimensional::Angle TraceRay(
    const RayDeflectionTexture &ray_deflection_texture,
    const RayInverseRadiusTexture &ray_inverse_radius_texture, Real p_r,
    dimensional::Angle delta, dimensional::Angle alpha, Real u_ic, Real u_oc,
    Real &u0, dimensional::Angle &phi0, Real &t0, Real &alpha0, Real &u1,
    dimensional::Angle &phi1, Real &t1, Real &alpha1);

}  // namespace preprocess
}  // namespace black_hole

#endif  // BLACK_HOLE_PREPROCESS_FUNCTIONS_H_
