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

/*<h2>black_hole/definitions.glsl</h2>

<p>This file defines the types which are used in the main
<a href="functions.glsl.html">functions</a> of our <a href="../paper.pdf">black
hole model</a>. A <a href="preprocess/definitions.h.html">C++ equivalent</a> of
this file allows the same functions to be compiled by a C++ compiler (in order
to unit test our <a href="preprocess/functions.h.html">preprocessing
functions</a>).

<p>To increase the code readability we introduce different "types" for
dimensionless quantities and for angles, as well as for the two precomputed
textures of our model:
*/

// The types used in the main functions of our black hole shader. C++ equivalent
// of these types are used to compile these functions with a C++ compiler, both
// to reuse them in order to precompute the textures they need, and for testing
// them.

// Angles and dimensionless quantities.
#define Angle float
#define Real float

// An angle and a time (in the 1st and 2nd components, respectively).
#define TimedAngle vec2

// An inverse distance and a time (in the 1st and 2nd components, respectively).
#define TimedInverseDistance vec2

// A 2D texture with TimedAngle values.
#define RayDeflectionTexture sampler2D

// A 2D texture with TimedInverseDistance values.
#define RayInverseRadiusTexture sampler2D
