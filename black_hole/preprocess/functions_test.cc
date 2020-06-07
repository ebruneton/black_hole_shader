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

/*<h2>black_hole/preprocess/functions_test.cc</h2>

<p>This file provides unit tests whose main goal is to check that ray deflection
and accretion disc intersections, obtained via the precomputed textures, are
close to reference values computed via direct numerical integration.

<p><i><b>Note</b>: the code uses the same notations as those defined in our
<a href="../../paper.pdf">black hole model</a></i>.
*/

#include "black_hole/preprocess/functions.h"

#include <limits>
#include <string>
#include "math/angle.h"
#include "test/test_case.h"

namespace black_hole {
namespace preprocess {

// Provides unit tests for the main functions of our black hole shader.

namespace {

using dimensional::Angle;
using dimensional::pi;
using dimensional::rad;

constexpr double kEpsilon = 1e-3;
constexpr double kMu = 4.0 / 27.0;
constexpr double kUic = 0.5;
constexpr double kUoc = 0.0;

/*
<p>For this we implement the reference function below, to compute the deflection
of a light ray, from a given origin and initial direction, by numerically
integrating the geodesic equation - i.e. <a href="../../paper.pdf">Eq. (8)</a>
in our model. Also we compute the deflection itself by computing the direction
of the vector joining two successive points (previous_x,previous_y) and (x,y)
along the  ray. By using this method, completely different from the tested code,
we increase the confidence in the test results.

<p><b>Note</b>: the $u_{max}$ parameter can be used to stop the integration when
$u$ becomes larger than this value. In this case the function returns the
deflection at this point.
*/

// Reference function to get the ray deflection, used to check that the results
// computed via our glsl functions match this reference.

Angle RayDeflectionReference(const Real p_r, const Angle delta,
                             const Real u_max = 1.0) {
  Real u = 1.0 / p_r;
  Real u_dot = -1.0 / (p_r * tan(delta));
  Angle phi = 0 * rad;
  constexpr Real dphi = 1e-6;

  Real previous_x = p_r;
  Real previous_y = 0;
  Real x = previous_x;
  Real y = previous_y;
  while (true) {
    u_dot = u_dot + (1.5 * u * u - u) * dphi;
    u = u + u_dot * dphi;
    phi = phi + dphi * rad;
    if (u >= 1.0) {
      return -1.0 * rad;
    } else if (u < 0.0 || u >= u_max) {
      Angle deflection = atan2(y - previous_y, x - previous_x) - delta;
      while (deflection < phi - pi) {
        deflection = deflection + 2.0 * pi;
      }
      return deflection;
    }
    previous_x = x;
    previous_y = y;
    x = cos(phi) / u;
    y = sin(phi) / u;
  }
}

/*
<p>We do the same for the accretion disc intersections: we implement the
reference function below to compute them for a light ray, from a given origin
and initial direction, by numerically integrating the geodesic equation. Also we
detect the intersections via the change of sign of the scalar product between
the current ray point vector and the normal to the disc. By using this method,
completely different from the tested code, we increase the confidence in the
test results.

<p><b>Note</b>: the $\varphi_{max}$ parameter can be used to stop the
integration when $\varphi$ becomes larger than this value. In this case the
function returns the inverse radius $u$ at this point.
*/

// Reference function to get the accretion disc intersections, used to check
// that the results obtained with our glsl functions match this reference.

Real RayDiscIntersectionsReference(
    const Real p_r, const Angle delta, const Angle alpha, const Real u_ic,
    const Real u_oc, Real *u0, Angle *phi0, Real *t0, Real *u1, Angle *phi1,
    Real *t1,
    const Angle phi_max = std::numeric_limits<double>::infinity() * rad) {
  Real u = 1.0 / p_r;
  Real u_dot = -1.0 / (p_r * tan(delta));
  Angle phi = 0 * rad;
  Real t = 0.0;
  const Real e = sqrt(u_dot * u_dot + u * u * (1.0 - u));
  constexpr Real dphi = 1e-5;

  *u0 = -1.0;
  *phi0 = 0.0 * rad;
  *t0 = -1.0;
  *u1 = -1.0;
  *phi1 = 0.0 * rad;
  *t1 = -1.0;

  const Real cos_alpha = cos(alpha);
  const Real sin_alpha = sin(alpha);
  Real previous_sign = sin_alpha;
  while (true) {
    u_dot = u_dot + (1.5 * u * u - u) * dphi;
    u = u + u_dot * dphi;
    phi = phi + dphi * rad;
    t = t + e / (u * u * (1.0 - u)) * dphi;
    if (u < 0.0 || u >= 1.0 || phi >= phi_max) {
      return u;
    }
    const Real sign = sin_alpha * cos(phi) - cos_alpha * sin(phi);
    if (u >= u_oc && u <= u_ic && sign * previous_sign < 0.0) {
      if (*u0 < 0.0) {
        *u0 = u;
        *t0 = t;
        *phi0 = phi;
      } else if (*u1 < 0.0) {
        *u1 = u;
        *t1 = t;
        *phi1 = phi;
      } else {
        assert(false);
      }
    }
    previous_sign = sign;
  }
}

/*
<p>Finally, to simplify the definition of some test cases, we provide a function
to compute the initial ray direction angle $\delta$ from $p^r$ and $e^2$. It
derives from the equations $\dot{u}(0) = -u(0)\cot\delta$ and
$\dot{u}^2 = e^2 - u^2 (1 - u)$:
*/

Angle GetDelta(const Real p_r, const Real e_square) {
  const Real u = 1.0 / p_r;
  return pi - atan(u / sqrt(e_square - u * u * (1.0 - u)));
}

/*
<p>The following class provides the test fixture for all our unit tests. It has
instance fields to store a ray origin and initial direction, as well as an
accretion disc orientation. It also has static fields, initialized lazily and at
most once, to store the precomputed textures of our model.
*/
class FunctionsTest : public dimensional::TestCase {
 public:
  template <typename T>
  FunctionsTest(const std::string &name, const T test, const Real p_r = 0.0,
                const Angle delta = 0.0 * rad, const Angle alpha = 0.0 * rad)
      : TestCase("FunctionsTest " + name, static_cast<Test>(test)),
        p_r_(p_r),
        delta_(delta),
        alpha_(alpha) {
    if (!initialized_) {
      ComputeRayDeflectionTexture(&ray_deflection_texture_);
      ComputeRayInverseRadiusTexture(&ray_inverse_radius_texture_);
      initialized_ = true;
    }
  }

  // Convenience constructor for the TestTraceRayIntersections() test case.
  FunctionsTest(const std::string &index, const Real p_r, const Angle delta,
                const Angle alpha)
      : FunctionsTest("TraceRayIntersections" + index,
                      &FunctionsTest::TestTraceRayIntersections, p_r, delta,
                      alpha) {}

/*
<p>We use this fixture to first test some low level functions, before testing
the higher level functions computing the deflection and accretion disc
intersections. The first unit test checks that
<code>GetUapsisFromEsquare</code> actually returns a root of the cubic equation
$e^2 - u^2 (1 - u) = 0$.
*/

  void TestGetUapsisFromEsquare() {
    Real e_square0 = 0.01;
    Real e_square1 = 0.1;
    Real e_square2 = 0.148;
    Real u_a0 = GetUapsisFromEsquare(e_square0);
    Real u_a1 = GetUapsisFromEsquare(e_square1);
    Real u_a2 = GetUapsisFromEsquare(e_square2);

    ExpectNear(e_square0, u_a0 * u_a0 * (1.0 - u_a0), Real(kEpsilon));
    ExpectNear(e_square1, u_a1 * u_a1 * (1.0 - u_a1), Real(kEpsilon));
    ExpectNear(e_square2, u_a2 * u_a2 * (1.0 - u_a2), Real(kEpsilon));
  }

/*
<p>This unit test checks that <code>GetRayDeflectionTextureUFromEsquare</code>
and its inverse <code>GetEsquareFromRayDeflectionTextureU</code> behave as
expected.
*/

  void TestGetRayDeflectionTextureUFromEsquare() {
    ExpectNear(0.5, GetRayDeflectionTextureUFromEsquare(0.0)(), kEpsilon);
    ExpectNear(0.01, GetRayDeflectionTextureUFromEsquare(kMu - 1e-6)(), 1e-2);
    ExpectNear(0.98, GetRayDeflectionTextureUFromEsquare(kMu + 1e-6)(), 1e-2);
    ExpectNear(0.5, GetRayDeflectionTextureUFromEsquare(1e4)(), kEpsilon);

    ExpectNear(0.3,
               GetRayDeflectionTextureUFromEsquare(
                   GetEsquareFromRayDeflectionTextureU(0.3))(),
               kEpsilon);
    ExpectNear(0.4,
               GetRayDeflectionTextureUFromEsquare(
                   GetEsquareFromRayDeflectionTextureU(0.4))(),
               kEpsilon);
    ExpectNear(0.6,
               GetRayDeflectionTextureUFromEsquare(
                   GetEsquareFromRayDeflectionTextureU(0.6))(),
               kEpsilon);
    ExpectNear(0.7,
               GetRayDeflectionTextureUFromEsquare(
                   GetEsquareFromRayDeflectionTextureU(0.7))(),
               kEpsilon);
  }

/*
<p>This test does the same for
<code>GetRayInverseRadiusTextureUFromEsquare</code> and its inverse
<code>GetEsquareFromRayInverseRadiusTextureU</code>.
*/

  void TestGetRayInverseRadiusTextureUFromEsquare() {
    ExpectNear(1.0, GetRayInverseRadiusTextureUFromEsquare(0.0)(), kEpsilon);
    ExpectNear(1e-3, GetRayInverseRadiusTextureUFromEsquare(100.0)(), kEpsilon);

    ExpectNear(0.01,
               GetRayInverseRadiusTextureUFromEsquare(
                   GetEsquareFromRayInverseRadiusTextureU(0.01))(),
               kEpsilon);
    ExpectNear(0.99,
               GetRayInverseRadiusTextureUFromEsquare(
                   GetEsquareFromRayInverseRadiusTextureU(0.99))(),
               kEpsilon);
  }

/*
<p>The following test checks that doing a lookup in the precomputed ray
deflection texture gives the same result as obtained via a numerical integration
of the ray equation.
*/

  void TestLookupRayDeflection() {
    auto test = [&](Real e_square, Real u) {
      const Real p_r = 1e6;
      const Angle delta = GetDelta(p_r, e_square);
      Angle expected = RayDeflectionReference(p_r, delta, u);
      TimedAngle ignored;
      Angle actual =
          LookupRayDeflection(ray_deflection_texture_, e_square, u, ignored).x;
      ExpectNear(expected.to(rad), actual.to(rad), kEpsilon);
    };

    test(kMu - 1e-3, 0.6);
    test(kMu - 1e-3, 0.633);
    test(kMu + 1e-3, 0.6);
    test(kMu + 1e-3, 0.95);
  }

/*
<p>And this test does the same for the ray inverse radius: it checks that doing
a lookup in the precomputed ray inverse radius texture gives the same result as
obtained via a numerical integration of the ray equation.
*/

  void TestLookupRayInverseRadius() {
    auto test = [&](Real e_square, Angle phi) {
      const Real p_r = 1e6;
      const Angle delta = GetDelta(p_r, e_square);
      Real u0, u1, t0, t1;
      Angle phi0, phi1;
      Real expected =
          RayDiscIntersectionsReference(p_r, delta, 0.0 * rad, kUic, kUoc,
                                        &u0, &phi0, &t0, &u1, &phi1, &t1, phi);
      Real actual =
          LookupRayInverseRadius(ray_inverse_radius_texture_, e_square, phi).x;
      ExpectNear(expected, actual, Real(kEpsilon));
    };

    test(kMu / 2, pi / 2.0);
    test(kMu - 0.1, pi / 3.0);
    test(kMu + 0.1, pi / 3.0);
    test(kMu * 2, pi / 2.0);
  }

/*
<p>One of the main unit test is the following, which checks that the combination
of the <span style="font-variant:small-caps;">Precompute</span> and
<span style="font-variant:small-caps;">TraceRay</span> procedures of our
<a href="../../paper.pdf">model</a> gives the same deflection as a direct
numerical integration of the geodesic equation.
*/

  void TestTraceRayDeflection() {
    auto test = [&](Real p_r, Angle delta) {
      Angle expected = RayDeflectionReference(p_r, delta);
      Real u0, u1, t0, t1, alpha0, alpha1;
      Angle phi0, phi1;
      Angle actual = TraceRay(
          ray_deflection_texture_, ray_inverse_radius_texture_, p_r, delta,
          0.0 * rad, kUic, kUoc, u0, phi0, t0, alpha0, u1, phi1, t1, alpha1);
      ExpectNear(expected.to(rad), actual.to(rad), kEpsilon);
    };

    test(5.0, pi);
    test(4.0, pi / 5.0);
    test(4.0, pi / 4.0);
    test(4.0, 3.0 * pi / 4.0);
    test(4.0, GetDelta(4.0, kMu - 1e-4));
    test(1.25, pi / 2);
    test(1.1, pi - GetDelta(1.1, kMu + 1e-3));
  }

/*
<p>The other main unit test checks that the combination of the <span
style="font-variant:small-caps;">Precompute</span> and <span
style="font-variant:small-caps;">TraceRay</span> procedures of our
<a href="../../paper.pdf">model</a> gives the same accretion disc intersections
as a direct numerical integration of the geodesic equation. This test is 
instantiated for several ray and accretion disc configurations at the end of
this file.
*/

  void TestTraceRayIntersections() {
    Real expected_u0, expected_u1, expected_t0, expected_t1, actual_u0,
        actual_u1, actual_t0, actual_t1, actual_alpha0, actual_alpha1;
    Angle expected_phi0, expected_phi1, actual_phi0, actual_phi1;
    RayDiscIntersectionsReference(p_r_, delta_, alpha_, kUic, kUoc,
                                  &expected_u0, &expected_phi0, &expected_t0,
                                  &expected_u1, &expected_phi1, &expected_t1);

    TraceRay(ray_deflection_texture_, ray_inverse_radius_texture_, p_r_, delta_,
             alpha_, kUic, kUoc, actual_u0, actual_phi0, actual_t0,
             actual_alpha0, actual_u1, actual_phi1, actual_t1, actual_alpha1);
    if (actual_alpha0 == 0.0) actual_u0 = -1.0;
    if (actual_alpha1 == 0.0) actual_u1 = -1.0;
    if (actual_u0 == -1.0) {
      actual_u0 = actual_u1;
      actual_phi0 = actual_phi1;
      actual_t0 = actual_t1;
      actual_u1 = -1.0;
    }

    ExpectNear(expected_u0, actual_u0, Real(kEpsilon));
    ExpectNear(expected_u1, actual_u1, Real(kEpsilon));
    bool check_time = expected_t0 < 40.0 && expected_t1 < 40.0;
    if (expected_u0 >= 0.0) {
      ExpectNear(expected_phi0.to(rad), actual_phi0.to(rad), kEpsilon);
      if (check_time) {
        ExpectNear(expected_t0, actual_t0, Real(0.1));
      }
    }
    if (expected_u1 >= 0.0) {
      ExpectNear(expected_phi1.to(rad), actual_phi1.to(rad), kEpsilon);
      if (check_time) {
        ExpectNear(expected_t1, actual_t1, Real(0.1));
      }
    }
  }

/*
<p>The rest of the file defines the instance and static fields of the test
fixture, and creates one instance of the test fixture for each unit test.
*/

 private:
  const Real p_r_;
  const Angle delta_;
  const Angle alpha_;

  static bool initialized_;
  static RayDeflectionTexture ray_deflection_texture_;
  static RayInverseRadiusTexture ray_inverse_radius_texture_;
};

bool FunctionsTest::initialized_ = false;
RayDeflectionTexture FunctionsTest::ray_deflection_texture_;
RayInverseRadiusTexture FunctionsTest::ray_inverse_radius_texture_;

FunctionsTest get_u_apsis_from_e_square(
    "GetUapsisFromEsquare", &FunctionsTest::TestGetUapsisFromEsquare);

FunctionsTest get_ray_deflection_texture_u_from_e_square(
    "GetRayDeflectionTextureUFromEsquare",
    &FunctionsTest::TestGetRayDeflectionTextureUFromEsquare);

FunctionsTest get_ray_inverse_radius_texture_u_from_e_square(
    "GetRayInverseRadiusTextureUFromEsquare",
    &FunctionsTest::TestGetRayInverseRadiusTextureUFromEsquare);

FunctionsTest lookup_ray_deflection("LookupRayDeflection",
                                    &FunctionsTest::TestLookupRayDeflection);

FunctionsTest lookup_ray_inverse_radius(
    "LookupRayInverseRadius", &FunctionsTest::TestLookupRayInverseRadius);

FunctionsTest trace_ray_deflection("TraceRayDeflection",
                                   &FunctionsTest::TestTraceRayDeflection);

FunctionsTest trace_ray_intersections[] = {
    // Ray escapes from black hole, initial direction towards black hole.
    FunctionsTest("1", 4.0, 3.0 * pi / 4.0, 3.0 * pi / 4.0),
    FunctionsTest("2", 20.0, GetDelta(20.0, kMu - 1e-4), 3.0 * pi / 4.0),
    FunctionsTest("3", 20.0, GetDelta(20.0, kMu - 1e-4), 0.01 * rad),
    FunctionsTest("4", 20.0, GetDelta(20.0, kMu - 1e-4), pi / 4.0),
    FunctionsTest("5", 20.0, GetDelta(20.0, kMu - 0.05), pi / 4.0),
    // Ray escapes from black hole, initial direction away from black hole.
    FunctionsTest("6", 4.0, 7.0 * pi / 16.0, pi / 4.0),
    // Ray falls into black hole, initial direction towards black hole.
    FunctionsTest("7", 4.0, 13.0 * pi / 16.0, pi / 8.0),
    // Ray falls into black hole, initial direction away from black hole.
    FunctionsTest("8", 1.1, pi - GetDelta(1.1, kMu + 0.05), 3.0 * pi / 4.0),
    FunctionsTest("9", 1.1, 1.26 * rad, pi / 4.0),
    // Other cases.
    FunctionsTest("10", 4.0, GetDelta(4.0, kMu - 1e-3), pi / 3.0),
    FunctionsTest("11", 4.0, pi / 4.0, pi / 3.0),
    FunctionsTest("12", 4.0, 3.0 * pi / 4.0, pi / 3.0),
    FunctionsTest("13", 4.0, 3.0 * pi / 4.0, 2.0 * pi / 3.0)};

}  // anonymous namespace

}  // namespace preprocess
}  // namespace black_hole
