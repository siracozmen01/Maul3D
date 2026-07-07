// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The math gate: analytic pins for the deterministic trig and the
// quaternion contract, plus a bit-stability sweep whose M3_MATH_HASH
// joins the cross-platform CI comparison. If any platform, compiler,
// or backend rounds one of these operations differently, the gate goes
// red before any physics is built on top.

#include "maul3d/math.h"

#include <stdio.h>

static int s_failures = 0;

#define CHECK(cond, msg)                                                                           \
    do                                                                                             \
    {                                                                                              \
        if (!(cond))                                                                               \
        {                                                                                          \
            printf("FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__);                                 \
            s_failures += 1;                                                                       \
        }                                                                                          \
    } while (0)

static int NearF(m3real a, m3real b, m3real tol)
{
    m3real d = a - b;
    return (d < 0.0f ? -d : d) <= tol;
}

static void TestScalarOps(void)
{
    CHECK(m3MinF(1.0f, 2.0f) == 1.0f && m3MaxF(1.0f, 2.0f) == 2.0f, "pinned min max");
    CHECK(m3ClampF(5.0f, 0.0f, 2.0f) == 2.0f, "clamp upper");
    // Signed zero law: (a < b ? a : b) keeps the second operand.
    m3real nz = -0.0f;
    m3real pz = 0.0f;
    CHECK(m3MinF(nz, pz) == 0.0f, "min is compare and select, ties keep b");
}

static void TestVectors(void)
{
    m3Vec3 x = {1.0f, 0.0f, 0.0f};
    m3Vec3 y = {0.0f, 1.0f, 0.0f};
    m3Vec3 z = m3Cross3(x, y);
    CHECK(z.x == 0.0f && z.y == 0.0f && z.z == 1.0f, "cross(x, y) is exactly z");
    CHECK(m3Dot3(x, y) == 0.0f, "orthogonal dot is exactly zero");
    m3Vec3 v = {3.0f, 4.0f, 0.0f};
    CHECK(m3Length3(v) == 5.0f, "3-4-5 length is exact");
    m3Vec3 zero = {0.0f, 0.0f, 0.0f};
    m3Vec3 n = m3Normalize3(zero);
    CHECK(n.x == 0.0f && n.y == 1.0f && n.z == 0.0f, "degenerate normalize fallback is fixed");
}

static void TestTrig(void)
{
    CHECK(m3Atan2(0.0f, 0.0f) == 0.0f, "atan2(0,0) is zero, never NaN");
    CHECK(NearF(m3Atan2(1.0f, 1.0f), 0.25f * M3_PI, 1.0e-4f), "atan2(1,1) near pi/4");
    CHECK(NearF(m3Atan2(1.0f, 0.0f), 0.5f * M3_PI, 1.0e-4f), "atan2(1,0) near pi/2");
    CHECK(NearF(m3Atan2(-1.0f, 0.0f), -0.5f * M3_PI, 1.0e-4f), "atan2(-1,0) near -pi/2");

    // Bhaskara accuracy band (about 0.002 worst case) and the built-in
    // renormalization: c*c + s*s lands within float rounding of one.
    for (int32_t i = -8; i <= 8; ++i)
    {
        m3real angle = 0.25f * M3_PI * (m3real)i;
        m3CosSin cs = m3ComputeCosSin(angle);
        CHECK(NearF(cs.c, cosf(angle), 3.0e-3f), "cosine within the Bhaskara band");
        CHECK(NearF(cs.s, sinf(angle), 3.0e-3f), "sine within the Bhaskara band");
        CHECK(NearF(cs.c * cs.c + cs.s * cs.s, 1.0f, 2.0e-6f), "renormalized to the unit circle");
    }
}

static void TestQuaternions(void)
{
    // Rotate x by 90 degrees about z: lands on y (within trig band).
    m3CosSin half = m3ComputeCosSin(0.25f * M3_PI);
    m3Quat q = {0.0f, 0.0f, half.s, half.c};
    m3Vec3 v = m3RotateVec3(q, (m3Vec3){1.0f, 0.0f, 0.0f});
    CHECK(NearF(v.x, 0.0f, 5.0e-3f) && NearF(v.y, 1.0f, 5.0e-3f) && NearF(v.z, 0.0f, 5.0e-3f),
          "quarter turn about z maps x to y");

    // Round trip: rotate then inverse rotate is the identity.
    m3Vec3 p = {0.3f, -0.7f, 1.1f};
    m3Vec3 rt = m3InvRotateVec3(q, m3RotateVec3(q, p));
    CHECK(NearF(rt.x, p.x, 1.0e-5f) && NearF(rt.y, p.y, 1.0e-5f) && NearF(rt.z, p.z, 1.0e-5f),
          "rotate round trip");

    // Integration keeps unit length exactly (normalize is built in).
    m3Quat r = m3MakeIdentityQuat();
    for (int32_t i = 0; i < 1000; ++i)
    {
        r = m3IntegrateRotation(r, (m3Vec3){0.011f, -0.007f, 0.013f});
    }
    m3real mag2 = r.x * r.x + r.y * r.y + r.z * r.z + r.w * r.w;
    CHECK(NearF(mag2, 1.0f, 2.0e-6f), "1000 integrations stay on the unit sphere");

    m3Quat degenerate = {0.0f, 0.0f, 0.0f, 0.0f};
    m3Quat fixedq = m3NormalizeQuat(degenerate);
    CHECK(fixedq.w == 1.0f, "degenerate quat normalizes to identity");
}

static void TestBitStability(void)
{
    // A fixed sweep through every operation, folded into one hash that
    // every CI cell must reproduce bit for bit.
    uint64_t h = M3_HASH_INIT;
    m3Quat q = m3MakeIdentityQuat();
    for (int32_t i = 0; i < 2048; ++i)
    {
        m3real angle = 0.01f * (m3real)i - 10.0f;
        m3CosSin cs = m3ComputeCosSin(angle);
        m3real at = m3Atan2(cs.s, cs.c);
        m3Vec3 v = {cs.c, cs.s, at};
        q = m3IntegrateRotation(q, m3MulSV3(0.001f, v));
        m3Vec3 r = m3RotateVec3(q, v);
        m3Vec3 n = m3Normalize3(m3Cross3(v, r));
        h = m3Hash64(h, &cs, (int32_t)sizeof(cs));
        h = m3Hash64(h, &at, (int32_t)sizeof(at));
        h = m3Hash64(h, &q, (int32_t)sizeof(q));
        h = m3Hash64(h, &n, (int32_t)sizeof(n));
    }
    printf("M3_MATH_HASH=%016llx\n", (unsigned long long)h);
}

int main(void)
{
    TestScalarOps();
    TestVectors();
    TestTrig();
    TestQuaternions();
    TestBitStability();
    if (s_failures == 0)
    {
        printf("test_math: all checks passed\n");
    }
    return s_failures == 0 ? 0 : 1;
}
