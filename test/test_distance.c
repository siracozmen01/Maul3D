// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// GJK gate: analytic pins plus the separating-axis ORACLE (the Maul2D
// recipe): for any separated pair the returned normal must TRULY
// separate the point sets, the projected gap along it must equal the
// reported distance, and rerunning with the warm cache must reproduce
// the same bits. A wrong closest point cannot satisfy the oracle.
// White box.

#include "world_internal.h"

#include <stdio.h>
#include <string.h>

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

static uint32_t s_state = 0x0DDBA11u;
static uint32_t NextU32(void)
{
    s_state += 0x9E3779B9u;
    uint32_t z = s_state;
    z = (z ^ (z >> 16)) * 0x85EBCA6Bu;
    z = (z ^ (z >> 13)) * 0xC2B2AE35u;
    return z ^ (z >> 16);
}
static m3real NextF(m3real lo, m3real hi)
{
    return lo + (hi - lo) * ((m3real)(NextU32() >> 8) * (1.0f / 16777216.0f));
}

static void TestAnalytic(void)
{
    // Two unit boxes, B three units along x: gap exactly 1.
    m3HullData boxData;
    m3BuildBoxHull(&boxData, (m3Vec3){1.0f, 1.0f, 1.0f});
    m3DistanceInput input;
    memset(&input, 0, sizeof(input));
    input.proxyA = (m3DistanceProxy){boxData.vertices, boxData.vertexCount, 0.0f};
    input.proxyB = (m3DistanceProxy){boxData.vertices, boxData.vertexCount, 0.0f};
    input.q = m3MakeIdentityQuat();
    input.p = (m3Vec3){3.0f, 0.0f, 0.0f};
    m3SimplexCache cache;
    memset(&cache, 0, sizeof(cache));
    m3DistanceOutput out = m3ShapeDistance(&input, &cache);
    CHECK(NearF(out.distance, 1.0f, 1.0e-5f), "box-box gap is analytic");
    CHECK(NearF(out.normal.x, 1.0f, 1.0e-5f), "the normal points from A to B");
    CHECK(NearF(out.pointA.x, 1.0f, 1.0e-4f) && NearF(out.pointB.x, 2.0f, 1.0e-4f),
          "witnesses sit on the facing faces");

    // Warm rerun: the cache reproduces the identical bits.
    m3DistanceOutput warm = m3ShapeDistance(&input, &cache);
    CHECK(memcmp(&warm.distance, &out.distance, 4) == 0, "the warm cache reproduces the bits");

    // Overlap: B at the origin, distance zero, never NaN.
    input.p = (m3Vec3){0.25f, 0.0f, 0.0f};
    memset(&cache, 0, sizeof(cache));
    out = m3ShapeDistance(&input, &cache);
    CHECK(out.distance == 0.0f, "an overlapped pair reports zero");
    CHECK(out.pointA.x == out.pointA.x, "and stays finite");

    // Point versus segment: the classic 3-4-5.
    m3Vec3 point[1] = {{0.0f, 0.0f, 0.0f}};
    m3Vec3 segment[2] = {{3.0f, 4.0f, 0.0f}, {3.0f, -4.0f, 0.0f}};
    input.proxyA = (m3DistanceProxy){point, 1, 0.0f};
    input.proxyB = (m3DistanceProxy){segment, 2, 0.0f};
    input.p = (m3Vec3){0.0f, 0.0f, 0.0f};
    memset(&cache, 0, sizeof(cache));
    out = m3ShapeDistance(&input, &cache);
    CHECK(NearF(out.distance, 3.0f, 1.0e-5f), "point-segment distance is analytic");

    // Sphere proxies: two points with radii, useRadii on.
    m3Vec3 pa[1] = {{0.0f, 0.0f, 0.0f}};
    m3Vec3 pb[1] = {{0.0f, 0.0f, 0.0f}};
    input.proxyA = (m3DistanceProxy){pa, 1, 0.5f};
    input.proxyB = (m3DistanceProxy){pb, 1, 0.25f};
    input.p = (m3Vec3){2.0f, 0.0f, 0.0f};
    input.useRadii = true;
    memset(&cache, 0, sizeof(cache));
    out = m3ShapeDistance(&input, &cache);
    CHECK(NearF(out.distance, 1.25f, 1.0e-5f), "radii subtract from the core distance");
    input.useRadii = false;
}

static void TestOracle(void)
{
    // Random rotated box pairs: every separated result must satisfy
    // the separating-axis oracle.
    m3HullData boxData;
    m3BuildBoxHull(&boxData, (m3Vec3){0.8f, 0.5f, 1.2f});
    int32_t separated = 0;
    int32_t overlapped = 0;
    for (int32_t iter = 0; iter < 500; ++iter)
    {
        m3Quat q = m3NormalizeQuat((m3Quat){NextF(-1.0f, 1.0f), NextF(-1.0f, 1.0f),
                                            NextF(-1.0f, 1.0f), NextF(-1.0f, 1.0f)});
        m3Vec3 p = {NextF(-2.5f, 2.5f), NextF(-2.5f, 2.5f), NextF(-2.5f, 2.5f)};

        m3DistanceInput input;
        memset(&input, 0, sizeof(input));
        input.proxyA = (m3DistanceProxy){boxData.vertices, boxData.vertexCount, 0.0f};
        input.proxyB = (m3DistanceProxy){boxData.vertices, boxData.vertexCount, 0.0f};
        input.q = q;
        input.p = p;
        m3SimplexCache cache;
        memset(&cache, 0, sizeof(cache));
        m3DistanceOutput out = m3ShapeDistance(&input, &cache);

        if (out.distance > 1.0e-4f)
        {
            separated += 1;
            // The oracle: project both point sets on the normal; the
            // gap between the sets must equal the reported distance.
            m3Vec3 n = out.normal;
            m3real maxA = -3.4e38f;
            for (int32_t v = 0; v < boxData.vertexCount; ++v)
            {
                m3real d = m3Dot3(n, boxData.vertices[v]);
                maxA = d > maxA ? d : maxA;
            }
            m3real minB = 3.4e38f;
            for (int32_t v = 0; v < boxData.vertexCount; ++v)
            {
                m3Vec3 w = m3Add3(m3RotateVec3(q, boxData.vertices[v]), p);
                m3real d = m3Dot3(n, w);
                minB = d < minB ? d : minB;
            }
            CHECK(minB - maxA > -1.0e-3f, "the normal truly separates the sets");
            CHECK(NearF(minB - maxA, out.distance, 2.0e-3f),
                  "the projected gap equals the reported distance");
        }
        else
        {
            overlapped += 1;
        }
    }
    // The sweep must exercise both outcomes to mean anything.
    CHECK(separated > 30 && overlapped > 30, "the sweep hits both regimes");
}

int main(void)
{
    TestAnalytic();
    TestOracle();
    if (s_failures == 0)
    {
        printf("test_distance: all checks passed\n");
    }
    return s_failures == 0 ? 0 : 1;
}
