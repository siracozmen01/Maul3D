// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// GJK distance, adapted from Box3D's distance.c (Copyright 2026 Erin
// Catto, MIT; Dirk Gregorius contributed portions upstream): the
// barycentric Voronoi solvers for segment, triangle, and tetrahedron
// simplices, the warm-startable vertex cache, and the main loop with
// the duplicate-support termination. One deviation, argued: the
// reference converts the quaternion to a matrix for speed; we rotate
// through the quaternion directly, the same arithmetic class our
// whole math layer pins, one less code path to keep deterministic.

#include "world_internal.h"

#include <float.h>
#include <string.h>

typedef struct m3SimplexVertex
{
    m3Vec3 wA; // support on A, frame A
    m3Vec3 wB; // support on B, frame A
    m3Vec3 w;  // wB - wA
    m3real a;  // barycentric weight
    int32_t indexA;
    int32_t indexB;
} m3SimplexVertex;

typedef struct m3Simplex
{
    m3SimplexVertex vertices[4];
    int32_t count;
} m3Simplex;

static m3real ScalarTriple(m3Vec3 a, m3Vec3 b, m3Vec3 c)
{
    return m3Dot3(a, m3Cross3(b, c));
}

static m3Vec3 Blend2(m3real a, m3Vec3 wa, m3real b, m3Vec3 wb)
{
    return (m3Vec3){a * wa.x + b * wb.x, a * wa.y + b * wb.y, a * wa.z + b * wb.z};
}

static m3Vec3 Blend3(m3real a, m3Vec3 wa, m3real b, m3Vec3 wb, m3real c, m3Vec3 wc)
{
    return (m3Vec3){a * wa.x + b * wb.x + c * wc.x, a * wa.y + b * wb.y + c * wc.y,
                    a * wa.z + b * wb.z + c * wc.z};
}

// Farthest vertex along the axis, first-vertex-origin shifted for
// precision (the reference technique).
static int32_t ProxySupport(const m3DistanceProxy* proxy, m3Vec3 axis)
{
    m3Vec3 origin = proxy->points[0];
    int32_t maxIndex = 0;
    m3real maxProjection = 0.0f;
    for (int32_t i = 1; i < proxy->count; ++i)
    {
        m3real projection = m3Dot3(axis, m3Sub3(proxy->points[i], origin));
        if (projection > maxProjection)
        {
            maxIndex = i;
            maxProjection = projection;
        }
    }
    return maxIndex;
}

// Barycentric helpers: the last element is the divisor.
static void BaryEdge(m3real out[3], m3Vec3 a, m3Vec3 b)
{
    m3Vec3 ab = m3Sub3(b, a);
    out[0] = m3Dot3(b, ab);
    out[1] = -m3Dot3(a, ab);
    out[2] = m3Dot3(ab, ab);
}

static void BaryTri(m3real out[4], m3Vec3 a, m3Vec3 b, m3Vec3 c)
{
    m3Vec3 ab = m3Sub3(b, a);
    m3Vec3 ac = m3Sub3(c, a);
    m3Vec3 n = m3Cross3(ab, ac);
    out[0] = m3Dot3(m3Cross3(b, c), n);
    out[1] = m3Dot3(m3Cross3(c, a), n);
    out[2] = m3Dot3(m3Cross3(a, b), n);
    out[3] = m3Dot3(n, n);
}

static void BaryTet(m3real out[5], m3Vec3 a, m3Vec3 b, m3Vec3 c, m3Vec3 d)
{
    m3Vec3 ab = m3Sub3(b, a);
    m3Vec3 ac = m3Sub3(c, a);
    m3Vec3 ad = m3Sub3(d, a);
    m3real divisor = ScalarTriple(ab, ac, ad);
    m3real sign = divisor < 0.0f ? -1.0f : 1.0f;
    out[0] = sign * ScalarTriple(b, c, d);
    out[1] = sign * ScalarTriple(a, d, c);
    out[2] = sign * ScalarTriple(a, b, d);
    out[3] = sign * ScalarTriple(a, c, b);
    out[4] = sign * divisor;
}

static m3real GetMetric(const m3Simplex* simplex)
{
    const m3SimplexVertex* vs = simplex->vertices;
    switch (simplex->count)
    {
    case 1:
        return 0.0f;
    case 2:
        return m3Length3(m3Sub3(vs[1].w, vs[0].w));
    case 3:
        return 0.5f * m3Length3(m3Cross3(m3Sub3(vs[1].w, vs[0].w), m3Sub3(vs[2].w, vs[0].w)));
    case 4:
        return ScalarTriple(m3Sub3(vs[1].w, vs[0].w), m3Sub3(vs[2].w, vs[0].w),
                            m3Sub3(vs[3].w, vs[0].w)) /
               6.0f;
    default:
        return 0.0f;
    }
}

static void WriteCache(m3SimplexCache* cache, const m3Simplex* simplex)
{
    cache->metric = GetMetric(simplex);
    cache->count = (uint16_t)simplex->count;
    for (int32_t i = 0; i < simplex->count; ++i)
    {
        cache->indexA[i] = (uint8_t)simplex->vertices[i].indexA;
        cache->indexB[i] = (uint8_t)simplex->vertices[i].indexB;
    }
}

static bool SolveSimplex2(m3Simplex* simplex)
{
    m3SimplexVertex* vs = simplex->vertices;
    m3Vec3 a = vs[0].w;
    m3Vec3 b = vs[1].w;
    m3Vec3 ab = m3Sub3(b, a);
    m3real divisor = m3Dot3(ab, ab);
    m3real u = m3Dot3(b, ab);
    m3real v = -m3Dot3(a, ab);

    if (v <= 0.0f)
    {
        simplex->count = 1;
        vs[0].a = 1.0f;
        return true;
    }
    if (u <= 0.0f)
    {
        simplex->count = 1;
        vs[0] = vs[1];
        vs[0].a = 1.0f;
        return true;
    }
    if (divisor <= 0.0f)
    {
        return false;
    }
    m3real denominator = 1.0f / divisor;
    vs[0].a = denominator * u;
    vs[1].a = denominator * v;
    return true;
}

static bool SolveSimplex3(m3Simplex* simplex)
{
    m3SimplexVertex* vs = simplex->vertices;
    m3SimplexVertex v1 = vs[0];
    m3SimplexVertex v2 = vs[1];
    m3SimplexVertex v3 = vs[2];

    m3real wAB[3];
    m3real wBC[3];
    m3real wCA[3];
    BaryEdge(wAB, v1.w, v2.w);
    BaryEdge(wBC, v2.w, v3.w);
    BaryEdge(wCA, v3.w, v1.w);

    if (wAB[1] <= 0.0f && wCA[0] <= 0.0f)
    {
        simplex->count = 1;
        vs[0] = v1;
        vs[0].a = 1.0f;
        return true;
    }
    if (wBC[1] <= 0.0f && wAB[0] <= 0.0f)
    {
        simplex->count = 1;
        vs[0] = v2;
        vs[0].a = 1.0f;
        return true;
    }
    if (wCA[1] <= 0.0f && wBC[0] <= 0.0f)
    {
        simplex->count = 1;
        vs[0] = v3;
        vs[0].a = 1.0f;
        return true;
    }

    m3real wABC[4];
    BaryTri(wABC, v1.w, v2.w, v3.w);

    if (wABC[2] <= 0.0f && wAB[0] > 0.0f && wAB[1] > 0.0f)
    {
        simplex->count = 2;
        vs[0] = v1;
        vs[1] = v2;
        if (wAB[2] <= 0.0f)
        {
            return false;
        }
        vs[0].a = wAB[0] / wAB[2];
        vs[1].a = wAB[1] / wAB[2];
        return true;
    }
    if (wABC[0] <= 0.0f && wBC[0] > 0.0f && wBC[1] > 0.0f)
    {
        simplex->count = 2;
        vs[0] = v2;
        vs[1] = v3;
        if (wBC[2] <= 0.0f)
        {
            return false;
        }
        vs[0].a = wBC[0] / wBC[2];
        vs[1].a = wBC[1] / wBC[2];
        return true;
    }
    if (wABC[1] <= 0.0f && wCA[0] > 0.0f && wCA[1] > 0.0f)
    {
        simplex->count = 2;
        vs[0] = v3;
        vs[1] = v1;
        if (wCA[2] <= 0.0f)
        {
            return false;
        }
        vs[0].a = wCA[0] / wCA[2];
        vs[1].a = wCA[1] / wCA[2];
        return true;
    }

    if (wABC[3] <= 0.0f)
    {
        return false;
    }
    vs[0].a = wABC[0] / wABC[3];
    vs[1].a = wABC[1] / wABC[3];
    vs[2].a = wABC[2] / wABC[3];
    return true;
}

static bool SolveSimplex4(m3Simplex* simplex)
{
    m3SimplexVertex* vs = simplex->vertices;
    m3SimplexVertex vA = vs[0];
    m3SimplexVertex vB = vs[1];
    m3SimplexVertex vC = vs[2];
    m3SimplexVertex vD = vs[3];

    m3real wAB[3];
    m3real wAC[3];
    m3real wAD[3];
    m3real wBC[3];
    m3real wCD[3];
    m3real wDB[3];
    BaryEdge(wAB, vA.w, vB.w);
    BaryEdge(wAC, vA.w, vC.w);
    BaryEdge(wAD, vA.w, vD.w);
    BaryEdge(wBC, vB.w, vC.w);
    BaryEdge(wCD, vC.w, vD.w);
    BaryEdge(wDB, vD.w, vB.w);

    // Vertex regions.
    if (wAB[1] <= 0.0f && wAC[1] <= 0.0f && wAD[1] <= 0.0f)
    {
        simplex->count = 1;
        vs[0] = vA;
        vs[0].a = 1.0f;
        return true;
    }
    if (wAB[0] <= 0.0f && wDB[0] <= 0.0f && wBC[1] <= 0.0f)
    {
        simplex->count = 1;
        vs[0] = vB;
        vs[0].a = 1.0f;
        return true;
    }
    if (wAC[0] <= 0.0f && wBC[0] <= 0.0f && wCD[1] <= 0.0f)
    {
        simplex->count = 1;
        vs[0] = vC;
        vs[0].a = 1.0f;
        return true;
    }
    if (wAD[0] <= 0.0f && wCD[0] <= 0.0f && wDB[1] <= 0.0f)
    {
        simplex->count = 1;
        vs[0] = vD;
        vs[0].a = 1.0f;
        return true;
    }

    // Edge regions.
    m3real wACB[4];
    m3real wABD[4];
    m3real wADC[4];
    m3real wBCD[4];
    BaryTri(wACB, vA.w, vC.w, vB.w);
    BaryTri(wABD, vA.w, vB.w, vD.w);
    BaryTri(wADC, vA.w, vD.w, vC.w);
    BaryTri(wBCD, vB.w, vC.w, vD.w);

    if (wABD[2] <= 0.0f && wACB[1] <= 0.0f && wAB[0] > 0.0f && wAB[1] > 0.0f)
    {
        simplex->count = 2;
        vs[0] = vA;
        vs[1] = vB;
        if (wAB[2] <= 0.0f)
        {
            return false;
        }
        vs[0].a = wAB[0] / wAB[2];
        vs[1].a = wAB[1] / wAB[2];
        return true;
    }
    if (wACB[2] <= 0.0f && wADC[1] <= 0.0f && wAC[0] > 0.0f && wAC[1] > 0.0f)
    {
        simplex->count = 2;
        vs[0] = vA;
        vs[1] = vC;
        if (wAC[2] <= 0.0f)
        {
            return false;
        }
        vs[0].a = wAC[0] / wAC[2];
        vs[1].a = wAC[1] / wAC[2];
        return true;
    }
    if (wADC[2] <= 0.0f && wABD[1] <= 0.0f && wAD[0] > 0.0f && wAD[1] > 0.0f)
    {
        simplex->count = 2;
        vs[0] = vA;
        vs[1] = vD;
        if (wAD[2] <= 0.0f)
        {
            return false;
        }
        vs[0].a = wAD[0] / wAD[2];
        vs[1].a = wAD[1] / wAD[2];
        return true;
    }
    if (wACB[0] <= 0.0f && wBCD[2] <= 0.0f && wBC[0] > 0.0f && wBC[1] > 0.0f)
    {
        simplex->count = 2;
        vs[0] = vB;
        vs[1] = vC;
        if (wBC[2] <= 0.0f)
        {
            return false;
        }
        vs[0].a = wBC[0] / wBC[2];
        vs[1].a = wBC[1] / wBC[2];
        return true;
    }
    if (wADC[0] <= 0.0f && wBCD[0] <= 0.0f && wCD[0] > 0.0f && wCD[1] > 0.0f)
    {
        simplex->count = 2;
        vs[0] = vC;
        vs[1] = vD;
        if (wCD[2] <= 0.0f)
        {
            return false;
        }
        vs[0].a = wCD[0] / wCD[2];
        vs[1].a = wCD[1] / wCD[2];
        return true;
    }
    if (wABD[0] <= 0.0f && wBCD[1] <= 0.0f && wDB[0] > 0.0f && wDB[1] > 0.0f)
    {
        simplex->count = 2;
        vs[0] = vD;
        vs[1] = vB;
        if (wDB[2] <= 0.0f)
        {
            return false;
        }
        vs[0].a = wDB[0] / wDB[2];
        vs[1].a = wDB[1] / wDB[2];
        return true;
    }

    // Face regions.
    m3real wABCD[5];
    BaryTet(wABCD, vA.w, vB.w, vC.w, vD.w);

    if (wABCD[3] < 0.0f && wACB[0] > 0.0f && wACB[1] > 0.0f && wACB[2] > 0.0f)
    {
        simplex->count = 3;
        vs[0] = vA;
        vs[1] = vC;
        vs[2] = vB;
        if (wACB[3] <= 0.0f)
        {
            return false;
        }
        vs[0].a = wACB[0] / wACB[3];
        vs[1].a = wACB[1] / wACB[3];
        vs[2].a = wACB[2] / wACB[3];
        return true;
    }
    if (wABCD[2] < 0.0f && wABD[0] > 0.0f && wABD[1] > 0.0f && wABD[2] > 0.0f)
    {
        simplex->count = 3;
        vs[0] = vA;
        vs[1] = vB;
        vs[2] = vD;
        if (wABD[3] <= 0.0f)
        {
            return false;
        }
        vs[0].a = wABD[0] / wABD[3];
        vs[1].a = wABD[1] / wABD[3];
        vs[2].a = wABD[2] / wABD[3];
        return true;
    }
    if (wABCD[1] < 0.0f && wADC[0] > 0.0f && wADC[1] > 0.0f && wADC[2] > 0.0f)
    {
        simplex->count = 3;
        vs[0] = vA;
        vs[1] = vD;
        vs[2] = vC;
        if (wADC[3] <= 0.0f)
        {
            return false;
        }
        vs[0].a = wADC[0] / wADC[3];
        vs[1].a = wADC[1] / wADC[3];
        vs[2].a = wADC[2] / wADC[3];
        return true;
    }
    if (wABCD[0] < 0.0f && wBCD[0] > 0.0f && wBCD[1] > 0.0f && wBCD[2] > 0.0f)
    {
        simplex->count = 3;
        vs[0] = vB;
        vs[1] = vC;
        vs[2] = vD;
        if (wBCD[3] <= 0.0f)
        {
            return false;
        }
        vs[0].a = wBCD[0] / wBCD[3];
        vs[1].a = wBCD[1] / wBCD[3];
        vs[2].a = wBCD[2] / wBCD[3];
        return true;
    }

    if (wABCD[4] <= 0.0f)
    {
        return false;
    }
    vs[0].a = wABCD[0] / wABCD[4];
    vs[1].a = wABCD[1] / wABCD[4];
    vs[2].a = wABCD[2] / wABCD[4];
    vs[3].a = wABCD[3] / wABCD[4];
    return true;
}

static void WitnessPoints(const m3Simplex* simplex, m3Vec3* pointA, m3Vec3* pointB)
{
    const m3SimplexVertex* vs = simplex->vertices;
    switch (simplex->count)
    {
    case 1:
        *pointA = vs[0].wA;
        *pointB = vs[0].wB;
        break;
    case 2:
        *pointA = Blend2(vs[0].a, vs[0].wA, vs[1].a, vs[1].wA);
        *pointB = Blend2(vs[0].a, vs[0].wB, vs[1].a, vs[1].wB);
        break;
    case 3:
        *pointA = Blend3(vs[0].a, vs[0].wA, vs[1].a, vs[1].wA, vs[2].a, vs[2].wA);
        *pointB = Blend3(vs[0].a, vs[0].wB, vs[1].a, vs[1].wB, vs[2].a, vs[2].wB);
        break;
    case 4:
    {
        // Overlap: force identical points, zero distance.
        m3Vec3 sum = m3Add3(Blend2(vs[0].a, vs[0].wA, vs[1].a, vs[1].wA),
                            Blend2(vs[2].a, vs[2].wA, vs[3].a, vs[3].wA));
        *pointA = sum;
        *pointB = sum;
        break;
    }
    default:
        *pointA = (m3Vec3){0.0f, 0.0f, 0.0f};
        *pointB = (m3Vec3){0.0f, 0.0f, 0.0f};
        break;
    }
}

m3DistanceOutput m3ShapeDistance(const m3DistanceInput* input, m3SimplexCache* cache)
{
    m3DistanceOutput output;
    memset(&output, 0, sizeof(output));

    const m3DistanceProxy* proxyA = &input->proxyA;
    const m3DistanceProxy* proxyB = &input->proxyB;

    // Seed the simplex from the cache; flush it when the metric moved
    // substantially (the reference heuristic).
    m3Simplex simplex;
    memset(&simplex, 0, sizeof(simplex));
    m3SimplexVertex* vs = simplex.vertices;
    simplex.count = cache->count <= 4 ? (int32_t)cache->count : 0;
    for (int32_t i = 0; i < simplex.count; ++i)
    {
        int32_t indexA = cache->indexA[i];
        int32_t indexB = cache->indexB[i];
        if (indexA >= proxyA->count || indexB >= proxyB->count)
        {
            simplex.count = 0;
            break;
        }
        vs[i].indexA = indexA;
        vs[i].indexB = indexB;
        vs[i].wA = proxyA->points[indexA];
        vs[i].wB = m3Add3(m3RotateVec3(input->q, proxyB->points[indexB]), input->p);
        vs[i].w = m3Sub3(vs[i].wB, vs[i].wA);
        vs[i].a = 0.0f;
    }
    if (simplex.count > 0)
    {
        m3real metric1 = cache->metric;
        m3real metric2 = GetMetric(&simplex);
        if (2.0f * metric1 < metric2 || metric2 < 0.5f * metric1 || metric2 < FLT_EPSILON)
        {
            simplex.count = 0;
        }
    }
    if (simplex.count == 0)
    {
        simplex.count = 1;
        vs[0].indexA = 0;
        vs[0].indexB = 0;
        vs[0].wA = proxyA->points[0];
        vs[0].wB = m3Add3(m3RotateVec3(input->q, proxyB->points[0]), input->p);
        vs[0].w = m3Sub3(vs[0].wB, vs[0].wA);
        vs[0].a = 0.0f;
    }

    m3Simplex backup;
    memset(&backup, 0, sizeof(backup));
    m3real distanceSq = FLT_MAX;
    m3Vec3 normal = {0.0f, 0.0f, 0.0f};

    int32_t iteration = 0;
    for (; iteration < M3_MAX_GJK_ITERATIONS; ++iteration)
    {
        bool solved = false;
        switch (simplex.count)
        {
        case 1:
            vs[0].a = 1.0f;
            solved = true;
            break;
        case 2:
            solved = SolveSimplex2(&simplex);
            break;
        case 3:
            solved = SolveSimplex3(&simplex);
            break;
        case 4:
            solved = SolveSimplex4(&simplex);
            break;
        default:
            break;
        }
        if (!solved)
        {
            // No progress: reconstruct the last good simplex.
            if (backup.count == 0)
            {
                break;
            }
            simplex = backup;
            break;
        }

        if (simplex.count == 4)
        {
            // Overlap: the origin is inside the tetrahedron.
            WitnessPoints(&simplex, &output.pointA, &output.pointB);
            output.iterations = iteration;
            return output;
        }

        m3real oldDistanceSq = distanceSq;
        m3Vec3 closest;
        switch (simplex.count)
        {
        case 1:
            closest = vs[0].w;
            break;
        case 2:
            closest = Blend2(vs[0].a, vs[0].w, vs[1].a, vs[1].w);
            break;
        default:
            closest = Blend3(vs[0].a, vs[0].w, vs[1].a, vs[1].w, vs[2].a, vs[2].w);
            break;
        }
        distanceSq = m3Dot3(closest, closest);
        if (distanceSq >= oldDistanceSq)
        {
            if (backup.count != 0)
            {
                simplex = backup;
            }
            break;
        }

        // The next search direction.
        m3Vec3 search;
        switch (simplex.count)
        {
        case 1:
            search = m3Neg3(vs[0].w);
            break;
        case 2:
        {
            m3Vec3 ab = m3Sub3(vs[1].w, vs[0].w);
            search = m3Cross3(m3Cross3(ab, m3Neg3(vs[0].w)), ab);
            break;
        }
        default:
        {
            m3Vec3 ab = m3Sub3(vs[1].w, vs[0].w);
            m3Vec3 ac = m3Sub3(vs[2].w, vs[0].w);
            m3Vec3 n = m3Cross3(ab, ac);
            search = m3Dot3(n, vs[0].w) < 0.0f ? n : m3Neg3(n);
            break;
        }
        }
        if (m3Dot3(search, search) < 1000.0f * FLT_MIN)
        {
            // The origin sits on the simplex: overlap.
            WitnessPoints(&simplex, &output.pointA, &output.pointB);
            output.iterations = iteration;
            return output;
        }
        normal = m3Neg3(search);

        int32_t indexA = ProxySupport(proxyA, m3Neg3(search));
        m3Vec3 supportA = proxyA->points[indexA];
        int32_t indexB = ProxySupport(proxyB, m3InvRotateVec3(input->q, search));
        m3Vec3 supportB = m3Add3(m3RotateVec3(input->q, proxyB->points[indexB]), input->p);

        backup = simplex;

        // Duplicate support: the main termination criterion.
        bool duplicate = false;
        for (int32_t i = 0; i < simplex.count; ++i)
        {
            if (vs[i].indexA == indexA && vs[i].indexB == indexB)
            {
                duplicate = true;
                break;
            }
        }
        if (duplicate)
        {
            break;
        }
        vs[simplex.count].indexA = indexA;
        vs[simplex.count].indexB = indexB;
        vs[simplex.count].wA = supportA;
        vs[simplex.count].wB = supportB;
        vs[simplex.count].w = m3Sub3(supportB, supportA);
        simplex.count += 1;
    }

    normal = m3Normalize3(normal);
    m3Vec3 pointA;
    m3Vec3 pointB;
    WitnessPoints(&simplex, &pointA, &pointB);
    WriteCache(cache, &simplex);

    output.pointA = pointA;
    output.pointB = pointB;
    output.distance = m3Length3(m3Sub3(pointA, pointB));
    output.normal = normal;
    output.iterations = iteration;

    if (input->useRadii)
    {
        m3real rA = proxyA->radius;
        m3real rB = proxyB->radius;
        output.distance = m3MaxF(0.0f, output.distance - rA - rB);
        output.pointA = m3Add3(output.pointA, m3MulSV3(rA, normal));
        output.pointB = m3Sub3(output.pointB, m3MulSV3(rB, normal));
    }
    return output;
}
