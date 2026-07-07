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

// ---------------------------------------------------------------
// Time of impact (2b-8), adapted from the reference distance.c.
// ---------------------------------------------------------------

m3Transform m3GetSweepTransform(const m3Sweep* sweep, m3real time)
{
    m3Transform transform;
    // NLerp with hemisphere correction: step rotations are small, but
    // the guard costs one compare and removes a whole failure class.
    m3Quat q2 = sweep->q2;
    if (sweep->q1.x * q2.x + sweep->q1.y * q2.y + sweep->q1.z * q2.z + sweep->q1.w * q2.w < 0.0f)
    {
        q2 = (m3Quat){-q2.x, -q2.y, -q2.z, -q2.w};
    }
    m3Quat q = {
        sweep->q1.x + time * (q2.x - sweep->q1.x), sweep->q1.y + time * (q2.y - sweep->q1.y),
        sweep->q1.z + time * (q2.z - sweep->q1.z), sweep->q1.w + time * (q2.w - sweep->q1.w)};
    transform.q = m3NormalizeQuat(q);
    m3Vec3 c = m3Add3(sweep->c1, m3MulSV3(time, m3Sub3(sweep->c2, sweep->c1)));
    m3Vec3 r = m3RotateVec3(transform.q, sweep->localCenter);
    transform.p.x = (double)(c.x - r.x);
    transform.p.y = (double)(c.y - r.y);
    transform.p.z = (double)(c.z - r.z);
    return transform;
}

// The TOI kernel runs in floats; this lifts the double transform back
// to the float relative frame the sweeps live in.
static m3Vec3 ToiTransformPoint(const m3Transform* xf, m3Vec3 p)
{
    m3Vec3 r = m3RotateVec3(xf->q, p);
    return (m3Vec3){(m3real)xf->p.x + r.x, (m3real)xf->p.y + r.y, (m3real)xf->p.z + r.z};
}

static int32_t ToiUniqueCount(int32_t vertexCount, const int32_t vertices[3])
{
    if (vertexCount == 1)
    {
        return 1;
    }
    if (vertexCount == 2)
    {
        return vertices[0] != vertices[1] ? 2 : 1;
    }
    if (vertices[0] != vertices[1] && vertices[0] != vertices[2] && vertices[1] != vertices[2])
    {
        return 3;
    }
    if (vertices[0] == vertices[1] && vertices[0] == vertices[2])
    {
        return 1;
    }
    return 2;
}

static int32_t ToiSupport(const m3DistanceProxy* proxy, m3Vec3 direction)
{
    int32_t best = 0;
    m3real bestDot = m3Dot3(proxy->points[0], direction);
    for (int32_t i = 1; i < proxy->count; ++i)
    {
        m3real d = m3Dot3(proxy->points[i], direction);
        if (d > bestDot)
        {
            bestDot = d;
            best = i;
        }
    }
    return best;
}

// Does the edge cross product flip direction across the sweep? If so
// the local-edge separation axis is unsafe (reference check).
static int ToiCheckFastEdges(const m3Sweep* sweepA, const m3Sweep* sweepB, m3Vec3 localEdgeA,
                             m3Vec3 localEdgeB, m3Vec3 axis0)
{
    m3Transform xfA = m3GetSweepTransform(sweepA, 1.0f);
    m3Transform xfB = m3GetSweepTransform(sweepB, 1.0f);
    m3Vec3 edgeA = m3RotateVec3(xfA.q, localEdgeA);
    m3Vec3 edgeB = m3RotateVec3(xfB.q, localEdgeB);
    m3Vec3 axis = m3Cross3(edgeA, edgeB);
    return m3Dot3(axis, axis0) < 0.0f;
}

typedef enum m3SeparationType
{
    m3_separationVertices,
    m3_separationEdges,
    m3_separationFaceA,
    m3_separationFaceB,
} m3SeparationType;

typedef struct m3SeparationFunction
{
    const m3DistanceProxy* proxyA;
    const m3DistanceProxy* proxyB;
    const m3Sweep* sweepA;
    const m3Sweep* sweepB;
    m3Vec3 witness1;
    m3Vec3 witness2;
    m3SeparationType type;
} m3SeparationFunction;

static m3SeparationFunction
ToiMakeSeparationFunction(const m3SimplexCache* cache, const m3DistanceProxy* proxyA,
                          const m3Sweep* sweepA, const m3DistanceProxy* proxyB,
                          const m3Sweep* sweepB, m3Vec3 worldNormal, m3real t1)
{
    m3SeparationFunction fcn;
    fcn.proxyA = proxyA;
    fcn.proxyB = proxyB;
    fcn.sweepA = sweepA;
    fcn.sweepB = sweepB;
    fcn.witness1 = worldNormal;
    fcn.witness2 = (m3Vec3){0.0f, 0.0f, 0.0f};
    fcn.type = m3_separationVertices;

    int32_t indexA[3] = {cache->indexA[0], cache->indexA[1], cache->indexA[2]};
    int32_t indexB[3] = {cache->indexB[0], cache->indexB[1], cache->indexB[2]};
    int32_t count = cache->count < 3 ? (int32_t)cache->count : 3;
    int32_t uniqueA = ToiUniqueCount(count, indexA);
    int32_t uniqueB = ToiUniqueCount(count, indexB);

    m3Transform xfA1 = m3GetSweepTransform(sweepA, t1);
    m3Transform xfB1 = m3GetSweepTransform(sweepB, t1);
    m3Vec3 deltaP = {(m3real)(xfB1.p.x - xfA1.p.x), (m3real)(xfB1.p.y - xfA1.p.y),
                     (m3real)(xfB1.p.z - xfA1.p.z)};

    if (count == 2 || (count == 3 && uniqueA <= 2 && uniqueB <= 2))
    {
        // Edge versus edge when both sides bring two unique vertices;
        // anything else keeps the world-axis witness.
        int32_t ia[2];
        int32_t ib[2];
        if (count == 2 && uniqueA == 2 && uniqueB == 2)
        {
            ia[0] = indexA[0];
            ia[1] = indexA[1];
            ib[0] = indexB[0];
            ib[1] = indexB[1];
        }
        else if (count == 3 && uniqueA == 2 && uniqueB == 2)
        {
            ia[0] = indexA[0];
            ia[1] = indexA[0] != indexA[1] ? indexA[1] : indexA[2];
            ib[0] = indexB[0];
            ib[1] = indexB[0] != indexB[1] ? indexB[1] : indexB[2];
        }
        else
        {
            return fcn; // vertex versus edge: world axis
        }
        m3Vec3 vA1 = proxyA->points[ia[0]];
        m3Vec3 localEdgeA = m3Normalize3(m3Sub3(proxyA->points[ia[1]], vA1));
        m3Vec3 edgeA = m3RotateVec3(xfA1.q, localEdgeA);
        m3Vec3 vB1 = proxyB->points[ib[0]];
        m3Vec3 localEdgeB = m3Normalize3(m3Sub3(proxyB->points[ib[1]], vB1));
        m3Vec3 edgeB = m3RotateVec3(xfB1.q, localEdgeB);
        m3Vec3 axis = m3Cross3(edgeA, edgeB);
        m3real len2 = m3Dot3(axis, axis);
        m3real tolerance2 = count == 2 ? 0.05f * 0.05f : 0.005f * 0.005f;
        if (len2 < tolerance2)
        {
            return fcn; // near parallel: world axis
        }
        m3Vec3 delta = m3Add3(m3Sub3(m3RotateVec3(xfB1.q, vB1), m3RotateVec3(xfA1.q, vA1)), deltaP);
        if (m3Dot3(delta, axis) < 0.0f)
        {
            axis = m3Neg3(axis);
            localEdgeB = m3Neg3(localEdgeB);
        }
        if (ToiCheckFastEdges(sweepA, sweepB, localEdgeA, localEdgeB, axis))
        {
            fcn.witness1 = m3Normalize3(axis); // unsafe: fixed world axis
            return fcn;
        }
        fcn.type = m3_separationEdges;
        fcn.witness1 = localEdgeA;
        fcn.witness2 = localEdgeB;
        return fcn;
    }

    if (count == 3 && uniqueA == 3)
    {
        m3Vec3 vA1 = proxyA->points[indexA[0]];
        m3Vec3 vA2 = proxyA->points[indexA[1]];
        m3Vec3 vA3 = proxyA->points[indexA[2]];
        m3Vec3 localAxisA = m3Normalize3(m3Cross3(m3Sub3(vA2, vA1), m3Sub3(vA3, vA1)));
        m3Vec3 axisA = m3RotateVec3(xfA1.q, localAxisA);
        m3Vec3 localPointA = m3MulSV3(1.0f / 3.0f, m3Add3(m3Add3(vA1, vA2), vA3));
        m3Vec3 localPointB = proxyB->points[indexB[0]];
        m3Vec3 delta = m3Add3(
            m3Sub3(m3RotateVec3(xfB1.q, localPointB), m3RotateVec3(xfA1.q, localPointA)), deltaP);
        if (m3Dot3(delta, axisA) < 0.0f)
        {
            localAxisA = m3Neg3(localAxisA);
        }
        fcn.type = m3_separationFaceA;
        fcn.witness1 = localAxisA;
        fcn.witness2 = localPointA;
        return fcn;
    }
    if (count == 3 && uniqueB == 3)
    {
        m3Vec3 vB1 = proxyB->points[indexB[0]];
        m3Vec3 vB2 = proxyB->points[indexB[1]];
        m3Vec3 vB3 = proxyB->points[indexB[2]];
        m3Vec3 localAxisB = m3Normalize3(m3Cross3(m3Sub3(vB2, vB1), m3Sub3(vB3, vB1)));
        m3Vec3 axisB = m3RotateVec3(xfB1.q, localAxisB);
        m3Vec3 localPointA = proxyA->points[indexA[0]];
        m3Vec3 localPointB = m3MulSV3(1.0f / 3.0f, m3Add3(m3Add3(vB1, vB2), vB3));
        m3Vec3 delta = m3Sub3(
            m3Sub3(m3RotateVec3(xfA1.q, localPointA), m3RotateVec3(xfB1.q, localPointB)), deltaP);
        if (m3Dot3(delta, axisB) < 0.0f)
        {
            localAxisB = m3Neg3(localAxisB);
        }
        fcn.type = m3_separationFaceB;
        fcn.witness1 = localAxisB;
        fcn.witness2 = localPointB;
        return fcn;
    }
    return fcn; // count == 1 or mixed: world axis witness
}

static m3real ToiFindMinSeparation(const m3SeparationFunction* fcn, int32_t* indexA,
                                   int32_t* indexB, m3real t)
{
    m3Transform xfA = m3GetSweepTransform(fcn->sweepA, t);
    m3Transform xfB = m3GetSweepTransform(fcn->sweepB, t);
    m3Vec3 deltaP = {(m3real)(xfB.p.x - xfA.p.x), (m3real)(xfB.p.y - xfA.p.y),
                     (m3real)(xfB.p.z - xfA.p.z)};

    switch (fcn->type)
    {
    case m3_separationVertices:
    {
        m3Vec3 axis = fcn->witness1;
        *indexA = ToiSupport(fcn->proxyA, m3InvRotateVec3(xfA.q, axis));
        *indexB = ToiSupport(fcn->proxyB, m3InvRotateVec3(xfB.q, m3Neg3(axis)));
        m3Vec3 delta = m3Add3(m3Sub3(m3RotateVec3(xfB.q, fcn->proxyB->points[*indexB]),
                                     m3RotateVec3(xfA.q, fcn->proxyA->points[*indexA])),
                              deltaP);
        return m3Dot3(delta, axis);
    }
    case m3_separationEdges:
    {
        m3Vec3 edgeA = m3RotateVec3(xfA.q, fcn->witness1);
        m3Vec3 edgeB = m3RotateVec3(xfB.q, fcn->witness2);
        m3Vec3 axis = m3Normalize3(m3Cross3(edgeA, edgeB));
        *indexA = ToiSupport(fcn->proxyA, m3InvRotateVec3(xfA.q, axis));
        *indexB = ToiSupport(fcn->proxyB, m3InvRotateVec3(xfB.q, m3Neg3(axis)));
        m3Vec3 delta = m3Add3(m3Sub3(m3RotateVec3(xfB.q, fcn->proxyB->points[*indexB]),
                                     m3RotateVec3(xfA.q, fcn->proxyA->points[*indexA])),
                              deltaP);
        return m3Dot3(delta, axis);
    }
    case m3_separationFaceA:
    {
        m3Vec3 normal = m3RotateVec3(xfA.q, fcn->witness1);
        *indexA = -1;
        m3Vec3 pointA = ToiTransformPoint(&xfA, fcn->witness2);
        *indexB = ToiSupport(fcn->proxyB, m3InvRotateVec3(xfB.q, m3Neg3(normal)));
        m3Vec3 pointB = ToiTransformPoint(&xfB, fcn->proxyB->points[*indexB]);
        return m3Dot3(m3Sub3(pointB, pointA), normal);
    }
    case m3_separationFaceB:
    default:
    {
        m3Vec3 normal = m3RotateVec3(xfB.q, fcn->witness1);
        *indexA = ToiSupport(fcn->proxyA, m3InvRotateVec3(xfA.q, m3Neg3(normal)));
        m3Vec3 pointA = ToiTransformPoint(&xfA, fcn->proxyA->points[*indexA]);
        *indexB = -1;
        m3Vec3 pointB = ToiTransformPoint(&xfB, fcn->witness2);
        return m3Dot3(m3Sub3(pointA, pointB), normal);
    }
    }
}

static m3real ToiEvaluateSeparation(const m3SeparationFunction* fcn, int32_t indexA, int32_t indexB,
                                    m3real t)
{
    m3Transform xfA = m3GetSweepTransform(fcn->sweepA, t);
    m3Transform xfB = m3GetSweepTransform(fcn->sweepB, t);

    switch (fcn->type)
    {
    case m3_separationVertices:
    {
        m3Vec3 pointA = ToiTransformPoint(&xfA, fcn->proxyA->points[indexA]);
        m3Vec3 pointB = ToiTransformPoint(&xfB, fcn->proxyB->points[indexB]);
        return m3Dot3(m3Sub3(pointB, pointA), fcn->witness1);
    }
    case m3_separationEdges:
    {
        m3Vec3 edgeA = m3RotateVec3(xfA.q, fcn->witness1);
        m3Vec3 edgeB = m3RotateVec3(xfB.q, fcn->witness2);
        m3Vec3 axis = m3Normalize3(m3Cross3(edgeA, edgeB));
        m3Vec3 pointA = ToiTransformPoint(&xfA, fcn->proxyA->points[indexA]);
        m3Vec3 pointB = ToiTransformPoint(&xfB, fcn->proxyB->points[indexB]);
        return m3Dot3(m3Sub3(pointB, pointA), axis);
    }
    case m3_separationFaceA:
    {
        m3Vec3 axis = m3RotateVec3(xfA.q, fcn->witness1);
        m3Vec3 pointA = ToiTransformPoint(&xfA, fcn->witness2);
        m3Vec3 pointB = ToiTransformPoint(&xfB, fcn->proxyB->points[indexB]);
        return m3Dot3(m3Sub3(pointB, pointA), axis);
    }
    case m3_separationFaceB:
    default:
    {
        m3Vec3 axis = m3RotateVec3(xfB.q, fcn->witness1);
        m3Vec3 pointA = ToiTransformPoint(&xfA, fcn->proxyA->points[indexA]);
        m3Vec3 pointB = ToiTransformPoint(&xfB, fcn->witness2);
        return m3Dot3(m3Sub3(pointA, pointB), axis);
    }
    }
}

static void ToiForceFixedAxis(m3SeparationFunction* fcn, m3real t)
{
    m3Transform xfA = m3GetSweepTransform(fcn->sweepA, t);
    m3Transform xfB = m3GetSweepTransform(fcn->sweepB, t);
    m3Vec3 edgeA = m3RotateVec3(xfA.q, fcn->witness1);
    m3Vec3 edgeB = m3RotateVec3(xfB.q, fcn->witness2);
    fcn->witness1 = m3Normalize3(m3Cross3(edgeA, edgeB));
    fcn->witness2 = (m3Vec3){0.0f, 0.0f, 0.0f};
    fcn->type = m3_separationVertices;
}

m3TOIOutput m3TimeOfImpact(const m3TOIInput* input)
{
    m3TOIOutput output;
    output.state = m3_toiStateUnknown;
    output.fraction = input->maxFraction;

    m3Sweep sweepA = input->sweepA;
    m3Sweep sweepB = input->sweepB;

    // Shift both sweeps so A starts at the origin (float hygiene).
    m3Vec3 origin = sweepA.c1;
    sweepA.c1 = m3Sub3(sweepA.c1, origin);
    sweepA.c2 = m3Sub3(sweepA.c2, origin);
    sweepB.c1 = m3Sub3(sweepB.c1, origin);
    sweepB.c2 = m3Sub3(sweepB.c2, origin);

    const m3real linearSlop = 0.005f;
    m3real totalRadius = input->proxyA.radius + input->proxyB.radius;
    m3real target = m3MaxF(linearSlop, totalRadius - linearSlop);
    m3real tolerance = 0.25f * linearSlop;
    m3real tMax = input->maxFraction;

    int32_t maxPushBackIterations = input->proxyA.count + input->proxyB.count;
    m3real t1 = 0.0f;
    const int32_t maxIterations = 25;
    int32_t distanceIterations = 0;

    m3SimplexCache cache;
    cache.count = 0;
    cache.metric = 0.0f;
    m3DistanceInput distanceInput;
    distanceInput.proxyA = input->proxyA;
    distanceInput.proxyB = input->proxyB;
    distanceInput.useRadii = false;

    for (;;)
    {
        m3Transform xfA = m3GetSweepTransform(&sweepA, t1);
        m3Transform xfB = m3GetSweepTransform(&sweepB, t1);
        // Relative pose of B in A's frame (the GJK contract).
        m3Quat conjA = {-xfA.q.x, -xfA.q.y, -xfA.q.z, xfA.q.w};
        distanceInput.q = m3MulQuat(conjA, xfB.q);
        m3Vec3 dp = {(m3real)(xfB.p.x - xfA.p.x), (m3real)(xfB.p.y - xfA.p.y),
                     (m3real)(xfB.p.z - xfA.p.z)};
        distanceInput.p = m3InvRotateVec3(xfA.q, dp);
        m3DistanceOutput distanceOutput = m3ShapeDistance(&distanceInput, &cache);

        distanceIterations += 1;

        if (distanceOutput.distance <= 0.0f)
        {
            // Started overlapped: continuous gives up, the discrete
            // deep-recovery kernels (2b-7) own this case.
            output.state = m3_toiStateOverlapped;
            output.fraction = 0.0f;
            break;
        }
        if (distanceOutput.distance <= target + tolerance)
        {
            output.state = m3_toiStateHit;
            output.fraction = t1;
            break;
        }
        if (distanceIterations == maxIterations)
        {
            output.state = m3_toiStateFailed;
            output.fraction = t1;
            break;
        }

        m3Vec3 worldNormal = m3RotateVec3(xfA.q, distanceOutput.normal);
        m3SeparationFunction function = ToiMakeSeparationFunction(
            &cache, &input->proxyA, &sweepA, &input->proxyB, &sweepB, worldNormal, t1);

        int done = 0;
        m3real t2 = tMax;
        int32_t pushBackIterations = 0;
        for (;;)
        {
            int32_t indexA;
            int32_t indexB;
            m3real s2 = ToiFindMinSeparation(&function, &indexA, &indexB, t2);
            if (s2 - target > tolerance)
            {
                output.state = m3_toiStateSeparated;
                output.fraction = tMax;
                done = 1;
                break;
            }
            if (s2 >= target - tolerance)
            {
                t1 = t2; // advance the sweeps
                break;
            }
            m3real s1 = ToiEvaluateSeparation(&function, indexA, indexB, t1);
            if (s1 < target - tolerance)
            {
                output.state = m3_toiStateFailed;
                output.fraction = t1;
                done = 1;
                break;
            }
            if (s1 <= target + tolerance)
            {
                output.state = m3_toiStateHit;
                output.fraction = t1;
                done = 1;
                break;
            }

            // Root of f(t) - target = 0: alternate false position
            // with bisection (the reference mix).
            int32_t rootIterationCount = 0;
            const int32_t maxRootIterations = 50;
            m3real a1 = t1;
            m3real a2 = t2;
            for (;;)
            {
                m3real t;
                if (rootIterationCount & 1)
                {
                    t = a1 + (target - s1) * (a2 - a1) / (s2 - s1);
                }
                else
                {
                    t = 0.5f * (a1 + a2);
                }
                rootIterationCount += 1;
                m3real s = ToiEvaluateSeparation(&function, indexA, indexB, t);
                m3real err = s - target;
                if ((err < 0.0f ? -err : err) <= tolerance)
                {
                    t2 = t;
                    break;
                }
                if (s > target)
                {
                    a1 = t;
                    s1 = s;
                }
                else
                {
                    a2 = t;
                    s2 = s;
                }
                if (rootIterationCount == maxRootIterations)
                {
                    break;
                }
            }

            if (rootIterationCount == maxRootIterations - 1 && function.type == m3_separationEdges)
            {
                // Failing edge case: pin the axis and restart.
                rootIterationCount = 0;
                t2 = tMax;
                ToiForceFixedAxis(&function, t1);
            }

            pushBackIterations += 1;
            if (pushBackIterations == maxPushBackIterations)
            {
                break;
            }
        }

        if (done)
        {
            break;
        }
    }
    return output;
}
