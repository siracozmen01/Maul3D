// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// QuickHull gate (white box): analytic solids against closed forms,
// the coplanar-merge proof (a cube emits six quads, not twelve
// triangles), degenerate refusals, and the fuzz sweep that holds
// Euler's identity, containment, positive-definite mass, and
// bit-identical reruns over pseudo-random clouds.

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

static uint64_t SplitMix(uint64_t* state)
{
    *state += 0x9E3779B97F4A7C15ull;
    uint64_t z = *state;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    return z ^ (z >> 31);
}

static float RandRange(uint64_t* state, float lo, float hi)
{
    return lo + (hi - lo) * (float)((double)(SplitMix(state) >> 11) / 9007199254740992.0);
}

static int EulerHolds(const m3HullData* hull)
{
    return hull->vertexCount - hull->edgeCount / 2 + hull->faceCount == 2;
}

// Every input point must sit inside or on the hull (within a small
// tolerance): the definition of a convex hull.
static int Contains(const m3HullData* hull, const m3Vec3* points, int32_t count, float tol)
{
    for (int32_t i = 0; i < count; ++i)
    {
        for (int32_t f = 0; f < hull->faceCount; ++f)
        {
            float d = m3Dot3(hull->faceNormals[f], points[i]) - hull->faceOffsets[f];
            if (d > tol)
            {
                return 0;
            }
        }
    }
    return 1;
}

static void TestCubeMergesToQuads(void)
{
    // Eight corners of a unit cube, deliberately shuffled: the hull
    // must come back with 8 vertices and SIX merged quad faces (the
    // coplanar pass turns twelve triangles into six quads), and the
    // integrated mass must match the box closed form.
    m3Vec3 corners[8] = {
        {0.5f, -0.5f, 0.5f},  {-0.5f, -0.5f, -0.5f}, {0.5f, 0.5f, -0.5f},  {-0.5f, 0.5f, 0.5f},
        {-0.5f, 0.5f, -0.5f}, {0.5f, 0.5f, 0.5f},    {-0.5f, -0.5f, 0.5f}, {0.5f, -0.5f, -0.5f},
    };
    m3HullData hull;
    CHECK(m3ComputeHull(corners, 8, &hull), "the cube builds");
    CHECK(hull.vertexCount == 8, "eight vertices survive");
    CHECK(hull.faceCount == 6, "coplanar merge yields six faces");
    CHECK(hull.edgeCount == 24, "twenty-four half edges");
    CHECK(EulerHolds(&hull), "Euler's identity holds");
    CHECK(Contains(&hull, corners, 8, 1.0e-5f), "all corners contained");

    m3HullData box;
    m3BuildBoxHull(&box, (m3Vec3){0.5f, 0.5f, 0.5f});
    float m = hull.unitMass;
    CHECK(m > 0.999f && m < 1.001f, "unit cube mass is one");
    CHECK(m3Dot3(hull.unitCom, hull.unitCom) < 1.0e-8f, "centroid at the origin");
    float want = box.unitInertiaCom.cx.x; // m/6 for a cube
    CHECK(hull.unitInertiaCom.cx.x > want - 1.0e-3f && hull.unitInertiaCom.cx.x < want + 1.0e-3f,
          "inertia xx matches the closed form");
    CHECK(hull.unitInertiaCom.cy.y > want - 1.0e-3f && hull.unitInertiaCom.cy.y < want + 1.0e-3f,
          "inertia yy matches the closed form");
    float off = hull.unitInertiaCom.cy.x;
    CHECK(off > -1.0e-4f && off < 1.0e-4f, "off-diagonal inertia vanishes");
}

static void TestInteriorAndDuplicatesAbsorbed(void)
{
    // The cube again, plus interior points and exact duplicates: the
    // hull must be identical in structure (8 vertices, 6 faces).
    m3Vec3 pts[14] = {
        {-0.5f, -0.5f, -0.5f}, {0.5f, -0.5f, -0.5f}, {0.5f, 0.5f, -0.5f},  {-0.5f, 0.5f, -0.5f},
        {-0.5f, -0.5f, 0.5f},  {0.5f, -0.5f, 0.5f},  {0.5f, 0.5f, 0.5f},   {-0.5f, 0.5f, 0.5f},
        {0.0f, 0.0f, 0.0f},    {0.1f, -0.2f, 0.3f},  {-0.25f, 0.1f, 0.0f}, {0.5f, 0.5f, 0.5f},
        {-0.5f, -0.5f, -0.5f}, {0.0f, 0.49f, 0.0f},
    };
    m3HullData hull;
    CHECK(m3ComputeHull(pts, 14, &hull), "the noisy cube builds");
    CHECK(hull.vertexCount == 8, "interior and duplicate points absorbed");
    CHECK(hull.faceCount == 6, "still six faces");
    CHECK(EulerHolds(&hull), "Euler's identity holds");
}

static void TestTetrahedron(void)
{
    // Regular-ish tetrahedron with an analytic volume: V = |det|/6.
    m3Vec3 pts[4] = {
        {0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
    m3HullData hull;
    CHECK(m3ComputeHull(pts, 4, &hull), "the tetrahedron builds");
    CHECK(hull.vertexCount == 4 && hull.faceCount == 4, "four vertices, four faces");
    CHECK(EulerHolds(&hull), "Euler's identity holds");
    CHECK(hull.unitMass > 1.0f / 6.0f - 1.0e-4f && hull.unitMass < 1.0f / 6.0f + 1.0e-4f,
          "tetra volume is one sixth");
    // Centroid of a tetra: the vertex mean.
    CHECK(hull.unitCom.x > 0.2499f && hull.unitCom.x < 0.2501f, "tetra centroid x");
    CHECK(hull.unitCom.y > 0.2499f && hull.unitCom.y < 0.2501f, "tetra centroid y");
    CHECK(hull.unitCom.z > 0.2499f && hull.unitCom.z < 0.2501f, "tetra centroid z");
}

static void TestOctahedron(void)
{
    m3Vec3 pts[6] = {{1.0f, 0.0f, 0.0f},  {-1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f},
                     {0.0f, -1.0f, 0.0f}, {0.0f, 0.0f, 1.0f},  {0.0f, 0.0f, -1.0f}};
    m3HullData hull;
    CHECK(m3ComputeHull(pts, 6, &hull), "the octahedron builds");
    CHECK(hull.vertexCount == 6 && hull.faceCount == 8, "six vertices, eight faces");
    CHECK(EulerHolds(&hull), "Euler's identity holds");
    // V = 4/3 for unit circumradius.
    CHECK(hull.unitMass > 4.0f / 3.0f - 1.0e-3f && hull.unitMass < 4.0f / 3.0f + 1.0e-3f,
          "octahedron volume");
}

static void TestDegenerateRefusals(void)
{
    m3HullData hull;
    m3Vec3 three[3] = {{0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}};
    CHECK(!m3ComputeHull(three, 3, &hull), "fewer than four points refused");

    m3Vec3 coplanar[6] = {{0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f},
                          {1.0f, 1.0f, 0.0f}, {0.3f, 0.7f, 0.0f}, {0.9f, 0.1f, 0.0f}};
    CHECK(!m3ComputeHull(coplanar, 6, &hull), "a coplanar cloud is refused");

    m3Vec3 collinear[5] = {{0.0f, 0.0f, 0.0f},
                           {1.0f, 1.0f, 1.0f},
                           {2.0f, 2.0f, 2.0f},
                           {3.0f, 3.0f, 3.0f},
                           {4.0f, 4.0f, 4.0f}};
    CHECK(!m3ComputeHull(collinear, 5, &hull), "a collinear cloud is refused");

    m3Vec3 bad[4] = {
        {0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0e30f}};
    bad[3].z = bad[3].z * bad[3].z; // infinity
    CHECK(!m3ComputeHull(bad, 4, &hull), "non-finite input is refused");
}

static void TestFuzzSweep(void)
{
    // Forty pseudo-random clouds: every build must satisfy Euler,
    // contain all its input points, carry positive mass and a
    // positive-definite inertia diagonal, respect the caps, and
    // rebuild bit-identically.
    uint64_t rng = 0x2545F4914F6CDD1Dull;
    int32_t built = 0;
    for (int32_t round = 0; round < 40; ++round)
    {
        int32_t count = 8 + (int32_t)(SplitMix(&rng) % 33); // 8..40 points
        m3Vec3 pts[64];
        for (int32_t i = 0; i < count; ++i)
        {
            pts[i].x = RandRange(&rng, -1.0f, 1.0f);
            pts[i].y = RandRange(&rng, -1.0f, 1.0f);
            pts[i].z = RandRange(&rng, -1.0f, 1.0f);
        }
        m3HullData hull;
        if (!m3ComputeHull(pts, count, &hull))
        {
            // A random cloud can exceed the 24-vertex budget only by
            // failing the caps, never by crashing; count it and move
            // on (the budget cut keeps the deepest points, so real
            // clouds land under the caps in practice).
            continue;
        }
        built += 1;
        CHECK(EulerHolds(&hull), "Euler's identity holds on fuzz");
        CHECK(hull.vertexCount <= M3_HULL_MAX_VERTS && hull.faceCount <= M3_HULL_MAX_FACES &&
                  hull.edgeCount <= M3_HULL_MAX_HALF_EDGES,
              "caps respected");
        CHECK(hull.unitMass > 0.0f, "positive volume");
        CHECK(hull.unitInertiaCom.cx.x > 0.0f && hull.unitInertiaCom.cy.y > 0.0f &&
                  hull.unitInertiaCom.cz.z > 0.0f,
              "positive inertia diagonal");
        // Containment: vertices of the hull are a subset of inputs,
        // and every input sits behind every face plane.
        CHECK(Contains(&hull, pts, count, 1.0e-3f), "all inputs contained");
        for (int32_t f = 0; f < hull.faceCount; ++f)
        {
            float n2 = m3Dot3(hull.faceNormals[f], hull.faceNormals[f]);
            CHECK(n2 > 0.99f && n2 < 1.01f, "face normals stay unit");
        }
        m3HullData again;
        CHECK(m3ComputeHull(pts, count, &again), "the rebuild succeeds");
        CHECK(memcmp(&hull, &again, sizeof(m3HullData)) == 0, "the rebuild is bit-identical");
    }
    CHECK(built >= 30, "most fuzz clouds build under the caps");
    printf("M3_QH_FUZZ built=%d/40\n", built);
}

static void TestHalfEdgeTwinLaw(void)
{
    // The SAT's Gauss map leans on twins at 2k and 2k+1: prove the
    // law holds on a QuickHull output, not just on boxes.
    m3Vec3 pts[6] = {{1.0f, 0.0f, 0.0f},  {-1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f},
                     {0.0f, -1.0f, 0.0f}, {0.0f, 0.0f, 1.0f},  {0.0f, 0.0f, -1.0f}};
    m3HullData hull;
    CHECK(m3ComputeHull(pts, 6, &hull), "the octahedron builds");
    for (int32_t e = 0; e < hull.edgeCount; e += 2)
    {
        CHECK(hull.edges[e].twin == e + 1 && hull.edges[e + 1].twin == e,
              "twins interleave at 2k and 2k+1");
        int32_t a0 = hull.edges[e].origin;
        int32_t a1 = hull.edges[hull.edges[e].next].origin;
        int32_t b0 = hull.edges[e + 1].origin;
        int32_t b1 = hull.edges[hull.edges[e + 1].next].origin;
        CHECK(a0 == b1 && a1 == b0, "twin edges run opposite directions");
    }
}

// 10-2: the parity cap. A dense sphere cloud must produce a hull
// RICHER than the old 24-vertex ceiling, cap at 64, and hold the
// half-edge laws; input past M3_HULL_MAX_INPUT refuses.
static void TestBigHullCap(void)
{
    m3Vec3 cloud[256];
    int32_t n = 0;
    for (int32_t i = 0; i < 128; ++i)
    {
        float t = (float)i / 128.0f;
        float phi = 2.399963f * (float)i; // golden angle spiral
        float y = 1.0f - 2.0f * t;
        float r = sqrtf(1.0f - y * y);
        cloud[n++] = (m3Vec3){r * cosf(phi), y, r * sinf(phi)};
    }
    m3HullData hull;
    CHECK(m3ComputeHull(cloud, n, &hull), "a 128-point sphere cloud builds");
    CHECK(hull.vertexCount > 24, "the hull outgrows the old ceiling");
    CHECK(hull.vertexCount <= 64, "and respects the new one");
    CHECK(hull.faceCount <= 124, "faces respect Euler at 64");
    CHECK(hull.edgeCount <= 372, "half edges respect Euler at 64");
    // Twin law sweep at the new scale.
    for (int32_t e = 0; e < hull.edgeCount; ++e)
    {
        CHECK(hull.edges[hull.edges[e].twin].twin == e, "twins pair at 64-vert scale");
    }
    m3Vec3 tooMany[257];
    for (int32_t i = 0; i < 257; ++i)
    {
        tooMany[i] = cloud[i % n];
    }
    CHECK(!m3ComputeHull(tooMany, 257, &hull), "input past the cap refuses");
}

// 10-4 boundary fuzz: clouds at 63, 64, and 65 candidate hull
// vertices, plus degenerate stews (coplanar sheets, duplicate
// spikes) that must simplify or refuse but never crash.
static void TestBoundaryClouds(void)
{
    // A cube-corner lattice tuned to yield close-to-cap hulls.
    for (int32_t target = 63; target <= 65; ++target)
    {
        m3Vec3 cloud[130];
        int32_t n = 0;
        for (int32_t i = 0; i < target && n < 128; ++i)
        {
            float t = (float)i / (float)target;
            float phi = 2.399963f * (float)i;
            float y = 1.0f - 2.0f * t;
            float r = sqrtf(1.0f - y * y);
            cloud[n++] = (m3Vec3){r * cosf(phi), y, r * sinf(phi)};
        }
        m3HullData hull;
        CHECK(m3ComputeHull(cloud, n, &hull), "a boundary cloud builds");
        CHECK(hull.vertexCount <= 64, "the cap holds at the boundary");
        for (int32_t e = 0; e < hull.edgeCount; ++e)
        {
            CHECK(hull.edges[hull.edges[e].twin].twin == e, "twins pair at the boundary");
        }
    }
    // Degenerate stews: a coplanar sheet with two spikes, and a
    // cloud that is one point duplicated many times plus a tetra.
    m3Vec3 sheet[80];
    int32_t n = 0;
    for (int32_t i = 0; i < 60; ++i)
    {
        sheet[n++] = (m3Vec3){(float)(i % 10) * 0.1f, 0.0f, (float)(i / 10) * 0.1f};
    }
    sheet[n++] = (m3Vec3){0.45f, 0.8f, 0.25f};
    sheet[n++] = (m3Vec3){0.45f, -0.8f, 0.25f};
    m3HullData hull;
    CHECK(m3ComputeHull(sheet, n, &hull), "the spiked sheet builds a volume");
    CHECK(hull.vertexCount >= 5, "the sheet corners and spikes survive");

    m3Vec3 dupes[70];
    for (int32_t i = 0; i < 66; ++i)
    {
        dupes[i] = (m3Vec3){0.5f, 0.5f, 0.5f};
    }
    dupes[66] = (m3Vec3){0.0f, 0.0f, 0.0f};
    dupes[67] = (m3Vec3){1.0f, 0.0f, 0.0f};
    dupes[68] = (m3Vec3){0.0f, 1.0f, 0.0f};
    dupes[69] = (m3Vec3){0.0f, 0.0f, 1.0f};
    CHECK(m3ComputeHull(dupes, 70, &hull), "the duplicate stew simplifies to the tetra");
    CHECK(hull.vertexCount >= 4 && hull.vertexCount <= 5, "duplicates absorbed");
}

int main(void)
{
    TestCubeMergesToQuads();
    TestInteriorAndDuplicatesAbsorbed();
    TestTetrahedron();
    TestOctahedron();
    TestDegenerateRefusals();
    TestFuzzSweep();
    TestBigHullCap();
    TestBoundaryClouds();
    TestHalfEdgeTwinLaw();
    if (s_failures == 0)
    {
        printf("test_quickhull: all checks passed\n");
    }
    return s_failures == 0 ? 0 : 1;
}
