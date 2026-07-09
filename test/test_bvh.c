// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Mesh BVH gate (2c-10): the brute-force referee the plan demands.
// The tree is a pruning layer, nothing more: for any query box the
// gathered set, filtered by the exact per-triangle test, must equal
// the full scan filtered by the same test. Plus build determinism
// (twin builds byte-identical), restore rebuild (a rolled-back world
// carries a byte-identical tree), and the degenerate boxes.

#include "world_internal.h"

#include <stdio.h>
#include <stdlib.h>
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

// Deterministic LCG (never libc rand: implementation-defined).
static uint32_t s_rng = 0x12345678u;
static uint32_t NextRand(void)
{
    s_rng = s_rng * 1664525u + 1013904223u;
    return s_rng;
}
static float RandRange(float lo, float hi)
{
    return lo + (hi - lo) * ((float)(NextRand() >> 8) / 16777216.0f);
}

// A bumpy heightfield: 1922 triangles, the realistic midphase load.
static m3WorldId MakeMeshWorld(m3ShapeId* meshShapeOut)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    def.meshCapacity = 2;
    m3WorldId world = m3CreateWorld(&def);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    float heights[32 * 32];
    for (int32_t z = 0; z < 32; ++z)
    {
        for (int32_t x = 0; x < 32; ++x)
        {
            // A deterministic bumpscape (no libm beyond the allowed).
            heights[z * 32 + x] = 0.25f * (float)((x * 7 + z * 13) % 5) - 0.5f;
        }
    }
    m3ShapeId meshShape = m3CreateHeightFieldShape(ground, &sd, heights, 32, 32, 1.0f);
    if (meshShapeOut != NULL)
    {
        *meshShapeOut = meshShape;
    }
    return world;
}

static const m3MeshData* MeshOf(m3WorldId worldId, m3ShapeId shape)
{
    m3World* world = m3WorldFromId(worldId);
    return &world->meshData[world->shapeMeshIndex[shape.index1 - 1]];
}

static const m3MeshBvh* BvhOf(m3WorldId worldId, m3ShapeId shape)
{
    m3World* world = m3WorldFromId(worldId);
    return &world->meshBvh[world->shapeMeshIndex[shape.index1 - 1]];
}

static void TriBox(const m3MeshData* mesh, int32_t t, m3Vec3* lo, m3Vec3* hi)
{
    m3Vec3 a = mesh->vertices[mesh->indices[3 * t + 0]];
    m3Vec3 b = mesh->vertices[mesh->indices[3 * t + 1]];
    m3Vec3 c = mesh->vertices[mesh->indices[3 * t + 2]];
    *lo = (m3Vec3){m3MinF(a.x, m3MinF(b.x, c.x)), m3MinF(a.y, m3MinF(b.y, c.y)),
                   m3MinF(a.z, m3MinF(b.z, c.z))};
    *hi = (m3Vec3){m3MaxF(a.x, m3MaxF(b.x, c.x)), m3MaxF(a.y, m3MaxF(b.y, c.y)),
                   m3MaxF(a.z, m3MaxF(b.z, c.z))};
}

static bool BoxesOverlap(m3Vec3 alo, m3Vec3 ahi, m3Vec3 blo, m3Vec3 bhi)
{
    return !(ahi.x < blo.x || alo.x > bhi.x || ahi.y < blo.y || alo.y > bhi.y || ahi.z < blo.z ||
             alo.z > bhi.z);
}

static void TestRefereeGather(void)
{
    m3ShapeId meshShape;
    m3WorldId world = MakeMeshWorld(&meshShape);
    const m3MeshData* mesh = MeshOf(world, meshShape);
    const m3MeshBvh* bvh = BvhOf(world, meshShape);
    CHECK(mesh->triangleCount == 2 * 31 * 31, "heightfield triangulates fully");
    CHECK(bvh->nodeCount > 100, "the tree actually built");

    uint16_t gathered[M3_MESH_MAX_TRIS];
    uint16_t brute[M3_MESH_MAX_TRIS];
    for (int32_t round = 0; round < 300; ++round)
    {
        m3Vec3 center = {RandRange(-4.0f, 36.0f), RandRange(-2.0f, 2.0f), RandRange(-4.0f, 36.0f)};
        m3Vec3 half = {RandRange(0.05f, 3.0f), RandRange(0.05f, 3.0f), RandRange(0.05f, 3.0f)};
        m3Vec3 qlo = m3Sub3(center, half);
        m3Vec3 qhi = m3Add3(center, half);

        int32_t gatherCount = m3MeshBvhGather(bvh, qlo, qhi, gathered);

        // The referee: full scan with the exact per-triangle test.
        int32_t bruteCount = 0;
        for (int32_t t = 0; t < mesh->triangleCount; ++t)
        {
            m3Vec3 tlo;
            m3Vec3 thi;
            TriBox(mesh, t, &tlo, &thi);
            if (BoxesOverlap(tlo, thi, qlo, qhi))
            {
                brute[bruteCount++] = (uint16_t)t;
            }
        }

        // Gather is ascending and duplicate-free.
        bool ascending = true;
        for (int32_t i = 1; i < gatherCount; ++i)
        {
            if (gathered[i] <= gathered[i - 1])
            {
                ascending = false;
            }
        }
        CHECK(ascending, "gather is strictly ascending");

        // Filtered by the exact test, gather equals brute exactly.
        int32_t filtered = 0;
        bool equal = true;
        for (int32_t i = 0; i < gatherCount; ++i)
        {
            m3Vec3 tlo;
            m3Vec3 thi;
            TriBox(mesh, gathered[i], &tlo, &thi);
            if (BoxesOverlap(tlo, thi, qlo, qhi))
            {
                if (filtered >= bruteCount || brute[filtered] != gathered[i])
                {
                    equal = false;
                    break;
                }
                filtered += 1;
            }
        }
        if (filtered != bruteCount)
        {
            equal = false;
        }
        CHECK(equal, "exact-filtered gather equals the brute-force scan");
        if (!equal)
        {
            break; // one detailed failure is enough
        }
    }

    // Degenerates: a box that misses everything, a box that covers it.
    int32_t none = m3MeshBvhGather(bvh, (m3Vec3){100.0f, 100.0f, 100.0f},
                                   (m3Vec3){101.0f, 101.0f, 101.0f}, gathered);
    CHECK(none == 0, "a far box gathers nothing");
    int32_t all = m3MeshBvhGather(bvh, (m3Vec3){-100.0f, -100.0f, -100.0f},
                                  (m3Vec3){100.0f, 100.0f, 100.0f}, gathered);
    CHECK(all == mesh->triangleCount, "a covering box gathers every triangle");
    m3DestroyWorld(world);
}

// Content equality for the pointer-based BVH (10-3): derived data
// must match by VALUE, and the struct now carries heap pointers.
static int BvhSame(const m3MeshBvh* a, const m3MeshBvh* b)
{
    if (a->nodeCount != b->nodeCount)
    {
        return 0;
    }
    if (a->nodeCount == 0)
    {
        return 1;
    }
    int32_t orderLen = 0;
    for (int32_t i = 0; i < a->nodeCount; ++i)
    {
        if (a->nodes[i].count > 0 && a->nodes[i].start + a->nodes[i].count > orderLen)
        {
            orderLen = a->nodes[i].start + a->nodes[i].count;
        }
    }
    return memcmp(a->nodes, b->nodes, (size_t)a->nodeCount * sizeof(m3MeshBvhNode)) == 0 &&
           memcmp(a->order, b->order, (size_t)orderLen * sizeof(uint16_t)) == 0;
}

static void TestBuildDeterminismAndRestore(void)
{
    // Twin builds are byte-identical, and a restore rebuilds the
    // exact same bytes (derived data follows content).
    m3ShapeId shapeA;
    m3WorldId a = MakeMeshWorld(&shapeA);
    m3ShapeId shapeB;
    m3WorldId b = MakeMeshWorld(&shapeB);
    const m3MeshBvh* bvhA = BvhOf(a, shapeA);
    const m3MeshBvh* bvhB = BvhOf(b, shapeB);
    CHECK(BvhSame(bvhA, bvhB), "twin builds are byte-identical");

    // Deep-copy the derived tree so the scrub below cannot alias it.
    m3MeshBvh beforeCopy;
    memset(&beforeCopy, 0, sizeof(beforeCopy));
    beforeCopy.nodeCount = bvhA->nodeCount;
    beforeCopy.nodes = (m3MeshBvhNode*)malloc((size_t)bvhA->nodeCount * sizeof(m3MeshBvhNode));
    memcpy(beforeCopy.nodes, bvhA->nodes, (size_t)bvhA->nodeCount * sizeof(m3MeshBvhNode));
    int32_t orderLen = 0;
    for (int32_t i = 0; i < bvhA->nodeCount; ++i)
    {
        if (bvhA->nodes[i].count > 0 && bvhA->nodes[i].start + bvhA->nodes[i].count > orderLen)
        {
            orderLen = bvhA->nodes[i].start + bvhA->nodes[i].count;
        }
    }
    beforeCopy.order = (uint16_t*)malloc((size_t)orderLen * sizeof(uint16_t));
    memcpy(beforeCopy.order, bvhA->order, (size_t)orderLen * sizeof(uint16_t));
    m3MeshBvh* before = &beforeCopy;

    int32_t bytes = m3World_SnapshotSize(a);
    uint8_t* snap = (uint8_t*)malloc((size_t)bytes);
    CHECK(m3World_Snapshot(a, snap, bytes) == bytes, "snapshot writes");

    // Scrub the live tree to prove restore really rebuilds it.
    m3World* worldA = m3WorldFromId(a);
    m3MeshBvhFree(&worldA->meshBvh[worldA->shapeMeshIndex[shapeA.index1 - 1]]);
    CHECK(m3World_Restore(a, snap, bytes), "restore accepts");
    CHECK(BvhSame(BvhOf(a, shapeA), before), "restore rebuilds a byte-identical tree");

    free(beforeCopy.nodes);
    free(beforeCopy.order);
    free(snap);
    m3DestroyWorld(a);
    m3DestroyWorld(b);
}

static void TestMeshSimUnperturbed(void)
{
    // A ball rolling down the bumpscape: 300 steps, then the curated
    // hash of twin worlds must agree (the BVH is invisible to the
    // simulation; cross-run identity is the cheapest referee here,
    // the DET golden and the bench hashes referee against history).
    uint64_t hashes[2];
    for (int32_t run = 0; run < 2; ++run)
    {
        m3ShapeId meshShape;
        m3WorldId world = MakeMeshWorld(&meshShape);
        m3BodyDef bd = m3DefaultBodyDef();
        bd.type = m3_dynamicBody;
        bd.position = (m3Pos3){16.0, 3.0, 16.0};
        m3BodyId ball = m3CreateBody(world, &bd);
        m3ShapeDef sd = m3DefaultShapeDef();
        m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.4f};
        m3CreateSphereShape(ball, &sd, &sphere);
        m3Body_SetLinearVelocity(ball, (m3Vec3){2.0f, 0.0f, 1.0f});
        for (int32_t i = 0; i < 300; ++i)
        {
            m3World_Step(world, 1.0f / 60.0f, 4);
        }
        hashes[run] = m3World_Hash(world);
        m3Pos3 p = m3Body_GetPosition(ball);
        CHECK(p.y > -2.0 && p.y < 3.0, "the ball stays on the bumpscape");
        m3DestroyWorld(world);
    }
    CHECK(hashes[0] == hashes[1], "mesh sim is bit-deterministic through the BVH");
}

int main(void)
{
    TestRefereeGather();
    TestBuildDeterminismAndRestore();
    TestMeshSimUnperturbed();
    if (s_failures == 0)
    {
        printf("test_bvh: all green\n");
        return 0;
    }
    printf("test_bvh: %d failure(s)\n", s_failures);
    return 1;
}
