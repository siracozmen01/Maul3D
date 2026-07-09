// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The voxel chunk gate (3-1): state in the grid, everything else
// derived. Twin builds agree to the byte, the greedy merge produces
// the analytically-known box counts, bodies rest on voxel floors at
// analytic heights, the chunk rides snapshot and journal like every
// other state, a rollback re-derives the identical surface, and the
// refusals are loud.

#include "world_internal.h"

#include <stddef.h>
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

static m3WorldId SmallWorld(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    def.voxelCapacity = 2;
    return m3CreateWorld(&def);
}

// One byte per voxel, x fastest (the create contract).
static void FillSlab(uint8_t* voxels, int32_t yTop)
{
    memset(voxels, 0, M3_VOXEL_COUNT);
    for (int32_t z = 0; z < M3_VOXEL_DIM; ++z)
    {
        for (int32_t y = 0; y < yTop; ++y)
        {
            for (int32_t x = 0; x < M3_VOXEL_DIM; ++x)
            {
                voxels[x + M3_VOXEL_DIM * (y + M3_VOXEL_DIM * z)] = 1;
            }
        }
    }
}

static const m3VoxelSurface* SurfaceOf(m3WorldId worldId, m3ShapeId shape)
{
    m3World* world = m3WorldFromId(worldId);
    return &world->voxelSurface[world->shapeVoxelIndex[shape.index1 - 1]];
}

// Content equality for the pointer-based BVH (10-3): derived data
// must match by VALUE, and the struct now carries heap pointers.
static int VoxBvhSame(const m3MeshBvh* a, const m3MeshBvh* b)
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

// Surfaces embed the pointer-based BVH now: compare the value
// region up to the tree, then the tree by content.
static int SurfaceSame(const m3VoxelSurface* a, const m3VoxelSurface* b)
{
    return memcmp(a, b, offsetof(m3VoxelSurface, bvh)) == 0 && VoxBvhSame(&a->bvh, &b->bvh);
}

static void TestGreedyMergeAnalytics(void)
{
    m3WorldId world = SmallWorld();
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    static uint8_t voxels[M3_VOXEL_COUNT];

    // A full chunk merges into exactly one box.
    memset(voxels, 1, sizeof(voxels));
    m3ShapeId full = m3CreateVoxelChunkShape(ground, &sd, voxels, NULL, 1.0f);
    CHECK(m3Shape_IsValid(full), "the full chunk creates");
    CHECK(SurfaceOf(world, full)->boxCount == 1, "a full chunk is one box");

    // A one-voxel-thick floor merges into one slab box.
    m3BodyDef gd2 = m3DefaultBodyDef();
    gd2.position = (m3Pos3){100.0, 0.0, 0.0};
    m3BodyId ground2 = m3CreateBody(world, &gd2);
    FillSlab(voxels, 1);
    m3ShapeId slab = m3CreateVoxelChunkShape(ground2, &sd, voxels, NULL, 1.0f);
    CHECK(m3Shape_IsValid(slab), "the slab creates");
    CHECK(SurfaceOf(world, slab)->boxCount == 1, "a flat floor is one box");
    m3DestroyWorld(world);

    // Two disjoint voxels are two boxes; payload rides along.
    m3WorldId w2 = SmallWorld();
    m3BodyId g3 = m3CreateBody(w2, &gd);
    memset(voxels, 0, sizeof(voxels));
    static uint16_t payload[M3_VOXEL_COUNT];
    memset(payload, 0, sizeof(payload));
    voxels[0] = 1;
    payload[0] = 7;
    int32_t far = 15 + M3_VOXEL_DIM * (15 + M3_VOXEL_DIM * 15);
    voxels[far] = 1;
    payload[far] = 9;
    m3ShapeId two = m3CreateVoxelChunkShape(g3, &sd, voxels, payload, 0.5f);
    CHECK(m3Shape_IsValid(two), "the two-voxel chunk creates");
    CHECK(SurfaceOf(w2, two)->boxCount == 2, "two isolated voxels are two boxes");
    m3World* wp = m3WorldFromId(w2);
    const m3VoxelChunkData* chunk = &wp->voxelData[wp->shapeVoxelIndex[two.index1 - 1]];
    CHECK(chunk->payload[0] == 7 && chunk->payload[far] == 9, "payload is carried as state");
    CHECK(chunk->filledCount == 2, "the filled count is exact");
    m3DestroyWorld(w2);
}

static void TestRestingOnVoxelFloor(void)
{
    // A 3-voxel-high floor with cell 0.5: the top face sits at
    // y = 1.5 in the chunk frame. A ball of radius 0.4 rests at
    // 1.9, a unit box rests with its center at 2.0, a lying capsule
    // of radius 0.2 rests at 1.7. Analytic, all three families.
    uint64_t hashes[2];
    for (int32_t run = 0; run < 2; ++run)
    {
        m3WorldId world = SmallWorld();
        m3BodyDef gd = m3DefaultBodyDef();
        gd.position = (m3Pos3){-4.0, 0.0, -4.0};
        m3BodyId ground = m3CreateBody(world, &gd);
        m3ShapeDef sd = m3DefaultShapeDef();
        static uint8_t voxels[M3_VOXEL_COUNT];
        FillSlab(voxels, 3);
        CHECK(m3Shape_IsValid(m3CreateVoxelChunkShape(ground, &sd, voxels, NULL, 0.5f)),
              "the floor chunk creates");

        m3BodyDef bd = m3DefaultBodyDef();
        bd.type = m3_dynamicBody;
        bd.position = (m3Pos3){0.0, 2.6, 0.0};
        m3BodyId ball = m3CreateBody(world, &bd);
        m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.4f};
        m3CreateSphereShape(ball, &sd, &sphere);

        bd.position = (m3Pos3){-2.0, 2.8, -2.0};
        m3BodyId crate = m3CreateBody(world, &bd);
        m3CreateBoxShape(crate, &sd, (m3Vec3){0.5f, 0.5f, 0.5f});

        bd.position = (m3Pos3){2.0, 2.4, 2.0};
        m3BodyId pill = m3CreateBody(world, &bd);
        m3Capsule capsule = {{-0.3f, 0.0f, 0.0f}, {0.3f, 0.0f, 0.0f}, 0.2f};
        m3CreateCapsuleShape(pill, &sd, &capsule);

        for (int32_t i = 0; i < 240; ++i)
        {
            m3World_Step(world, 1.0f / 60.0f, 4);
        }
        m3Pos3 pBall = m3Body_GetPosition(ball);
        m3Pos3 pCrate = m3Body_GetPosition(crate);
        m3Pos3 pPill = m3Body_GetPosition(pill);
        CHECK(pBall.y > 1.85 && pBall.y < 1.95, "the ball rests at the analytic height");
        CHECK(pCrate.y > 1.95 && pCrate.y < 2.05, "the crate rests flat at the analytic height");
        CHECK(pPill.y > 1.65 && pPill.y < 1.75, "the capsule rests at the analytic height");
        hashes[run] = m3World_Hash(world);
        m3DestroyWorld(world);
    }
    CHECK(hashes[0] == hashes[1], "voxel resting scenes are bit-deterministic twins");
}

static void TestSnapshotJournalAndDerivedRebuild(void)
{
    static uint8_t journal[262144];
    m3WorldId world = SmallWorld();
    CHECK(m3World_JournalBegin(world, journal, (int32_t)sizeof(journal)), "the journal arms");

    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    static uint8_t voxels[M3_VOXEL_COUNT];
    FillSlab(voxels, 2);
    // A tower in one corner makes the surface non-trivial.
    for (int32_t y = 2; y < 10; ++y)
    {
        voxels[3 + M3_VOXEL_DIM * (y + M3_VOXEL_DIM * 3)] = 1;
    }
    m3ShapeId chunkShape = m3CreateVoxelChunkShape(ground, &sd, voxels, NULL, 0.5f);
    CHECK(m3Shape_IsValid(chunkShape), "the tower chunk creates");

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){1.5, 3.0, 1.5};
    m3BodyId ball = m3CreateBody(world, &bd);
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.3f};
    m3CreateSphereShape(ball, &sd, &sphere);
    for (int32_t i = 0; i < 90; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    int32_t bytes = m3World_JournalEnd(world);
    CHECK(bytes > 0, "the voxel session records");

    // Journal replay re-mints the chunk and rebuilds the surface.
    m3WorldId replayed = SmallWorld();
    CHECK(m3World_JournalReplay(replayed, journal, bytes), "the voxel session replays");
    CHECK(m3World_Hash(replayed) == m3World_Hash(world), "the replay is bit-identical");
    CHECK(SurfaceSame(SurfaceOf(replayed, chunkShape), SurfaceOf(world, chunkShape)),
          "the replayed surface is byte-identical");
    m3DestroyWorld(replayed);

    // Snapshot, scrub the derived surface, restore: bit-identical
    // state AND a byte-identical rebuilt surface (the derived law).
    int32_t snapBytes = m3World_SnapshotSize(world);
    uint8_t* snap = (uint8_t*)malloc((size_t)snapBytes);
    CHECK(m3World_Snapshot(world, snap, snapBytes) == snapBytes, "the voxel snapshot writes");
    m3VoxelSurface* before = (m3VoxelSurface*)malloc(sizeof(m3VoxelSurface));
    memcpy(before, SurfaceOf(world, chunkShape), sizeof(m3VoxelSurface));
    uint64_t hashBefore = m3World_Hash(world);

    m3World* wp = m3WorldFromId(world);
    // The scrub must FREE the embedded tree first (10-3 ownership):
    // a raw memset wipes live pointers and LSAN convicts the leak.
    m3MeshBvhFree(&wp->voxelSurface[wp->shapeVoxelIndex[chunkShape.index1 - 1]].bvh);
    memset(&wp->voxelSurface[wp->shapeVoxelIndex[chunkShape.index1 - 1]], 0,
           sizeof(m3VoxelSurface));
    for (int32_t i = 0; i < 30; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m3World_Restore(world, snap, snapBytes), "the voxel snapshot restores");
    CHECK(m3World_Hash(world) == hashBefore, "the restore is bit-identical");
    // The value region IS the surface; the embedded tree is its
    // pure function (build determinism holds elsewhere in this
    // suite), and saved copies must not chase stale pointers.
    CHECK(memcmp(SurfaceOf(world, chunkShape), before, offsetof(m3VoxelSurface, bvh)) == 0,
          "the restore re-derives a byte-identical surface");
    free(before);
    free(snap);
    m3DestroyWorld(world);
}

static void TestQueriesAgainstVoxels(void)
{
    m3WorldId world = SmallWorld();
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    static uint8_t voxels[M3_VOXEL_COUNT];
    FillSlab(voxels, 4);
    m3ShapeId chunkShape = m3CreateVoxelChunkShape(ground, &sd, voxels, NULL, 1.0f);
    CHECK(m3Shape_IsValid(chunkShape), "the query chunk creates");

    // A downward ray hits the top face at the analytic fraction.
    m3RayHit hit =
        m3World_CastRayClosest(world, (m3Pos3){8.0, 10.0, 8.0}, (m3Vec3){0.0f, -10.0f, 0.0f});
    CHECK(hit.hit && hit.shape.index1 == chunkShape.index1, "the ray hits the chunk");
    CHECK(hit.fraction > 0.599f && hit.fraction < 0.601f, "the ray fraction is analytic");
    CHECK(hit.normal.y > 0.99f, "the entry normal is the top face");

    // A ray born inside the slab misses (the ray contract).
    hit = m3World_CastRayClosest(world, (m3Pos3){8.0, 2.0, 8.0}, (m3Vec3){0.0f, -5.0f, 0.0f});
    CHECK(!hit.hit, "a ray born inside reports a miss");

    // Containment: inside a filled voxel, above the slab, outside.
    CHECK(m3World_PointInside(world, (m3Pos3){8.0, 3.5, 8.0}).index1 == chunkShape.index1,
          "a point in a filled voxel is inside");
    CHECK(m3World_PointInside(world, (m3Pos3){8.0, 4.5, 8.0}).index1 == 0,
          "a point above the slab is outside");
    CHECK(m3World_PointInside(world, (m3Pos3){-1.0, 1.0, 8.0}).index1 == 0,
          "a point before the chunk origin is outside");

    // Overlaps: the sphere reach and the AABB both see the chunk.
    m3ShapeId found[4];
    int32_t n = m3World_OverlapSphere(world, (m3Pos3){8.0, 4.3, 8.0}, 0.5f, found, 4);
    CHECK(n == 1 && found[0].index1 == chunkShape.index1, "the sphere reach finds the chunk");
    n = m3World_OverlapSphere(world, (m3Pos3){8.0, 5.5, 8.0}, 0.5f, found, 4);
    CHECK(n == 0, "out of reach finds nothing");
    m3DestroyWorld(world);
}

static void TestVoxelRefusals(void)
{
    m3WorldId world = SmallWorld();
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3BodyDef dd = m3DefaultBodyDef();
    dd.type = m3_dynamicBody;
    m3BodyId mover = m3CreateBody(world, &dd);
    m3ShapeDef sd = m3DefaultShapeDef();
    static uint8_t voxels[M3_VOXEL_COUNT];
    memset(voxels, 1, sizeof(voxels));

    CHECK(!m3Shape_IsValid(m3CreateVoxelChunkShape(mover, &sd, voxels, NULL, 1.0f)),
          "a dynamic body refuses a chunk (3-1 is static level geometry)");
    m3ShapeDef sensor = m3DefaultShapeDef();
    sensor.isSensor = true;
    CHECK(!m3Shape_IsValid(m3CreateVoxelChunkShape(ground, &sensor, voxels, NULL, 1.0f)),
          "a sensor chunk refuses (sensor volumes are convex)");
    CHECK(!m3Shape_IsValid(m3CreateVoxelChunkShape(ground, &sd, voxels, NULL, 0.0f)),
          "zero cell size refuses");
    CHECK(!m3Shape_IsValid(m3CreateVoxelChunkShape(ground, &sd, NULL, NULL, 1.0f)),
          "a null grid refuses");
    memset(voxels, 0, sizeof(voxels));
    CHECK(!m3Shape_IsValid(m3CreateVoxelChunkShape(ground, &sd, voxels, NULL, 1.0f)),
          "an empty grid refuses");

    // Pool exhaustion: capacity 2 fills, the third refuses.
    memset(voxels, 1, sizeof(voxels));
    m3BodyDef g2 = m3DefaultBodyDef();
    g2.position = (m3Pos3){40.0, 0.0, 0.0};
    m3BodyId ground2 = m3CreateBody(world, &g2);
    CHECK(m3Shape_IsValid(m3CreateVoxelChunkShape(ground, &sd, voxels, NULL, 1.0f)),
          "chunk one fills");
    CHECK(m3Shape_IsValid(m3CreateVoxelChunkShape(ground2, &sd, voxels, NULL, 1.0f)),
          "chunk two fills");
    CHECK(!m3Shape_IsValid(m3CreateVoxelChunkShape(ground, &sd, voxels, NULL, 1.0f)),
          "the voxel pool refuses past capacity");
    m3World_Step(world, 1.0f / 60.0f, 4);
    CHECK(m3World_Hash(world) != 0, "the refused world still steps");
    m3DestroyWorld(world);
}

static void TestEditsCarveAndWake(void)
{
    // The headline of 3-2: a crate sleeps on a voxel floor, the
    // floor under it is carved away, the crate wakes and falls to
    // the lower level. Analytic before and after heights.
    m3WorldId world = SmallWorld();
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    static uint8_t voxels[M3_VOXEL_COUNT];
    FillSlab(voxels, 4); // floor top at y = 4 with cell 1
    m3ShapeId chunkShape = m3CreateVoxelChunkShape(ground, &sd, voxels, NULL, 1.0f);
    CHECK(m3Shape_IsValid(chunkShape), "the carve floor creates");

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){8.0, 4.6, 8.0};
    m3BodyId crate = m3CreateBody(world, &bd);
    m3CreateBoxShape(crate, &sd, (m3Vec3){0.5f, 0.5f, 0.5f});

    for (int32_t i = 0; i < 240; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3Pos3 p = m3Body_GetPosition(crate);
    CHECK(p.y > 4.45 && p.y < 4.55, "the crate rests on the intact floor");
    m3World* wp = m3WorldFromId(world);
    int32_t crateSlot = crate.index1 - 1;
    CHECK(wp->awake[crateSlot] == 0, "the crate fell asleep");

    // Carve a 4x1x4 pocket in the top layer under the crate.
    int32_t lo[3] = {6, 3, 6};
    int32_t hi[3] = {9, 3, 9};
    CHECK(m3VoxelChunk_ClearBox(chunkShape, lo, hi) == 16, "the carve clears sixteen voxels");
    CHECK(wp->awake[crateSlot] == 1, "the vanishing floor wakes the sleeper");

    for (int32_t i = 0; i < 240; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    p = m3Body_GetPosition(crate);
    CHECK(p.y > 3.45 && p.y < 3.55, "the crate settles one voxel lower in the pocket");

    // Single-voxel edits reshape the surface analytically: refill
    // one pocket voxel and the surface gains a box.
    int32_t before = SurfaceOf(world, chunkShape)->boxCount;
    CHECK(m3VoxelChunk_SetVoxel(chunkShape, 6, 3, 6, 42), "a refill lands");
    CHECK(SurfaceOf(world, chunkShape)->boxCount == before + 1, "the refill adds one box");
    const m3VoxelChunkData* chunk = &wp->voxelData[wp->shapeVoxelIndex[chunkShape.index1 - 1]];
    CHECK(chunk->payload[6 + M3_VOXEL_DIM * (3 + M3_VOXEL_DIM * 6)] == 42,
          "the refill carries its payload");

    // A payload-only set changes state but not one derived byte.
    static m3VoxelSurface surfBefore;
    memcpy(&surfBefore, SurfaceOf(world, chunkShape), sizeof(m3VoxelSurface));
    uint64_t hashBefore = m3World_Hash(world);
    CHECK(m3VoxelChunk_SetVoxel(chunkShape, 6, 3, 6, 43), "the payload update lands");
    CHECK(m3World_Hash(world) != hashBefore, "payload is state: the hash moves");
    CHECK(memcmp(&surfBefore, SurfaceOf(world, chunkShape), offsetof(m3VoxelSurface, bvh)) == 0,
          "payload is not geometry: the surface does not move");
    m3DestroyWorld(world);
}

static void TestEditRollbackReedit(void)
{
    // Gap 9's core promise: edits are state transitions inside the
    // rollback delta. Edit, snapshot, diverge with MORE edits, roll
    // back, redo the divergence, and land on the identical bits
    // with a byte-identical derived surface.
    static uint8_t journal[262144];
    m3WorldId world = SmallWorld();
    CHECK(m3World_JournalBegin(world, journal, (int32_t)sizeof(journal)), "the journal arms");
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    static uint8_t voxels[M3_VOXEL_COUNT];
    FillSlab(voxels, 5);
    m3ShapeId chunkShape = m3CreateVoxelChunkShape(ground, &sd, voxels, NULL, 0.5f);

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){4.0, 4.0, 4.0};
    m3BodyId ball = m3CreateBody(world, &bd);
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.3f};
    m3CreateSphereShape(ball, &sd, &sphere);

    CHECK(m3VoxelChunk_ClearVoxel(chunkShape, 8, 4, 8), "the first carve lands");
    for (int32_t i = 0; i < 60; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }

    // The composition rule (the manual states it, this test honors
    // it): the journal closes BEFORE the rollback block. A restore
    // is not an op, so a journal spanning one would replay a longer
    // history than the world lived. The first draft of this test
    // violated the rule and the engine correctly refused to make
    // the mismatched timelines agree.
    int32_t bytes = m3World_JournalEnd(world);
    CHECK(bytes > 0, "the edit session records");
    m3WorldId replayed = SmallWorld();
    CHECK(m3World_JournalReplay(replayed, journal, bytes), "the edit session replays");
    CHECK(m3World_Hash(replayed) == m3World_Hash(world), "the edit replay is bit-identical");
    m3DestroyWorld(replayed);

    int32_t snapBytes = m3World_SnapshotSize(world);
    uint8_t* snap = (uint8_t*)malloc((size_t)snapBytes);
    CHECK(m3World_Snapshot(world, snap, snapBytes) == snapBytes, "the mid-edit snapshot writes");

    // The divergence, twice.
    uint64_t firstTimeline = 0;
    static m3VoxelSurface firstSurface;
    for (int32_t run = 0; run < 2; ++run)
    {
        int32_t lo[3] = {7, 4, 7};
        int32_t hi[3] = {9, 4, 9};
        CHECK(m3VoxelChunk_ClearBox(chunkShape, lo, hi) >= 0, "the divergent carve lands");
        CHECK(m3VoxelChunk_SetVoxel(chunkShape, 0, 5, 0, 7), "the divergent build lands");
        for (int32_t i = 0; i < 90; ++i)
        {
            m3World_Step(world, 1.0f / 60.0f, 4);
        }
        if (run == 0)
        {
            firstTimeline = m3World_Hash(world);
            memcpy(&firstSurface, SurfaceOf(world, chunkShape), sizeof(m3VoxelSurface));
            CHECK(m3World_Restore(world, snap, snapBytes), "the rollback lands");
        }
        else
        {
            CHECK(m3World_Hash(world) == firstTimeline,
                  "edit, roll back, reedit: bit-identical timelines");
            CHECK(memcmp(&firstSurface, SurfaceOf(world, chunkShape),
                         offsetof(m3VoxelSurface, bvh)) == 0,
                  "the re-derived surface is byte-identical");
        }
    }

    free(snap);
    m3DestroyWorld(world);
}

static void TestEditRefusalsAndEmptyEnd(void)
{
    m3WorldId world = SmallWorld();
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    static uint8_t voxels[M3_VOXEL_COUNT];
    memset(voxels, 0, sizeof(voxels));
    voxels[0] = 1;
    m3ShapeId chunkShape = m3CreateVoxelChunkShape(ground, &sd, voxels, NULL, 1.0f);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.5, 3.0, 0.5};
    m3BodyId ball = m3CreateBody(world, &bd);
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.4f};
    m3ShapeId ballShape = m3CreateSphereShape(ball, &sd, &sphere);

    CHECK(!m3VoxelChunk_SetVoxel(chunkShape, 16, 0, 0, 0), "out-of-range x refuses");
    CHECK(!m3VoxelChunk_ClearVoxel(chunkShape, 0, -1, 0), "negative y refuses");
    CHECK(!m3VoxelChunk_SetVoxel(ballShape, 0, 0, 0, 0), "a non-voxel shape refuses edits");
    int32_t lo[3] = {2, 0, 0};
    int32_t hi[3] = {1, 0, 0};
    CHECK(m3VoxelChunk_ClearBox(chunkShape, lo, hi) == -1, "an inverted region refuses");
    m3ShapeId stale = chunkShape;
    stale.generation += 1;
    CHECK(!m3VoxelChunk_ClearVoxel(stale, 0, 0, 0), "a stale id refuses edits");

    // Clear the last voxel: the chunk stays a valid shape that
    // collides with nothing, and the ball falls straight through
    // where the floor voxel used to be.
    CHECK(m3VoxelChunk_ClearVoxel(chunkShape, 0, 0, 0), "the last voxel clears");
    CHECK(m3Shape_IsValid(chunkShape), "the empty chunk stays a valid shape");
    CHECK(SurfaceOf(world, chunkShape)->boxCount == 0, "the empty surface has no boxes");
    for (int32_t i = 0; i < 90; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3Pos3 p = m3Body_GetPosition(ball);
    CHECK(p.y < -5.0, "the ball falls through the vanished floor");
    m3DestroyWorld(world);
}

static void TestFractureBridge(void)
{
    // Two pillars on the base layer carry a span at y = 8. Cutting
    // one joint leaves the span anchored through the other pillar
    // (zero events: the negative case matters). Cutting the second
    // joint strands the middle span: exactly one island, eight
    // voxels, hand-counted, removed from the grid and delivered
    // with its recipe.
    m3WorldId world = SmallWorld();
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    static uint8_t voxels[M3_VOXEL_COUNT];
    memset(voxels, 0, sizeof(voxels));
    for (int32_t y = 0; y <= 8; ++y)
    {
        voxels[2 + M3_VOXEL_DIM * (y + M3_VOXEL_DIM * 8)] = 1;  // pillar at x = 2
        voxels[13 + M3_VOXEL_DIM * (y + M3_VOXEL_DIM * 8)] = 1; // pillar at x = 13
    }
    for (int32_t x = 3; x <= 12; ++x)
    {
        voxels[x + M3_VOXEL_DIM * (8 + M3_VOXEL_DIM * 8)] = 1; // the span
    }
    m3ShapeId bridge = m3CreateVoxelChunkShape(ground, &sd, voxels, NULL, 1.0f);
    CHECK(m3Shape_IsValid(bridge), "the bridge creates");
    m3World* wp = m3WorldFromId(world);
    const m3VoxelChunkData* chunk = &wp->voxelData[wp->shapeVoxelIndex[bridge.index1 - 1]];
    CHECK(chunk->filledCount == 28, "the bridge is twenty-eight voxels");

    // Cut the left joint: still anchored through the right pillar.
    int32_t count = 0;
    CHECK(m3VoxelChunk_ClearVoxel(bridge, 3, 8, 8), "the left cut lands");
    m3World_FragmentEvents(world, &count);
    CHECK(count == 0, "a span anchored through one pillar drops nothing");

    // Snapshot here: the rollback must re-derive the same fracture.
    int32_t snapBytes = m3World_SnapshotSize(world);
    uint8_t* snap = (uint8_t*)malloc((size_t)snapBytes);
    CHECK(m3World_Snapshot(world, snap, snapBytes) == snapBytes, "the pre-cut snapshot writes");

    uint64_t hashes[2];
    for (int32_t run = 0; run < 2; ++run)
    {
        // Cut the right joint: the middle span (x = 4..11) hangs.
        CHECK(m3VoxelChunk_ClearVoxel(bridge, 12, 8, 8), "the right cut lands");
        const m3FragmentEvent* events = m3World_FragmentEvents(world, &count);
        CHECK(count == 1, "the stranded span is exactly one island");
        CHECK(events[0].voxelCount == 8, "the island is the hand-counted eight voxels");
        CHECK(events[0].chunkShape.index1 == bridge.index1, "the event names its chunk");
        CHECK(events[0].boundsLo[0] == 4 && events[0].boundsHi[0] == 11 &&
                  events[0].boundsLo[1] == 8 && events[0].boundsHi[1] == 8,
              "the island bounds are analytic");
        CHECK(events[0].mass > 7.99f && events[0].mass < 8.01f, "unit cells weigh eight");
        CHECK(events[0].comChunk.x > 7.99f && events[0].comChunk.x < 8.01f &&
                  events[0].comChunk.y > 8.49f && events[0].comChunk.y < 8.51f,
              "the island center of mass is analytic");
        CHECK(events[0].recipeStart >= 0 && events[0].recipeCount == 8, "the recipe fits");
        int32_t recipeTotal = 0;
        const uint16_t* recipe = m3World_FragmentRecipe(world, &recipeTotal);
        bool exact = true;
        for (int32_t k = 0; k < 8; ++k)
        {
            uint16_t v = recipe[events[0].recipeStart + k];
            int32_t x = v % M3_VOXEL_DIM;
            int32_t y = (v / M3_VOXEL_DIM) % M3_VOXEL_DIM;
            int32_t z = v / (M3_VOXEL_DIM * M3_VOXEL_DIM);
            if (x != 4 + k || y != 8 || z != 8)
            {
                exact = false;
            }
        }
        CHECK(exact, "the recipe decodes to exactly the stranded voxels in canonical order");
        CHECK(chunk->filledCount == 18, "the island left the grid with the cut");
        CHECK(m3World_FragmentEventsDropped(world) == 0, "nothing was dropped");

        // The pillars remain and still collide: a ray finds one.
        m3RayHit hit =
            m3World_CastRayClosest(world, (m3Pos3){2.5, 20.0, 8.5}, (m3Vec3){0.0f, -15.0f, 0.0f});
        CHECK(hit.hit, "the surviving pillar still answers rays");

        hashes[run] = m3World_Hash(world);
        if (run == 0)
        {
            // A step clears the transient streams.
            m3World_Step(world, 1.0f / 60.0f, 4);
            m3World_FragmentEvents(world, &count);
            CHECK(count == 0, "the next step clears fragment events");
            // Roll back to before the second cut and do it again.
            CHECK(m3World_Restore(world, snap, snapBytes), "the fracture rollback lands");
            m3World_FragmentEvents(world, &count);
            CHECK(count == 0, "restore clears fragment events");
            CHECK(chunk->filledCount == 27, "the rollback restores the pre-cut grid");
        }
    }
    CHECK(hashes[0] == hashes[1], "cut, roll back, recut: the fracture is bit-identical");
    free(snap);
    m3DestroyWorld(world);
}

static void TestFractureTwoIslands(void)
{
    // A column with two arms joined through one connector voxel:
    // clearing the connector strands BOTH arms in one edit. Two
    // events, canonical order (the lower linear seed first).
    m3WorldId world = SmallWorld();
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    static uint8_t voxels[M3_VOXEL_COUNT];
    memset(voxels, 0, sizeof(voxels));
    for (int32_t y = 0; y <= 5; ++y)
    {
        voxels[8 + M3_VOXEL_DIM * (y + M3_VOXEL_DIM * 8)] = 1; // the column
    }
    voxels[6 + M3_VOXEL_DIM * (5 + M3_VOXEL_DIM * 8)] = 1; // arm A: x 6..7
    voxels[7 + M3_VOXEL_DIM * (5 + M3_VOXEL_DIM * 8)] = 1;
    voxels[9 + M3_VOXEL_DIM * (5 + M3_VOXEL_DIM * 8)] = 1; // arm B: x 9..10
    voxels[10 + M3_VOXEL_DIM * (5 + M3_VOXEL_DIM * 8)] = 1;
    m3ShapeId tee = m3CreateVoxelChunkShape(ground, &sd, voxels, NULL, 1.0f);
    CHECK(m3Shape_IsValid(tee), "the tee creates");

    CHECK(m3VoxelChunk_ClearVoxel(tee, 8, 5, 8), "the connector clears");
    int32_t count = 0;
    const m3FragmentEvent* events = m3World_FragmentEvents(world, &count);
    CHECK(count == 2, "both arms strand in one edit");
    CHECK(events[0].voxelCount == 2 && events[1].voxelCount == 2, "each arm is two voxels");
    CHECK(events[0].boundsLo[0] == 6 && events[1].boundsLo[0] == 9,
          "events arrive in canonical seed order: the lower island first");
    m3DestroyWorld(world);
}

static void TestSeamWeldingCoverage(void)
{
    // White-box: welded borders read as covered, unwelded borders
    // as exposed, and destroying the neighbor un-welds.
    m3WorldId world = SmallWorld();
    m3ShapeDef sd = m3DefaultShapeDef();
    static uint8_t voxels[M3_VOXEL_COUNT];
    FillSlab(voxels, 4);

    m3BodyDef gd = m3DefaultBodyDef();
    gd.position = (m3Pos3){0.0, 0.0, 0.0};
    m3BodyId groundA = m3CreateBody(world, &gd);
    m3ShapeId chunkA = m3CreateVoxelChunkShape(groundA, &sd, voxels, NULL, 1.0f);
    gd.position = (m3Pos3){16.0, 0.0, 0.0}; // exactly one extent: welds
    m3BodyId groundB = m3CreateBody(world, &gd);
    m3ShapeId chunkB = m3CreateVoxelChunkShape(groundB, &sd, voxels, NULL, 1.0f);

    const m3VoxelSurface* surfA = SurfaceOf(world, chunkA);
    CHECK(surfA->boxCount == 1, "the slab is one box");
    CHECK((surfA->boxCovered[0] & 2u) != 0, "the welded +x border reads covered");
    CHECK((surfA->boxCovered[0] & 1u) == 0, "the open -x border reads exposed");
    CHECK((surfA->boxCovered[0] & 8u) == 0, "the top face reads exposed");
    CHECK((surfA->boxCovered[0] & 4u) == 0, "the world-facing bottom reads exposed");

    // Destroying the neighbor un-welds the border.
    m3DestroyBody(groundB);
    CHECK(!m3Shape_IsValid(chunkB), "the neighbor chunk went with its body");
    CHECK((SurfaceOf(world, chunkA)->boxCovered[0] & 2u) == 0,
          "the border is exposed again after the neighbor vanishes");
    m3DestroyWorld(world);
}

static void TestRollingAcrossSeams(void)
{
    // The headline: a ball rolls across an INTRA-chunk box seam and
    // then a CHUNK-chunk border, and neither seam kicks it. Chunk A
    // is built so the greedy merge must split it into two boxes with
    // a flush internal wall (the lower layer is missing under the
    // second half); the walking surface is flat at y = 4 the whole
    // way. Twin worlds for determinism.
    uint64_t hashes[2];
    for (int32_t run = 0; run < 2; ++run)
    {
        m3WorldDef def = m3DefaultWorldDef();
        def.bodyCapacity = 8;
        def.shapeCapacity = 8;
        def.voxelCapacity = 2;
        m3WorldId world = m3CreateWorld(&def);
        m3ShapeDef sd = m3DefaultShapeDef();
        static uint8_t voxels[M3_VOXEL_COUNT];

        // Chunk A: x 0..7 filled y 0..3; x 8..15 filled y 1..3 only.
        memset(voxels, 0, sizeof(voxels));
        for (int32_t z = 0; z < M3_VOXEL_DIM; ++z)
        {
            for (int32_t x = 0; x < M3_VOXEL_DIM; ++x)
            {
                for (int32_t y = x < 8 ? 0 : 1; y <= 3; ++y)
                {
                    voxels[x + M3_VOXEL_DIM * (y + M3_VOXEL_DIM * z)] = 1;
                }
            }
        }
        m3BodyDef gd = m3DefaultBodyDef();
        gd.position = (m3Pos3){0.0, 0.0, 0.0};
        m3BodyId groundA = m3CreateBody(world, &gd);
        m3ShapeId chunkA = m3CreateVoxelChunkShape(groundA, &sd, voxels, NULL, 1.0f);
        CHECK(SurfaceOf(world, chunkA)->boxCount == 2, "the split floor is two boxes");

        // Chunk B: a full-width slab, welded at x = 16.
        FillSlab(voxels, 4);
        gd.position = (m3Pos3){16.0, 0.0, 0.0};
        m3BodyId groundB = m3CreateBody(world, &gd);
        m3ShapeId chunkB = m3CreateVoxelChunkShape(groundB, &sd, voxels, NULL, 1.0f);
        CHECK(m3Shape_IsValid(chunkB), "chunk B creates");

        m3BodyDef bd = m3DefaultBodyDef();
        bd.type = m3_dynamicBody;
        bd.position = (m3Pos3){4.0, 4.5, 8.0};
        m3BodyId ball = m3CreateBody(world, &bd);
        m3ShapeDef ballDef = m3DefaultShapeDef();
        ballDef.friction = 0.2f; // keep it rolling the whole runway
        m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.5f};
        m3CreateSphereShape(ball, &ballDef, &sphere);
        m3Body_SetLinearVelocity(ball, (m3Vec3){4.0f, 0.0f, 0.0f});

        double maxHeightDeviation = 0.0;
        float maxVerticalSpeed = 0.0f;
        for (int32_t i = 0; i < 420; ++i)
        {
            m3World_Step(world, 1.0f / 60.0f, 4);
            m3Pos3 p = m3Body_GetPosition(ball);
            double dev = p.y - 4.5;
            if (dev < 0.0)
            {
                dev = -dev;
            }
            if (dev > maxHeightDeviation)
            {
                maxHeightDeviation = dev;
            }
            m3Vec3 v = m3Body_GetLinearVelocity(ball);
            float vy = v.y < 0.0f ? -v.y : v.y;
            if (i > 30 && vy > maxVerticalSpeed)
            {
                maxVerticalSpeed = vy; // after the initial settle
            }
        }
        m3Pos3 p = m3Body_GetPosition(ball);
        CHECK(p.x > 20.0, "the ball crossed both seams and kept going");
        CHECK(maxHeightDeviation < 0.05, "no seam lifted or dropped the ball");
        CHECK(maxVerticalSpeed < 0.25f, "no seam kicked the ball vertically");
        m3Vec3 v = m3Body_GetLinearVelocity(ball);
        CHECK(v.x > 1.0f, "the roll survived with real forward speed");
        hashes[run] = m3World_Hash(world);
        m3DestroyWorld(world);
    }
    CHECK(hashes[0] == hashes[1], "the seam crossing is bit-deterministic");
}

static void TestVoxelMeetsHeightfield(void)
{
    // The junction contract: a ball resting where a voxel block and
    // a heightfield floor meet settles clean (two ordinary pairs,
    // no special case), sleeps, and twins agree.
    uint64_t hashes[2];
    for (int32_t run = 0; run < 2; ++run)
    {
        m3WorldId world = SmallWorld();
        m3ShapeDef sd = m3DefaultShapeDef();
        m3BodyDef gd = m3DefaultBodyDef();
        gd.position = (m3Pos3){-8.0, 0.0, -8.0};
        m3BodyId field = m3CreateBody(world, &gd);
        float heights[16 * 16];
        for (int32_t i = 0; i < 16 * 16; ++i)
        {
            heights[i] = 0.0f; // a flat field: the junction is exact
        }
        CHECK(m3Shape_IsValid(m3CreateHeightFieldShape(field, &sd, heights, 16, 16, 1.0f)),
              "the field creates");

        static uint8_t voxels[M3_VOXEL_COUNT];
        FillSlab(voxels, 2); // a low voxel block ON the field
        m3BodyDef vd = m3DefaultBodyDef();
        vd.position = (m3Pos3){0.0, 0.0, -3.0};
        m3BodyId block = m3CreateBody(world, &vd);
        CHECK(m3Shape_IsValid(m3CreateVoxelChunkShape(block, &sd, voxels, NULL, 0.5f)),
              "the block creates");

        // The ball drops right at the wall base: it touches the
        // heightfield floor and the voxel wall at once.
        m3BodyDef bd = m3DefaultBodyDef();
        bd.type = m3_dynamicBody;
        bd.position = (m3Pos3){-0.8, 1.5, -1.0};
        m3BodyId ball = m3CreateBody(world, &bd);
        m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.4f};
        m3CreateSphereShape(ball, &sd, &sphere);

        for (int32_t i = 0; i < 300; ++i)
        {
            m3World_Step(world, 1.0f / 60.0f, 4);
        }
        m3Pos3 p = m3Body_GetPosition(ball);
        CHECK(p.y > 0.35 && p.y < 0.45, "the ball rests on the field at the wall base");
        m3World* wp = m3WorldFromId(world);
        CHECK(wp->awake[ball.index1 - 1] == 0, "the junction rest is stable enough to sleep");
        hashes[run] = m3World_Hash(world);
        m3DestroyWorld(world);
    }
    CHECK(hashes[0] == hashes[1], "the junction scene is bit-deterministic");
}

static void TestBulletVsVoxelWall(void)
{
    // The two-sided promise of voxel CCD: a one-voxel wall stops a
    // two-hundred-per-second bullet, and the SAME wall with its
    // voxel cleared the step before is a real hole. Twins.
    uint64_t hashes[2];
    for (int32_t run = 0; run < 2; ++run)
    {
        for (int32_t withHole = 0; withHole < 2; ++withHole)
        {
            m3WorldDef def = m3DefaultWorldDef();
            def.gravity = (m3Vec3){0.0f, 0.0f, 0.0f}; // pure flight
            def.bodyCapacity = 8;
            def.shapeCapacity = 8;
            def.voxelCapacity = 1;
            m3WorldId world = m3CreateWorld(&def);
            m3ShapeDef sd = m3DefaultShapeDef();
            static uint8_t voxels[M3_VOXEL_COUNT];
            memset(voxels, 0, sizeof(voxels));
            // A one-voxel-thick wall at x = 8 spanning the chunk,
            // anchored through the base layer (a floor column under
            // it keeps fracture out of this test's way).
            for (int32_t z = 0; z < M3_VOXEL_DIM; ++z)
            {
                for (int32_t y = 0; y < M3_VOXEL_DIM; ++y)
                {
                    voxels[8 + M3_VOXEL_DIM * (y + M3_VOXEL_DIM * z)] = 1;
                }
            }
            m3BodyDef gd = m3DefaultBodyDef();
            m3BodyId ground = m3CreateBody(world, &gd);
            m3ShapeId wall = m3CreateVoxelChunkShape(ground, &sd, voxels, NULL, 1.0f);
            CHECK(m3Shape_IsValid(wall), "the wall creates");

            if (withHole != 0)
            {
                // Open the hole on the bullet's exact path, THEN
                // fire: destruction opens real holes. The clear is
                // a 3x3 window so the bullet's radius fits.
                int32_t lo[3] = {8, 7, 7};
                int32_t hi[3] = {8, 9, 9};
                CHECK(m3VoxelChunk_ClearBox(wall, lo, hi) == 9, "the window opens");
            }

            m3BodyDef bd = m3DefaultBodyDef();
            bd.type = m3_dynamicBody;
            bd.isBullet = true;
            bd.position = (m3Pos3){2.0, 8.5, 8.5};
            bd.linearVelocity = (m3Vec3){200.0f, 0.0f, 0.0f};
            m3BodyId bullet = m3CreateBody(world, &bd);
            m3Sphere slug = {{0.0f, 0.0f, 0.0f}, 0.2f};
            m3CreateSphereShape(bullet, &sd, &slug);

            for (int32_t i = 0; i < 30; ++i)
            {
                m3World_Step(world, 1.0f / 60.0f, 4);
            }
            m3Pos3 p = m3Body_GetPosition(bullet);
            if (withHole != 0)
            {
                CHECK(p.x > 20.0, "the bullet flies through the opened window");
            }
            else
            {
                CHECK(p.x < 8.6, "the one-voxel wall stops the bullet");
            }
            if (withHole != 0)
            {
                hashes[run] = m3World_Hash(world);
            }
            m3DestroyWorld(world);
        }
    }
    CHECK(hashes[0] == hashes[1], "the shot through the window is bit-deterministic");
}

static void TestShapeCastsAgainstVoxels(void)
{
    m3WorldId world = SmallWorld();
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    static uint8_t voxels[M3_VOXEL_COUNT];
    FillSlab(voxels, 4); // top face at y = 4
    m3ShapeId chunkShape = m3CreateVoxelChunkShape(ground, &sd, voxels, NULL, 1.0f);

    // A sphere cast straight down: center stops at 4 + r, so the
    // fraction is (10 - 4.3 - slop band) / 8 within the TOI band.
    m3RayHit hit = m3World_CastSphereClosest(world, (m3Pos3){8.0, 10.0, 8.0}, 0.3f,
                                             (m3Vec3){0.0f, -8.0f, 0.0f});
    CHECK(hit.hit && hit.shape.index1 == chunkShape.index1, "the sphere cast hits the chunk");
    CHECK(hit.fraction > 0.70f && hit.fraction < 0.72f, "the voxel cast fraction is analytic");

    // A capsule cast onto the top face, and a start-overlapped cast.
    hit = m3World_CastCapsuleClosest(world, (m3Pos3){8.0, 6.0, 8.0}, (m3Vec3){-0.4f, 0.0f, 0.0f},
                                     (m3Vec3){0.4f, 0.0f, 0.0f}, 0.2f, (m3Vec3){0.0f, -4.0f, 0.0f});
    CHECK(hit.hit && hit.fraction > 0.44f && hit.fraction < 0.46f,
          "the capsule cast fraction is analytic");
    hit = m3World_CastSphereClosest(world, (m3Pos3){8.0, 3.9, 8.0}, 0.3f,
                                    (m3Vec3){0.0f, -2.0f, 0.0f});
    CHECK(hit.hit && hit.fraction == 0.0f, "a cast born inside the chunk reports zero");

    // A box cast lands on the chunk face at the same analytic
    // height as the sphere (4-1: generic casts see voxels through
    // the shared branch).
    m3Quat identity = {0.0f, 0.0f, 0.0f, 1.0f};
    hit = m3World_CastBoxClosest(world, (m3Pos3){8.0, 10.0, 8.0}, (m3Vec3){0.4f, 0.4f, 0.4f},
                                 identity, (m3Vec3){0.0f, -8.0f, 0.0f});
    CHECK(hit.hit && hit.shape.index1 == chunkShape.index1, "the box cast hits the chunk");
    CHECK(hit.fraction > 0.69f && hit.fraction < 0.71f, "the voxel box-cast fraction is analytic");

    // A cast that misses the chunk entirely.
    hit = m3World_CastSphereClosest(world, (m3Pos3){40.0, 10.0, 8.0}, 0.3f,
                                    (m3Vec3){0.0f, -8.0f, 0.0f});
    CHECK(!hit.hit, "a cast beside the chunk misses");
    m3DestroyWorld(world);
}

static void TestFillWeightsFragments(void)
{
    // Gap 6: half-destroyed voxels weigh half, and the fragment
    // center of mass leans toward the heavy end. A hanging two-arm
    // island with fills 255 and 85 (a 3:1 ratio) has its analytic
    // weighted center at the quarter point.
    m3WorldId world = SmallWorld();
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    static uint8_t voxels[M3_VOXEL_COUNT];
    memset(voxels, 0, sizeof(voxels));
    for (int32_t y = 0; y <= 5; ++y)
    {
        voxels[8 + M3_VOXEL_DIM * (y + M3_VOXEL_DIM * 8)] = 1; // column
    }
    voxels[9 + M3_VOXEL_DIM * (5 + M3_VOXEL_DIM * 8)] = 1;  // arm near
    voxels[10 + M3_VOXEL_DIM * (5 + M3_VOXEL_DIM * 8)] = 1; // arm far
    m3ShapeId tee = m3CreateVoxelChunkShape(ground, &sd, voxels, NULL, 1.0f);

    CHECK(m3VoxelChunk_SetFill(tee, 9, 5, 8, 255), "the near arm keeps full fill");
    CHECK(m3VoxelChunk_SetFill(tee, 10, 5, 8, 85), "the far arm wears to a third");

    CHECK(m3VoxelChunk_ClearVoxel(tee, 8, 5, 8), "the connector clears");
    int32_t count = 0;
    const m3FragmentEvent* events = m3World_FragmentEvents(world, &count);
    CHECK(count == 1, "the two-arm island strands as one fragment");
    // Mass: (255 + 85) / 255 cells = 4/3 of a unit cell.
    CHECK(events[0].mass > 1.32f && events[0].mass < 1.34f, "the fragment weighs its fills");
    // Weighted COM: (3 * 9.5 + 1 * 10.5) / 4 = 9.75 along x.
    CHECK(events[0].comChunk.x > 9.74f && events[0].comChunk.x < 9.76f,
          "the center of mass leans toward the full voxel");
    m3DestroyWorld(world);
}

static void TestSetFillContracts(void)
{
    m3WorldId world = SmallWorld();
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    static uint8_t voxels[M3_VOXEL_COUNT];
    FillSlab(voxels, 2);
    m3ShapeId chunkShape = m3CreateVoxelChunkShape(ground, &sd, voxels, NULL, 1.0f);

    CHECK(!m3VoxelChunk_SetFill(chunkShape, 0, 5, 0, 100), "fill on an empty voxel refuses");
    CHECK(!m3VoxelChunk_SetFill(chunkShape, 0, 0, 0, 0), "zero fill refuses (that is a clear)");
    CHECK(!m3VoxelChunk_SetFill(chunkShape, 16, 0, 0, 100), "out-of-range fill refuses");

    // Fill is state (the hash moves) and never geometry (the
    // surface does not move by a single byte).
    static m3VoxelSurface before;
    memcpy(&before, SurfaceOf(world, chunkShape), sizeof(m3VoxelSurface));
    uint64_t hashBefore = m3World_Hash(world);
    CHECK(m3VoxelChunk_SetFill(chunkShape, 3, 1, 3, 77), "the fill lands");
    CHECK(m3World_Hash(world) != hashBefore, "fill is state: the hash moves");
    CHECK(memcmp(&before, SurfaceOf(world, chunkShape), offsetof(m3VoxelSurface, bvh)) == 0,
          "fill is not geometry: the surface is untouched");

    // And it rides the snapshot like every other bit of state.
    int32_t snapBytes = m3World_SnapshotSize(world);
    uint8_t* snap = (uint8_t*)malloc((size_t)snapBytes);
    CHECK(m3World_Snapshot(world, snap, snapBytes) == snapBytes, "the fill snapshot writes");
    uint64_t at77 = m3World_Hash(world);
    CHECK(m3VoxelChunk_SetFill(chunkShape, 3, 1, 3, 200), "the second fill lands");
    CHECK(m3World_Restore(world, snap, snapBytes), "the fill rollback lands");
    CHECK(m3World_Hash(world) == at77, "fill rides the rollback delta");
    free(snap);
    m3DestroyWorld(world);
}

static void TestEmbeddedRecovery(void)
{
    // Gap 7: a ball born INSIDE a solid voxel block walks out
    // through the nearest exposed face with bounded speed, then
    // rests on the surface. Never a launch, never a NaN, twins.
    uint64_t hashes[2];
    for (int32_t run = 0; run < 2; ++run)
    {
        m3WorldId world = SmallWorld();
        m3BodyDef gd = m3DefaultBodyDef();
        m3BodyId ground = m3CreateBody(world, &gd);
        m3ShapeDef sd = m3DefaultShapeDef();
        static uint8_t voxels[M3_VOXEL_COUNT];
        FillSlab(voxels, 6); // a solid slab, top face at y = 6
        m3ShapeId block = m3CreateVoxelChunkShape(ground, &sd, voxels, NULL, 1.0f);
        CHECK(m3Shape_IsValid(block), "the solid block creates");

        m3BodyDef bd = m3DefaultBodyDef();
        bd.type = m3_dynamicBody;
        bd.position = (m3Pos3){8.5, 4.5, 8.5}; // buried: nearest exit is up
        m3BodyId ball = m3CreateBody(world, &bd);
        m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.4f};
        m3CreateSphereShape(ball, &sd, &sphere);

        float maxSpeed = 0.0f;
        for (int32_t i = 0; i < 600; ++i)
        {
            m3World_Step(world, 1.0f / 60.0f, 4);
            m3Vec3 v = m3Body_GetLinearVelocity(ball);
            float speed2 = v.x * v.x + v.y * v.y + v.z * v.z;
            if (speed2 > maxSpeed)
            {
                maxSpeed = speed2;
            }
        }
        m3Pos3 p = m3Body_GetPosition(ball);
        CHECK(p.y > 6.3 && p.y < 6.5, "the buried ball surfaces and rests on top");
        CHECK(maxSpeed < 100.0f, "the recovery speed stays bounded (no launch)");
        // PointInside finds the ball's own sphere (a center is
        // always inside its own shape); the assertion is that the
        // CHUNK no longer contains it.
        CHECK(m3World_PointInside(world, p).index1 != block.index1,
              "the rest position is outside the solid block");
        hashes[run] = m3World_Hash(world);
        m3DestroyWorld(world);
    }
    CHECK(hashes[0] == hashes[1], "the embedded recovery is bit-deterministic");
}

int main(void)
{
    TestGreedyMergeAnalytics();
    TestRestingOnVoxelFloor();
    TestSnapshotJournalAndDerivedRebuild();
    TestQueriesAgainstVoxels();
    TestVoxelRefusals();
    TestEditsCarveAndWake();
    TestEditRollbackReedit();
    TestEditRefusalsAndEmptyEnd();
    TestFractureBridge();
    TestFractureTwoIslands();
    TestSeamWeldingCoverage();
    TestRollingAcrossSeams();
    TestVoxelMeetsHeightfield();
    TestBulletVsVoxelWall();
    TestShapeCastsAgainstVoxels();
    TestFillWeightsFragments();
    TestSetFillContracts();
    TestEmbeddedRecovery();
    if (s_failures == 0)
    {
        printf("test_voxel: all green\n");
        return 0;
    }
    printf("test_voxel: %d failure(s)\n", s_failures);
    return 1;
}
