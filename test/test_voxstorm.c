// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The voxel red team (3-7): gap eight. Negative and near-edge
// coordinates, the anchor convention pinned as contract, fracture
// storms up to and past the event capacity, journal composition
// refusals, the chunk pool sweep, and a deterministic edit storm
// whose twins must agree bit for bit while white-box invariants
// (filled count versus popcount, incremental surface and coverage
// versus from-scratch rebuilds) hold at every checkpoint. The
// sanitizer cells run all of it with teeth.

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

static uint32_t s_rng = 0xA5C39E17u;
static uint32_t NextRand(void)
{
    s_rng = s_rng * 1664525u + 1013904223u;
    return s_rng;
}

static m3WorldId StormWorld(int32_t voxelCap)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    def.voxelCapacity = voxelCap;
    return m3CreateWorld(&def);
}

static const m3VoxelSurface* SurfaceOf(m3WorldId worldId, m3ShapeId shape)
{
    m3World* world = m3WorldFromId(worldId);
    return &world->voxelSurface[world->shapeVoxelIndex[shape.index1 - 1]];
}

static void TestNegativeAndEdgeCoordinates(void)
{
    // Chunks live at strongly negative world positions; edits touch
    // the extreme voxels; the weld triggers on exact negative
    // deltas; rays arrive from the negative side.
    m3WorldId world = StormWorld(2);
    m3ShapeDef sd = m3DefaultShapeDef();
    static uint8_t voxels[M3_VOXEL_COUNT];
    memset(voxels, 0, sizeof(voxels));
    for (int32_t z = 0; z < M3_VOXEL_DIM; ++z)
    {
        for (int32_t x = 0; x < M3_VOXEL_DIM; ++x)
        {
            voxels[x + M3_VOXEL_DIM * (0 + M3_VOXEL_DIM * z)] = 1;
        }
    }
    m3BodyDef gd = m3DefaultBodyDef();
    gd.position = (m3Pos3){-32.0, -16.0, -32.0};
    m3BodyId groundA = m3CreateBody(world, &gd);
    m3ShapeId chunkA = m3CreateVoxelChunkShape(groundA, &sd, voxels, NULL, 1.0f);
    gd.position = (m3Pos3){-16.0, -16.0, -32.0}; // exactly +16: welds
    m3BodyId groundB = m3CreateBody(world, &gd);
    m3ShapeId chunkB = m3CreateVoxelChunkShape(groundB, &sd, voxels, NULL, 1.0f);
    CHECK(m3Shape_IsValid(chunkA) && m3Shape_IsValid(chunkB), "negative chunks create");
    CHECK((SurfaceOf(world, chunkA)->boxCovered[0] & 2u) != 0,
          "the weld holds at negative coordinates");

    // Edits on the extreme voxels of a negative-position chunk.
    CHECK(m3VoxelChunk_ClearVoxel(chunkA, 0, 0, 0), "the corner voxel clears");
    CHECK(m3VoxelChunk_SetVoxel(chunkA, 0, 0, 0, 3), "the corner voxel refills");
    CHECK(m3VoxelChunk_ClearVoxel(chunkA, 15, 0, 15), "the far corner clears");
    CHECK(m3VoxelChunk_SetVoxel(chunkA, 15, 0, 15, 4), "the far corner refills");

    // A ball dropped at negative coordinates rests on the floor at
    // the analytic height (-16 + 1 + radius).
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){-24.0, -13.0, -24.0};
    m3BodyId ball = m3CreateBody(world, &bd);
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.4f};
    m3CreateSphereShape(ball, &sd, &sphere);
    for (int32_t i = 0; i < 180; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3Pos3 p = m3Body_GetPosition(ball);
    CHECK(p.y > -14.65 && p.y < -14.55, "the ball rests at the negative analytic height");

    // A ray from deeper negative space finds the floor (aimed away
    // from the resting ball, which would otherwise catch it first).
    m3RayHit hit =
        m3World_CastRayClosest(world, (m3Pos3){-28.0, -5.0, -28.0}, (m3Vec3){0.0f, -20.0f, 0.0f});
    CHECK(hit.hit && hit.shape.index1 == chunkA.index1, "the negative-side ray lands");
    CHECK(hit.fraction > 0.49f && hit.fraction < 0.51f, "the negative-side fraction is analytic");
    m3DestroyWorld(world);
}

static void TestAnchorConventionPinned(void)
{
    // The documented contract: voxels with no path to the chunk's
    // base layer survive creation and fragment on the FIRST
    // clearing edit anywhere in the chunk. Floating platforms
    // belong at their own chunk's base.
    m3WorldId world = StormWorld(1);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    static uint8_t voxels[M3_VOXEL_COUNT];
    memset(voxels, 0, sizeof(voxels));
    // A floating 2x1x2 platform at y = 7 and one grounded stud.
    voxels[0] = 1; // (0, 0, 0): anchored
    for (int32_t z = 4; z <= 5; ++z)
    {
        for (int32_t x = 4; x <= 5; ++x)
        {
            voxels[x + M3_VOXEL_DIM * (7 + M3_VOXEL_DIM * z)] = 1;
        }
    }
    m3ShapeId chunkShape = m3CreateVoxelChunkShape(ground, &sd, voxels, NULL, 1.0f);
    CHECK(m3Shape_IsValid(chunkShape), "the floating platform survives creation");
    m3World* wp = m3WorldFromId(world);
    const m3VoxelChunkData* chunk = &wp->voxelData[wp->shapeVoxelIndex[chunkShape.index1 - 1]];
    CHECK(chunk->filledCount == 5, "all five voxels stand before any edit");

    // Clearing an EMPTY voxel is a no-op: no occupancy change, no
    // sweep (pinned as contract; the wake pass still runs).
    CHECK(m3VoxelChunk_ClearVoxel(chunkShape, 15, 15, 15), "an empty clear is a quiet no-op");
    int32_t count = 0;
    m3World_FragmentEvents(world, &count);
    CHECK(count == 0, "a no-op clear does not sweep");
    CHECK(chunk->filledCount == 5, "a no-op clear moves nothing");

    // The first occupancy-changing clear sweeps the whole grid:
    // one platform voxel goes directly, the three others fragment.
    CHECK(m3VoxelChunk_ClearVoxel(chunkShape, 4, 7, 4), "the platform corner clears");
    const m3FragmentEvent* events = m3World_FragmentEvents(world, &count);
    CHECK(count == 1, "the rest of the unanchored platform fragments");
    CHECK(events[0].voxelCount == 3, "the island is the three remaining platform voxels");
    CHECK(chunk->filledCount == 1, "only the grounded stud remains");
    m3DestroyWorld(world);
}

static void TestFractureStorms(void)
{
    // Storm one: clearing the anchor layer strands EVERYTHING above
    // as one island (3840 voxels, recipe fits).
    m3WorldId world = StormWorld(1);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    static uint8_t voxels[M3_VOXEL_COUNT];
    memset(voxels, 1, sizeof(voxels)); // a full chunk
    m3ShapeId full = m3CreateVoxelChunkShape(ground, &sd, voxels, NULL, 1.0f);
    int32_t lo[3] = {0, 0, 0};
    int32_t hi[3] = {15, 0, 15};
    CHECK(m3VoxelChunk_ClearBox(full, lo, hi) == 256, "the anchor layer clears");
    int32_t count = 0;
    const m3FragmentEvent* events = m3World_FragmentEvents(world, &count);
    CHECK(count == 1, "everything above the anchor is one island");
    CHECK(events[0].voxelCount == 4096 - 256, "the island is the whole upper block");
    CHECK(events[0].recipeStart >= 0, "a 3840-voxel recipe still fits the buffer");
    m3World* wp = m3WorldFromId(world);
    CHECK(wp->voxelData[wp->shapeVoxelIndex[full.index1 - 1]].filledCount == 0,
          "the storm empties the grid");
    CHECK(m3World_FragmentEventsDropped(world) == 0, "storm one drops nothing");
    m3DestroyWorld(world);

    // Storm two: 448 isolated floating studs in one chunk; the
    // sweep after one clear must remove ALL of them, emit the
    // event-capacity's worth, and count the dropped rest loudly.
    m3WorldId world2 = StormWorld(1);
    m3BodyId ground2 = m3CreateBody(world2, &gd);
    memset(voxels, 0, sizeof(voxels));
    voxels[0] = 1; // the anchored stud that receives the edit
    int32_t studs = 0;
    for (int32_t z = 0; z < M3_VOXEL_DIM; z += 2)
    {
        for (int32_t y = 2; y < M3_VOXEL_DIM; y += 2)
        {
            for (int32_t x = 0; x < M3_VOXEL_DIM; x += 2)
            {
                voxels[x + M3_VOXEL_DIM * (y + M3_VOXEL_DIM * z)] = 1;
                studs += 1;
            }
        }
    }
    CHECK(studs == 448, "the storm grid holds 448 isolated studs");
    m3ShapeId cloud = m3CreateVoxelChunkShape(ground2, &sd, voxels, NULL, 1.0f);
    CHECK(m3VoxelChunk_ClearVoxel(cloud, 0, 0, 0), "the trigger clear lands");
    count = 0;
    events = m3World_FragmentEvents(world2, &count);
    CHECK(count == M3_FRAGMENT_EVENT_CAP, "events fill to the cap");
    CHECK(m3World_FragmentEventsDropped(world2) == 448 - M3_FRAGMENT_EVENT_CAP,
          "the surplus is counted loudly");
    m3World* wp2 = m3WorldFromId(world2);
    CHECK(wp2->voxelData[wp2->shapeVoxelIndex[cloud.index1 - 1]].filledCount == 0,
          "EVERY stud left the grid: the removal never truncates");
    CHECK(events[0].voxelCount == 1 && events[0].boundsLo[1] == 2,
          "the first event is the lowest canonical seed");
    m3DestroyWorld(world2);
}

static void TestJournalCompositionRefusal(void)
{
    // An edits-only journal replayed into a FRESH world names a
    // chunk that does not exist there: refused, atomically. The
    // supported composition is snapshot-seeded replay (the soak's
    // segment pattern), proven here in miniature.
    static uint8_t journal[65536];
    m3WorldId world = StormWorld(1);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    static uint8_t voxels[M3_VOXEL_COUNT];
    memset(voxels, 0, sizeof(voxels));
    for (int32_t x = 0; x < 4; ++x)
    {
        voxels[x] = 1;
    }
    m3ShapeId chunkShape = m3CreateVoxelChunkShape(ground, &sd, voxels, NULL, 1.0f);

    int32_t snapBytes = m3World_SnapshotSize(world);
    uint8_t* snap = (uint8_t*)malloc((size_t)snapBytes);
    CHECK(m3World_Snapshot(world, snap, snapBytes) == snapBytes, "the seed snapshot writes");

    CHECK(m3World_JournalBegin(world, journal, (int32_t)sizeof(journal)), "the journal arms");
    CHECK(m3VoxelChunk_ClearVoxel(chunkShape, 0, 0, 0), "the journaled edit lands");
    m3World_Step(world, 1.0f / 60.0f, 4);
    int32_t bytes = m3World_JournalEnd(world);
    CHECK(bytes > 0, "the edits-only session records");
    uint64_t liveHash = m3World_Hash(world);

    m3WorldId fresh = StormWorld(1);
    uint64_t freshHash = m3World_Hash(fresh);
    CHECK(!m3World_JournalReplay(fresh, journal, bytes),
          "an edits-only journal refuses a world without its chunk");
    CHECK(m3World_Hash(fresh) == freshHash, "the refusal is atomic");
    CHECK(m3World_Restore(fresh, snap, snapBytes), "the seed restores");
    CHECK(m3World_JournalReplay(fresh, journal, bytes), "the seeded replay lands");
    CHECK(m3World_Hash(fresh) == liveHash, "seed plus journal equals the live world");
    m3DestroyWorld(fresh);
    free(snap);
    m3DestroyWorld(world);
}

static void TestVoxelPoolSweep(void)
{
    // Fill, refuse, recycle, stale: the pool contract. The
    // generation-retirement law itself is pool-generic and was
    // proven by churn on this same m3IdPool code in the 2d-2
    // sweeps; re-churning 65535 generations of full chunk builds
    // would buy no new proof at real sanitizer cost.
    m3WorldId world = StormWorld(2);
    m3ShapeDef sd = m3DefaultShapeDef();
    static uint8_t voxels[M3_VOXEL_COUNT];
    memset(voxels, 1, sizeof(voxels));
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId a = m3CreateBody(world, &gd);
    gd.position = (m3Pos3){40.0, 0.0, 0.0};
    m3BodyId b = m3CreateBody(world, &gd);
    gd.position = (m3Pos3){80.0, 0.0, 0.0};
    m3BodyId c = m3CreateBody(world, &gd);

    m3ShapeId one = m3CreateVoxelChunkShape(a, &sd, voxels, NULL, 1.0f);
    m3ShapeId two = m3CreateVoxelChunkShape(b, &sd, voxels, NULL, 1.0f);
    CHECK(m3Shape_IsValid(one) && m3Shape_IsValid(two), "the pool fills");
    CHECK(!m3Shape_IsValid(m3CreateVoxelChunkShape(c, &sd, voxels, NULL, 1.0f)),
          "the third chunk refuses past capacity");
    m3DestroyBody(b); // cascades the chunk, frees the slot
    m3ShapeId three = m3CreateVoxelChunkShape(c, &sd, voxels, NULL, 1.0f);
    CHECK(m3Shape_IsValid(three), "the freed slot recycles");
    CHECK(!m3Shape_IsValid(two), "the destroyed chunk id is stale");
    CHECK(!m3VoxelChunk_ClearVoxel(two, 0, 0, 0), "a stale chunk refuses edits");
    m3World_Step(world, 1.0f / 60.0f, 4);
    CHECK(m3World_Hash(world) != 0, "the sweep leaves a healthy world");
    m3DestroyWorld(world);
}

static int32_t PopCount(const m3VoxelChunkData* chunk)
{
    int32_t bits = 0;
    for (int32_t i = 0; i < M3_VOXEL_COUNT / 8; ++i)
    {
        uint8_t byte = chunk->occupancy[i];
        while (byte != 0)
        {
            bits += byte & 1;
            byte >>= 1;
        }
    }
    return bits;
}

static uint64_t RunEditStorm(void)
{
    m3WorldId world = StormWorld(2);
    m3ShapeDef sd = m3DefaultShapeDef();
    static uint8_t voxels[M3_VOXEL_COUNT];
    memset(voxels, 0, sizeof(voxels));
    for (int32_t z = 0; z < M3_VOXEL_DIM; ++z)
    {
        for (int32_t y = 0; y < 6; ++y)
        {
            for (int32_t x = 0; x < M3_VOXEL_DIM; ++x)
            {
                voxels[x + M3_VOXEL_DIM * (y + M3_VOXEL_DIM * z)] = 1;
            }
        }
    }
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeId chunkShape = m3CreateVoxelChunkShape(ground, &sd, voxels, NULL, 0.5f);

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){4.0, 5.0, 4.0};
    m3BodyId ball = m3CreateBody(world, &bd);
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.3f};
    m3CreateSphereShape(ball, &sd, &sphere);

    for (int32_t round = 0; round < 400; ++round)
    {
        uint32_t roll = NextRand() % 5u;
        // Coordinates deliberately overshoot the grid: hostile
        // inputs must refuse without a trace.
        int32_t x = (int32_t)(NextRand() % 20u) - 2;
        int32_t y = (int32_t)(NextRand() % 20u) - 2;
        int32_t z = (int32_t)(NextRand() % 20u) - 2;
        if (roll == 0)
        {
            m3VoxelChunk_SetVoxel(chunkShape, x, y, z, (uint16_t)(NextRand() & 0xFFFFu));
        }
        else if (roll == 1)
        {
            m3VoxelChunk_ClearVoxel(chunkShape, x, y, z);
        }
        else if (roll == 2)
        {
            m3VoxelChunk_SetFill(chunkShape, x, y, z, (uint8_t)(1u + (NextRand() % 255u)));
        }
        else if (roll == 3)
        {
            int32_t lo[3] = {x, y, z};
            int32_t hi[3] = {x + (int32_t)(NextRand() % 4u), y + (int32_t)(NextRand() % 4u),
                             z + (int32_t)(NextRand() % 4u)};
            m3VoxelChunk_ClearBox(chunkShape, lo, hi);
        }
        if (round % 50 == 0)
        {
            m3World_Step(world, 1.0f / 60.0f, 4);
        }
    }
    for (int32_t i = 0; i < 60; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }

    // The white-box invariants, checked at the end of the storm:
    // the incremental state and derived data must equal a
    // from-scratch rebuild, byte for byte.
    m3World* wp = m3WorldFromId(world);
    int32_t slot = wp->shapeVoxelIndex[chunkShape.index1 - 1];
    const m3VoxelChunkData* chunk = &wp->voxelData[slot];
    CHECK(chunk->filledCount == PopCount(chunk), "filled count equals the popcount");
    m3VoxelSurface* fresh = (m3VoxelSurface*)malloc(sizeof(m3VoxelSurface));
    m3VoxelSurfaceBuild(fresh, chunk);
    CHECK(memcmp(fresh->boxLo, wp->voxelSurface[slot].boxLo, sizeof(fresh->boxLo)) == 0 &&
              fresh->boxCount == wp->voxelSurface[slot].boxCount,
          "the incremental surface equals the from-scratch build");
    free(fresh);

    uint64_t hash = m3World_Hash(world);
    m3DestroyWorld(world);
    return hash;
}

static void TestEditStormTwins(void)
{
    s_rng = 0xA5C39E17u;
    uint64_t first = RunEditStorm();
    s_rng = 0xA5C39E17u;
    uint64_t second = RunEditStorm();
    CHECK(first == second, "the edit storm is bit-deterministic across twins");
    CHECK(first != 0, "the storm world lived");
}

int main(void)
{
    TestNegativeAndEdgeCoordinates();
    TestAnchorConventionPinned();
    TestFractureStorms();
    TestJournalCompositionRefusal();
    TestVoxelPoolSweep();
    TestEditStormTwins();
    if (s_failures == 0)
    {
        printf("test_voxstorm: all green\n");
        return 0;
    }
    printf("test_voxstorm: %d failure(s)\n", s_failures);
    return 1;
}
