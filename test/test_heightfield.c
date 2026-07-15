// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The native terrain gate (19-1): a grid heightfield lands as raw
// samples (the low-memory path beside meshes), refuses hostile
// grids in both doors, and rides the journal and the snapshot
// byte for byte. Contacts open in 19-2.

#include "maul3d/body.h"
#include "maul3d/shape.h"

#include <math.h>
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

static void FillHills(float* h, int32_t nx, int32_t nz)
{
    for (int32_t z = 0; z < nz; ++z)
    {
        for (int32_t x = 0; x < nx; ++x)
        {
            h[z * nx + x] = 0.25f * (float)((x * 7 + z * 3) % 5);
        }
    }
}

static void TestCreateAndWalls(void)
{
    m3WorldDef wd = m3DefaultWorldDef();
    wd.bodyCapacity = 8;
    wd.shapeCapacity = 8;
    m3WorldId world = m3CreateWorld(&wd);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    static float hills[16 * 16];
    FillHills(hills, 16, 16);
    m3ShapeId terrain = m3CreateHeightFieldGridShape(ground, &sd, hills, 16, 16, 0.5f);
    CHECK(m3Shape_IsValid(terrain), "the native grid lands");

    m3BodyDef dd = m3DefaultBodyDef();
    dd.type = m3_dynamicBody;
    dd.position = (m3Pos3){0.0, 5.0, 0.0};
    m3BodyId mover = m3CreateBody(world, &dd);
    uint64_t before = m3World_Hash(world);
    CHECK(!m3Shape_IsValid(m3CreateHeightFieldGridShape(mover, &sd, hills, 16, 16, 0.5f)),
          "a dynamic body refuses terrain");
    CHECK(!m3Shape_IsValid(m3CreateHeightFieldGridShape(ground, &sd, hills, 1, 16, 0.5f)),
          "a one-column grid refuses");
    CHECK(!m3Shape_IsValid(m3CreateHeightFieldGridShape(ground, &sd, hills, 16, 16, 0.0f)),
          "a zero cell refuses");
    static float poisoned[16 * 16];
    FillHills(poisoned, 16, 16);
    poisoned[40] = NAN;
    CHECK(!m3Shape_IsValid(m3CreateHeightFieldGridShape(ground, &sd, poisoned, 16, 16, 0.5f)),
          "a NaN sample refuses");
    CHECK(m3World_Hash(world) == before, "refused terrain moved no bits");
    m3DestroyBody(mover);

    m3DestroyShape(terrain);
    CHECK(!m3Shape_IsValid(terrain), "destroyed terrain goes stale");
    m3DestroyWorld(world);
}

static void TestSnapshotAndReplay(void)
{
    static uint8_t journal[262144];
    static uint8_t snapA[2097152];
    static uint8_t snapB[2097152];
    m3WorldDef wd = m3DefaultWorldDef();
    wd.bodyCapacity = 8;
    wd.shapeCapacity = 8;
    m3WorldId world = m3CreateWorld(&wd);
    bool recording = m3World_JournalBegin(world, journal, (int32_t)sizeof(journal));
    CHECK(recording, "the terrain session records");
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    static float hills[32 * 24];
    FillHills(hills, 32, 24);
    m3ShapeId terrain = m3CreateHeightFieldGridShape(ground, &sd, hills, 32, 24, 0.25f);
    CHECK(m3Shape_IsValid(terrain), "the recorded grid lands");
    for (int32_t i = 0; i < 30; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    int32_t bytes = m3World_JournalEnd(world);
    CHECK(bytes > 0, "the session closes");

    int32_t sizeA = m3World_Snapshot(world, snapA, (int32_t)sizeof(snapA));
    CHECK(sizeA > 0, "the terrain snapshot fits");
    CHECK(m3World_Restore(world, snapA, sizeA), "the terrain restore lands");
    int32_t sizeB = m3World_Snapshot(world, snapB, (int32_t)sizeof(snapB));
    CHECK(sizeB == sizeA && memcmp(snapA, snapB, (size_t)sizeA) == 0,
          "the round-tripped snapshot is byte-identical");

    m3WorldDef fresh = wd;
    m3WorldId replayed = m3CreateWorld(&fresh);
    CHECK(m3World_JournalReplay(replayed, journal, bytes), "the terrain session replays");
    CHECK(m3World_Hash(replayed) == m3World_Hash(world), "the replay is bit-identical");
    m3DestroyWorld(replayed);
    m3DestroyWorld(world);
}

int main(void)
{
    TestCreateAndWalls();
    TestSnapshotAndReplay();
    if (s_failures == 0)
    {
        printf("test_heightfield: all passed\n");
        return 0;
    }
    printf("test_heightfield: %d FAILURES\n", s_failures);
    return 1;
}
