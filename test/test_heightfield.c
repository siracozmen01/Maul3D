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

static void TestContacts(void)
{
    // 19-2: bodies land ON native terrain. A ball rests on a flat
    // grid, a box slides across it without ghost snags, and a ball
    // on a slope rolls downhill.
    m3WorldDef wd = m3DefaultWorldDef();
    wd.bodyCapacity = 16;
    wd.shapeCapacity = 16;
    m3WorldId world = m3CreateWorld(&wd);
    m3BodyDef gd = m3DefaultBodyDef();
    gd.position = (m3Pos3){-8.0, 0.0, -8.0};
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    static float flat[33 * 33]; // zeros: a flat 16m field
    m3ShapeId terrain = m3CreateHeightFieldGridShape(ground, &sd, flat, 33, 33, 0.5f);
    CHECK(m3Shape_IsValid(terrain), "the flat field lands");

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 2.0, 0.0};
    m3BodyId ball = m3CreateBody(world, &bd);
    m3Sphere s = {{0.0f, 0.0f, 0.0f}, 0.4f};
    m3CreateSphereShape(ball, &sd, &s);

    bd.position = (m3Pos3){-4.0, 1.0, 3.0};
    bd.linearVelocity = (m3Vec3){4.0f, 0.0f, 0.0f};
    m3BodyId slider = m3CreateBody(world, &bd);
    m3ShapeDef bs = m3DefaultShapeDef();
    bs.friction = 0.05f;
    m3CreateBoxShape(slider, &bs, (m3Vec3){0.3f, 0.3f, 0.3f});

    double maxSliderY = 0.0;
    for (int32_t i = 0; i < 300; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
        if (i > 60)
        {
            double y = m3Body_GetPosition(slider).y;
            maxSliderY = y > maxSliderY ? y : maxSliderY;
        }
    }
    double ballY = m3Body_GetPosition(ball).y;
    CHECK(ballY > 0.3 && ballY < 0.5, "the ball rests on the field");
    CHECK(m3Body_GetPosition(slider).x > -1.0, "the slick box slides across cells");
    CHECK(maxSliderY < 0.45, "no ghost snag pops the slider");
    m3DestroyWorld(world);

    // The slope: heights rise along x; a ball rolls downhill (-x).
    m3WorldDef wd2 = m3DefaultWorldDef();
    wd2.bodyCapacity = 8;
    wd2.shapeCapacity = 8;
    m3WorldId hill = m3CreateWorld(&wd2);
    m3BodyDef gd2 = m3DefaultBodyDef();
    gd2.position = (m3Pos3){-8.0, 0.0, -8.0};
    m3BodyId ground2 = m3CreateBody(hill, &gd2);
    static float ramp[33 * 33];
    for (int32_t z = 0; z < 33; ++z)
    {
        for (int32_t x = 0; x < 33; ++x)
        {
            ramp[z * 33 + x] = 0.3f * (float)x;
        }
    }
    CHECK(m3Shape_IsValid(m3CreateHeightFieldGridShape(ground2, &sd, ramp, 33, 33, 0.5f)),
          "the ramp lands");
    m3BodyDef rb = m3DefaultBodyDef();
    rb.type = m3_dynamicBody;
    rb.position = (m3Pos3){0.0, 6.0, 0.0}; // the ramp is 4.8 high here
    m3BodyId roller = m3CreateBody(hill, &rb);
    m3CreateSphereShape(roller, &sd, &s);
    for (int32_t i = 0; i < 300; ++i)
    {
        m3World_Step(hill, 1.0f / 60.0f, 4);
    }
    CHECK(m3Body_GetPosition(roller).x < -1.0, "the ball rolls downhill");
    m3DestroyWorld(hill);
}

static void TestContactTwins(void)
{
    uint64_t hashes[2];
    for (int32_t run = 0; run < 2; ++run)
    {
        m3WorldDef wd = m3DefaultWorldDef();
        wd.bodyCapacity = 16;
        wd.shapeCapacity = 16;
        m3WorldId w = m3CreateWorld(&wd);
        m3BodyDef gd = m3DefaultBodyDef();
        gd.position = (m3Pos3){-4.0, 0.0, -4.0};
        m3BodyId ground = m3CreateBody(w, &gd);
        m3ShapeDef sd = m3DefaultShapeDef();
        static float bumps[17 * 17];
        for (int32_t i = 0; i < 17 * 17; ++i)
        {
            bumps[i] = 0.15f * (float)((i * 13) % 7);
        }
        m3CreateHeightFieldGridShape(ground, &sd, bumps, 17, 17, 0.5f);
        m3BodyDef bd = m3DefaultBodyDef();
        bd.type = m3_dynamicBody;
        for (int32_t k = 0; k < 5; ++k)
        {
            bd.position = (m3Pos3){-2.0 + (double)k, 3.0 + 0.3 * (double)k, 0.5 * (double)(k % 3)};
            m3BodyId b = m3CreateBody(w, &bd);
            if (k % 2 == 0)
            {
                m3Sphere sp = {{0.0f, 0.0f, 0.0f}, 0.3f};
                m3CreateSphereShape(b, &sd, &sp);
            }
            else
            {
                m3CreateBoxShape(b, &sd, (m3Vec3){0.25f, 0.25f, 0.25f});
            }
        }
        for (int32_t i = 0; i < 240; ++i)
        {
            m3World_Step(w, 1.0f / 60.0f, 4);
        }
        hashes[run] = m3World_Hash(w);
        m3DestroyWorld(w);
    }
    CHECK(hashes[0] == hashes[1], "twin terrain worlds are bit-identical");
}

int main(void)
{
    TestCreateAndWalls();
    TestSnapshotAndReplay();
    TestContacts();
    TestContactTwins();
    if (s_failures == 0)
    {
        printf("test_heightfield: all passed\n");
        return 0;
    }
    printf("test_heightfield: %d FAILURES\n", s_failures);
    return 1;
}
