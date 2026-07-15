// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The rebuilt tree gate (17-4): m3World_RebuildBroadphase replaces
// a lopsided insertion-order tree with a balanced one WITHOUT
// touching a single answer. Physics stays bit-identical (pairs are
// canonical regardless of tree shape), queries return the same
// shapes, the op journals, and the snapshot carries the new shape.
// Plus the 4-6 carry check: a crate on a gliding kinematic
// platform rides along by friction.

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

static m3WorldId City(void)
{
    // A worst-case insertion order: a long strip of statics created
    // left to right (the classic lopsided tree), plus a rain of
    // dynamic balls.
    m3WorldDef wd = m3DefaultWorldDef();
    wd.bodyCapacity = 64;
    wd.shapeCapacity = 64;
    m3WorldId world = m3CreateWorld(&wd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3BodyDef bd = m3DefaultBodyDef();
    for (int32_t i = 0; i < 24; ++i)
    {
        bd.position = (m3Pos3){-23.0 + 2.0 * (double)i, 0.0, 0.0};
        m3BodyId post = m3CreateBody(world, &bd);
        m3CreateBoxShape(post, &sd, (m3Vec3){1.0f, 0.5f, 1.0f});
    }
    m3BodyDef db = m3DefaultBodyDef();
    db.type = m3_dynamicBody;
    for (int32_t i = 0; i < 8; ++i)
    {
        db.position = (m3Pos3){-14.0 + 4.0 * (double)i, 2.0 + 0.3 * (double)(i % 3), 0.0};
        m3BodyId ball = m3CreateBody(world, &db);
        m3Sphere s = {{0.0f, 0.0f, 0.0f}, 0.4f};
        m3CreateSphereShape(ball, &sd, &s);
    }
    return world;
}

static void TestRebuildChangesNoAnswer(void)
{
    // Twin cities: one rebuilds, one keeps its lopsided tree. The
    // tree is never hashed and pairs are canonical, so 180 stepped
    // frames must land on the SAME hash, and a query must return
    // the same shapes before and after the rebuild.
    m3WorldId plain = City();
    m3WorldId rebuilt = City();

    m3ShapeId before[64];
    int32_t beforeCount = m3World_OverlapSphere(rebuilt, (m3Pos3){0.0, 0.5, 0.0}, 3.0f, before, 64);
    m3World_RebuildBroadphase(rebuilt);
    m3ShapeId after[64];
    int32_t afterCount = m3World_OverlapSphere(rebuilt, (m3Pos3){0.0, 0.5, 0.0}, 3.0f, after, 64);
    CHECK(beforeCount == afterCount, "the rebuild keeps the overlap count");
    bool same = beforeCount == afterCount;
    for (int32_t i = 0; same && i < beforeCount; ++i)
    {
        same = before[i].index1 == after[i].index1;
    }
    CHECK(same, "the rebuild keeps the overlap list, in order");

    m3RayHit h1 =
        m3World_CastRayClosest(plain, (m3Pos3){0.0, 5.0, 0.0}, (m3Vec3){0.0f, -10.0f, 0.0f});
    m3RayHit h2 =
        m3World_CastRayClosest(rebuilt, (m3Pos3){0.0, 5.0, 0.0}, (m3Vec3){0.0f, -10.0f, 0.0f});
    CHECK(h1.hit && h2.hit && h1.shape.index1 == h2.shape.index1 && h1.fraction == h2.fraction,
          "the rebuilt tree casts the same ray");

    for (int32_t i = 0; i < 180; ++i)
    {
        m3World_Step(plain, 1.0f / 60.0f, 4);
        m3World_Step(rebuilt, 1.0f / 60.0f, 4);
    }
    CHECK(m3World_Hash(plain) == m3World_Hash(rebuilt),
          "physics is bit-identical across the rebuild");
    m3DestroyWorld(plain);
    m3DestroyWorld(rebuilt);
}

static void TestRebuildTwinsReplayRollback(void)
{
    static uint8_t journal[262144];
    static uint8_t snap[4194304];
    uint64_t hashes[2];
    for (int32_t run = 0; run < 2; ++run)
    {
        m3WorldId w = City();
        // City() creates before the journal starts, so record a
        // fresh world instead: journal from birth.
        m3DestroyWorld(w);
        m3WorldDef wd = m3DefaultWorldDef();
        wd.bodyCapacity = 64;
        wd.shapeCapacity = 64;
        w = m3CreateWorld(&wd);
        bool recording = run == 0 && m3World_JournalBegin(w, journal, (int32_t)sizeof(journal));
        m3ShapeDef sd = m3DefaultShapeDef();
        m3BodyDef bd = m3DefaultBodyDef();
        for (int32_t i = 0; i < 16; ++i)
        {
            bd.position = (m3Pos3){-15.0 + 2.0 * (double)i, 0.0, 0.0};
            m3CreateBoxShape(m3CreateBody(w, &bd), &sd, (m3Vec3){1.0f, 0.5f, 1.0f});
        }
        m3BodyDef db = m3DefaultBodyDef();
        db.type = m3_dynamicBody;
        db.position = (m3Pos3){0.0, 2.0, 0.0};
        m3BodyId ball = m3CreateBody(w, &db);
        m3Sphere s = {{0.0f, 0.0f, 0.0f}, 0.4f};
        m3CreateSphereShape(ball, &sd, &s);
        m3World_RebuildBroadphase(w);
        int32_t snapBytes = 0;
        for (int32_t i = 0; i < 120; ++i)
        {
            if (i == 40 && run == 0)
            {
                snapBytes = m3World_Snapshot(w, snap, (int32_t)sizeof(snap));
                CHECK(snapBytes > 0, "the rebuilt snapshot fits");
            }
            if (i == 60)
            {
                m3World_RebuildBroadphase(w); // mid-run, journaled
            }
            m3World_Step(w, 1.0f / 60.0f, 4);
        }
        hashes[run] = m3World_Hash(w);
        if (recording)
        {
            int32_t bytes = m3World_JournalEnd(w);
            CHECK(bytes > 0, "the rebuild session records");
            m3WorldDef fresh = wd;
            m3WorldId replayed = m3CreateWorld(&fresh);
            CHECK(m3World_JournalReplay(replayed, journal, bytes), "the rebuild replays");
            CHECK(m3World_Hash(replayed) == hashes[0], "the replay is bit-identical");
            m3DestroyWorld(replayed);
            CHECK(m3World_Restore(w, snap, snapBytes), "the mid-run restore lands");
            for (int32_t i = 40; i < 120; ++i)
            {
                if (i == 60)
                {
                    m3World_RebuildBroadphase(w);
                }
                m3World_Step(w, 1.0f / 60.0f, 4);
            }
            CHECK(m3World_Hash(w) == hashes[0], "the re-run lands on the same bits");
        }
        m3DestroyWorld(w);
    }
    CHECK(hashes[0] == hashes[1], "twin rebuild sessions are bit-identical");
}

static void TestPlatformCarry(void)
{
    // The 4-6 carry check, rigid-body edition: a crate resting on a
    // kinematic platform gliding sideways rides along by friction.
    m3WorldDef wd = m3DefaultWorldDef();
    wd.bodyCapacity = 8;
    wd.shapeCapacity = 8;
    m3WorldId world = m3CreateWorld(&wd);
    m3BodyDef pd = m3DefaultBodyDef();
    pd.type = m3_kinematicBody;
    pd.position = (m3Pos3){0.0, 1.0, 0.0};
    m3BodyId platform = m3CreateBody(world, &pd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.friction = 0.8f;
    m3CreateBoxShape(platform, &sd, (m3Vec3){1.5f, 0.2f, 1.5f});
    m3BodyDef cd = m3DefaultBodyDef();
    cd.type = m3_dynamicBody;
    cd.position = (m3Pos3){0.0, 1.5, 0.0};
    m3BodyId crate = m3CreateBody(world, &cd);
    m3CreateBoxShape(crate, &sd, (m3Vec3){0.25f, 0.25f, 0.25f});
    for (int32_t i = 0; i < 240; ++i)
    {
        double t = (double)(i + 1) / 60.0;
        m3Body_SetTargetTransform(platform, (m3Pos3){0.8 * t, 1.0, 0.0},
                                  (m3Quat){0.0f, 0.0f, 0.0f, 1.0f});
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    double px = m3Body_GetPosition(platform).x;
    double cx = m3Body_GetPosition(crate).x;
    CHECK(px > 3.0, "the platform genuinely glides");
    CHECK(fabs(cx - px) < 0.4, "the crate rides the platform");
    CHECK(m3Body_GetPosition(crate).y > 1.2, "the crate stays aboard");
    m3DestroyWorld(world);
}

int main(void)
{
    TestRebuildChangesNoAnswer();
    TestRebuildTwinsReplayRollback();
    TestPlatformCarry();
    if (s_failures == 0)
    {
        printf("test_rebuild: all passed\n");
        return 0;
    }
    printf("test_rebuild: %d FAILURES\n", s_failures);
    return 1;
}
