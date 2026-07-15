// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The water gate (18-1): a world-anchored box of water floats what
// should float, sinks what should sink, carries what drifts into
// its current, wakes sleepers when the tide arrives or leaves, and
// rides twins, the journal, and the snapshot. No fluid sim BY LAW.

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

static m3WorldId Pool(m3WaterVolumeId* outWater, float flowX)
{
    // A floor at y = 0 and a 4-meter-deep basin of water above it.
    m3WorldDef wd = m3DefaultWorldDef();
    wd.bodyCapacity = 16;
    wd.shapeCapacity = 16;
    m3WorldId world = m3CreateWorld(&wd);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    m3CreatePlaneShape(ground, &sd, &floor);
    m3WaterVolumeDef wdf = m3DefaultWaterVolumeDef();
    wdf.lo = (m3Pos3){-10.0, 0.0, -10.0};
    wdf.hi = (m3Pos3){10.0, 4.0, 10.0};
    wdf.flow = (m3Vec3){flowX, 0.0f, 0.0f};
    m3WaterVolumeId water = m3CreateWaterVolume(world, &wdf);
    if (outWater != NULL)
    {
        *outWater = water;
    }
    return world;
}

static m3BodyId DropCrate(m3WorldId world, double x, double y, float density)
{
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){x, y, 0.0};
    m3BodyId crate = m3CreateBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.density = density;
    m3CreateBoxShape(crate, &sd, (m3Vec3){0.3f, 0.3f, 0.3f});
    return crate;
}

static void TestFloatAndSink(void)
{
    m3WaterVolumeId water;
    m3WorldId world = Pool(&water, 0.0f);
    CHECK(m3WaterVolume_IsValid(water), "the basin fills");
    m3BodyId cork = DropCrate(world, -2.0, 6.0, 400.0f);  // lighter than water
    m3BodyId anvil = DropCrate(world, 2.0, 6.0, 3000.0f); // heavier than water
    for (int32_t i = 0; i < 420; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    double corkY = m3Body_GetPosition(cork).y;
    double anvilY = m3Body_GetPosition(anvil).y;
    CHECK(corkY > 2.5 && corkY < 4.6, "the light crate floats near the surface");
    CHECK(anvilY < 0.8, "the dense crate sinks to the floor");
    m3DestroyWorld(world);
}

static void TestFlowCarries(void)
{
    m3WorldId still = Pool(NULL, 0.0f);
    m3WorldId river = Pool(NULL, 1.5f);
    m3BodyId a = DropCrate(still, 0.0, 3.5, 400.0f);
    m3BodyId b = DropCrate(river, 0.0, 3.5, 400.0f);
    for (int32_t i = 0; i < 300; ++i)
    {
        m3World_Step(still, 1.0f / 60.0f, 4);
        m3World_Step(river, 1.0f / 60.0f, 4);
    }
    double ax = m3Body_GetPosition(a).x;
    double bx = m3Body_GetPosition(b).x;
    CHECK(fabs(ax) < 0.2, "still water leaves the floater in place");
    CHECK(bx > 2.0, "the current carries the floater downstream");
    m3DestroyWorld(still);
    m3DestroyWorld(river);
}

static void TestTideWakesSleepers(void)
{
    // A crate falls asleep on the dry floor; the tide comes in and
    // it wakes and floats off. Then the tide goes out under a
    // floater and it wakes to fall.
    m3WorldDef wd = m3DefaultWorldDef();
    wd.bodyCapacity = 16;
    wd.shapeCapacity = 16;
    m3WorldId world = m3CreateWorld(&wd);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    m3CreatePlaneShape(ground, &sd, &floor);
    m3BodyId crate = DropCrate(world, 0.0, 0.6, 300.0f);
    for (int32_t i = 0; i < 240; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(!m3Body_IsAwake(crate), "the crate sleeps on the dry floor");
    m3WaterVolumeDef wdf = m3DefaultWaterVolumeDef();
    wdf.lo = (m3Pos3){-5.0, 0.0, -5.0};
    wdf.hi = (m3Pos3){5.0, 3.0, 5.0};
    m3WaterVolumeId tide = m3CreateWaterVolume(world, &wdf);
    CHECK(m3Body_IsAwake(crate), "the incoming tide wakes the sleeper");
    for (int32_t i = 0; i < 420; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m3Body_GetPosition(crate).y > 1.5, "the woken crate floats up");
    m3DestroyWaterVolume(tide);
    for (int32_t i = 0; i < 300; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m3Body_GetPosition(crate).y < 0.8, "the ebb drops the floater back");
    m3DestroyWorld(world);
}

static void TestWaterWalls(void)
{
    m3WorldDef wd = m3DefaultWorldDef();
    wd.bodyCapacity = 8;
    wd.shapeCapacity = 8;
    m3WorldId world = m3CreateWorld(&wd);
    uint64_t before = m3World_Hash(world);
    m3WaterVolumeDef def = m3DefaultWaterVolumeDef();
    def.hi = def.lo; // empty box
    CHECK(!m3WaterVolume_IsValid(m3CreateWaterVolume(world, &def)), "an empty box refuses");
    def = m3DefaultWaterVolumeDef();
    def.density = -5.0f;
    CHECK(!m3WaterVolume_IsValid(m3CreateWaterVolume(world, &def)), "negative density refuses");
    def = m3DefaultWaterVolumeDef();
    def.lo.x = (double)NAN;
    CHECK(!m3WaterVolume_IsValid(m3CreateWaterVolume(world, &def)), "a NaN corner refuses");
    def = m3DefaultWaterVolumeDef();
    def.linearDrag = -1.0f;
    CHECK(!m3WaterVolume_IsValid(m3CreateWaterVolume(world, &def)), "negative drag refuses");
    CHECK(m3World_Hash(world) == before, "refused water moved no bits");
    // The ninth volume refuses loudly.
    m3WaterVolumeId ids[8];
    def = m3DefaultWaterVolumeDef();
    for (int32_t k = 0; k < 8; ++k)
    {
        def.lo = (m3Pos3){(double)k * 3.0, 0.0, 0.0};
        def.hi = (m3Pos3){(double)k * 3.0 + 2.0, 2.0, 2.0};
        ids[k] = m3CreateWaterVolume(world, &def);
        CHECK(m3WaterVolume_IsValid(ids[k]), "eight volumes fit");
    }
    def.lo = (m3Pos3){30.0, 0.0, 0.0};
    def.hi = (m3Pos3){32.0, 2.0, 2.0};
    CHECK(!m3WaterVolume_IsValid(m3CreateWaterVolume(world, &def)), "the ninth refuses");
    m3DestroyWaterVolume(ids[3]);
    CHECK(!m3WaterVolume_IsValid(ids[3]), "a destroyed volume goes stale");
    m3DestroyWaterVolume(ids[3]); // stale destroy: quiet no-op
    m3DestroyWorld(world);
}

static void TestWaterTwinsReplayRollback(void)
{
    static uint8_t journal[131072];
    static uint8_t snap[2097152];
    uint64_t hashes[2];
    for (int32_t run = 0; run < 2; ++run)
    {
        m3WorldDef wd = m3DefaultWorldDef();
        wd.bodyCapacity = 16;
        wd.shapeCapacity = 16;
        m3WorldId w = m3CreateWorld(&wd);
        bool recording = run == 0 && m3World_JournalBegin(w, journal, (int32_t)sizeof(journal));
        m3BodyDef gd = m3DefaultBodyDef();
        m3BodyId ground = m3CreateBody(w, &gd);
        m3ShapeDef sd = m3DefaultShapeDef();
        m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
        m3CreatePlaneShape(ground, &sd, &floor);
        m3WaterVolumeDef wdf = m3DefaultWaterVolumeDef();
        wdf.lo = (m3Pos3){-6.0, 0.0, -6.0};
        wdf.hi = (m3Pos3){6.0, 3.0, 6.0};
        wdf.flow = (m3Vec3){0.4f, 0.0f, 0.0f};
        m3WaterVolumeId water = m3CreateWaterVolume(w, &wdf);
        m3BodyDef bd = m3DefaultBodyDef();
        bd.type = m3_dynamicBody;
        bd.position = (m3Pos3){-2.0, 4.0, 0.0};
        m3BodyId cork = m3CreateBody(w, &bd);
        m3ShapeDef cs = m3DefaultShapeDef();
        cs.density = 400.0f;
        m3CreateBoxShape(cork, &cs, (m3Vec3){0.3f, 0.3f, 0.3f});
        int32_t snapBytes = 0;
        for (int32_t i = 0; i < 180; ++i)
        {
            if (i == 60 && run == 0)
            {
                snapBytes = m3World_Snapshot(w, snap, (int32_t)sizeof(snap));
                CHECK(snapBytes > 0, "the wet snapshot fits");
            }
            if (i == 120)
            {
                m3DestroyWaterVolume(water); // the ebb, journaled
            }
            m3World_Step(w, 1.0f / 60.0f, 4);
        }
        hashes[run] = m3World_Hash(w);
        if (recording)
        {
            int32_t bytes = m3World_JournalEnd(w);
            CHECK(bytes > 0, "the wet session records");
            m3WorldDef fresh = wd;
            m3WorldId replayed = m3CreateWorld(&fresh);
            CHECK(m3World_JournalReplay(replayed, journal, bytes), "the water replays");
            CHECK(m3World_Hash(replayed) == hashes[0], "the replay is bit-identical");
            m3DestroyWorld(replayed);
            CHECK(m3World_Restore(w, snap, snapBytes), "the wet restore lands");
            for (int32_t i = 60; i < 180; ++i)
            {
                if (i == 120)
                {
                    m3DestroyWaterVolume(water);
                }
                m3World_Step(w, 1.0f / 60.0f, 4);
            }
            CHECK(m3World_Hash(w) == hashes[0], "the re-run lands on the same bits");
        }
        m3DestroyWorld(w);
    }
    CHECK(hashes[0] == hashes[1], "twin wet sessions are bit-identical");
}

int main(void)
{
    TestFloatAndSink();
    TestFlowCarries();
    TestTideWakesSleepers();
    TestWaterWalls();
    TestWaterTwinsReplayRollback();
    if (s_failures == 0)
    {
        printf("test_water: all passed\n");
        return 0;
    }
    printf("test_water: %d FAILURES\n", s_failures);
    return 1;
}
