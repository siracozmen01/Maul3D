// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The fields gate (11-3): wind streams a flag deterministically
// with gust continuity across rollback, and conveyors carry
// crates at belt speed with the reference tangentVelocity law the
// central-friction port carried at zero until this slice.

#include "maul3d/shape.h"
#include "maul3d/softbody.h"

#include <math.h>
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

static m3WorldId PlaneWorld(m3ShapeId* outFloor)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    def.softBodyCapacity = 4;
    m3WorldId world = m3CreateWorld(&def);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane fl = {{0.0f, 1.0f, 0.0f}, 0.0f};
    m3ShapeId fs = m3CreatePlaneShape(ground, &sd, &fl);
    if (outFloor != NULL)
    {
        *outFloor = fs;
    }
    return world;
}

static void TestWindStreamsTheFlag(void)
{
    // A cloth pinned along one edge streams in a gusty wind: it
    // rises from the vertical hang, twins agree to the bit, and
    // the gust phase survives rollback (same wave, same bits).
    static uint8_t snap[2097152];
    uint64_t hashes[2];
    for (int32_t run = 0; run < 2; ++run)
    {
        m3WorldId world = PlaneWorld(NULL);
        m3SoftBodyDef cd = m3DefaultSoftBodyDef();
        cd.countX = 6;
        cd.countY = 6;
        cd.countZ = 1;
        cd.spacing = 0.25f;
        cd.radius = 0.08f;
        cd.position = (m3Pos3){0.0, 2.0, 0.0};
        m3SoftBodyId flag = m3CreateSoftBody(world, &cd);
        for (int32_t r = 0; r < 6; ++r)
        {
            m3SoftBody_PinParticle(flag, r * 6); // the mast column
        }
        m3World_SetWind(world, (m3Vec3){1.0f, 0.0f, 0.0f}, 6.0f, 0.8f, 0.5f);
        int32_t snapBytes = 0;
        for (int32_t i = 0; i < 240; ++i)
        {
            m3World_Step(world, 1.0f / 60.0f, 4);
            if (i == 120 && run == 0)
            {
                snapBytes = m3World_Snapshot(world, snap, (int32_t)sizeof(snap));
                CHECK(snapBytes > 0, "the mid-gust snapshot fits");
            }
        }
        // The free edge must stream downwind, not hang.
        double freeX = 0.0;
        for (int32_t r = 0; r < 6; ++r)
        {
            freeX += m3SoftBody_GetParticlePosition(flag, r * 6 + 5).x;
        }
        freeX /= 6.0;
        CHECK(freeX > 0.8, "the flag streams downwind");
        uint64_t final = m3World_Hash(world);
        hashes[run] = final;
        if (run == 0)
        {
            CHECK(m3World_Restore(world, snap, snapBytes), "the mid-gust restore lands");
            for (int32_t i = 121; i < 240; ++i)
            {
                m3World_Step(world, 1.0f / 60.0f, 4);
            }
            CHECK(m3World_Hash(world) == final,
                  "the gust wave continues across rollback bit-exactly");
        }
        m3DestroyWorld(world);
    }
    CHECK(hashes[0] == hashes[1], "windy twins are bit-identical");
}

static void TestConveyorCarries(void)
{
    // A crate dropped on a belt reaches belt speed; on the second
    // half the belt reverses and the crate follows.
    m3ShapeId floor;
    m3WorldId world = PlaneWorld(&floor);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 0.5, 0.0};
    m3BodyId crate = m3CreateBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3CreateBoxShape(crate, &sd, (m3Vec3){0.4f, 0.4f, 0.4f});
    m3Shape_SetSurfaceVelocity(floor, (m3Vec3){1.5f, 0.0f, 0.0f});
    for (int32_t i = 0; i < 120; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3Vec3 v = m3Body_GetLinearVelocity(crate);
    CHECK(fabsf(v.x - 1.5f) < 0.1f, "the crate rides the belt at belt speed");

    m3Shape_SetSurfaceVelocity(floor, (m3Vec3){-1.0f, 0.0f, 0.0f});
    for (int32_t i = 0; i < 180; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    v = m3Body_GetLinearVelocity(crate);
    CHECK(fabsf(v.x + 1.0f) < 0.1f, "the reversed belt carries it back");
    m3DestroyWorld(world);
}

static void TestFieldsReplay(void)
{
    // Wind and conveyor ops through the journal, bit-exact.
    static uint8_t journal[131072];
    uint64_t hashes[2];
    for (int32_t run = 0; run < 2; ++run)
    {
        m3WorldDef def = m3DefaultWorldDef();
        def.bodyCapacity = 16;
        def.shapeCapacity = 16;
        def.softBodyCapacity = 4;
        m3WorldId world = m3CreateWorld(&def);
        bool recording = run == 0 && m3World_JournalBegin(world, journal, (int32_t)sizeof(journal));
        m3BodyDef gd = m3DefaultBodyDef();
        m3BodyId ground = m3CreateBody(world, &gd);
        m3ShapeDef sg = m3DefaultShapeDef();
        m3Plane fl = {{0.0f, 1.0f, 0.0f}, 0.0f};
        m3ShapeId floor = m3CreatePlaneShape(ground, &sg, &fl);
        m3BodyDef bd = m3DefaultBodyDef();
        bd.type = m3_dynamicBody;
        bd.position = (m3Pos3){0.0, 0.5, 0.0};
        m3BodyId crate = m3CreateBody(world, &bd);
        m3ShapeDef sd = m3DefaultShapeDef();
        m3CreateBoxShape(crate, &sd, (m3Vec3){0.3f, 0.3f, 0.3f});
        m3SoftBodyDef cd = m3DefaultSoftBodyDef();
        cd.countX = 4;
        cd.countY = 4;
        cd.countZ = 1;
        cd.spacing = 0.25f;
        cd.radius = 0.08f;
        cd.position = (m3Pos3){2.0, 2.0, 0.0};
        m3SoftBodyId flag = m3CreateSoftBody(world, &cd);
        m3SoftBody_PinParticle(flag, 0);
        m3SoftBody_PinParticle(flag, 12);
        for (int32_t i = 0; i < 120; ++i)
        {
            if (i == 20)
            {
                m3World_SetWind(world, (m3Vec3){0.0f, 0.0f, 1.0f}, 4.0f, 1.2f, 0.3f);
            }
            if (i == 40)
            {
                m3Shape_SetSurfaceVelocity(floor, (m3Vec3){0.8f, 0.0f, 0.0f});
            }
            if (i == 80)
            {
                m3World_SetWind(world, (m3Vec3){0.0f, 0.0f, 1.0f}, 2.0f, 1.2f, 0.3f);
            }
            m3World_Step(world, 1.0f / 60.0f, 4);
        }
        uint64_t final = m3World_Hash(world);
        hashes[run] = final;
        if (recording)
        {
            int32_t bytes = m3World_JournalEnd(world);
            CHECK(bytes > 0, "the fields session records");
            m3WorldDef fdef = m3DefaultWorldDef();
            fdef.bodyCapacity = 16;
            fdef.shapeCapacity = 16;
            fdef.softBodyCapacity = 4;
            m3WorldId fresh = m3CreateWorld(&fdef);
            CHECK(m3World_JournalReplay(fresh, journal, bytes), "the fields session replays");
            CHECK(m3World_Hash(fresh) == final, "the replay is bit-identical");
            m3DestroyWorld(fresh);
        }
        m3DestroyWorld(world);
    }
    CHECK(hashes[0] == hashes[1], "field twins are bit-identical");
}

static void TestHostileFields(void)
{
    m3ShapeId floor;
    m3WorldId world = PlaneWorld(&floor);
    m3World_SetWind(world, (m3Vec3){3.0f, 0.0f, 0.0f}, 5.0f, 1.0f, 0.2f); // non-unit dir
    m3World_SetWind(world, (m3Vec3){NAN, 0.0f, 0.0f}, 5.0f, 1.0f, 0.2f);
    m3World_SetWind(world, (m3Vec3){1.0f, 0.0f, 0.0f}, -5.0f, 1.0f, 0.2f);
    m3Shape_SetSurfaceVelocity(floor, (m3Vec3){NAN, 0.0f, 0.0f});
    m3ShapeId stale = floor;
    stale.index1 += 500;
    m3Shape_SetSurfaceVelocity(stale, (m3Vec3){1.0f, 0.0f, 0.0f});
    uint64_t before = m3World_Hash(world);
    m3World_Step(world, 1.0f / 60.0f, 4);
    m3World_Step(world, 1.0f / 60.0f, 4);
    (void)before;
    CHECK(m3World_IsValid(world), "hostile fields left the world whole");
    m3DestroyWorld(world);
}

int main(void)
{
    TestWindStreamsTheFlag();
    TestConveyorCarries();
    TestFieldsReplay();
    TestHostileFields();
    if (s_failures == 0)
    {
        printf("test_fields: all green\n");
        return 0;
    }
    printf("test_fields: %d failure(s)\n", s_failures);
    return 1;
}
