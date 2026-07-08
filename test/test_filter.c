// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The filter gate (8-1): categories, masks, and groups decide who
// touches whom, queries carry their own filter, sensors and the
// continuous phase obey the same rule, and filtered worlds
// journal, replay, and roll back onto identical bits.

#include "maul3d/shape.h"

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

enum
{
    CAT_GROUND = 1u << 0,
    CAT_DEBRIS = 1u << 1,
    CAT_PLAYER = 1u << 2,
    CAT_GHOST = 1u << 3
};

static m3WorldId GroundWorld(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 32;
    def.shapeCapacity = 32;
    m3WorldId world = m3CreateWorld(&def);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.categoryBits = CAT_GROUND;
    sd.maskBits = ~0ull;
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    m3CreatePlaneShape(ground, &sd, &floor);
    return world;
}

static m3BodyId DropBall(m3WorldId world, double x, uint64_t category, uint64_t mask)
{
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){x, 2.0, 0.0};
    m3BodyId body = m3CreateBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.categoryBits = category;
    sd.maskBits = mask;
    m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.5f};
    m3CreateSphereShape(body, &sd, &ball);
    return body;
}

static void TestMaskGate(void)
{
    // A ghost whose mask excludes the ground falls straight
    // through; a debris ball with a full mask lands. The gate is
    // symmetric: BOTH directions must agree, so a one-sided mask
    // still separates the pair.
    m3WorldId world = GroundWorld();
    m3BodyId solid = DropBall(world, -2.0, CAT_DEBRIS, ~0ull);
    m3BodyId ghost = DropBall(world, 2.0, CAT_GHOST, CAT_DEBRIS); // sees debris only
    for (int32_t i = 0; i < 120; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m3Body_GetPosition(solid).y > 0.4, "the full-mask ball lands on the plane");
    CHECK(m3Body_GetPosition(ghost).y < -3.0, "the masked ghost falls through the world");
    m3DestroyWorld(world);
}

static void TestGroupOverride(void)
{
    // Two debris balls whose masks EXCLUDE each other but share a
    // positive group: the group forces the collision. Then the
    // same pair with a negative group: never, even with full
    // masks.
    m3WorldId world = GroundWorld();
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 0.5, 0.0};
    m3BodyId a = m3CreateBody(world, &bd);
    m3ShapeDef sa = m3DefaultShapeDef();
    sa.categoryBits = CAT_DEBRIS;
    sa.maskBits = CAT_GROUND; // does NOT see debris
    sa.groupIndex = 7;
    m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.5f};
    m3CreateSphereShape(a, &sa, &ball);
    bd.position = (m3Pos3){0.4, 2.0, 0.0}; // will fall onto a
    m3BodyId b = m3CreateBody(world, &bd);
    m3CreateSphereShape(b, &sa, &ball);
    for (int32_t i = 0; i < 120; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    // The positive group forced the pair: b rests on or beside a,
    // not inside it.
    m3Pos3 pa = m3Body_GetPosition(a);
    m3Pos3 pb = m3Body_GetPosition(b);
    double dx = pb.x - pa.x;
    double dy = pb.y - pa.y;
    double dz = pb.z - pa.z;
    CHECK(dx * dx + dy * dy + dz * dz > 0.9 * 0.9, "a positive group forces the collision");

    // Negative group: full masks, never collide.
    m3BodyDef cd = m3DefaultBodyDef();
    cd.type = m3_dynamicBody;
    cd.position = (m3Pos3){5.0, 0.5, 0.0};
    m3BodyId c = m3CreateBody(world, &cd);
    m3ShapeDef sc = m3DefaultShapeDef();
    sc.groupIndex = -3;
    m3CreateSphereShape(c, &sc, &ball);
    cd.position = (m3Pos3){5.0, 1.49, 0.0}; // starts exactly on top
    m3BodyId d = m3CreateBody(world, &cd);
    m3CreateSphereShape(d, &sc, &ball);
    for (int32_t i = 0; i < 120; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3Pos3 pd = m3Body_GetPosition(d);
    CHECK(pd.y < 1.1 && pd.y > 0.4, "a negative group never collides: d sinks into c's space");
    m3DestroyWorld(world);
}

static void TestQueryFilters(void)
{
    // The ray sees what its filter admits: a debris-only filter
    // skips the player capsule standing in the beam, and the
    // filtered sphere cast agrees. All-hits respects it too.
    m3WorldId world = GroundWorld();
    m3BodyDef bd = m3DefaultBodyDef();
    bd.position = (m3Pos3){0.0, 1.0, 0.0};
    m3BodyId player = m3CreateBody(world, &bd);
    m3ShapeDef sp = m3DefaultShapeDef();
    sp.categoryBits = CAT_PLAYER;
    m3Capsule cap = {{0.0f, -0.5f, 0.0f}, {0.0f, 0.5f, 0.0f}, 0.4f};
    m3CreateCapsuleShape(player, &sp, &cap);
    bd.position = (m3Pos3){0.0, 1.0, 3.0};
    m3BodyId crate = m3CreateBody(world, &bd);
    m3ShapeDef sc = m3DefaultShapeDef();
    sc.categoryBits = CAT_DEBRIS;
    m3CreateBoxShape(crate, &sc, (m3Vec3){0.5f, 0.5f, 0.5f});

    m3Pos3 origin = {0.0, 1.0, -3.0};
    m3Vec3 dir = {0.0f, 0.0f, 8.0f};
    m3RayHit any = m3World_CastRayClosest(world, origin, dir);
    CHECK(any.hit && any.point.z < 1.0, "the unfiltered ray stops at the player");
    m3QueryFilter debrisOnly = m3DefaultQueryFilter();
    debrisOnly.maskBits = CAT_DEBRIS;
    m3RayHit through = m3World_CastRayClosestEx(world, origin, dir, debrisOnly);
    CHECK(through.hit && through.point.z > 2.0, "the filtered ray skips the player");

    m3RayHit cast = m3World_CastSphereClosestEx(world, origin, 0.2f, dir, debrisOnly);
    CHECK(cast.hit && cast.point.z > 2.0, "the filtered sphere cast agrees");

    m3RayHit hits[8];
    int32_t n = m3World_CastRayAllEx(world, origin, dir, hits, 8, debrisOnly);
    for (int32_t k = 0; k < n; ++k)
    {
        CHECK(hits[k].shape.index1 != 0, "all-hits entries are real");
        CHECK(hits[k].point.z > 2.0, "all-hits skips the player too");
    }
    m3DestroyWorld(world);
}

static void TestFilteredReplayAndRollback(void)
{
    // Filters ride the def through the journal: a filtered world
    // replays and rolls back onto identical bits, and twin runs
    // agree.
    static uint8_t journal[262144];
    static uint8_t snap[786432];
    uint64_t hashes[2];
    for (int32_t run = 0; run < 2; ++run)
    {
        m3WorldDef def = m3DefaultWorldDef();
        def.bodyCapacity = 32;
        def.shapeCapacity = 32;
        m3WorldId world = m3CreateWorld(&def);
        bool recording = run == 0 && m3World_JournalBegin(world, journal, (int32_t)sizeof(journal));
        m3BodyDef gd = m3DefaultBodyDef();
        m3BodyId ground = m3CreateBody(world, &gd);
        m3ShapeDef sg = m3DefaultShapeDef();
        sg.categoryBits = CAT_GROUND;
        m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
        m3CreatePlaneShape(ground, &sg, &floor);
        DropBall(world, -1.0, CAT_DEBRIS, ~0ull);
        DropBall(world, 1.0, CAT_GHOST, CAT_DEBRIS);
        DropBall(world, 0.0, CAT_DEBRIS, CAT_GROUND | CAT_DEBRIS);
        int32_t snapBytes = 0;
        for (int32_t i = 0; i < 90; ++i)
        {
            m3World_Step(world, 1.0f / 60.0f, 4);
            if (i == 45 && run == 0)
            {
                snapBytes = m3World_Snapshot(world, snap, (int32_t)sizeof(snap));
                CHECK(snapBytes > 0, "the filtered snapshot fits");
            }
        }
        hashes[run] = m3World_Hash(world);
        if (recording)
        {
            int32_t bytes = m3World_JournalEnd(world);
            CHECK(bytes > 0, "the filtered session records");
            m3WorldId fresh = m3CreateWorld(&def);
            CHECK(m3World_JournalReplay(fresh, journal, bytes), "the filtered session replays");
            CHECK(m3World_Hash(fresh) == hashes[0], "the replay is bit-identical");
            m3DestroyWorld(fresh);
            CHECK(m3World_Restore(world, snap, snapBytes), "the filtered restore lands");
            for (int32_t i = 46; i < 90; ++i)
            {
                m3World_Step(world, 1.0f / 60.0f, 4);
            }
            CHECK(m3World_Hash(world) == hashes[0], "the re-fall is bit-identical");
        }
        m3DestroyWorld(world);
    }
    CHECK(hashes[0] == hashes[1], "filtered twins are bit-identical");
}

int main(void)
{
    TestMaskGate();
    TestGroupOverride();
    TestQueryFilters();
    TestFilteredReplayAndRollback();
    if (s_failures == 0)
    {
        printf("test_filter: all green\n");
        return 0;
    }
    printf("test_filter: %d failure(s)\n", s_failures);
    return 1;
}
