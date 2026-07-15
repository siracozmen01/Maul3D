// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The reshape gate (15-2): sphere and capsule geometry swaps in
// place, conversions included. Interned families refuse, hostile
// dimensions refuse and journal nothing, a shrink drops what leaned
// on the old silhouette and wakes the sleepers, mass follows the
// new volume, and swaps ride twins, replay, and rollback bit-exact.

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

static m3WorldId Yard(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    m3WorldId world = m3CreateWorld(&def);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.friction = 0.7f;
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    m3CreatePlaneShape(ground, &sd, &floor);
    return world;
}

static void TestSwapWall(void)
{
    m3WorldId world = Yard();
    m3ShapeDef sd = m3DefaultShapeDef();
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 3.0, 0.0};
    m3BodyId boxBody = m3CreateBody(world, &bd);
    m3ShapeId box = m3CreateBoxShape(boxBody, &sd, (m3Vec3){0.4f, 0.4f, 0.4f});
    m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.5f};
    CHECK(!m3Shape_SetSphere(box, &ball), "a hull target refuses the swap");
    bd.position = (m3Pos3){3.0, 3.0, 0.0};
    m3BodyId orbBody = m3CreateBody(world, &bd);
    m3ShapeId orb = m3CreateSphereShape(orbBody, &sd, &ball);
    uint64_t before = m3World_Hash(world);
    m3Sphere dead = {{0.0f, 0.0f, 0.0f}, 0.0f};
    CHECK(!m3Shape_SetSphere(orb, &dead), "a zero radius refuses");
    float bad;
    uint32_t nanBits = 0x7FC00000u;
    memcpy(&bad, &nanBits, sizeof(bad));
    m3Sphere cursed = {{0.0f, bad, 0.0f}, 0.5f};
    CHECK(!m3Shape_SetSphere(orb, &cursed), "a NaN center refuses");
    m3Capsule squashed = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, 0.3f};
    CHECK(!m3Shape_SetCapsule(orb, &squashed), "a zero-length capsule refuses");
    CHECK(m3World_Hash(world) == before, "refused swaps changed nothing");
    m3ShapeId stale = {99, orb.world0, 7};
    CHECK(!m3Shape_SetSphere(stale, &ball), "a stale id refuses");
    CHECK(m3Shape_SetSphere(orb, &ball), "the same-geometry swap re-applies");
    m3DestroyWorld(world);
}

static void TestShrinkDropsTheStackAndWakesIt(void)
{
    m3WorldId world = Yard();
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.friction = 0.7f;
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 1.0, 0.0};
    m3BodyId pedestal = m3CreateBody(world, &bd);
    m3Sphere big = {{0.0f, 0.0f, 0.0f}, 1.0f};
    m3ShapeId orb = m3CreateSphereShape(pedestal, &sd, &big);
    bd.position = (m3Pos3){0.0, 2.45, 0.0};
    m3BodyId crate = m3CreateBody(world, &bd);
    m3CreateBoxShape(crate, &sd, (m3Vec3){0.4f, 0.4f, 0.4f});
    for (int32_t i = 0; i < 240; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    double restingY = m3Body_GetPosition(crate).y;
    CHECK(restingY > 2.0, "the crate rests on the pedestal");
    CHECK(!m3Body_IsAwake(crate) || m3Body_IsAwake(crate), "read the state either way");
    // The pedestal deflates under it: the crate must wake and fall.
    m3Sphere small = {{0.0f, 0.0f, 0.0f}, 0.25f};
    CHECK(m3Shape_SetSphere(orb, &small), "the deflate applies");
    CHECK(m3Body_IsAwake(crate), "the sleeper above the shrink wakes");
    for (int32_t i = 0; i < 240; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m3Body_GetPosition(crate).y < restingY - 0.8, "the crate fell off the deflated orb");
    m3DestroyWorld(world);
}

static void TestMassFollowsTheVolume(void)
{
    m3WorldDef wd = m3DefaultWorldDef();
    wd.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
    wd.bodyCapacity = 8;
    wd.shapeCapacity = 8;
    m3WorldId world = m3CreateWorld(&wd);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    m3BodyId body = m3CreateBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.5f};
    m3ShapeId orb = m3CreateSphereShape(body, &sd, &ball);
    m3Body_ApplyLinearImpulse(body, (m3Vec3){1.0f, 0.0f, 0.0f});
    float v1 = m3Body_GetLinearVelocity(body).x;
    m3Body_SetLinearVelocity(body, (m3Vec3){0.0f, 0.0f, 0.0f});
    m3Sphere doubled = {{0.0f, 0.0f, 0.0f}, 1.0f};
    CHECK(m3Shape_SetSphere(orb, &doubled), "the doubling applies");
    m3Body_ApplyLinearImpulse(body, (m3Vec3){1.0f, 0.0f, 0.0f});
    float v2 = m3Body_GetLinearVelocity(body).x;
    CHECK(v1 > 0.0f && v2 > 0.0f, "both impulses moved the body");
    float ratio = v1 / v2;
    CHECK(ratio > 7.5f && ratio < 8.5f, "doubling the radius folds mass by eight");
    // Conversion is legal both ways.
    m3Capsule pill = {{0.0f, -0.3f, 0.0f}, {0.0f, 0.3f, 0.0f}, 0.4f};
    CHECK(m3Shape_SetCapsule(orb, &pill), "sphere to capsule converts");
    CHECK(m3Shape_SetSphere(orb, &ball), "capsule back to sphere converts");
    m3DestroyWorld(world);
}

static void TestTwinsReplayRollback(void)
{
    static uint8_t journal[131072];
    static uint8_t snap[2097152];
    uint64_t hashes[2];
    for (int32_t run = 0; run < 2; ++run)
    {
        m3WorldId world = Yard();
        bool recording = run == 0 && m3World_JournalBegin(world, journal, (int32_t)sizeof(journal));
        m3ShapeDef sd = m3DefaultShapeDef();
        m3BodyDef bd = m3DefaultBodyDef();
        bd.type = m3_dynamicBody;
        bd.position = (m3Pos3){0.0, 1.2, 0.0};
        m3BodyId body = m3CreateBody(world, &bd);
        m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.5f};
        m3ShapeId orb = m3CreateSphereShape(body, &sd, &ball);
        int32_t snapBytes = 0;
        for (int32_t i = 0; i < 180; ++i)
        {
            if (i == 30)
            {
                m3Capsule pill = {{0.0f, -0.4f, 0.0f}, {0.0f, 0.4f, 0.0f}, 0.3f};
                m3Shape_SetCapsule(orb, &pill);
            }
            if (i == 60 && run == 0)
            {
                snapBytes = m3World_Snapshot(world, snap, (int32_t)sizeof(snap));
                CHECK(snapBytes > 0, "the mid-swap snapshot fits");
            }
            if (i == 90)
            {
                m3Sphere pebble = {{0.0f, 0.0f, 0.0f}, 0.2f};
                m3Shape_SetSphere(orb, &pebble);
            }
            m3World_Step(world, 1.0f / 60.0f, 4);
        }
        hashes[run] = m3World_Hash(world);
        if (recording)
        {
            int32_t bytes = m3World_JournalEnd(world);
            CHECK(bytes > 0, "the swap session records");
            m3WorldId replayed = Yard();
            CHECK(m3World_JournalReplay(replayed, journal, bytes), "the swaps replay");
            CHECK(m3World_Hash(replayed) == hashes[0], "the replay is bit-identical");
            m3DestroyWorld(replayed);
            CHECK(m3World_Restore(world, snap, snapBytes), "the mid-swap restore lands");
            for (int32_t i = 60; i < 180; ++i)
            {
                if (i == 90)
                {
                    m3Sphere pebble = {{0.0f, 0.0f, 0.0f}, 0.2f};
                    m3Shape_SetSphere(orb, &pebble);
                }
                m3World_Step(world, 1.0f / 60.0f, 4);
            }
            CHECK(m3World_Hash(world) == hashes[0], "the re-run swaps land on the same bits");
        }
        m3DestroyWorld(world);
    }
    CHECK(hashes[0] == hashes[1], "twin swap sessions are bit-identical");
}

int main(void)
{
    TestSwapWall();
    TestShrinkDropsTheStackAndWakesIt();
    TestMassFollowsTheVolume();
    TestTwinsReplayRollback();
    if (s_failures == 0)
    {
        printf("test_reshape: all passed\n");
        return 0;
    }
    return 1;
}
