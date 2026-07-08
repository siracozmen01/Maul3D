// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The force gate (8-2): host forces and impulses land by the
// analytic books (F = m a, T = I alpha, off-center impulses split
// into the linear and angular ledgers), live for exactly one step
// when they are forces, journal and roll back to the bit, and
// refuse hostile or non-dynamic targets quietly.

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

static m3WorldId FreeSpace(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    def.gravity = (m3Vec3){0.0f, 0.0f, 0.0f}; // clean ledgers
    return m3CreateWorld(&def);
}

static m3BodyId UnitCube(m3WorldId world, m3Pos3 at)
{
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = at;
    bd.linearDamping = 0.0f; // clean analytic books
    bd.angularDamping = 0.0f;
    m3BodyId body = m3CreateBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    // half extents 0.5: volume 1, density 1: mass 1, I = m/6 = 1/6.
    m3CreateBoxShape(body, &sd, (m3Vec3){0.5f, 0.5f, 0.5f});
    return body;
}

static void TestForceAndTorqueBooks(void)
{
    m3WorldId world = FreeSpace();
    m3BodyId cube = UnitCube(world, (m3Pos3){0.0, 0.0, 0.0});

    // A 2 N force held for sixty steps of one second total on a
    // one kilogram cube: v = F t / m = 2 m/s. The force must be
    // re-applied per step (it clears after each one).
    for (int32_t i = 0; i < 60; ++i)
    {
        m3Body_ApplyForce(cube, (m3Vec3){2.0f, 0.0f, 0.0f});
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3Vec3 v = m3Body_GetLinearVelocity(cube);
    CHECK(fabsf(v.x - 2.0f) < 0.01f, "F equals m a over one second");

    // The accumulator clears: sixty more steps with NO application
    // add nothing.
    for (int32_t i = 0; i < 60; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3Vec3 v2 = m3Body_GetLinearVelocity(cube);
    CHECK(fabsf(v2.x - v.x) < 1.0e-6f, "a force lives for exactly one step");

    // Torque: T = I alpha; unit cube I = 1/6, T = 0.5 N m for one
    // second: w = T t / I = 3 rad/s.
    // Far from the drifting force cube: the first draft parked
    // this at x = 5 and the 2 m/s drifter sailed straight through
    // the spin measurement.
    m3BodyId top = UnitCube(world, (m3Pos3){0.0, 50.0, 0.0});
    for (int32_t i = 0; i < 60; ++i)
    {
        m3Body_ApplyTorque(top, (m3Vec3){0.0f, 0.5f, 0.0f});
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3Vec3 w = m3Body_GetAngularVelocity(top);
    CHECK(fabsf(w.y - 3.0f) < 0.02f, "T equals I alpha over one second");
    m3DestroyWorld(world);
}

static void TestImpulseBooks(void)
{
    m3WorldId world = FreeSpace();
    m3BodyId cube = UnitCube(world, (m3Pos3){0.0, 0.0, 0.0});

    // A 3 kg m/s impulse at the center: v = J / m = 3, no spin.
    m3Body_ApplyLinearImpulse(cube, (m3Vec3){3.0f, 0.0f, 0.0f});
    m3Vec3 v = m3Body_GetLinearVelocity(cube);
    m3Vec3 w = m3Body_GetAngularVelocity(cube);
    CHECK(fabsf(v.x - 3.0f) < 1.0e-6f, "a center impulse is J over m");
    CHECK(fabsf(w.x) + fabsf(w.y) + fabsf(w.z) < 1.0e-6f, "a center impulse adds no spin");

    // The same impulse at a corner-height point: SAME linear
    // change, plus spin = invI (r x J): r = (0, 0.5, 0),
    // J = (3,0,0): r x J = (0,0,-1.5), invI = 6: w.z = -9.
    m3BodyId two = UnitCube(world, (m3Pos3){5.0, 0.0, 0.0});
    m3Body_ApplyLinearImpulseAtPoint(two, (m3Vec3){3.0f, 0.0f, 0.0f}, (m3Pos3){5.0, 0.5, 0.0});
    m3Vec3 v2 = m3Body_GetLinearVelocity(two);
    m3Vec3 w2 = m3Body_GetAngularVelocity(two);
    CHECK(fabsf(v2.x - 3.0f) < 1.0e-5f, "an off-center impulse keeps the linear ledger");
    CHECK(fabsf(w2.z + 9.0f) < 0.01f, "the angular ledger takes invI times r cross J");

    // Angular impulse direct: L = 0.5: w = invI L = 3.
    m3BodyId three = UnitCube(world, (m3Pos3){10.0, 0.0, 0.0});
    m3Body_ApplyAngularImpulse(three, (m3Vec3){0.0f, 0.0f, 0.5f});
    CHECK(fabsf(m3Body_GetAngularVelocity(three).z - 3.0f) < 0.01f,
          "an angular impulse is invI times L");
    m3DestroyWorld(world);
}

static void TestForceReplayAndRollback(void)
{
    // A force-and-impulse script journals from birth, replays
    // bit-exact, and a mid-script rollback re-lands the same bits.
    static uint8_t journal[131072];
    static uint8_t snap[786432];
    uint64_t hashes[2];
    for (int32_t run = 0; run < 2; ++run)
    {
        m3WorldDef def = m3DefaultWorldDef();
        def.bodyCapacity = 16;
        def.shapeCapacity = 16;
        m3WorldId world = m3CreateWorld(&def);
        bool recording = run == 0 && m3World_JournalBegin(world, journal, (int32_t)sizeof(journal));
        m3BodyDef gd = m3DefaultBodyDef();
        m3BodyId ground = m3CreateBody(world, &gd);
        m3ShapeDef sg = m3DefaultShapeDef();
        m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
        m3CreatePlaneShape(ground, &sg, &floor);
        m3BodyId cube = UnitCube(world, (m3Pos3){0.0, 2.0, 0.0});

        int32_t snapBytes = 0;
        for (int32_t i = 0; i < 120; ++i)
        {
            if (i % 7 == 3)
            {
                m3Body_ApplyForce(cube, (m3Vec3){1.5f, 0.0f, 0.4f});
            }
            if (i == 40)
            {
                m3Body_ApplyLinearImpulseAtPoint(cube, (m3Vec3){0.0f, 2.0f, 0.0f},
                                                 m3Body_GetPosition(cube));
            }
            if (i == 80)
            {
                m3Body_ApplyTorque(cube, (m3Vec3){0.0f, 0.8f, 0.0f});
            }
            m3World_Step(world, 1.0f / 60.0f, 4);
            if (i == 60)
            {
                // Part of the SCRIPT in both runs: a force applied
                // between steps, with run zero snapshotting right
                // after it: pending accumulators are state and
                // must survive the trip.
                m3Body_ApplyForce(cube, (m3Vec3){0.0f, 0.0f, -2.0f});
                if (run == 0)
                {
                    snapBytes = m3World_Snapshot(world, snap, (int32_t)sizeof(snap));
                    CHECK(snapBytes > 0, "the mid-script snapshot fits");
                }
            }
        }
        uint64_t final = m3World_Hash(world);
        if (recording)
        {
            int32_t bytes = m3World_JournalEnd(world);
            CHECK(bytes > 0, "the force session records");
            m3WorldId fresh = m3CreateWorld(&def);
            CHECK(m3World_JournalReplay(fresh, journal, bytes), "the force session replays");
            CHECK(m3World_Hash(fresh) == final, "the replay is bit-identical");
            m3DestroyWorld(fresh);
            CHECK(m3World_Restore(world, snap, snapBytes), "the mid-script restore lands");
            for (int32_t i = 61; i < 120; ++i)
            {
                if (i % 7 == 3)
                {
                    m3Body_ApplyForce(cube, (m3Vec3){1.5f, 0.0f, 0.4f});
                }
                if (i == 80)
                {
                    m3Body_ApplyTorque(cube, (m3Vec3){0.0f, 0.8f, 0.0f});
                }
                m3World_Step(world, 1.0f / 60.0f, 4);
            }
            CHECK(m3World_Hash(world) == final, "the re-run is bit-identical");
        }
        hashes[run] = final;
        m3DestroyWorld(world);
    }
    CHECK(hashes[0] == hashes[1], "force twins are bit-identical");
}

static void TestForceContracts(void)
{
    m3WorldId world = FreeSpace();
    m3BodyId cube = UnitCube(world, (m3Pos3){0.0, 0.0, 0.0});
    m3BodyDef sd2 = m3DefaultBodyDef();
    sd2.position = (m3Pos3){3.0, 0.0, 0.0};
    m3BodyId rock = m3CreateBody(world, &sd2); // static

    float bad;
    uint32_t nanBits = 0x7FC00000u;
    memcpy(&bad, &nanBits, sizeof(bad));
    m3Body_ApplyForce(cube, (m3Vec3){bad, 0.0f, 0.0f});          // hostile: no-op
    m3Body_ApplyLinearImpulse(rock, (m3Vec3){5.0f, 0.0f, 0.0f}); // static: no-op
    m3World_Step(world, 1.0f / 60.0f, 4);
    CHECK(fabsf(m3Body_GetLinearVelocity(cube).x) < 1.0e-6f, "a NaN force never lands");
    CHECK(m3Body_GetPosition(rock).x == 3.0, "a static target never moves");

    m3DestroyBody(cube);
    m3Body_ApplyForce(cube, (m3Vec3){1.0f, 0.0f, 0.0f}); // stale: no-op
    m3World_Step(world, 1.0f / 60.0f, 4);
    m3DestroyWorld(world);
}

int main(void)
{
    TestForceAndTorqueBooks();
    TestImpulseBooks();
    TestForceReplayAndRollback();
    TestForceContracts();
    if (s_failures == 0)
    {
        printf("test_force: all green\n");
        return 0;
    }
    printf("test_force: %d failure(s)\n", s_failures);
    return 1;
}
