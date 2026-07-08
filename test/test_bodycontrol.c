// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The runtime-control gate (8-3): teleports wake both
// neighborhoods, the kinematic servo lands ON its target, type
// flips rebuild the books, disabled bodies vanish everywhere,
// motion locks hold their axes exact, sleep knobs obey, and every
// op journals and rolls back onto identical bits.

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

static m3WorldId PlaneWorld(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 32;
    def.shapeCapacity = 32;
    m3WorldId world = m3CreateWorld(&def);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    m3CreatePlaneShape(ground, &sd, &floor);
    return world;
}

static m3BodyId Crate(m3WorldId world, m3Pos3 at)
{
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = at;
    m3BodyId body = m3CreateBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3CreateBoxShape(body, &sd, (m3Vec3){0.5f, 0.5f, 0.5f});
    return body;
}

static void TestTeleportWakes(void)
{
    // A crate sleeps on the plane; another crate teleports right
    // above it: BOTH the sleeper and the traveler end awake, and
    // the sleeper takes the landing.
    m3WorldId world = PlaneWorld();
    m3BodyId sleeper = Crate(world, (m3Pos3){0.0, 0.5, 0.0});
    m3BodyId traveler = Crate(world, (m3Pos3){10.0, 0.5, 0.0});
    for (int32_t i = 0; i < 240; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(!m3Body_IsAwake(sleeper), "the crate sleeps before the teleport");
    m3Body_SetTransform(traveler, (m3Pos3){0.0, 2.0, 0.0}, (m3Quat){0.0f, 0.0f, 0.0f, 1.0f});
    CHECK(m3Body_IsAwake(sleeper), "the arrival neighborhood wakes");
    for (int32_t i = 0; i < 120; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3Pos3 t = m3Body_GetPosition(traveler);
    CHECK(t.y > 1.3 && t.y < 1.7, "the traveler lands on the sleeper");
    m3DestroyWorld(world);
}

static void TestKinematicServo(void)
{
    // The servo lands the kinematic slab exactly on its target
    // after one step, then holds (the order clears).
    m3WorldId world = PlaneWorld();
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_kinematicBody;
    bd.position = (m3Pos3){0.0, 2.0, 0.0};
    m3BodyId slab = m3CreateBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3CreateBoxShape(slab, &sd, (m3Vec3){1.0f, 0.2f, 1.0f});

    m3Body_SetTargetTransform(slab, (m3Pos3){1.5, 2.5, -0.5}, (m3Quat){0.0f, 0.0f, 0.0f, 1.0f});
    m3World_Step(world, 1.0f / 60.0f, 4);
    m3Pos3 p = m3Body_GetPosition(slab);
    CHECK(fabs(p.x - 1.5) < 1.0e-4 && fabs(p.y - 2.5) < 1.0e-4 && fabs(p.z + 0.5) < 1.0e-4,
          "the servo lands on the target in one step");
    // Documented contract: the servo clears its order but leaves
    // the exit velocity on the body; the host zeroes it or issues
    // the next target. Zero it here and prove no further pull.
    m3Body_SetLinearVelocity(slab, (m3Vec3){0.0f, 0.0f, 0.0f});
    m3Body_SetAngularVelocity(slab, (m3Vec3){0.0f, 0.0f, 0.0f});
    m3World_Step(world, 1.0f / 60.0f, 4);
    m3Pos3 q = m3Body_GetPosition(slab);
    CHECK(fabs(q.x - p.x) < 1.0e-6 && fabs(q.y - p.y) < 1.0e-6,
          "the order cleared: no further servo pull");
    m3DestroyWorld(world);
}

static void TestTypeFlipAndDisable(void)
{
    // A static shelf HOVERS with a crate on top, then turns
    // DYNAMIC: both fall (a grounded pillar would just stand, the
    // first draft of this test learned that premise the hard way).
    // Then a crate disables mid-scene: contacts and queries lose
    // it, and enabling brings it back awake.
    m3WorldId world = PlaneWorld();
    m3BodyDef pd = m3DefaultBodyDef();
    pd.position = (m3Pos3){0.0, 3.0, 0.0};
    m3BodyId shelf = m3CreateBody(world, &pd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3CreateBoxShape(shelf, &sd, (m3Vec3){1.0f, 0.2f, 1.0f});
    m3BodyId crate = Crate(world, (m3Pos3){0.0, 3.8, 0.0});
    for (int32_t i = 0; i < 120; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    double restY = m3Body_GetPosition(crate).y;
    CHECK(restY > 3.5, "the crate rests on the hovering static shelf");
    m3Body_SetType(shelf, m3_dynamicBody);
    for (int32_t i = 0; i < 180; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m3Body_GetPosition(crate).y < 1.8, "the freed shelf drops its rider");

    // Disable: the crate vanishes from a ray and from contacts.
    m3BodyId lid = Crate(world, (m3Pos3){10.0, 0.5, 0.0});
    for (int32_t i = 0; i < 60; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3RayHit before =
        m3World_CastRayClosest(world, (m3Pos3){10.0, 5.0, 0.0}, (m3Vec3){0.0f, -6.0f, 0.0f});
    CHECK(before.hit && before.point.y > 0.9, "the ray sees the enabled crate");
    m3Body_SetEnabled(lid, false);
    CHECK(!m3Body_IsEnabled(lid), "disabled reads back");
    m3RayHit after =
        m3World_CastRayClosest(world, (m3Pos3){10.0, 5.0, 0.0}, (m3Vec3){0.0f, -6.0f, 0.0f});
    CHECK(after.hit && after.point.y < 0.1, "the ray passes through the disabled crate");
    m3Body_SetEnabled(lid, true);
    CHECK(m3Body_IsEnabled(lid) && m3Body_IsAwake(lid), "enabling wakes the body");
    m3DestroyWorld(world);
}

static void TestMotionLocks(void)
{
    // An upright 2.5D crate: linear z and ALL spin locked (a free
    // angular z let the first draft's crate trip over its leading
    // edge and tumble home: physical, but noise here). Shoved
    // diagonally, it slides in x only and never rotates.
    m3WorldId world = PlaneWorld();
    m3BodyId crate = Crate(world, (m3Pos3){0.0, 0.5, 0.0});
    // lock linear z (bit 2), angular x y z (bits 3, 4, 5).
    m3Body_SetMotionLocks(crate, (1u << 2) | (1u << 3) | (1u << 4) | (1u << 5));
    CHECK(m3Body_GetMotionLocks(crate) == 0x3Cu, "locks read back");
    m3Body_ApplyLinearImpulse(crate, (m3Vec3){3.0f, 0.0f, 3.0f});
    m3Body_ApplyAngularImpulse(crate, (m3Vec3){0.0f, 0.4f, 0.0f});
    for (int32_t i = 0; i < 90; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3Pos3 p = m3Body_GetPosition(crate);
    m3Quat q = m3Body_GetRotation(crate);
    // The criterion is the lock ASYMMETRY (x free, z frozen), not
    // the slide distance: rev 20 overeats friction on shoved boxes
    // (the cross-pass budget defect, convicted 8-3, fixed by the
    // central-friction slice), so the exact distance is that
    // slice's law, not this one's.
    CHECK(p.x > 0.05, "the unlocked axis carries the shove");
    CHECK(fabs(p.z) < 1.0e-4, "the locked linear axis never moves");
    CHECK(fabsf(q.x) < 1.0e-4f && fabsf(q.y) < 1.0e-4f && fabsf(q.z) < 1.0e-4f,
          "the locked spin axes never turn");
    m3DestroyWorld(world);
}

static void TestSleepKnobsAndControlReplay(void)
{
    // A can-never-sleep crate stays awake forever; a forced sleep
    // freezes one instantly. The whole control script journals and
    // rolls back bit-exact.
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
        m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
        m3CreatePlaneShape(ground, &sg, &floor);
        m3BodyId restless = Crate(world, (m3Pos3){0.0, 0.5, 0.0});
        m3BodyId normal = Crate(world, (m3Pos3){3.0, 0.5, 0.0});
        m3Body_SetSleepControls(restless, 0.0f, false);

        int32_t snapBytes = 0;
        for (int32_t i = 0; i < 260; ++i)
        {
            if (i == 40)
            {
                m3Body_SetMotionLocks(normal, 1u << 4);
            }
            if (i == 80)
            {
                m3Body_SetTransform(restless, (m3Pos3){0.0, 3.0, 1.0},
                                    (m3Quat){0.0f, 0.0f, 0.0f, 1.0f});
            }
            if (i == 160)
            {
                m3Body_SetAwake(normal, false); // force-sleep
            }
            m3World_Step(world, 1.0f / 60.0f, 4);
            if (i == 120 && run == 0)
            {
                snapBytes = m3World_Snapshot(world, snap, (int32_t)sizeof(snap));
                CHECK(snapBytes > 0, "the control snapshot fits");
            }
        }
        CHECK(m3Body_IsAwake(restless), "the can-never-sleep crate is still awake");
        CHECK(!m3Body_IsAwake(normal), "the forced sleeper stays down");
        uint64_t final = m3World_Hash(world);
        if (recording)
        {
            int32_t bytes = m3World_JournalEnd(world);
            CHECK(bytes > 0, "the control session records");
            m3WorldId fresh = m3CreateWorld(&def);
            CHECK(m3World_JournalReplay(fresh, journal, bytes), "the control session replays");
            CHECK(m3World_Hash(fresh) == final, "the replay is bit-identical");
            m3DestroyWorld(fresh);
            CHECK(m3World_Restore(world, snap, snapBytes), "the control restore lands");
            for (int32_t i = 121; i < 260; ++i)
            {
                if (i == 160)
                {
                    m3Body_SetAwake(normal, false);
                }
                m3World_Step(world, 1.0f / 60.0f, 4);
            }
            CHECK(m3World_Hash(world) == final, "the re-run is bit-identical");
        }
        hashes[run] = final;
        m3DestroyWorld(world);
    }
    CHECK(hashes[0] == hashes[1], "control twins are bit-identical");
}

int main(void)
{
    TestTeleportWakes();
    TestKinematicServo();
    TestTypeFlipAndDisable();
    TestMotionLocks();
    TestSleepKnobsAndControlReplay();
    if (s_failures == 0)
    {
        printf("test_bodycontrol: all green\n");
        return 0;
    }
    printf("test_bodycontrol: %d failure(s)\n", s_failures);
    return 1;
}
