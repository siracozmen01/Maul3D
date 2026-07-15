// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The introspection gate (14-1): counters report the live world
// exactly, the profile fills after a step, and above all READING IS
// PURE: a twin that polls counters and profile every tick lands on
// the same bits as a twin that never looks.

#include "maul3d/body.h"
#include "maul3d/joint.h"
#include "maul3d/shape.h"
#include "maul3d/softbody.h"

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

static m3WorldId BuildYard(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 32;
    def.shapeCapacity = 32;
    def.jointCapacity = 4;
    def.softBodyCapacity = 2;
    def.voxelCapacity = 2;
    m3WorldId world = m3CreateWorld(&def);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    m3CreatePlaneShape(ground, &sd, &floor);
    for (int32_t i = 0; i < 3; ++i)
    {
        m3BodyDef bd = m3DefaultBodyDef();
        bd.type = m3_dynamicBody;
        bd.position = (m3Pos3){0.0, 0.55 + 1.1 * (double)i, 0.0};
        m3CreateBoxShape(m3CreateBody(world, &bd), &sd, (m3Vec3){0.5f, 0.5f, 0.5f});
    }
    return world;
}

static void TestCountsMatchTheScene(void)
{
    m3WorldId world = BuildYard();
    m3Counters c = m3World_GetCounters(world);
    CHECK(c.bodyCount == 4, "one ground and three crates");
    CHECK(c.shapeCount == 4, "one plane and three boxes");
    CHECK(c.jointCount == 0, "no joints yet");
    CHECK(c.hullCount >= 1, "boxes intern hull slabs");
    CHECK(c.islandCount == 0 && c.colorCount == 0, "step statistics are zero before a step");
    CHECK(c.snapshotBytes > 0, "the snapshot size reports");
    CHECK(c.snapshotBytes == m3World_SnapshotSize(world), "the two size paths agree");

    // A joint arrives, then a lattice: the counters follow.
    m3BodyDef ad = m3DefaultBodyDef();
    ad.type = m3_dynamicBody;
    ad.position = (m3Pos3){3.0, 2.0, 0.0};
    m3BodyId a = m3CreateBody(world, &ad);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3CreateBoxShape(a, &sd, (m3Vec3){0.3f, 0.3f, 0.3f});
    ad.position = (m3Pos3){3.0, 1.0, 0.0};
    m3BodyId b = m3CreateBody(world, &ad);
    m3CreateBoxShape(b, &sd, (m3Vec3){0.3f, 0.3f, 0.3f});
    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_sphericalJoint;
    jd.bodyA = a;
    jd.bodyB = b;
    m3JointId link = m3CreateJoint(&jd);
    m3SoftBodyDef sbd = m3DefaultSoftBodyDef();
    sbd.position = (m3Pos3){-4.0, 3.0, 0.0};
    sbd.countX = 2;
    sbd.countY = 2;
    sbd.countZ = 2;
    sbd.spacing = 0.5f;
    m3SoftBodyId jelly = m3CreateSoftBody(world, &sbd);
    CHECK(m3SoftBody_IsValid(jelly), "the lattice creates");
    c = m3World_GetCounters(world);
    CHECK(c.bodyCount == 6, "six bodies now");
    CHECK(c.jointCount == 1, "the joint counts");
    CHECK(c.softBodyCount == 1, "the lattice counts");

    for (int32_t i = 0; i < 60; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    c = m3World_GetCounters(world);
    CHECK(c.contactCount > 0, "the settled stack has contacts");
    CHECK(c.awakeCount >= 1, "something is awake");
    CHECK(c.islandCount >= 1, "the step counted its islands");
    CHECK(c.colorCount >= 1, "the step counted its colors");
    CHECK(c.treeHeight >= 1, "the tree has height");
    CHECK(c.scratchPeak > 0, "the step used scratch");
    CHECK(c.scratchPeak <= c.scratchCapacity, "the peak fits the reserve");

    // Destruction shrinks the census.
    m3DestroyJoint(link);
    m3DestroyBody(b);
    c = m3World_GetCounters(world);
    CHECK(c.bodyCount == 5, "the destroyed body left the count");
    CHECK(c.jointCount == 0, "the destroyed joint left the count");
    m3DestroyWorld(world);
    c = m3World_GetCounters(world);
    CHECK(c.bodyCount == 0 && c.shapeCount == 0 && c.snapshotBytes == 0,
          "a dead world reads all zeros");
}

static void TestProfileFillsAfterAStep(void)
{
    m3WorldId world = BuildYard();
    m3Profile p = m3World_GetProfile(world);
    CHECK(p.step == 0.0f, "the profile is zero before the first step");
    for (int32_t i = 0; i < 10; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    p = m3World_GetProfile(world);
    CHECK(p.step > 0.0f, "the step timed itself");
    CHECK(p.broadphase >= 0.0f && p.narrowphase >= 0.0f && p.solve >= 0.0f,
          "phase times are never negative");
    CHECK(p.step >= p.solve, "no phase exceeds the whole");
    m3DestroyWorld(world);
}

static void TestReadingIsPure(void)
{
    // The determinism guard: one twin polls the observers every
    // tick, the other never looks, and the bits must agree.
    uint64_t hashes[2];
    for (int32_t run = 0; run < 2; ++run)
    {
        m3WorldId world = BuildYard();
        for (int32_t i = 0; i < 120; ++i)
        {
            if (run == 0)
            {
                m3Counters c = m3World_GetCounters(world);
                m3Profile p = m3World_GetProfile(world);
                (void)c;
                (void)p;
            }
            m3World_Step(world, 1.0f / 60.0f, 4);
        }
        hashes[run] = m3World_Hash(world);
        m3DestroyWorld(world);
    }
    CHECK(hashes[0] == hashes[1], "watching the world never changes it");
}

static int s_assertSeen = 0;

static int AssertCatcher(const char* condition, const char* file, int line)
{
    (void)condition;
    (void)file;
    (void)line;
    s_assertSeen += 1;
    return 1; // handled: no abort
}

static void TestNamesAndHooks(void)
{
    static uint8_t journal[65536];
    static uint8_t snap[2097152];
    m3WorldId world = BuildYard();
    CHECK(m3World_JournalBegin(world, journal, (int32_t)sizeof(journal)), "the tape opens");
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){5.0, 1.0, 0.0};
    m3BodyId hero = m3CreateBody(world, &bd);
    CHECK(m3Body_GetName(hero)[0] == 0, "a new body is unnamed");
    m3Body_SetName(hero, "the crate of destiny");
    CHECK(strcmp(m3Body_GetName(hero), "the crate of destiny") == 0, "the name reads back");
    m3Body_SetName(hero, "a name far too long to fit inside thirty one bytes of storage");
    CHECK(strlen(m3Body_GetName(hero)) == 31, "long names truncate to the cap");
    m3Body_SetName(hero, "hero");
    uint64_t before = m3World_Hash(world);
    m3Body_SetName(hero, "renamed");
    CHECK(m3World_Hash(world) == before, "a name is never a hash input");
    int32_t bytes = m3World_JournalEnd(world);
    CHECK(bytes > 0, "the naming session records");
    m3WorldId twin = BuildYard();
    CHECK(m3World_JournalReplay(twin, journal, bytes), "the naming session replays");
    m3BodyId twinHero = {hero.index1, (uint16_t)(twin.index1 - 1), hero.generation};
    CHECK(strcmp(m3Body_GetName(twinHero), "renamed") == 0, "the replayed name lands");
    m3DestroyWorld(twin);
    int32_t snapBytes = m3World_Snapshot(world, snap, (int32_t)sizeof(snap));
    CHECK(snapBytes > 0, "the named snapshot fits");
    m3Body_SetName(hero, "clobbered");
    CHECK(m3World_Restore(world, snap, snapBytes), "the restore lands");
    CHECK(strcmp(m3Body_GetName(hero), "renamed") == 0, "the snapshot carries the name");
    m3BodyId stale = {99, hero.world0, 7};
    m3Body_SetName(stale, "ghost");
    CHECK(m3Body_GetName(stale)[0] == 0, "a stale id stays nameless");
    m3DestroyWorld(world);

    // The assert hook: installed, fired, uninstalled, no abort.
    m3SetAssertHandler(AssertCatcher);
    m3AssertFail("introspection-probe", __FILE__, __LINE__);
    CHECK(s_assertSeen == 1, "the handler caught the failure");
    m3SetAssertHandler(NULL);
}

static void TestContactReadback(void)
{
    m3WorldId world = BuildYard();
    for (int32_t i = 0; i < 90; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    // The stack settled: the middle crate touches above and below.
    m3Counters c = m3World_GetCounters(world);
    CHECK(c.contactCount > 0, "the yard has contacts");
    m3ContactData data[8];
    int32_t total = 0;
    int32_t maxPer = 0;
    // The readback needs real ids: build a fresh two-crate stack.
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){10.0, 0.55, 0.0};
    m3BodyId a = m3CreateBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3ShapeId shapeA = m3CreateBoxShape(a, &sd, (m3Vec3){0.5f, 0.5f, 0.5f});
    bd.position = (m3Pos3){10.0, 1.6, 0.0};
    m3BodyId b = m3CreateBody(world, &bd);
    m3CreateBoxShape(b, &sd, (m3Vec3){0.5f, 0.5f, 0.5f});
    for (int32_t i = 0; i < 90; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    total = m3Body_GetContactData(a, data, 8);
    CHECK(total >= 1, "the crate reports its touches");
    maxPer = data[0].pointCount;
    CHECK(maxPer >= 1 && maxPer <= 4, "the manifold point count is sane");
    CHECK(data[0].normalImpulses[0] >= 0.0f, "normal impulses are never negative");
    double y = data[0].points[0].y;
    CHECK(y > -0.2 && y < 1.7, "the contact points sit near the stack");
    int32_t viaShape = m3Shape_GetContactData(shapeA, data, 8);
    CHECK(viaShape == total, "the shape view agrees with the body view");
    m3ContactData none[2];
    m3BodyId stale = {99, a.world0, 7};
    CHECK(m3Body_GetContactData(stale, none, 2) == 0, "a stale id reads zero contacts");
    m3DestroyWorld(world);
}

int main(void)
{
    TestCountsMatchTheScene();
    TestProfileFillsAfterAStep();
    TestReadingIsPure();
    TestNamesAndHooks();
    TestContactReadback();
    if (s_failures == 0)
    {
        printf("test_introspection: all passed\n");
        return 0;
    }
    return 1;
}
