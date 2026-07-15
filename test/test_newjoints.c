// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The new joint gate (16-1): FILTER is rowless by law (its whole
// effect is the connected-pair collision filter; a falling body
// wearing one falls FREE, never welded), and PARALLEL locks two
// axes parallel while every translation and the shared twist stay
// free. Both ride twins, replay, rollback, and hostile walls.

#include "maul3d/body.h"
#include "maul3d/joint.h"
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
    def.jointCapacity = 8;
    m3WorldId world = m3CreateWorld(&def);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    m3CreatePlaneShape(ground, &sd, &floor);
    return world;
}

static void TestFilterIsRowlessAndFilters(void)
{
    // Two overlapping crates wearing a filter: they fall through
    // each other and land side by side inside the same space; the
    // moment the filter dies, the overlap resolves apart.
    m3WorldId world = Yard();
    m3ShapeDef sd = m3DefaultShapeDef();
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 3.0, 0.0};
    m3BodyId a = m3CreateBody(world, &bd);
    m3CreateBoxShape(a, &sd, (m3Vec3){0.4f, 0.4f, 0.4f});
    bd.position = (m3Pos3){0.2, 3.2, 0.0};
    m3BodyId b = m3CreateBody(world, &bd);
    m3CreateBoxShape(b, &sd, (m3Vec3){0.4f, 0.4f, 0.4f});
    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_filterJoint;
    jd.bodyA = a;
    jd.bodyB = b;
    m3JointId filter = m3CreateJoint(&jd);
    CHECK(m3Joint_IsValid(filter), "the filter creates");
    for (int32_t i = 0; i < 180; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    // Rowless proof: both crates rest ON THE FLOOR (y about 0.4),
    // still overlapping horizontally; a phantom weld would hold b
    // 0.2 above a instead.
    double ya = m3Body_GetPosition(a).y;
    double yb = m3Body_GetPosition(b).y;
    CHECK(ya > 0.3 && ya < 0.5, "crate a rests on the floor");
    CHECK(yb > 0.3 && yb < 0.5, "crate b overlaps a freely on the floor");
    m3DestroyJoint(filter);
    for (int32_t i = 0; i < 180; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    double dx = fabs(m3Body_GetPosition(a).x - m3Body_GetPosition(b).x);
    CHECK(dx > 0.6, "without the filter the overlap resolves apart");
    m3DestroyWorld(world);
}

static void TestParallelKeepsAxesParallel(void)
{
    // Two floating plates joined parallel about y: a torque about x
    // on one drags the other's tilt along (the lock), while a shove
    // translates it freely and a twist about y stays private.
    m3WorldDef wd = m3DefaultWorldDef();
    wd.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
    wd.bodyCapacity = 8;
    wd.shapeCapacity = 8;
    wd.jointCapacity = 4;
    m3WorldId world = m3CreateWorld(&wd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 2.0, 0.0};
    m3BodyId a = m3CreateBody(world, &bd);
    m3CreateBoxShape(a, &sd, (m3Vec3){0.6f, 0.05f, 0.6f});
    bd.position = (m3Pos3){2.0, 2.0, 0.0};
    m3BodyId b = m3CreateBody(world, &bd);
    m3CreateBoxShape(b, &sd, (m3Vec3){0.6f, 0.05f, 0.6f});
    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_parallelJoint;
    jd.bodyA = a;
    jd.bodyB = b;
    jd.localAxisA = (m3Vec3){0.0f, 1.0f, 0.0f};
    jd.localAxisB = (m3Vec3){0.0f, 1.0f, 0.0f};
    CHECK(m3Joint_IsValid(m3CreateJoint(&jd)), "the parallel joint creates");
    // Translation stays free: a shove separates them further.
    m3Body_ApplyLinearImpulse(b, (m3Vec3){2.0f, 0.0f, 0.0f});
    for (int32_t i = 0; i < 30; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m3Body_GetPosition(b).x > 2.5, "translation is free");
    // The lock: torque body a about x, body b tilts WITH it.
    m3Body_ApplyAngularImpulse(a, (m3Vec3){0.6f, 0.0f, 0.0f});
    for (int32_t i = 0; i < 60; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3Vec3 wa = m3Body_GetAngularVelocity(a);
    m3Vec3 wb = m3Body_GetAngularVelocity(b);
    CHECK(fabsf(wa.x - wb.x) < 0.05f, "the tilt rate is shared through the lock");
    CHECK(fabsf(wa.x) > 0.1f, "the pair genuinely tilts");
    // The twist about the shared axis stays private to b.
    m3Body_ApplyAngularImpulse(b, (m3Vec3){0.0f, 1.0f, 0.0f});
    for (int32_t i = 0; i < 30; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    wa = m3Body_GetAngularVelocity(a);
    wb = m3Body_GetAngularVelocity(b);
    CHECK(fabsf(wb.y) > fabsf(wa.y) + 0.2f, "the twist stays private");
    m3DestroyWorld(world);
}

static void TestWallsTwinsAndReplay(void)
{
    // Hostile walls: a parallel joint with a zero axis refuses, a
    // filter with zero axes creates (it has no geometry to check).
    m3WorldId world = Yard();
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 2.0, 0.0};
    m3BodyId a = m3CreateBody(world, &bd);
    bd.position = (m3Pos3){1.0, 2.0, 0.0};
    m3BodyId b = m3CreateBody(world, &bd);
    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_parallelJoint;
    jd.bodyA = a;
    jd.bodyB = b;
    jd.localAxisA = (m3Vec3){0.0f, 0.0f, 0.0f};
    CHECK(!m3Joint_IsValid(m3CreateJoint(&jd)), "a zero-axis parallel refuses");
    jd.type = m3_filterJoint;
    CHECK(m3Joint_IsValid(m3CreateJoint(&jd)), "a filter needs no axes");
    m3DestroyWorld(world);

    // Twins and replay through both types plus a mid-run rollback.
    static uint8_t journal[131072];
    static uint8_t snap[2097152];
    uint64_t hashes[2];
    for (int32_t run = 0; run < 2; ++run)
    {
        m3WorldId w = Yard();
        bool recording = run == 0 && m3World_JournalBegin(w, journal, (int32_t)sizeof(journal));
        m3ShapeDef sd = m3DefaultShapeDef();
        m3BodyDef cd = m3DefaultBodyDef();
        cd.type = m3_dynamicBody;
        cd.position = (m3Pos3){0.0, 2.0, 0.0};
        m3BodyId p = m3CreateBody(w, &cd);
        m3CreateBoxShape(p, &sd, (m3Vec3){0.4f, 0.1f, 0.4f});
        cd.position = (m3Pos3){0.1, 2.6, 0.0};
        m3BodyId q = m3CreateBody(w, &cd);
        m3CreateBoxShape(q, &sd, (m3Vec3){0.4f, 0.1f, 0.4f});
        m3JointDef pj = m3DefaultJointDef();
        pj.type = m3_parallelJoint;
        pj.bodyA = p;
        pj.bodyB = q;
        pj.localAxisA = (m3Vec3){0.0f, 1.0f, 0.0f};
        pj.localAxisB = (m3Vec3){0.0f, 1.0f, 0.0f};
        m3CreateJoint(&pj);
        pj.type = m3_filterJoint;
        m3CreateJoint(&pj);
        int32_t snapBytes = 0;
        for (int32_t i = 0; i < 180; ++i)
        {
            if (i == 60 && run == 0)
            {
                snapBytes = m3World_Snapshot(w, snap, (int32_t)sizeof(snap));
                CHECK(snapBytes > 0, "the mid-run snapshot fits");
            }
            m3World_Step(w, 1.0f / 60.0f, 4);
        }
        hashes[run] = m3World_Hash(w);
        if (recording)
        {
            int32_t bytes = m3World_JournalEnd(w);
            CHECK(bytes > 0, "the joint session records");
            m3WorldId replayed = Yard();
            CHECK(m3World_JournalReplay(replayed, journal, bytes), "the session replays");
            CHECK(m3World_Hash(replayed) == hashes[0], "the replay is bit-identical");
            m3DestroyWorld(replayed);
            CHECK(m3World_Restore(w, snap, snapBytes), "the mid-run restore lands");
            for (int32_t i = 60; i < 180; ++i)
            {
                m3World_Step(w, 1.0f / 60.0f, 4);
            }
            CHECK(m3World_Hash(w) == hashes[0], "the re-run lands on the same bits");
        }
        m3DestroyWorld(w);
    }
    CHECK(hashes[0] == hashes[1], "twin joint sessions are bit-identical");
}

static double ArmDroop(bool spring, bool motor, float budget)
{
    // A hinged arm under gravity, driven toward horizontal: the
    // droop below the target measures what the drive could not
    // afford. Four states, one scene.
    m3WorldDef wd = m3DefaultWorldDef();
    wd.bodyCapacity = 8;
    wd.shapeCapacity = 8;
    wd.jointCapacity = 4;
    m3WorldId world = m3CreateWorld(&wd);
    m3BodyDef ad = m3DefaultBodyDef();
    ad.position = (m3Pos3){0.0, 3.0, 0.0};
    m3BodyId anchor = m3CreateBody(world, &ad);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.8, 3.0, 0.0};
    m3BodyId arm = m3CreateBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3CreateBoxShape(arm, &sd, (m3Vec3){0.7f, 0.08f, 0.08f});
    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_revoluteJoint;
    jd.bodyA = anchor;
    jd.bodyB = arm;
    jd.localAnchorB = (m3Vec3){-0.8f, 0.0f, 0.0f};
    jd.localAxisA = (m3Vec3){0.0f, 0.0f, 1.0f};
    jd.localAxisB = (m3Vec3){0.0f, 0.0f, 1.0f};
    jd.enableMotor = motor;
    jd.motorSpeed = 0.0f;
    jd.maxMotorEffort = budget;
    m3JointId joint = m3CreateJoint(&jd);
    if (spring)
    {
        m3Joint_SetSpring(joint, true, 6.0f, 1.0f);
        m3Joint_SetTargetAngle(joint, 0.0f);
    }
    for (int32_t i = 0; i < 240; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    double droop = 3.0 - m3Body_GetPosition(arm).y;
    m3DestroyWorld(world);
    return droop;
}

static void TestFourStateDrive(void)
{
    // off: the arm falls and hangs.
    double off = ArmDroop(false, false, 0.0f);
    CHECK(off > 0.5, "state OFF lets the arm hang");
    // velocity only (speed 0, generous budget): a brake, not a hold;
    // it resists but gravity still wins ground slowly.
    double braked = ArmDroop(false, true, 50.0f);
    CHECK(braked < off, "state VELOCITY brakes the fall");
    // position only: the spring holds the arm near horizontal.
    double held = ArmDroop(true, false, 0.0f);
    CHECK(held < 0.15, "state POSITION holds the target");
    // position + velocity with a STARVED shared budget: the drive
    // may not spend more than the motor's allowance, so the arm
    // sags well below the unbudgeted hold (16-2).
    double starved = ArmDroop(true, true, 0.05f);
    CHECK(starved > held + 0.1, "the shared budget honestly starves the drive");
    // position + velocity with a rich budget matches the hold.
    double rich = ArmDroop(true, true, 50.0f);
    CHECK(rich < 0.2, "a rich shared budget still holds");
}

static void TestWheelSteering(void)
{
    // A hub and a rim on a wheel joint in zero gravity: the steer
    // drive yaws the rim about the strut to the target, reads back
    // through GetSteerAngle, honors the journal, and refuses the
    // hostile wall.
    static uint8_t journal[65536];
    uint64_t hashes[2];
    for (int32_t run = 0; run < 2; ++run)
    {
        m3WorldDef wd = m3DefaultWorldDef();
        wd.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
        wd.bodyCapacity = 8;
        wd.shapeCapacity = 8;
        wd.jointCapacity = 4;
        m3WorldId world = m3CreateWorld(&wd);
        bool recording = run == 0 && m3World_JournalBegin(world, journal, (int32_t)sizeof(journal));
        m3BodyDef hd = m3DefaultBodyDef();
        hd.position = (m3Pos3){0.0, 2.0, 0.0};
        m3BodyId hub = m3CreateBody(world, &hd);
        m3BodyDef rd = m3DefaultBodyDef();
        rd.type = m3_dynamicBody;
        rd.position = (m3Pos3){0.0, 1.4, 0.0};
        m3BodyId rim = m3CreateBody(world, &rd);
        m3ShapeDef sd = m3DefaultShapeDef();
        m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.3f};
        m3CreateSphereShape(rim, &sd, &ball);
        m3JointDef jd = m3DefaultJointDef();
        jd.type = m3_wheelJoint;
        jd.bodyA = hub;
        jd.bodyB = rim;
        jd.localAxisA = (m3Vec3){0.0f, -1.0f, 0.0f}; // strut down
        jd.localAxisB = (m3Vec3){0.0f, 0.0f, 1.0f};  // axle
        m3JointId wheel = m3CreateJoint(&jd);
        CHECK(m3Joint_IsValid(wheel), "the wheel creates");
        m3Joint_SetSteer(wheel, true, 0.5f, 8.0f, 1.0f, 0.0f);
        for (int32_t i = 0; i < 120; ++i)
        {
            m3World_Step(world, 1.0f / 60.0f, 4);
        }
        float angle = m3Joint_GetSteerAngle(wheel);
        CHECK(fabsf(angle - 0.5f) < 0.05f, "the rim yaws to the steer target");
        m3Joint_SetSteer(wheel, true, -0.5f, 8.0f, 1.0f, 0.0f);
        for (int32_t i = 0; i < 120; ++i)
        {
            m3World_Step(world, 1.0f / 60.0f, 4);
        }
        CHECK(fabsf(m3Joint_GetSteerAngle(wheel) + 0.5f) < 0.05f, "the steer reverses");
        // The hostile wall runs in BOTH runs so the twins stay
        // symmetric: a refused steer never touches state.
        m3Joint_SetSteer(wheel, true, 2.0f, 8.0f, 1.0f, 0.0f); // beyond the range
        CHECK(m3Joint_GetSteerAngle(wheel) > -1.1f, "an absurd target refused, the drive holds");
        hashes[run] = m3World_Hash(world);
        if (recording)
        {
            int32_t bytes = m3World_JournalEnd(world);
            CHECK(bytes > 0, "the steering session records");
            m3WorldDef fresh = wd;
            m3WorldId replayed = m3CreateWorld(&fresh);
            CHECK(m3World_JournalReplay(replayed, journal, bytes), "the steering replays");
            CHECK(m3World_Hash(replayed) == hashes[0], "the replay is bit-identical");
            m3DestroyWorld(replayed);
        }
        m3DestroyWorld(world);
    }
    CHECK(hashes[0] == hashes[1], "twin steering sessions are bit-identical");

    // The type wall lives outside the twins: a spherical never
    // steers, and asking changes nothing.
    m3WorldDef wd = m3DefaultWorldDef();
    wd.bodyCapacity = 8;
    wd.shapeCapacity = 8;
    wd.jointCapacity = 4;
    m3WorldId world = m3CreateWorld(&wd);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.position = (m3Pos3){0.0, 2.0, 0.0};
    m3BodyId a = m3CreateBody(world, &bd);
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 1.0, 0.0};
    m3BodyId b = m3CreateBody(world, &bd);
    m3JointDef jd = m3DefaultJointDef();
    jd.bodyA = a;
    jd.bodyB = b;
    m3JointId ball = m3CreateJoint(&jd);
    uint64_t before = m3World_Hash(world);
    m3Joint_SetSteer(ball, true, 0.3f, 8.0f, 1.0f, 0.0f);
    CHECK(m3Joint_GetSteerAngle(ball) == 0.0f, "a spherical never steers");
    CHECK(m3World_Hash(world) == before, "the refused steer moved no bits");
    m3DestroyWorld(world);
}

int main(void)
{
    TestFilterIsRowlessAndFilters();
    TestParallelKeepsAxesParallel();
    TestWallsTwinsAndReplay();
    TestFourStateDrive();
    TestWheelSteering();
    if (s_failures == 0)
    {
        printf("test_newjoints: all passed\n");
        return 0;
    }
    return 1;
}
