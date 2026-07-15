// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The transmission gate (16-6): a GEAR holds spinA + ratio * spinB
// at its create value across two bodies hinged on a common frame
// (counter-rotation under positive ratio, the external mesh), and a
// PULLEY holds length1 + ratio * length2 across two world anchors
// (rigid both ways). Both ride twins, replay, rollback, and the
// hostile walls.

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

static void TestGearCouplesSpin(void)
{
    // Two disks hinged about z on a static frame, meshed 2:1 in
    // zero gravity: kick one and the mesh splits the spin with the
    // opposite sign, and keeps the split.
    m3WorldDef wd = m3DefaultWorldDef();
    wd.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
    wd.bodyCapacity = 8;
    wd.shapeCapacity = 8;
    wd.jointCapacity = 8;
    m3WorldId world = m3CreateWorld(&wd);
    m3BodyDef fd = m3DefaultBodyDef();
    m3BodyId frame = m3CreateBody(world, &fd);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){-0.5, 2.0, 0.0};
    m3BodyId gearA = m3CreateBody(world, &bd);
    bd.position = (m3Pos3){0.5, 2.0, 0.0};
    m3BodyId gearB = m3CreateBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3CreateBoxShape(gearA, &sd, (m3Vec3){0.3f, 0.3f, 0.1f});
    m3CreateBoxShape(gearB, &sd, (m3Vec3){0.3f, 0.3f, 0.1f});
    m3JointDef hd = m3DefaultJointDef();
    hd.type = m3_revoluteJoint;
    hd.bodyA = frame;
    hd.bodyB = gearA;
    hd.localAnchorA = (m3Vec3){-0.5f, 2.0f, 0.0f};
    hd.localAxisA = (m3Vec3){0.0f, 0.0f, 1.0f};
    hd.localAxisB = (m3Vec3){0.0f, 0.0f, 1.0f};
    CHECK(m3Joint_IsValid(m3CreateJoint(&hd)), "hinge A creates");
    hd.bodyB = gearB;
    hd.localAnchorA = (m3Vec3){0.5f, 2.0f, 0.0f};
    CHECK(m3Joint_IsValid(m3CreateJoint(&hd)), "hinge B creates");
    m3JointDef gd = m3DefaultJointDef();
    gd.type = m3_gearJoint;
    gd.bodyA = gearA;
    gd.bodyB = gearB;
    gd.localAxisA = (m3Vec3){0.0f, 0.0f, 1.0f};
    gd.localAxisB = (m3Vec3){0.0f, 0.0f, 1.0f};
    gd.ratio = 2.0f;
    CHECK(m3Joint_IsValid(m3CreateJoint(&gd)), "the gear creates");
    m3Body_ApplyAngularImpulse(gearA, (m3Vec3){0.0f, 0.0f, 0.05f});
    for (int32_t i = 0; i < 120; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3Vec3 wA = m3Body_GetAngularVelocity(gearA);
    m3Vec3 wB = m3Body_GetAngularVelocity(gearB);
    CHECK(fabsf(wA.z) > 0.05f, "the kicked gear genuinely spins");
    CHECK(wA.z * wB.z < 0.0f, "positive ratio counter-rotates (the external mesh)");
    CHECK(fabsf(wA.z + 2.0f * wB.z) < 0.02f, "the mesh holds spinA + 2 spinB");
    m3DestroyWorld(world);
}

static void PulleyLengths(m3BodyId a, m3BodyId b, double* len1, double* len2)
{
    m3Pos3 pa = m3Body_GetPosition(a);
    m3Pos3 pb = m3Body_GetPosition(b);
    double dax = pa.x + 1.0;
    double day = pa.y - 4.0;
    double dbx = pb.x - 1.0;
    double dby = pb.y - 4.0;
    *len1 = sqrt(dax * dax + day * day + pa.z * pa.z);
    *len2 = sqrt(dbx * dbx + dby * dby + pb.z * pb.z);
}

static void TestPulleyTradesRope(void)
{
    // A heavy crate and a light one hang from ropes over two world
    // anchors: the heavy side sinks, the light side rises, and
    // length1 + ratio * length2 keeps its create value throughout.
    for (int32_t pass = 0; pass < 2; ++pass)
    {
        float ratio = pass == 0 ? 1.0f : 2.0f;
        m3WorldDef wd = m3DefaultWorldDef();
        wd.bodyCapacity = 8;
        wd.shapeCapacity = 8;
        wd.jointCapacity = 4;
        m3WorldId world = m3CreateWorld(&wd);
        m3BodyDef bd = m3DefaultBodyDef();
        bd.type = m3_dynamicBody;
        bd.position = (m3Pos3){-1.0, 2.0, 0.0};
        m3BodyId heavy = m3CreateBody(world, &bd);
        bd.position = (m3Pos3){1.0, 2.0, 0.0};
        m3BodyId light = m3CreateBody(world, &bd);
        m3ShapeDef sd = m3DefaultShapeDef();
        m3CreateBoxShape(heavy, &sd, (m3Vec3){0.5f, 0.5f, 0.5f});
        m3CreateBoxShape(light, &sd, (m3Vec3){0.2f, 0.2f, 0.2f});
        m3JointDef jd = m3DefaultJointDef();
        jd.type = m3_pulleyJoint;
        jd.bodyA = heavy;
        jd.bodyB = light;
        jd.groundAnchorA = (m3Pos3){-1.0, 4.0, 0.0};
        jd.groundAnchorB = (m3Pos3){1.0, 4.0, 0.0};
        jd.ratio = ratio;
        CHECK(m3Joint_IsValid(m3CreateJoint(&jd)), "the pulley creates");
        double constant = 2.0 + (double)ratio * 2.0;
        for (int32_t i = 0; i < 150; ++i)
        {
            m3World_Step(world, 1.0f / 60.0f, 4);
        }
        double len1;
        double len2;
        PulleyLengths(heavy, light, &len1, &len2);
        CHECK(m3Body_GetPosition(heavy).y < 1.9, "the heavy side sinks");
        CHECK(m3Body_GetPosition(light).y > 2.1, "the light side rises");
        CHECK(fabs(len1 + (double)ratio * len2 - constant) < 0.03,
              "the rope law holds through the trade");
        m3DestroyWorld(world);
    }
}

static void TestTransmissionWalls(void)
{
    // Hostile defs refuse loudly and leak nothing.
    m3WorldDef wd = m3DefaultWorldDef();
    wd.bodyCapacity = 8;
    wd.shapeCapacity = 8;
    wd.jointCapacity = 4;
    m3WorldId world = m3CreateWorld(&wd);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 2.0, 0.0};
    m3BodyId a = m3CreateBody(world, &bd);
    bd.position = (m3Pos3){1.0, 2.0, 0.0};
    m3BodyId b = m3CreateBody(world, &bd);
    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_gearJoint;
    jd.bodyA = a;
    jd.bodyB = b;
    jd.ratio = 0.0f;
    CHECK(!m3Joint_IsValid(m3CreateJoint(&jd)), "a zero-ratio gear refuses");
    jd.ratio = 1.0f;
    jd.localAxisA = (m3Vec3){0.0f, 0.0f, 0.0f};
    CHECK(!m3Joint_IsValid(m3CreateJoint(&jd)), "a zero-axis gear refuses");
    jd = m3DefaultJointDef();
    jd.type = m3_pulleyJoint;
    jd.bodyA = a;
    jd.bodyB = b;
    jd.groundAnchorA = (m3Pos3){0.0, 4.0, 0.0};
    jd.groundAnchorB = (m3Pos3){1.0, 4.0, 0.0};
    jd.ratio = -1.0f;
    CHECK(!m3Joint_IsValid(m3CreateJoint(&jd)), "a negative-ratio pulley refuses");
    jd.ratio = 0.0f;
    CHECK(!m3Joint_IsValid(m3CreateJoint(&jd)), "a zero-ratio pulley refuses");
    jd.ratio = 1.0f;
    jd.groundAnchorA = (m3Pos3){(double)NAN, 4.0, 0.0};
    CHECK(!m3Joint_IsValid(m3CreateJoint(&jd)), "a NaN world anchor refuses");
    jd.groundAnchorA = (m3Pos3){0.0, 2.0, 0.0}; // ON the body anchor
    CHECK(!m3Joint_IsValid(m3CreateJoint(&jd)), "a rope end on its pulley refuses");
    m3DestroyWorld(world);
}

static void TestTransmissionTwinsAndReplay(void)
{
    // One scene wearing both new types: twins, a journaled replay,
    // and a mid-run rollback all land on the same bits (the pulley
    // world anchors ride the new snapshot blocks).
    static uint8_t journal[131072];
    static uint8_t snap[2097152];
    uint64_t hashes[2];
    for (int32_t run = 0; run < 2; ++run)
    {
        m3WorldDef wd = m3DefaultWorldDef();
        wd.bodyCapacity = 16;
        wd.shapeCapacity = 16;
        wd.jointCapacity = 8;
        m3WorldId w = m3CreateWorld(&wd);
        bool recording = run == 0 && m3World_JournalBegin(w, journal, (int32_t)sizeof(journal));
        m3BodyDef fd = m3DefaultBodyDef();
        m3BodyId frame = m3CreateBody(w, &fd);
        m3BodyDef bd = m3DefaultBodyDef();
        bd.type = m3_dynamicBody;
        m3ShapeDef sd = m3DefaultShapeDef();
        // The gear pair on the frame.
        bd.position = (m3Pos3){-0.5, 2.0, 0.0};
        m3BodyId ga = m3CreateBody(w, &bd);
        m3CreateBoxShape(ga, &sd, (m3Vec3){0.3f, 0.3f, 0.1f});
        bd.position = (m3Pos3){0.5, 2.0, 0.0};
        m3BodyId gb = m3CreateBody(w, &bd);
        m3CreateBoxShape(gb, &sd, (m3Vec3){0.3f, 0.3f, 0.1f});
        m3JointDef hd = m3DefaultJointDef();
        hd.type = m3_revoluteJoint;
        hd.bodyA = frame;
        hd.bodyB = ga;
        hd.localAnchorA = (m3Vec3){-0.5f, 2.0f, 0.0f};
        hd.localAxisA = (m3Vec3){0.0f, 0.0f, 1.0f};
        hd.localAxisB = (m3Vec3){0.0f, 0.0f, 1.0f};
        m3CreateJoint(&hd);
        hd.bodyB = gb;
        hd.localAnchorA = (m3Vec3){0.5f, 2.0f, 0.0f};
        m3CreateJoint(&hd);
        m3JointDef gd = m3DefaultJointDef();
        gd.type = m3_gearJoint;
        gd.bodyA = ga;
        gd.bodyB = gb;
        gd.localAxisA = (m3Vec3){0.0f, 0.0f, 1.0f};
        gd.localAxisB = (m3Vec3){0.0f, 0.0f, 1.0f};
        gd.ratio = 2.0f;
        m3CreateJoint(&gd);
        m3Body_ApplyAngularImpulse(ga, (m3Vec3){0.0f, 0.0f, 0.05f});
        // The pulley pair beside it.
        bd.position = (m3Pos3){-3.0, 2.0, 0.0};
        m3BodyId heavy = m3CreateBody(w, &bd);
        m3CreateBoxShape(heavy, &sd, (m3Vec3){0.5f, 0.5f, 0.5f});
        bd.position = (m3Pos3){3.0, 2.0, 0.0};
        m3BodyId light = m3CreateBody(w, &bd);
        m3CreateBoxShape(light, &sd, (m3Vec3){0.2f, 0.2f, 0.2f});
        m3JointDef pd = m3DefaultJointDef();
        pd.type = m3_pulleyJoint;
        pd.bodyA = heavy;
        pd.bodyB = light;
        pd.groundAnchorA = (m3Pos3){-3.0, 4.0, 0.0};
        pd.groundAnchorB = (m3Pos3){3.0, 4.0, 0.0};
        pd.ratio = 1.5f;
        m3CreateJoint(&pd);
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
            CHECK(bytes > 0, "the transmission session records");
            m3WorldDef fresh = wd;
            m3WorldId replayed = m3CreateWorld(&fresh);
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
    CHECK(hashes[0] == hashes[1], "twin transmission sessions are bit-identical");
}

int main(void)
{
    TestGearCouplesSpin();
    TestPulleyTradesRope();
    TestTransmissionWalls();
    TestTransmissionTwinsAndReplay();
    if (s_failures == 0)
    {
        printf("test_gearpulley: all passed\n");
        return 0;
    }
    printf("test_gearpulley: %d FAILURES\n", s_failures);
    return 1;
}
