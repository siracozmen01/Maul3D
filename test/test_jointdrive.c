// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The joint runtime gate (8-6a): limits and motors bind mid-run,
// the collide toggle separates an overlapped pair, readback lands
// in the m*g band on a hanging load, breakage fires exactly once
// at the analytic threshold, and every op replays bit-exact.

#include "maul3d/joint.h"
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

static m3WorldId SmallWorld(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    def.jointCapacity = 8;
    return m3CreateWorld(&def);
}

static m3BodyId Box(m3WorldId world, m3Pos3 at, float half, int dynamic)
{
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = dynamic ? m3_dynamicBody : m3_staticBody;
    bd.position = at;
    m3BodyId body = m3CreateBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3CreateBoxShape(body, &sd, (m3Vec3){half, half, half});
    return body;
}

static void StepN(m3WorldId world, int32_t n)
{
    for (int32_t i = 0; i < n; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
}

static void TestRuntimeMotorAndLimits(void)
{
    // A hinge door: the runtime motor swings it, the runtime limit
    // stops the swing at the analytic angle.
    m3WorldId world = SmallWorld();
    m3BodyId post = Box(world, (m3Pos3){0.0, 2.0, 0.0}, 0.2f, 0);
    m3BodyId door = Box(world, (m3Pos3){0.8, 2.0, 0.0}, 0.4f, 1);
    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_revoluteJoint;
    jd.bodyA = post;
    jd.bodyB = door;
    jd.localAnchorA = (m3Vec3){0.4f, 0.0f, 0.0f};
    jd.localAnchorB = (m3Vec3){-0.4f, 0.0f, 0.0f};
    jd.localAxisA = (m3Vec3){0.0f, 1.0f, 0.0f};
    jd.localAxisB = (m3Vec3){0.0f, 1.0f, 0.0f};
    m3JointId hinge = m3CreateJoint(&jd);
    StepN(world, 30);
    CHECK(fabsf(m3Joint_GetAngle(hinge)) < 0.05f, "the unmotored door hangs still");

    m3Joint_SetMotor(hinge, true, 2.0f, 100.0f);
    StepN(world, 60);
    float swung = m3Joint_GetAngle(hinge);
    CHECK(fabsf(swung) > 0.8f, "the runtime motor swings the door");

    m3Joint_SetMotor(hinge, true, -2.0f, 100.0f);
    m3Joint_SetLimits(hinge, true, -0.5f, 0.5f);
    StepN(world, 120);
    float held = m3Joint_GetAngle(hinge);
    CHECK(held > -0.65f && held < -0.35f, "the runtime limit holds the swing at the stop");
    m3DestroyWorld(world);
}

static void TestCollideToggle(void)
{
    // Two overlapped jointed boxes: connected pairs skip contacts
    // by default, so they interpenetrate in peace; the runtime
    // toggle turns contacts on and the pair pushes apart.
    m3WorldId world = SmallWorld();
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sg = m3DefaultShapeDef();
    m3Plane fl = {{0.0f, 1.0f, 0.0f}, 0.0f};
    m3CreatePlaneShape(ground, &sg, &fl);
    m3BodyId a = Box(world, (m3Pos3){0.0, 0.5, 0.0}, 0.5f, 1);
    m3BodyId b = Box(world, (m3Pos3){0.6, 0.5, 0.0}, 0.5f, 1);
    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_sphericalJoint;
    jd.bodyA = a;
    jd.bodyB = b;
    jd.localAnchorA = (m3Vec3){0.3f, 0.6f, 0.0f};
    jd.localAnchorB = (m3Vec3){-0.3f, 0.6f, 0.0f};
    m3JointId link = m3CreateJoint(&jd);
    StepN(world, 60);
    double gap0 = m3Body_GetPosition(b).x - m3Body_GetPosition(a).x;
    CHECK(gap0 < 0.95, "connected boxes overlap in peace");
    CHECK(!m3Joint_GetCollideConnected(link), "the flag reads back off");
    m3Joint_SetCollideConnected(link, true);
    CHECK(m3Joint_GetCollideConnected(link), "the flag reads back on");
    StepN(world, 120);
    double gap1 = m3Body_GetPosition(b).x - m3Body_GetPosition(a).x;
    CHECK(gap1 > gap0 + 0.2, "contacts on: the pair pushes apart");
    m3DestroyWorld(world);
}

static void TestReadbackBand(void)
{
    // A 1 kg crate hanging from a static anchor: the joint carries
    // m*g = 10 N once settled. Torque stays near zero.
    m3WorldId world = SmallWorld();
    m3BodyId anchor = Box(world, (m3Pos3){0.0, 5.0, 0.0}, 0.2f, 0);
    m3BodyId crate = Box(world, (m3Pos3){0.0, 3.8, 0.0}, 0.5f, 1);
    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_sphericalJoint;
    jd.bodyA = anchor;
    jd.bodyB = crate;
    jd.localAnchorA = (m3Vec3){0.0f, -0.2f, 0.0f};
    jd.localAnchorB = (m3Vec3){0.0f, 0.5f, 0.0f};
    m3JointId rope = m3CreateJoint(&jd);
    StepN(world, 180);
    m3real force = m3Joint_GetConstraintForce(rope);
    m3real torque = m3Joint_GetConstraintTorque(rope);
    CHECK(force > 8.0f && force < 12.0f, "the rope carries the m*g band");
    CHECK(torque < 2.0f, "a ball joint carries no torque worth naming");
    m3DestroyWorld(world);
}

static void TestBreakage(void)
{
    // The same hanging crate, heavier: a cap ABOVE the load holds
    // forever; a cap UNDER the load breaks exactly once, the event
    // carries the id, and the crate falls free.
    for (int32_t pass = 0; pass < 2; ++pass)
    {
        m3WorldId world = SmallWorld();
        m3BodyId anchor = Box(world, (m3Pos3){0.0, 6.0, 0.0}, 0.2f, 0);
        m3BodyId crate = Box(world, (m3Pos3){0.0, 4.3, 0.0}, 1.0f, 1); // mass 8, weight 80
        m3JointDef jd = m3DefaultJointDef();
        jd.type = m3_sphericalJoint;
        jd.bodyA = anchor;
        jd.bodyB = crate;
        jd.localAnchorA = (m3Vec3){0.0f, -0.2f, 0.0f};
        jd.localAnchorB = (m3Vec3){0.0f, 1.0f, 0.0f};
        m3JointId rope = m3CreateJoint(&jd);
        StepN(world, 60); // settle first, break laws read clean loads
        m3Joint_SetBreakThresholds(rope, pass == 0 ? 200.0f : 40.0f, 0.0f);
        int32_t breaks = 0;
        bool idMatches = true;
        for (int32_t i = 0; i < 120; ++i)
        {
            m3World_Step(world, 1.0f / 60.0f, 4);
            int32_t n = 0;
            const m3JointBreakEvent* ev = m3World_JointBreakEvents(world, &n);
            for (int32_t k = 0; k < n; ++k)
            {
                if (ev[k].joint.index1 != rope.index1)
                {
                    idMatches = false;
                }
            }
            breaks += n;
        }
        if (pass == 0)
        {
            CHECK(breaks == 0, "a cap above the load holds");
            CHECK(m3Joint_IsValid(rope), "the held joint stays valid");
        }
        else
        {
            CHECK(breaks == 1, "a cap under the load breaks exactly once");
            CHECK(idMatches, "the break event carries the joint id");
            CHECK(!m3Joint_IsValid(rope), "the broken joint destroyed itself");
            CHECK(m3Body_GetPosition(crate).y < 2.0, "the freed crate falls");
        }
        m3DestroyWorld(world);
    }
}

static void TestSpringServo(void)
{
    // The hinge door driven by the runtime angle spring: with the
    // hinge axis vertical, gravity has no say, and the door parks
    // in the target band. 8-6b.
    m3WorldId world = SmallWorld();
    m3BodyId post = Box(world, (m3Pos3){0.0, 2.0, 0.0}, 0.2f, 0);
    m3BodyId door = Box(world, (m3Pos3){0.8, 2.0, 0.0}, 0.4f, 1);
    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_revoluteJoint;
    jd.bodyA = post;
    jd.bodyB = door;
    jd.localAnchorA = (m3Vec3){0.4f, 0.0f, 0.0f};
    jd.localAnchorB = (m3Vec3){-0.4f, 0.0f, 0.0f};
    jd.localAxisA = (m3Vec3){0.0f, 1.0f, 0.0f};
    jd.localAxisB = (m3Vec3){0.0f, 1.0f, 0.0f};
    m3JointId hinge = m3CreateJoint(&jd);
    m3Joint_SetSpring(hinge, true, 8.0f, 1.0f);
    m3Joint_SetTargetAngle(hinge, 0.6f);
    StepN(world, 240);
    float angle = m3Joint_GetAngle(hinge);
    CHECK(fabsf(fabsf(angle) - 0.6f) < 0.1f, "the angle servo parks in the target band");
    m3DestroyWorld(world);
}

static void TestSpringDampingContrast(void)
{
    // The same slider, two damping ratios: the underdamped run
    // overshoots its target, the critically damped run does not.
    // Gravity-free so the axis owns the whole story.
    double overshoot[2];
    for (int32_t pass = 0; pass < 2; ++pass)
    {
        m3WorldDef def = m3DefaultWorldDef();
        def.bodyCapacity = 8;
        def.shapeCapacity = 8;
        def.jointCapacity = 4;
        def.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
        m3WorldId world = m3CreateWorld(&def);
        m3BodyId rail = Box(world, (m3Pos3){0.0, 2.0, 0.0}, 0.2f, 0);
        m3BodyId slider = Box(world, (m3Pos3){0.0, 2.0, 0.0}, 0.3f, 1);
        m3JointDef jd = m3DefaultJointDef();
        jd.type = m3_prismaticJoint;
        jd.bodyA = rail;
        jd.bodyB = slider;
        jd.localAxisA = (m3Vec3){1.0f, 0.0f, 0.0f};
        jd.localAxisB = (m3Vec3){1.0f, 0.0f, 0.0f};
        m3JointId slide = m3CreateJoint(&jd);
        m3Joint_SetSpring(slide, true, 2.0f, pass == 0 ? 0.05f : 1.5f);
        m3Joint_SetTargetTranslation(slide, 1.0f);
        double peak = 0.0;
        for (int32_t i = 0; i < 300; ++i)
        {
            m3World_Step(world, 1.0f / 60.0f, 4);
            double t = (double)m3Joint_GetTranslation(slide);
            if (t > peak)
            {
                peak = t;
            }
        }
        overshoot[pass] = peak - 1.0;
        double rest = (double)m3Joint_GetTranslation(slide);
        CHECK(fabs(rest - 1.0) < 0.08, "the translation servo settles at the target");
        m3DestroyWorld(world);
    }
    CHECK(overshoot[0] > 0.1, "the underdamped spring overshoots");
    CHECK(overshoot[1] < 0.05, "the damped spring does not");
}

static void TestSphericalRotationDrive(void)
{
    // A floating crate driven to a quarter turn about y by the
    // rotation spring. Gravity-free; the drive owns the pose.
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    def.jointCapacity = 4;
    def.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
    m3WorldId world = m3CreateWorld(&def);
    m3BodyId anchor = Box(world, (m3Pos3){0.0, 2.0, 0.0}, 0.2f, 0);
    m3BodyId crate = Box(world, (m3Pos3){0.0, 2.0, 1.2}, 0.4f, 1);
    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_sphericalJoint;
    jd.bodyA = anchor;
    jd.bodyB = crate;
    jd.localAnchorA = (m3Vec3){0.0f, 0.0f, 0.6f};
    jd.localAnchorB = (m3Vec3){0.0f, 0.0f, -0.6f};
    // The target lives in the joint frames (frame z = the local
    // axis, the documented reference semantic). Put the frame's z
    // on world y, so a frame-z target turns the crate about y.
    jd.localAxisA = (m3Vec3){0.0f, 1.0f, 0.0f};
    jd.localAxisB = (m3Vec3){0.0f, 1.0f, 0.0f};
    m3JointId ball = m3CreateJoint(&jd);
    float s = sinf(0.25f * M3_PI * 0.5f);
    float c = cosf(0.25f * M3_PI * 0.5f);
    m3Quat target = {0.0f, 0.0f, s, c}; // 45 degrees about frame z
    m3Joint_SetSpring(ball, true, 6.0f, 1.0f);
    m3Joint_SetTargetRotation(ball, target);
    StepN(world, 300);
    m3Quat q = m3Body_GetRotation(crate);
    m3Quat want = {0.0f, s, 0.0f, c}; // 45 degrees about world y
    float dot = fabsf(q.x * want.x + q.y * want.y + q.z * want.z + q.w * want.w);
    CHECK(dot > 0.98f, "the rotation drive lands the crate near the target pose");
    m3DestroyWorld(world);
}

static void TestRuntimeOpsReplay(void)
{
    // The whole 8-6a op family under the journal: twins land on the
    // same bits and a fresh replay reproduces the final hash.
    static uint8_t journal[131072];
    uint64_t hashes[2];
    for (int32_t run = 0; run < 2; ++run)
    {
        m3WorldDef def = m3DefaultWorldDef();
        def.bodyCapacity = 16;
        def.shapeCapacity = 16;
        def.jointCapacity = 8;
        m3WorldId world = m3CreateWorld(&def);
        bool recording = run == 0 && m3World_JournalBegin(world, journal, (int32_t)sizeof(journal));
        m3BodyId post = Box(world, (m3Pos3){0.0, 2.0, 0.0}, 0.2f, 0);
        m3BodyId door = Box(world, (m3Pos3){0.8, 2.0, 0.0}, 0.4f, 1);
        m3JointDef jd = m3DefaultJointDef();
        jd.type = m3_revoluteJoint;
        jd.bodyA = post;
        jd.bodyB = door;
        jd.localAnchorA = (m3Vec3){0.4f, 0.0f, 0.0f};
        jd.localAnchorB = (m3Vec3){-0.4f, 0.0f, 0.0f};
        jd.localAxisA = (m3Vec3){0.0f, 1.0f, 0.0f};
        jd.localAxisB = (m3Vec3){0.0f, 1.0f, 0.0f};
        m3JointId hinge = m3CreateJoint(&jd);
        StepN(world, 20);
        m3Joint_SetMotor(hinge, true, 3.0f, 50.0f);
        StepN(world, 20);
        m3Joint_SetLimits(hinge, true, -0.3f, 0.3f);
        m3Joint_SetCollideConnected(hinge, true);
        StepN(world, 20);
        m3Joint_SetBreakThresholds(hinge, 30.0f, 25.0f);
        m3Joint_SetSpring(hinge, true, 5.0f, 0.7f);
        m3Joint_SetTargetAngle(hinge, 0.2f);
        StepN(world, 60);
        uint64_t final = m3World_Hash(world);
        hashes[run] = final;
        if (recording)
        {
            int32_t bytes = m3World_JournalEnd(world);
            CHECK(bytes > 0, "the runtime session records");
            m3WorldDef freshDef = m3DefaultWorldDef();
            freshDef.bodyCapacity = 16;
            freshDef.shapeCapacity = 16;
            freshDef.jointCapacity = 8;
            m3WorldId fresh = m3CreateWorld(&freshDef);
            CHECK(m3World_JournalReplay(fresh, journal, bytes), "the runtime session replays");
            CHECK(m3World_Hash(fresh) == final, "the replay is bit-identical");
            m3DestroyWorld(fresh);
        }
        m3DestroyWorld(world);
    }
    CHECK(hashes[0] == hashes[1], "runtime-op twins are bit-identical");
}

static void TestHostileRuntime(void)
{
    m3WorldId world = SmallWorld();
    m3BodyId a = Box(world, (m3Pos3){0.0, 2.0, 0.0}, 0.2f, 0);
    m3BodyId b = Box(world, (m3Pos3){1.0, 2.0, 0.0}, 0.4f, 1);
    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_sphericalJoint;
    jd.bodyA = a;
    jd.bodyB = b;
    m3JointId link = m3CreateJoint(&jd);
    m3Joint_SetLimits(link, true, 1.0f, -1.0f); // lower > upper refused
    m3Joint_SetMotor(link, true, NAN, 10.0f);
    m3Joint_SetMotor(link, true, 1.0f, -5.0f);
    m3Joint_SetBreakThresholds(link, -1.0f, 0.0f);
    m3Joint_SetSpring(link, true, 0.0f, 1.0f); // zero hertz refused
    m3Joint_SetSpring(link, true, NAN, 1.0f);  // NaN refused
    m3Joint_SetTargetAngle(link, 0.5f);        // ball joint: wrong type, no-op
    m3Joint_SetTargetTranslation(link, 0.5f);  // wrong type, no-op
    m3Joint_SetTargetRotation(link, (m3Quat){3.0f, 0.0f, 0.0f, 1.0f}); // non-unit refused
    m3JointId stale = link;
    stale.index1 += 99;
    m3Joint_SetMotor(stale, true, 1.0f, 1.0f);
    CHECK(m3Joint_GetConstraintForce(stale) == 0.0f, "stale id reads zero");
    CHECK(m3Joint_GetAngle(link) == 0.0f, "GetAngle on a ball joint reads zero");
    CHECK(m3Joint_GetTranslation(link) == 0.0f, "GetTranslation on a ball joint reads zero");
    StepN(world, 10);
    CHECK(m3Joint_IsValid(link), "hostile args left the joint whole");
    m3DestroyWorld(world);
}

int main(void)
{
    TestRuntimeMotorAndLimits();
    TestCollideToggle();
    TestReadbackBand();
    TestBreakage();
    TestSpringServo();
    TestSpringDampingContrast();
    TestSphericalRotationDrive();
    TestRuntimeOpsReplay();
    TestHostileRuntime();
    if (s_failures == 0)
    {
        printf("test_jointdrive: all green\n");
        return 0;
    }
    printf("test_jointdrive: %d failure(s)\n", s_failures);
    return 1;
}
