// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Joint gate (2c-2): the spherical point constraint against analytic
// pendulum physics, a hanging chain, and the determinism spine
// (rollback, replay with minted joint ids, sleep coupling).

#include "maul3d/joint.h"
#include "maul3d/shape.h"

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

static void StepN(m3WorldId world, int32_t steps)
{
    for (int32_t i = 0; i < steps; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
}

static void TestPendulum(void)
{
    // A point pendulum of length one released horizontal: the bob
    // must pass the bottom near sqrt(2 g L) and the rod length must
    // hold within the soft-constraint band the whole swing.
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m3WorldId world = m3CreateWorld(&def);

    m3BodyDef ad = m3DefaultBodyDef();
    m3BodyId anchor = m3CreateBody(world, &ad); // shapeless static pivot

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){1.0, 0.0, 0.0};
    m3BodyId bob = m3CreateBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.1f};
    m3CreateSphereShape(bob, &sd, &ball);

    m3JointDef jd = m3DefaultJointDef();
    jd.bodyA = anchor;
    jd.bodyB = bob;
    jd.localAnchorA = (m3Vec3){0.0f, 0.0f, 0.0f};
    jd.localAnchorB = (m3Vec3){-1.0f, 0.0f, 0.0f}; // the rod end on the bob
    m3JointId joint = m3CreateJoint(&jd);
    CHECK(m3Joint_IsValid(joint), "the pendulum joint creates");

    double vMax = 0.0;
    double maxStretch = 0.0;
    for (int32_t i = 0; i < 120; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
        m3Pos3 p = m3Body_GetPosition(bob);
        m3Vec3 v = m3Body_GetLinearVelocity(bob);
        double speed = (double)m3Dot3(v, v);
        vMax = speed > vMax ? speed : vMax;
        m3Quat q = m3Body_GetRotation(bob);
        m3Vec3 arm = m3RotateVec3(q, (m3Vec3){-1.0f, 0.0f, 0.0f});
        double px = p.x + (double)arm.x;
        double py = p.y + (double)arm.y;
        double pz = p.z + (double)arm.z;
        double drift = px * px + py * py + pz * pz; // pinned point vs origin
        maxStretch = drift > maxStretch ? drift : maxStretch;
    }
    // sqrt(2 * 10 * 1) = 4.47; the soft constraint bleeds a little.
    CHECK(vMax > 15.0 && vMax < 21.0, "the bob passes the bottom in the energy band");
    CHECK(maxStretch < 0.005, "the pinned point holds inside the soft band");
    m3DestroyWorld(world);
}

static void TestChainSettlesAndSleeps(void)
{
    // Four links hang from a static anchor, settle vertically, and
    // the whole articulated island crosses into sleep TOGETHER (the
    // joint-union proof); the sleeping chain is bit-frozen.
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    m3WorldId world = m3CreateWorld(&def);

    m3BodyDef ad = m3DefaultBodyDef();
    ad.position = (m3Pos3){0.0, 4.0, 0.0};
    m3BodyId anchor = m3CreateBody(world, &ad);

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    // A frictionless multi-pendulum is chaotic and conserves energy
    // forever; damping is how a rope stops being a perpetual motion
    // machine. Start near vertical, bleed the sway out.
    bd.linearDamping = 1.5f;
    bd.angularDamping = 1.5f;
    m3ShapeDef sd = m3DefaultShapeDef();
    m3BodyId previous = anchor;
    m3BodyId links[4];
    for (int32_t k = 0; k < 4; ++k)
    {
        bd.position = (m3Pos3){0.05 * (double)(k + 1), 4.0 - 0.5 * (double)(k + 1), 0.0};
        links[k] = m3CreateBody(world, &bd);
        m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.12f};
        m3CreateSphereShape(links[k], &sd, &ball);

        m3JointDef jd = m3DefaultJointDef();
        jd.bodyA = previous;
        jd.bodyB = links[k];
        jd.localAnchorA = (m3Vec3){0.0f, k == 0 ? 0.0f : -0.25f, 0.0f};
        jd.localAnchorB = (m3Vec3){0.0f, 0.25f, 0.0f};
        CHECK(m3Joint_IsValid(m3CreateJoint(&jd)), "a chain link joins");
        previous = links[k];
    }

    StepN(world, 900); // settle plus the sleep bar
    for (int32_t k = 0; k < 4; ++k)
    {
        m3Pos3 p = m3Body_GetPosition(links[k]);
        CHECK(p.x > -0.05 && p.x < 0.05, "the chain hangs vertical");
        m3Vec3 v = m3Body_GetLinearVelocity(links[k]);
        CHECK(v.x == 0.0f && v.y == 0.0f && v.z == 0.0f, "the settled link sleeps");
    }
    m3Pos3 before = m3Body_GetPosition(links[3]);
    StepN(world, 30);
    m3Pos3 after = m3Body_GetPosition(links[3]);
    CHECK(before.x == after.x && before.y == after.y && before.z == after.z,
          "the sleeping chain is bit-frozen");

    // Waking the TOP link must wake the whole articulated island.
    m3Body_SetLinearVelocity(links[0], (m3Vec3){2.0f, 0.0f, 0.0f});
    StepN(world, 30);
    m3Pos3 tail = m3Body_GetPosition(links[3]);
    CHECK(tail.x != after.x || tail.y != after.y, "the joint union wakes the whole chain");
    m3DestroyWorld(world);
}

static void TestJointDeterminismSpine(void)
{
    // Joints ride every spine: a journaled pendulum session replays
    // with identical minted ids and bits; a mid-swing snapshot
    // restores and reruns bit-exact; destroying the joint mid-flight
    // journals and replays too.
    uint8_t journal[32768];
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m3WorldId world = m3CreateWorld(&def);
    CHECK(m3World_JournalBegin(world, journal, (int32_t)sizeof(journal)), "journal arms");

    m3BodyDef ad = m3DefaultBodyDef();
    m3BodyId anchor = m3CreateBody(world, &ad);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.8, 0.0, 0.0};
    m3BodyId bob = m3CreateBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.1f};
    m3CreateSphereShape(bob, &sd, &ball);
    m3JointDef jd = m3DefaultJointDef();
    jd.bodyA = anchor;
    jd.bodyB = bob;
    jd.localAnchorB = (m3Vec3){-0.8f, 0.0f, 0.0f};
    m3JointId joint = m3CreateJoint(&jd);
    CHECK(m3Joint_IsValid(joint), "the journaled joint creates");

    StepN(world, 60);
    m3DestroyJoint(joint); // the bob flies free, journaled
    StepN(world, 30);

    uint64_t h1 = m3World_Hash(world);
    int32_t bytes = m3World_JournalEnd(world);
    CHECK(bytes > 0, "the joint session recorded");
    m3WorldId twin = m3CreateWorld(&def);
    CHECK(m3World_JournalReplay(twin, journal, bytes), "the joint session replays");
    CHECK(m3World_Hash(twin) == h1, "the replay is bit-identical, joint ids included");
    m3DestroyWorld(twin);

    // Rollback with a live joint.
    m3WorldId w2 = m3CreateWorld(&def);
    m3BodyId anchor2 = m3CreateBody(w2, &ad);
    bd.position = (m3Pos3){0.8, 0.0, 0.0};
    m3BodyId bob2 = m3CreateBody(w2, &bd);
    m3CreateSphereShape(bob2, &sd, &ball);
    jd.bodyA = anchor2;
    jd.bodyB = bob2;
    CHECK(m3Joint_IsValid(m3CreateJoint(&jd)), "the rollback joint creates");
    StepN(w2, 40);
    int32_t snapBytes = m3World_SnapshotSize(w2);
    void* snap = malloc((size_t)snapBytes);
    CHECK(m3World_Snapshot(w2, snap, snapBytes) == snapBytes, "snapshot writes");
    StepN(w2, 50);
    uint64_t after = m3World_Hash(w2);
    CHECK(m3World_Restore(w2, snap, snapBytes), "snapshot restores");
    StepN(w2, 50);
    CHECK(m3World_Hash(w2) == after, "the pendulum rolls back bit-exact");
    free(snap);
    m3DestroyWorld(w2);
    m3DestroyWorld(world);
}

static void TestJointContracts(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    def.jointCapacity = 1;
    m3WorldId world = m3CreateWorld(&def);
    m3BodyDef ad = m3DefaultBodyDef();
    m3BodyId a = m3CreateBody(world, &ad);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){1.0, 0.0, 0.0};
    m3BodyId b = m3CreateBody(world, &bd);
    m3BodyId c = m3CreateBody(world, &bd);

    m3JointDef jd = m3DefaultJointDef();
    jd.bodyA = a;
    jd.bodyB = a;
    CHECK(!m3Joint_IsValid(m3CreateJoint(&jd)), "a self-joint is refused");
    jd.bodyB = m3CreateBody(world, &ad); // static-static
    jd.bodyA = a;
    CHECK(!m3Joint_IsValid(m3CreateJoint(&jd)), "an immovable pair is refused");
    jd.bodyB = b;
    m3JointId first = m3CreateJoint(&jd);
    CHECK(m3Joint_IsValid(first), "the single slot fills");
    jd.bodyB = c;
    CHECK(!m3Joint_IsValid(m3CreateJoint(&jd)), "the joint pool refuses past capacity");

    // Destroying a body cascades its joints.
    m3DestroyBody(b);
    CHECK(!m3Joint_IsValid(first), "destroying a body destroys its joints");
    m3World_Step(world, 1.0f / 60.0f, 4);
    m3DestroyWorld(world);
}

static m3BodyId MakeDoor(m3WorldId world, m3JointDef* jdOut)
{
    // A door panel hinged to a static post on the y axis: the hinge
    // line passes through x = 0, the panel extends to +x.
    m3BodyDef pd = m3DefaultBodyDef();
    m3BodyId post = m3CreateBody(world, &pd);

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.5, 0.0, 0.0};
    m3BodyId door = m3CreateBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3CreateBoxShape(door, &sd, (m3Vec3){0.5f, 1.0f, 0.05f});

    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_revoluteJoint;
    jd.bodyA = post;
    jd.bodyB = door;
    jd.localAnchorA = (m3Vec3){0.0f, 0.0f, 0.0f};
    jd.localAnchorB = (m3Vec3){-0.5f, 0.0f, 0.0f};
    jd.localAxisA = (m3Vec3){0.0f, 1.0f, 0.0f};
    jd.localAxisB = (m3Vec3){0.0f, 1.0f, 0.0f};
    *jdOut = jd;
    return door;
}

static double DoorAngle(m3BodyId door)
{
    // The hinge axis is world y and the door starts along +x, so the
    // yaw of the body quaternion IS the door angle (collinearity
    // keeps the other two rotations locked).
    m3Quat q = m3Body_GetRotation(door);
    return 2.0 * (double)m3Atan2(q.y, q.w);
}

static void TestDoorSwings(void)
{
    // Gravity off: a pushed door spins about the hinge axis only,
    // the hinge line never drifts, and no off-axis rotation leaks.
    m3WorldDef def = m3DefaultWorldDef();
    def.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m3WorldId world = m3CreateWorld(&def);
    m3JointDef jd;
    m3BodyId door = MakeDoor(world, &jd);
    CHECK(m3Joint_IsValid(m3CreateJoint(&jd)), "the hinge creates");
    m3Body_SetAngularVelocity(door, (m3Vec3){0.0f, 2.0f, 0.0f});

    double maxDrift = 0.0;
    double maxOffAxis = 0.0;
    for (int32_t i = 0; i < 240; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
        m3Pos3 p = m3Body_GetPosition(door);
        m3Quat q = m3Body_GetRotation(door);
        m3Vec3 hinge = m3RotateVec3(q, (m3Vec3){-0.5f, 0.0f, 0.0f});
        double hx = p.x + (double)hinge.x;
        double hy = p.y + (double)hinge.y;
        double hz = p.z + (double)hinge.z;
        double drift = hx * hx + hy * hy + hz * hz;
        maxDrift = drift > maxDrift ? drift : maxDrift;
        double off = (double)(q.x * q.x + q.z * q.z); // yaw-only means x, z stay zero
        maxOffAxis = off > maxOffAxis ? off : maxOffAxis;
    }
    CHECK(maxDrift < 0.0005, "the hinge line never drifts");
    CHECK(maxOffAxis < 0.001, "no off-axis rotation leaks");
    m3DestroyWorld(world);
}

static void TestDoorLimits(void)
{
    // Limits at plus and minus a quarter turn: a hard shove never
    // carries the door past the stops (plus the speculative band).
    m3WorldDef def = m3DefaultWorldDef();
    def.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m3WorldId world = m3CreateWorld(&def);
    m3JointDef jd;
    m3BodyId door = MakeDoor(world, &jd);
    jd.enableLimit = true;
    jd.lowerAngle = -0.7854f;
    jd.upperAngle = 0.7854f;
    CHECK(m3Joint_IsValid(m3CreateJoint(&jd)), "the limited hinge creates");
    m3Body_SetAngularVelocity(door, (m3Vec3){0.0f, 12.0f, 0.0f});

    double maxAngle = 0.0;
    for (int32_t i = 0; i < 240; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
        if (i == 120)
        {
            m3Body_SetAngularVelocity(door, (m3Vec3){0.0f, -12.0f, 0.0f});
        }
        double a = DoorAngle(door);
        double mag = a < 0.0 ? -a : a;
        maxAngle = mag > maxAngle ? mag : maxAngle;
    }
    CHECK(maxAngle < 0.83, "the stops hold against a hard shove both ways");
    CHECK(maxAngle > 0.70, "the door actually reached the stops");
    m3DestroyWorld(world);
}

static void TestDoorMotor(void)
{
    // A strong motor reaches its commanded speed; a feeble motor
    // against the same door proves the torque cap (it cannot).
    for (int32_t variant = 0; variant < 2; ++variant)
    {
        m3WorldDef def = m3DefaultWorldDef();
        def.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
        def.bodyCapacity = 8;
        def.shapeCapacity = 8;
        m3WorldId world = m3CreateWorld(&def);
        m3JointDef jd;
        m3BodyId door = MakeDoor(world, &jd);
        jd.enableMotor = true;
        jd.motorSpeed = 2.0f;
        jd.maxMotorTorque = variant == 0 ? 50.0f : 0.02f;
        CHECK(m3Joint_IsValid(m3CreateJoint(&jd)), "the motored hinge creates");

        for (int32_t i = 0; i < 120; ++i)
        {
            m3World_Step(world, 1.0f / 60.0f, 4);
        }
        m3Vec3 w = m3Body_GetAngularVelocity(door);
        if (variant == 0)
        {
            CHECK(w.y > 1.9f && w.y < 2.1f, "the strong motor reaches its speed");
        }
        else
        {
            CHECK(w.y < 1.0f, "the feeble motor is honestly torque-capped");
        }
        m3DestroyWorld(world);
    }
}

int main(void)
{
    TestPendulum();
    TestChainSettlesAndSleeps();
    TestJointDeterminismSpine();
    TestJointContracts();
    TestDoorSwings();
    TestDoorLimits();
    TestDoorMotor();
    if (s_failures == 0)
    {
        printf("test_joints: all checks passed\n");
    }
    return s_failures == 0 ? 0 : 1;
}
