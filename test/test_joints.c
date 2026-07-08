// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Joint gate (2c-2): the spherical point constraint against analytic
// pendulum physics, a hanging chain, and the determinism spine
// (rollback, replay with minted joint ids, sleep coupling).

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

static double RandD(uint64_t* state, double lo, double hi)
{
    *state += 0x9E3779B97F4A7C15ull;
    uint64_t z = *state;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    z = z ^ (z >> 31);
    return lo + (hi - lo) * ((double)(z >> 11) / 9007199254740992.0);
}

static m3Quat RandQuat(uint64_t* state)
{
    m3Quat q = {(m3real)RandD(state, -1.0, 1.0), (m3real)RandD(state, -1.0, 1.0),
                (m3real)RandD(state, -1.0, 1.0), (m3real)RandD(state, -1.0, 1.0)};
    return m3NormalizeQuat(q);
}

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
    jd.lowerLimit = -0.7854f;
    jd.upperLimit = 0.7854f;
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
        jd.maxMotorEffort = variant == 0 ? 50.0f : 0.02f;
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

static void TestPrismaticJacobianFiniteDifference(void)
{
    // The plan's hard requirement: the reference flags its prismatic
    // Jacobian simplification as untested, so the FULL form we ship
    // is held to the numerics here. For random poses and velocities:
    // the analytic Cdot rows (axis and both perps, with the
    // cross(rA + d, axis) arms) must match (C(x + h v) - C(x)) / h.
    uint64_t state = 0xA5A5A5A5ull;
    int32_t checked = 0;
    for (int32_t round = 0; round < 20; ++round)
    {
        // Random poses near the origin, random anchors, random axis.
        m3Vec3 pA = {(m3real)RandD(&state, -1.0, 1.0), (m3real)RandD(&state, -1.0, 1.0),
                     (m3real)RandD(&state, -1.0, 1.0)};
        m3Vec3 pB = {(m3real)RandD(&state, -1.0, 1.0), (m3real)RandD(&state, -1.0, 1.0),
                     (m3real)RandD(&state, -1.0, 1.0)};
        m3Quat qA = RandQuat(&state);
        m3Quat qB = RandQuat(&state);
        m3Vec3 laA = {(m3real)RandD(&state, -0.5, 0.5), (m3real)RandD(&state, -0.5, 0.5),
                      (m3real)RandD(&state, -0.5, 0.5)};
        m3Vec3 laB = {(m3real)RandD(&state, -0.5, 0.5), (m3real)RandD(&state, -0.5, 0.5),
                      (m3real)RandD(&state, -0.5, 0.5)};
        m3Vec3 axisLocal = m3Normalize3((m3Vec3){(m3real)RandD(&state, -1.0, 1.0),
                                                 (m3real)RandD(&state, -1.0, 1.0),
                                                 (m3real)RandD(&state, -1.0, 1.0)});
        m3Vec3 vA = {(m3real)RandD(&state, -2.0, 2.0), (m3real)RandD(&state, -2.0, 2.0),
                     (m3real)RandD(&state, -2.0, 2.0)};
        m3Vec3 vB = {(m3real)RandD(&state, -2.0, 2.0), (m3real)RandD(&state, -2.0, 2.0),
                     (m3real)RandD(&state, -2.0, 2.0)};
        m3Vec3 wA = {(m3real)RandD(&state, -2.0, 2.0), (m3real)RandD(&state, -2.0, 2.0),
                     (m3real)RandD(&state, -2.0, 2.0)};
        m3Vec3 wB = {(m3real)RandD(&state, -2.0, 2.0), (m3real)RandD(&state, -2.0, 2.0),
                     (m3real)RandD(&state, -2.0, 2.0)};

        // C rows as a function of the pose.
        // rA = rot(qA, laA), rB = rot(qB, laB),
        // d = pB + rB - pA - rA, axis/perps fixed in A.
        // Any perpendicular pair does for the numerics; the test is
        // self-contained math and needs no engine internals.
        m3Vec3 up = axisLocal.x * axisLocal.x < 0.9f ? (m3Vec3){1.0f, 0.0f, 0.0f}
                                                     : (m3Vec3){0.0f, 1.0f, 0.0f};
        m3Vec3 t1 = m3Normalize3(m3Cross3(axisLocal, up));
        m3Vec3 t2 = m3Cross3(axisLocal, t1);

        // Analytic Cdot with the FULL arms.
        m3Vec3 rA = m3RotateVec3(qA, laA);
        m3Vec3 rB = m3RotateVec3(qB, laB);
        m3Vec3 d = m3Sub3(m3Add3(pB, rB), m3Add3(pA, rA));
        m3Vec3 axis = m3RotateVec3(qA, axisLocal);
        m3Vec3 pY = m3RotateVec3(qA, t1);
        m3Vec3 pZ = m3RotateVec3(qA, t2);
        m3Vec3 vRel = m3Sub3(m3Sub3(m3Add3(vB, m3Cross3(wB, rB)), vA), m3Cross3(wA, m3Add3(rA, d)));
        double cdot[3] = {(double)m3Dot3(vRel, axis), (double)m3Dot3(vRel, pY),
                          (double)m3Dot3(vRel, pZ)};

        // Numeric Cdot by CENTRAL difference: the one-sided form
        // drowns in float cancellation; central kills the O(h) term
        // and h = 1e-3 keeps the round-off noise two decades down.
        const double h = 1.0e-3;
        m3Vec3 pAp = m3Add3(pA, m3MulSV3((m3real)h, vA));
        m3Vec3 pBp = m3Add3(pB, m3MulSV3((m3real)h, vB));
        m3Quat qAp = m3IntegrateRotation(qA, m3MulSV3((m3real)h, wA));
        m3Quat qBp = m3IntegrateRotation(qB, m3MulSV3((m3real)h, wB));
        m3Vec3 pAm = m3Sub3(pA, m3MulSV3((m3real)h, vA));
        m3Vec3 pBm = m3Sub3(pB, m3MulSV3((m3real)h, vB));
        m3Quat qAm = m3IntegrateRotation(qA, m3MulSV3(-(m3real)h, wA));
        m3Quat qBm = m3IntegrateRotation(qB, m3MulSV3(-(m3real)h, wB));

        m3Vec3 rAp = m3RotateVec3(qAp, laA);
        m3Vec3 rBp = m3RotateVec3(qBp, laB);
        m3Vec3 dp2 = m3Sub3(m3Add3(pBp, rBp), m3Add3(pAp, rAp));
        m3Vec3 axP = m3RotateVec3(qAp, axisLocal);
        m3Vec3 pYp = m3RotateVec3(qAp, t1);
        m3Vec3 pZp = m3RotateVec3(qAp, t2);
        m3Vec3 rAm = m3RotateVec3(qAm, laA);
        m3Vec3 rBm = m3RotateVec3(qBm, laB);
        m3Vec3 dm2 = m3Sub3(m3Add3(pBm, rBm), m3Add3(pAm, rAm));
        m3Vec3 axM = m3RotateVec3(qAm, axisLocal);
        m3Vec3 pYm = m3RotateVec3(qAm, t1);
        m3Vec3 pZm = m3RotateVec3(qAm, t2);
        double cp[3] = {(double)m3Dot3(dp2, axP), (double)m3Dot3(dp2, pYp),
                        (double)m3Dot3(dp2, pZp)};
        double cm[3] = {(double)m3Dot3(dm2, axM), (double)m3Dot3(dm2, pYm),
                        (double)m3Dot3(dm2, pZm)};
        for (int32_t k = 0; k < 3; ++k)
        {
            double fd = (cp[k] - cm[k]) / (2.0 * h);
            double err = fd - cdot[k];
            err = err < 0.0 ? -err : err;
            double scale = 1.0;
            double mag = cdot[k] < 0.0 ? -cdot[k] : cdot[k];
            scale = mag > 1.0 ? mag : 1.0;
            CHECK(err / scale < 5.0e-3, "the full prismatic Jacobian matches the numerics");
            checked += 1;
        }
    }
    CHECK(checked == 60, "all sixty rows were checked");
}

static void TestElevator(void)
{
    // A motored vertical slider lifts a heavy platform to its upper
    // stop and holds it there against gravity: the elevator.
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m3WorldId world = m3CreateWorld(&def);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId shaft = m3CreateBody(world, &gd);

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 0.5, 0.0};
    m3BodyId car = m3CreateBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3CreateBoxShape(car, &sd, (m3Vec3){0.5f, 0.1f, 0.5f});

    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_prismaticJoint;
    jd.bodyA = shaft;
    jd.bodyB = car;
    jd.localAnchorA = (m3Vec3){0.0f, 0.5f, 0.0f}; // the rail zero
    jd.localAnchorB = (m3Vec3){0.0f, 0.0f, 0.0f};
    jd.localAxisA = (m3Vec3){0.0f, 1.0f, 0.0f};
    jd.localAxisB = (m3Vec3){0.0f, 1.0f, 0.0f};
    jd.enableLimit = true;
    jd.lowerLimit = 0.0f;
    jd.upperLimit = 2.0f;
    jd.enableMotor = true;
    jd.motorSpeed = 1.0f;
    jd.maxMotorEffort = 200.0f; // plenty against gravity
    CHECK(m3Joint_IsValid(m3CreateJoint(&jd)), "the elevator joint creates");

    StepN(world, 300); // five seconds: 2 m of travel plus the hold
    m3Pos3 p = m3Body_GetPosition(car);
    CHECK(p.y > 2.40 && p.y < 2.60, "the car rides to the upper stop and holds");
    CHECK(p.x > -0.01 && p.x < 0.01 && p.z > -0.01 && p.z < 0.01, "the rail holds sideways");
    m3Quat q = m3Body_GetRotation(car);
    CHECK(q.w > 0.999f, "the rotation lock holds the car level");

    // Cut the motor by destroying and recreating without it: simpler,
    // command the motor downward with a feeble force: gravity wins.
    m3DestroyWorld(world);
}

static void TestSliderGravityAlongRail(void)
{
    // A frictionless slider tilted 30 degrees: the car accelerates
    // along the rail at g sin(theta), the classic inclined plane
    // WITHOUT contact (the joint IS the plane). Analytic check.
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m3WorldId world = m3CreateWorld(&def);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId rail = m3CreateBody(world, &gd);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    m3BodyId car = m3CreateBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.2f};
    m3CreateSphereShape(car, &sd, &ball);

    // Axis 30 degrees off horizontal in the xz-free xy plane.
    m3Vec3 axis = {0.8660254f, -0.5f, 0.0f};
    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_prismaticJoint;
    jd.bodyA = rail;
    jd.bodyB = car;
    jd.localAxisA = axis;
    jd.localAxisB = axis;
    CHECK(m3Joint_IsValid(m3CreateJoint(&jd)), "the tilted slider creates");

    StepN(world, 60); // one second
    m3Vec3 v = m3Body_GetLinearVelocity(car);
    // v = g sin(30) t along the axis = 5 m/s; components (4.33, -2.5).
    m3real speed = sqrtf(m3Dot3(v, v));
    CHECK(speed > 4.7f && speed < 5.3f, "the car obeys g sin(theta) on the rail");
    CHECK(v.x > 4.0f && v.y < -2.2f, "the velocity points down the rail");
    m3DestroyWorld(world);
}

static double TwistOf(m3Quat q)
{
    double tw =
        (double)q.w < 0.0 ? atan2(-(double)q.z, -(double)q.w) : atan2((double)q.z, (double)q.w);
    return 2.0 * tw;
}

static double SwingOf(m3Quat q)
{
    double x = sqrt((double)q.z * (double)q.z + (double)q.w * (double)q.w);
    double y = sqrt((double)q.x * (double)q.x + (double)q.y * (double)q.y);
    return 2.0 * atan2(y, x);
}

static void TestTwistJacobianFiniteDifference(void)
{
    // The second flagged Jacobian: the reference's twist row is
    // coneAxis + tan(theta/2) * perp and carries the author's own
    // verify-me todo. Claim: d(twist)/dt = dot(wB - wA, J_twist),
    // and d(swing)/dt = dot(wB - wA, swingAxis). Both held to
    // central differences over random moderate poses (swing kept
    // under two radians, away from the antipodal singularity).
    uint64_t state = 0x5EED5EEDull;
    int32_t checked = 0;
    for (int32_t round = 0; round < 20; ++round)
    {
        m3Quat qA = RandQuat(&state);
        // qB = qA composed with a bounded random rotation.
        m3Vec3 rot = {(m3real)RandD(&state, -0.8, 0.8), (m3real)RandD(&state, -0.8, 0.8),
                      (m3real)RandD(&state, -0.8, 0.8)};
        m3Quat qB = m3IntegrateRotation(qA, rot);
        m3Vec3 wA = {(m3real)RandD(&state, -2.0, 2.0), (m3real)RandD(&state, -2.0, 2.0),
                     (m3real)RandD(&state, -2.0, 2.0)};
        m3Vec3 wB = {(m3real)RandD(&state, -2.0, 2.0), (m3real)RandD(&state, -2.0, 2.0),
                     (m3real)RandD(&state, -2.0, 2.0)};

        m3Vec3 coneAxis = m3RotateVec3(qA, (m3Vec3){0.0f, 0.0f, 1.0f});
        m3Vec3 twistAxis = m3RotateVec3(qB, (m3Vec3){0.0f, 0.0f, 1.0f});
        m3Vec3 swing = m3Cross3(coneAxis, twistAxis);
        m3real len2 = m3Dot3(swing, swing);
        if (len2 < 1.0e-6f)
        {
            continue; // aligned pose: the axis is arbitrary, skip
        }
        swing = m3MulSV3(1.0f / sqrtf(len2), swing);
        m3Quat conjA = {-qA.x, -qA.y, -qA.z, qA.w};
        m3Quat relQ = m3NormalizeQuat(m3MulQuat(conjA, qB));
        m3real denom = relQ.z * relQ.z + relQ.w * relQ.w;
        if (denom < 1.0e-6f)
        {
            continue; // near the swing singularity
        }
        m3real tanHalf = sqrtf((relQ.x * relQ.x + relQ.y * relQ.y) / denom);
        m3Vec3 perp = m3Cross3(swing, coneAxis);
        m3Vec3 twistJac = m3Add3(coneAxis, m3MulSV3(tanHalf, perp));

        double analyticTwist = (double)m3Dot3(m3Sub3(wB, wA), twistJac);
        double analyticSwing = (double)m3Dot3(m3Sub3(wB, wA), swing);

        const double h = 1.0e-3;
        m3Quat qAp = m3IntegrateRotation(qA, m3MulSV3((m3real)h, wA));
        m3Quat qBp = m3IntegrateRotation(qB, m3MulSV3((m3real)h, wB));
        m3Quat qAm = m3IntegrateRotation(qA, m3MulSV3(-(m3real)h, wA));
        m3Quat qBm = m3IntegrateRotation(qB, m3MulSV3(-(m3real)h, wB));
        m3Quat cAp = {-qAp.x, -qAp.y, -qAp.z, qAp.w};
        m3Quat cAm = {-qAm.x, -qAm.y, -qAm.z, qAm.w};
        m3Quat relP = m3NormalizeQuat(m3MulQuat(cAp, qBp));
        m3Quat relM = m3NormalizeQuat(m3MulQuat(cAm, qBm));

        double fdTwist = (TwistOf(relP) - TwistOf(relM)) / (2.0 * h);
        double fdSwing = (SwingOf(relP) - SwingOf(relM)) / (2.0 * h);

        double scaleT = analyticTwist < 0.0 ? -analyticTwist : analyticTwist;
        scaleT = scaleT > 1.0 ? scaleT : 1.0;
        double errT = fdTwist - analyticTwist;
        errT = errT < 0.0 ? -errT : errT;
        CHECK(errT / scaleT < 1.0e-2, "the flagged twist Jacobian matches the numerics");

        double scaleS = analyticSwing < 0.0 ? -analyticSwing : analyticSwing;
        scaleS = scaleS > 1.0 ? scaleS : 1.0;
        double errS = fdSwing - analyticSwing;
        errS = errS < 0.0 ? -errS : errS;
        CHECK(errS / scaleS < 1.0e-2, "the swing axis row matches the numerics");
        checked += 1;
    }
    CHECK(checked >= 15, "enough non-degenerate poses were checked");
}

static void TestShoulderCone(void)
{
    // An arm on a spherical joint with a 30-degree cone: dropped from
    // horizontal (swing 90 degrees), it must be caught by the cone
    // and settle hanging inside it, never outside the band.
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m3WorldId world = m3CreateWorld(&def);
    m3BodyDef ad = m3DefaultBodyDef();
    // The socket frame points its z DOWN so the cone opens downward.
    m3BodyId socket = m3CreateBody(world, &ad);

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.6, 0.0, 0.0}; // arm sticks out horizontal
    bd.linearDamping = 1.0f;
    bd.angularDamping = 1.0f;
    m3BodyId arm = m3CreateBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Capsule capsule = {{-0.5f, 0.0f, 0.0f}, {0.3f, 0.0f, 0.0f}, 0.1f};
    m3CreateCapsuleShape(arm, &sd, &capsule);

    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_sphericalJoint;
    jd.bodyA = socket;
    jd.bodyB = arm;
    jd.localAnchorA = (m3Vec3){0.0f, 0.0f, 0.0f};
    jd.localAnchorB = (m3Vec3){-0.6f, 0.0f, 0.0f};
    // The cone axis points along +x, HORIZONTAL: gravity wants the
    // arm hanging straight down (swing 90 degrees), the cone stops
    // the droop at 30 degrees below the horizon. Pointing the cone
    // downward would make hanging the cone CENTER and catch nothing
    // (the first version of this test learned that the hard way).
    jd.localAxisA = (m3Vec3){1.0f, 0.0f, 0.0f};
    jd.localAxisB = (m3Vec3){1.0f, 0.0f, 0.0f}; // the arm's long axis
    jd.enableCone = true;
    jd.coneAngle = 0.5236f; // 30 degrees
    CHECK(m3Joint_IsValid(m3CreateJoint(&jd)), "the shoulder creates");

    StepN(world, 480);
    m3Pos3 p = m3Body_GetPosition(arm);
    double len = sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    CHECK(len > 0.55 && len < 0.65, "the arm still hangs from the socket");
    double droop = -p.y / len; // sine of the angle below the horizon
    CHECK(droop > 0.44 && droop < 0.56, "the cone catches the droop at thirty degrees");
    CHECK(p.x > 0.4, "the arm stays out along the cone axis");
    m3DestroyWorld(world);
}

static void TestTwistClamp(void)
{
    // A spinning body on a twist-limited spherical joint: the twist
    // angle must stay inside the band while the spin bleeds into the
    // stops (gravity off, aligned frames, pure twist geometry).
    m3WorldDef def = m3DefaultWorldDef();
    def.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m3WorldId world = m3CreateWorld(&def);
    m3BodyDef ad = m3DefaultBodyDef();
    m3BodyId anchor = m3CreateBody(world, &ad);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 0.0, 0.0};
    m3BodyId spinner = m3CreateBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3CreateBoxShape(spinner, &sd, (m3Vec3){0.4f, 0.1f, 0.4f});

    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_sphericalJoint;
    jd.bodyA = anchor;
    jd.bodyB = spinner;
    jd.localAxisA = (m3Vec3){0.0f, 0.0f, 1.0f};
    jd.localAxisB = (m3Vec3){0.0f, 0.0f, 1.0f};
    jd.enableLimit = true;
    jd.lowerLimit = -0.5f;
    jd.upperLimit = 0.5f;
    CHECK(m3Joint_IsValid(m3CreateJoint(&jd)), "the twist-limited joint creates");
    m3Body_SetAngularVelocity(spinner, (m3Vec3){0.0f, 0.0f, 6.0f});

    double maxTwist = 0.0;
    for (int32_t i = 0; i < 240; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
        if (i == 120)
        {
            m3Body_SetAngularVelocity(spinner, (m3Vec3){0.0f, 0.0f, -6.0f});
        }
        m3Quat q = m3Body_GetRotation(spinner);
        double tw = TwistOf(q);
        double mag = tw < 0.0 ? -tw : tw;
        maxTwist = mag > maxTwist ? mag : maxTwist;
    }
    CHECK(maxTwist < 0.55, "the twist stops hold from both directions");
    CHECK(maxTwist > 0.42, "the stops were actually reached");
    m3DestroyWorld(world);
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
    TestPrismaticJacobianFiniteDifference();
    TestElevator();
    TestSliderGravityAlongRail();
    TestTwistJacobianFiniteDifference();
    TestShoulderCone();
    TestTwistClamp();
    if (s_failures == 0)
    {
        printf("test_joints: all checks passed\n");
    }
    return s_failures == 0 ? 0 : 1;
}
