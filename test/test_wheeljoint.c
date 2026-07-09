// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The wheel joint gate (12-2): the OPTIONAL rigid-wheel path. A
// jointed cart drives on real contacts (no rays anywhere), rolls a
// rubble field without sinking through anything, twins to the bit,
// snaps a capped axle deterministically and plows on three wheels,
// replays its whole session from the journal, and the hostile wall
// refuses geometry that is not a wheel. The raycast vehicle is
// untouched by all of it.

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

static m3WorldId PlaneWorld(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 64;
    def.shapeCapacity = 64;
    def.jointCapacity = 16;
    m3WorldId world = m3CreateWorld(&def);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    m3CreatePlaneShape(ground, &sd, &floor);
    return world;
}

#define CART_WHEELS 4

typedef struct JointCart
{
    m3BodyId chassis;
    m3BodyId wheels[CART_WHEELS];
    m3JointId joints[CART_WHEELS];
} JointCart;

// A contact-wheeled cart: a 300 kg box chassis and four sphere
// wheels on wheel joints. Suspension is the joint's 8-6b spring
// plus travel limits; drive is the joint's spin motor. Wheels are
// bodies: everything below them is real contact, never a ray.
static JointCart MakeJointCart(m3WorldId world, m3Pos3 at)
{
    JointCart cart;
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = at;
    cart.chassis = m3CreateBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.density = 300.0f;
    m3CreateBoxShape(cart.chassis, &sd, (m3Vec3){1.0f, 0.25f, 0.5f});
    for (int32_t w = 0; w < CART_WHEELS; ++w)
    {
        m3Vec3 local = {(w & 1) != 0 ? 0.8f : -0.8f, -0.35f, (w & 2) != 0 ? 0.45f : -0.45f};
        m3BodyDef wd = m3DefaultBodyDef();
        wd.type = m3_dynamicBody;
        wd.position =
            (m3Pos3){at.x + (double)local.x, at.y + (double)local.y, at.z + (double)local.z};
        cart.wheels[w] = m3CreateBody(world, &wd);
        m3ShapeDef ws = m3DefaultShapeDef();
        ws.density = 120.0f; // ~13.6 kg per wheel: real unsprung mass
        m3Sphere tire = {{0.0f, 0.0f, 0.0f}, 0.3f};
        m3CreateSphereShape(cart.wheels[w], &ws, &tire);
        m3JointDef jd = m3DefaultJointDef();
        jd.type = m3_wheelJoint;
        jd.bodyA = cart.chassis;
        jd.bodyB = cart.wheels[w];
        jd.localAnchorA = local;
        jd.localAnchorB = (m3Vec3){0.0f, 0.0f, 0.0f};
        jd.localAxisA = (m3Vec3){0.0f, -1.0f, 0.0f}; // strut: chassis down
        jd.localAxisB = (m3Vec3){0.0f, 0.0f, 1.0f};  // axle: wheel z
        jd.enableLimit = true;
        jd.lowerLimit = -0.1f;
        jd.upperLimit = 0.1f;
        cart.joints[w] = m3CreateJoint(&jd);
        m3Joint_SetSpring(cart.joints[w], true, 4.0f, 0.7f);
        m3Joint_SetTargetTranslation(cart.joints[w], 0.0f);
    }
    return cart;
}

static void DriveCart(const JointCart* cart, float speed, float torque)
{
    for (int32_t w = 0; w < CART_WHEELS; ++w)
    {
        m3Joint_SetMotor(cart->joints[w], true, speed, torque);
    }
}

static void TestCartRollsOnFlat(void)
{
    // Settle, drive, arrive: the motorized spin becomes forward
    // travel through nothing but tire contact friction. The
    // suspension carries the chassis inside its travel books.
    m3WorldId world = PlaneWorld();
    JointCart cart = MakeJointCart(world, (m3Pos3){0.0, 0.66, 0.0});
    for (int32_t i = 0; i < 180; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    for (int32_t w = 0; w < CART_WHEELS; ++w)
    {
        m3real t = m3Joint_GetTranslation(cart.joints[w]);
        CHECK(t > -0.11f && t < 0.11f, "the settled suspension sits inside its travel");
    }
    double startX = m3Body_GetPosition(cart.chassis).x;
    DriveCart(&cart, -20.0f, 60.0f); // negative spin about +z rolls +x
    for (int32_t i = 0; i < 300; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    double gain = m3Body_GetPosition(cart.chassis).x - startX;
    CHECK(gain > 2.0, "the motored cart drives forward on contact friction");
    m3Vec3 spin = m3Body_GetAngularVelocity(cart.wheels[0]);
    CHECK(spin.z < -5.0f, "the wheels genuinely spin about their axles");
    m3DestroyWorld(world);
}

static void TestRubbleFieldTwins(void)
{
    // The plan's promise: a joint-wheeled cart crosses a rubble
    // field without ray tunneling artifacts, because there are no
    // rays: every wheel is a body and every strike is a contact.
    // The honest assertion is that no wheel center ever sinks below
    // its radius, and that twin runs land on one hash.
    uint64_t hashes[2];
    for (int32_t run = 0; run < 2; ++run)
    {
        m3WorldId world = PlaneWorld();
        // Deterministic rubble: a low wall of loose bricks across
        // the cart's path, plus scattered chunks behind it.
        m3ShapeDef sd = m3DefaultShapeDef();
        sd.density = 400.0f;
        for (int32_t i = 0; i < 12; ++i)
        {
            m3BodyDef bd = m3DefaultBodyDef();
            bd.type = m3_dynamicBody;
            double col = (double)(i % 4);
            double row = (double)(i / 4);
            bd.position = (m3Pos3){3.0 + row * 0.8, 0.14 + row * 0.02, -0.9 + col * 0.6};
            m3BodyId brick = m3CreateBody(world, &bd);
            m3CreateBoxShape(brick, &sd, (m3Vec3){0.22f, 0.12f, 0.22f});
        }
        JointCart cart = MakeJointCart(world, (m3Pos3){0.0, 0.66, 0.0});
        for (int32_t i = 0; i < 120; ++i)
        {
            m3World_Step(world, 1.0f / 60.0f, 4);
        }
        DriveCart(&cart, -25.0f, 90.0f);
        bool sank = false;
        for (int32_t i = 0; i < 600; ++i)
        {
            m3World_Step(world, 1.0f / 60.0f, 4);
            for (int32_t w = 0; w < CART_WHEELS; ++w)
            {
                if (m3Body_GetPosition(cart.wheels[w]).y < 0.24)
                {
                    sank = true; // radius 0.3 minus honest contact slop
                }
            }
        }
        CHECK(!sank, "no wheel ever sinks through the world");
        CHECK(m3Body_GetPosition(cart.chassis).x > 3.0, "the cart genuinely crosses the rubble");
        hashes[run] = m3World_Hash(world);
        m3DestroyWorld(world);
    }
    CHECK(hashes[0] == hashes[1], "twin rubble runs are bit-identical");
}

static void TestBrokenAxleDeterministic(void)
{
    // The 8-6 break contract on an axle: cap the front-left joint's
    // torque, overdrive it, and the axle snaps in-step. The wheel
    // rolls away, the cart plows on three, and a rollback through
    // the break re-runs to the same bits.
    static uint8_t snap[2097152];
    uint64_t hashes[2];
    for (int32_t run = 0; run < 2; ++run)
    {
        m3WorldId world = PlaneWorld();
        JointCart cart = MakeJointCart(world, (m3Pos3){0.0, 0.66, 0.0});
        for (int32_t i = 0; i < 120; ++i)
        {
            m3World_Step(world, 1.0f / 60.0f, 4);
        }
        m3Joint_SetBreakThresholds(cart.joints[1], 0.0f, 25.0f);
        DriveCart(&cart, -30.0f, 90.0f);
        int32_t snapBytes = 0;
        int32_t breakStep = -1;
        for (int32_t i = 0; i < 480; ++i)
        {
            if (i == 10 && run == 0)
            {
                snapBytes = m3World_Snapshot(world, snap, (int32_t)sizeof(snap));
                CHECK(snapBytes > 0, "the pre-break snapshot fits");
            }
            m3World_Step(world, 1.0f / 60.0f, 4);
            if (breakStep < 0 && !m3Joint_IsValid(cart.joints[1]))
            {
                breakStep = i;
                int32_t count = 0;
                m3World_JointBreakEvents(world, &count);
                CHECK(count > 0, "the break emits its event");
            }
        }
        CHECK(breakStep >= 0, "the capped axle snaps under overdrive");
        CHECK(m3Joint_IsValid(cart.joints[0]), "the uncapped axles hold");
        double dx = m3Body_GetPosition(cart.wheels[1]).x - m3Body_GetPosition(cart.chassis).x;
        double dz = m3Body_GetPosition(cart.wheels[1]).z - m3Body_GetPosition(cart.chassis).z;
        CHECK(dx * dx + dz * dz > 1.0, "the freed wheel leaves the cart");
        hashes[run] = m3World_Hash(world);
        if (run == 0)
        {
            CHECK(m3World_Restore(world, snap, snapBytes), "the pre-break restore lands");
            for (int32_t i = 10; i < 480; ++i)
            {
                m3World_Step(world, 1.0f / 60.0f, 4);
            }
            CHECK(m3World_Hash(world) == hashes[0], "the re-run breaks on the same bits");
        }
        m3DestroyWorld(world);
    }
    CHECK(hashes[0] == hashes[1], "twin break runs are bit-identical");
}

static void TestJournalReplay(void)
{
    // The whole cart life journals: creates, springs, limits,
    // motors, a break cap, the drive. Replay lands on the final
    // hash with zero divergence.
    static uint8_t journal[131072];
    m3WorldId world = PlaneWorld();
    CHECK(m3World_JournalBegin(world, journal, (int32_t)sizeof(journal)), "the journal opens");
    JointCart cart = MakeJointCart(world, (m3Pos3){0.0, 0.66, 0.0});
    m3Joint_SetBreakThresholds(cart.joints[2], 0.0f, 25.0f);
    for (int32_t i = 0; i < 360; ++i)
    {
        if (i == 60)
        {
            DriveCart(&cart, -30.0f, 90.0f);
        }
        if (i == 200)
        {
            m3Joint_SetMotor(cart.joints[0], false, 0.0f, 0.0f); // one dead motor
        }
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    uint64_t final = m3World_Hash(world);
    int32_t bytes = m3World_JournalEnd(world);
    CHECK(bytes > 0, "the cart session records");
    m3WorldId replayed = PlaneWorld();
    CHECK(m3World_JournalReplay(replayed, journal, bytes), "the cart session replays");
    CHECK(m3World_Hash(replayed) == final, "the replayed cart is bit-identical");
    m3DestroyWorld(replayed);
    m3DestroyWorld(world);
}

static void TestHostileWall(void)
{
    // Geometry that is not a wheel refuses loudly at create; type
    // gates on the read and drive APIs hold; a small honest skew is
    // snapped and accepted.
    m3WorldId world = PlaneWorld();
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 1.0, 0.0};
    m3BodyId a = m3CreateBody(world, &bd);
    bd.position = (m3Pos3){0.0, 0.4, 0.0};
    m3BodyId b = m3CreateBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.density = 100.0f;
    m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.2f};
    m3CreateSphereShape(a, &sd, &ball);
    m3CreateSphereShape(b, &sd, &ball);

    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_wheelJoint;
    jd.bodyA = a;
    jd.bodyB = b;
    jd.localAxisA = (m3Vec3){0.0f, -1.0f, 0.0f};
    jd.localAxisB = (m3Vec3){0.0f, -1.0f, 0.0f}; // axle along the strut
    CHECK(!m3Joint_IsValid(m3CreateJoint(&jd)), "an axle along the strut refuses");

    jd.localAxisB = (m3Vec3){0.0f, 0.0f, 0.0f};
    CHECK(!m3Joint_IsValid(m3CreateJoint(&jd)), "a zero axle refuses");

    jd.localAxisB = (m3Vec3){0.0f, 0.05f, 1.0f}; // ~2.9 degrees of skew
    m3JointId wheel = m3CreateJoint(&jd);
    CHECK(m3Joint_IsValid(wheel), "a small skew snaps and lands");
    CHECK(m3Joint_GetAngle(wheel) == 0.0f, "the spin angle starts at the create pose");

    m3Joint_SetTargetAngle(wheel, 1.0f); // the wheel's drive is its
                                         // suspension: angle targets
                                         // stay revolute-only
    m3Joint_SetSpring(wheel, true, 3.0f, 0.5f);
    m3Joint_SetTargetTranslation(wheel, 0.05f);
    for (int32_t i = 0; i < 60; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m3Joint_IsValid(wheel), "the wheel survives its first minute");

    // Regression: the older type gates stay closed to the wheel's
    // readers and vice versa.
    m3JointDef rd = m3DefaultJointDef();
    rd.type = m3_revoluteJoint;
    rd.bodyA = a;
    rd.bodyB = b;
    rd.localAxisA = (m3Vec3){0.0f, 0.0f, 1.0f};
    rd.localAxisB = (m3Vec3){0.0f, 0.0f, 1.0f};
    m3JointId hinge = m3CreateJoint(&rd);
    CHECK(m3Joint_GetTranslation(hinge) == 0.0f, "a hinge has no translation to read");
    m3DestroyWorld(world);
}

int main(void)
{
    TestCartRollsOnFlat();
    TestRubbleFieldTwins();
    TestBrokenAxleDeterministic();
    TestJournalReplay();
    TestHostileWall();
    if (s_failures == 0)
    {
        printf("test_wheeljoint: all passed\n");
        return 0;
    }
    return 1;
}
