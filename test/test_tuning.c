// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The tuning gate (8-4): runtime materials bind next step, density
// rebuilds mass on request, world knobs change behavior in the
// direction physics demands, equal-knob worlds twin bit-exact,
// differing knobs diverge detectably, and every op replays.

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

static m3WorldId PlaneWorld(m3ShapeId* outFloor)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 32;
    def.shapeCapacity = 32;
    m3WorldId world = m3CreateWorld(&def);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    m3ShapeId floorShape = m3CreatePlaneShape(ground, &sd, &floor);
    if (outFloor != NULL)
    {
        *outFloor = floorShape;
    }
    return world;
}

static m3BodyId Crate(m3WorldId world, m3Pos3 at, m3ShapeId* outShape)
{
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = at;
    m3BodyId body = m3CreateBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3ShapeId shape = m3CreateBoxShape(body, &sd, (m3Vec3){0.5f, 0.5f, 0.5f});
    if (outShape != NULL)
    {
        *outShape = shape;
    }
    return body;
}

static void StepN(m3WorldId world, int32_t n)
{
    for (int32_t i = 0; i < n; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
}

static void TestFrictionSetterBinds(void)
{
    // The same shove on the same settled crate: after the floor
    // turns frictionless at runtime, the crate keeps its speed.
    double vx[2];
    for (int32_t pass = 0; pass < 2; ++pass)
    {
        m3ShapeId floor;
        m3WorldId world = PlaneWorld(&floor);
        m3ShapeId crateShape;
        m3BodyId crate = Crate(world, (m3Pos3){0.0, 0.5, 0.0}, &crateShape);
        StepN(world, 60);
        if (pass == 1)
        {
            m3Shape_SetFriction(floor, 0.0f);
            m3Shape_SetFriction(crateShape, 0.0f);
            CHECK(m3Shape_GetFriction(floor) == 0.0f, "friction reads back");
        }
        m3Body_ApplyLinearImpulse(crate, (m3Vec3){2.0f, 0.0f, 0.0f});
        StepN(world, 60);
        vx[pass] = (double)m3Body_GetLinearVelocity(crate).x;
        m3DestroyWorld(world);
    }
    CHECK(vx[0] < 0.1, "default friction stops the shove");
    CHECK(vx[1] > 1.9, "runtime-zeroed friction keeps it sliding");
}

static void TestRestitutionAndThreshold(void)
{
    // A dead ball turns bouncy at runtime; then a raised world
    // threshold kills the bounce again.
    for (int32_t mode = 0; mode < 3; ++mode)
    {
        m3WorldId world = PlaneWorld(NULL);
        m3BodyDef bd = m3DefaultBodyDef();
        bd.type = m3_dynamicBody;
        bd.position = (m3Pos3){0.0, 2.0, 0.0};
        m3BodyId ball = m3CreateBody(world, &bd);
        m3ShapeDef sd = m3DefaultShapeDef();
        m3Sphere s = {{0.0f, 0.0f, 0.0f}, 0.5f};
        m3ShapeId shape = m3CreateSphereShape(ball, &sd, &s);
        if (mode >= 1)
        {
            m3Shape_SetRestitution(shape, 0.8f);
        }
        if (mode == 2)
        {
            m3World_SetRestitutionThreshold(world, 100.0f);
        }
        double peak = 0.0;
        bool falling = true;
        for (int32_t i = 0; i < 180; ++i)
        {
            m3World_Step(world, 1.0f / 60.0f, 4);
            double y = m3Body_GetPosition(ball).y;
            double vy = (double)m3Body_GetLinearVelocity(ball).y;
            if (falling && vy > 0.01)
            {
                falling = false;
            }
            if (!falling && y > peak)
            {
                peak = y;
            }
        }
        if (mode == 0)
        {
            CHECK(peak < 0.7, "the dead ball stays down");
        }
        else if (mode == 1)
        {
            CHECK(peak > 1.0, "the runtime-bouncy ball bounces");
        }
        else
        {
            CHECK(peak < 0.7, "the raised threshold kills the bounce");
        }
        m3DestroyWorld(world);
    }
}

static void TestDensityRebuildsMass(void)
{
    // F = ma with runtime density: after densifying 8x with a mass
    // rebuild, the same force accelerates an eighth as much.
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    def.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
    m3WorldId world = m3CreateWorld(&def);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.linearDamping = 0.0f;
    bd.angularDamping = 0.0f;
    m3BodyId body = m3CreateBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3ShapeId shape = m3CreateBoxShape(body, &sd, (m3Vec3){0.5f, 0.5f, 0.5f});

    m3Body_ApplyLinearImpulse(body, (m3Vec3){1.0f, 0.0f, 0.0f});
    float v1 = m3Body_GetLinearVelocity(body).x;
    m3Body_SetLinearVelocity(body, (m3Vec3){0.0f, 0.0f, 0.0f});

    m3Shape_SetDensity(shape, 8.0f, true);
    CHECK(fabsf(m3Shape_GetDensity(shape) - 8.0f) < 1.0e-6f, "density reads back");
    m3Body_ApplyLinearImpulse(body, (m3Vec3){1.0f, 0.0f, 0.0f});
    float v8 = m3Body_GetLinearVelocity(body).x;
    CHECK(fabsf(v1 - 8.0f * v8) < 1.0e-3f, "8x density means an eighth the kick");

    // Without the rebuild flag the books do not move.
    m3Shape_SetDensity(shape, 1.0f, false);
    m3Body_SetLinearVelocity(body, (m3Vec3){0.0f, 0.0f, 0.0f});
    m3Body_ApplyLinearImpulse(body, (m3Vec3){1.0f, 0.0f, 0.0f});
    float vStale = m3Body_GetLinearVelocity(body).x;
    CHECK(fabsf(vStale - v8) < 1.0e-6f, "no rebuild flag: mass books unchanged");
    m3DestroyWorld(world);
}

static void TestGravityAndSpeedCap(void)
{
    // Flip gravity: the crate lifts off. Cap the speed: it obeys.
    m3WorldId world = PlaneWorld(NULL);
    m3BodyId crate = Crate(world, (m3Pos3){0.0, 0.5, 0.0}, NULL);
    StepN(world, 30);
    m3World_SetGravity(world, (m3Vec3){0.0f, 20.0f, 0.0f});
    m3Vec3 g = m3World_GetGravity(world);
    CHECK(g.y == 20.0f, "gravity reads back");
    // Contract: gravity does not wake sleepers (the reference
    // behavior); the crate napped during the settle, so disturb it.
    m3Body_SetAwake(crate, true);
    StepN(world, 60);
    CHECK(m3Body_GetPosition(crate).y > 3.0, "flipped gravity lifts the crate");

    m3World_SetMaximumLinearSpeed(world, 5.0f);
    m3Body_ApplyLinearImpulse(crate, (m3Vec3){1000.0f, 0.0f, 0.0f});
    m3World_Step(world, 1.0f / 60.0f, 4);
    m3Vec3 v = m3Body_GetLinearVelocity(crate);
    double speed =
        sqrt((double)v.x * (double)v.x + (double)v.y * (double)v.y + (double)v.z * (double)v.z);
    CHECK(speed < 5.001, "the speed cap holds");
    m3DestroyWorld(world);
}

static void TestSleepAndContinuousToggles(void)
{
    // Sleeping off wakes the sleeper and keeps everyone up; on
    // again lets it drop back off. Continuous off tunnels the
    // bullet the continuous phase exists to stop.
    m3WorldId world = PlaneWorld(NULL);
    m3BodyId crate = Crate(world, (m3Pos3){0.0, 0.5, 0.0}, NULL);
    StepN(world, 240);
    CHECK(!m3Body_IsAwake(crate), "the crate sleeps under defaults");
    m3World_EnableSleeping(world, false);
    CHECK(!m3World_IsSleepingEnabled(world), "toggle reads back");
    CHECK(m3Body_IsAwake(crate), "sleeping off wakes the sleeper");
    StepN(world, 240);
    CHECK(m3Body_IsAwake(crate), "nobody sleeps while sleeping is off");
    m3World_EnableSleeping(world, true);
    StepN(world, 240);
    CHECK(!m3Body_IsAwake(crate), "sleeping on lets it drop off again");
    m3DestroyWorld(world);

    // The bullet-vs-wall proof, with the phase off then on.
    for (int32_t pass = 0; pass < 2; ++pass)
    {
        m3WorldDef def = m3DefaultWorldDef();
        def.bodyCapacity = 8;
        def.shapeCapacity = 8;
        def.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
        m3WorldId w = m3CreateWorld(&def);
        m3BodyDef wallDef = m3DefaultBodyDef();
        wallDef.position = (m3Pos3){5.0, 0.0, 0.0};
        m3BodyId wall = m3CreateBody(w, &wallDef);
        m3ShapeDef sd = m3DefaultShapeDef();
        m3CreateBoxShape(wall, &sd, (m3Vec3){0.05f, 2.0f, 2.0f});
        (void)wall;
        m3BodyDef bulletDef = m3DefaultBodyDef();
        bulletDef.type = m3_dynamicBody;
        bulletDef.isBullet = true;
        bulletDef.position = (m3Pos3){0.0, 0.0, 0.0};
        m3BodyId bullet = m3CreateBody(w, &bulletDef);
        m3Sphere s = {{0.0f, 0.0f, 0.0f}, 0.1f};
        m3CreateSphereShape(bullet, &sd, &s);
        if (pass == 1)
        {
            m3World_EnableContinuous(w, false);
            CHECK(!m3World_IsContinuousEnabled(w), "continuous toggle reads back");
        }
        m3Body_SetLinearVelocity(bullet, (m3Vec3){300.0f, 0.0f, 0.0f});
        StepN(w, 10);
        double x = m3Body_GetPosition(bullet).x;
        if (pass == 0)
        {
            CHECK(x < 5.0, "continuous on: the bullet stops at the wall");
        }
        else
        {
            CHECK(x > 6.0, "continuous off: the bullet tunnels (the phase earns its keep)");
        }
        m3DestroyWorld(w);
    }
}

static void TestKnobTwinsAndReplay(void)
{
    // Twin law: equal knob scripts twin bit-exact; a differing knob
    // diverges detectably. And the whole op family replays.
    static uint8_t journal[262144];
    uint64_t twinHash[2];
    for (int32_t run = 0; run < 2; ++run)
    {
        // The journal must open on the EMPTY world: a floor created
        // before JournalBegin does not exist in the replay target
        // (the first draft learned that contract the hard way).
        m3WorldDef def = m3DefaultWorldDef();
        def.bodyCapacity = 32;
        def.shapeCapacity = 32;
        m3WorldId world = m3CreateWorld(&def);
        bool recording = run == 0 && m3World_JournalBegin(world, journal, (int32_t)sizeof(journal));
        m3BodyDef gd = m3DefaultBodyDef();
        m3BodyId ground = m3CreateBody(world, &gd);
        m3ShapeDef sg = m3DefaultShapeDef();
        m3Plane fl = {{0.0f, 1.0f, 0.0f}, 0.0f};
        m3ShapeId floor = m3CreatePlaneShape(ground, &sg, &fl);
        m3ShapeId crateShape;
        m3BodyId crate = Crate(world, (m3Pos3){0.0, 3.0, 0.0}, &crateShape);
        StepN(world, 30);
        m3World_SetContactTuning(world, 45.0f, 5.0f, 1.5f);
        m3Shape_SetFriction(floor, 0.2f);
        m3Shape_SetRestitution(crateShape, 0.4f);
        m3World_SetRestitutionThreshold(world, 0.5f);
        m3World_SetMaximumLinearSpeed(world, 50.0f);
        m3Body_ApplyLinearImpulse(crate, (m3Vec3){3.0f, 0.0f, 1.0f});
        StepN(world, 60);
        m3World_EnableSleeping(world, false);
        m3Shape_SetRollingResistance(crateShape, 0.05f);
        m3Shape_SetDensity(crateShape, 2.0f, true);
        StepN(world, 60);
        uint64_t final = m3World_Hash(world);
        twinHash[run] = final;
        if (recording)
        {
            int32_t bytes = m3World_JournalEnd(world);
            CHECK(bytes > 0, "the knob session records");
            m3WorldDef freshDef = m3DefaultWorldDef();
            freshDef.bodyCapacity = 32;
            freshDef.shapeCapacity = 32;
            m3WorldId fresh = m3CreateWorld(&freshDef);
            CHECK(m3World_JournalReplay(fresh, journal, bytes), "the knob session replays");
            CHECK(m3World_Hash(fresh) == final, "the knob replay is bit-identical");
            m3DestroyWorld(fresh);
        }
        m3DestroyWorld(world);
    }
    CHECK(twinHash[0] == twinHash[1], "equal knob twins are bit-identical");

    // A single differing knob must diverge.
    uint64_t split[2];
    for (int32_t run = 0; run < 2; ++run)
    {
        m3WorldId world = PlaneWorld(NULL);
        m3BodyId crate = Crate(world, (m3Pos3){0.0, 3.0, 0.0}, NULL);
        (void)crate;
        m3World_SetContactTuning(world, run == 0 ? 30.0f : 15.0f, 10.0f, 3.0f);
        StepN(world, 90);
        split[run] = m3World_Hash(world);
        m3DestroyWorld(world);
    }
    CHECK(split[0] != split[1], "a differing knob diverges the hash");
}

static void TestHostileKnobs(void)
{
    // Garbage in: nothing happens, loudly nothing.
    m3ShapeId floor;
    m3WorldId world = PlaneWorld(&floor);
    float f0 = m3Shape_GetFriction(floor);
    m3Shape_SetFriction(floor, -1.0f);
    m3Shape_SetFriction(floor, NAN);
    CHECK(m3Shape_GetFriction(floor) == f0, "hostile friction refused");
    m3Shape_SetDensity(floor, 0.0f, true);
    m3Shape_SetDensity(floor, -5.0f, true);
    m3World_SetContactTuning(world, 0.0f, 10.0f, 3.0f);
    m3World_SetContactTuning(world, 30.0f, 10.0f, NAN);
    m3World_SetMaximumLinearSpeed(world, 0.0f);
    m3World_SetGravity(world, (m3Vec3){NAN, 0.0f, 0.0f});
    m3Vec3 g = m3World_GetGravity(world);
    CHECK(g.y < 0.0f && g.x == 0.0f, "hostile gravity refused");
    m3ShapeId stale = floor;
    stale.index1 += 1000;
    m3Shape_SetFriction(stale, 0.5f);
    CHECK(m3Shape_GetFriction(stale) == 0.0f, "stale id reads zero, sets nothing");
    m3DestroyWorld(world);
}

int main(void)
{
    TestFrictionSetterBinds();
    TestRestitutionAndThreshold();
    TestDensityRebuildsMass();
    TestGravityAndSpeedCap();
    TestSleepAndContinuousToggles();
    TestKnobTwinsAndReplay();
    TestHostileKnobs();
    if (s_failures == 0)
    {
        printf("test_tuning: all green\n");
        return 0;
    }
    printf("test_tuning: %d failure(s)\n", s_failures);
    return 1;
}
