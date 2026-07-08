// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The events gate (8-5): hit events fire once per impact at the
// analytic speed and only for opted-in shapes; body move events
// mirror every mover's transform and mark the sleep edge; the
// pre-solve veto builds a one-way platform under the loud purity
// contract; streams replay identically on twins.

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
    m3ShapeId fs = m3CreatePlaneShape(ground, &sd, &floor);
    if (outFloor != NULL)
    {
        *outFloor = fs;
    }
    return world;
}

static void TestHitEvents(void)
{
    // A ball dropped from 2m hits at sqrt(2*g*(h - r)) with g = 10:
    // about 5.48 m/s. Opted in, the hit fires once with that speed;
    // opted out or gated by a raised threshold, silence.
    for (int32_t mode = 0; mode < 3; ++mode)
    {
        m3ShapeId floor;
        m3WorldId world = PlaneWorld(&floor);
        m3BodyDef bd = m3DefaultBodyDef();
        bd.type = m3_dynamicBody;
        bd.position = (m3Pos3){0.0, 2.0, 0.0};
        m3BodyId ball = m3CreateBody(world, &bd);
        m3ShapeDef sd = m3DefaultShapeDef();
        m3Sphere s = {{0.0f, 0.0f, 0.0f}, 0.5f};
        m3ShapeId shape = m3CreateSphereShape(ball, &sd, &s);
        (void)shape;
        if (mode >= 1)
        {
            m3Shape_EnableHitEvents(floor, true);
            CHECK(m3Shape_AreHitEventsEnabled(floor), "hit flag reads back");
        }
        if (mode == 2)
        {
            m3World_SetHitEventThreshold(world, 50.0f);
        }
        int32_t totalHits = 0;
        double speed = 0.0;
        double normalY = 0.0;
        double pointY = 1.0e9;
        for (int32_t i = 0; i < 90; ++i)
        {
            m3World_Step(world, 1.0f / 60.0f, 4);
            int32_t n = 0;
            const m3HitEvent* ev = m3World_HitEvents(world, &n);
            if (n > 0)
            {
                speed = (double)ev[0].approachSpeed;
                normalY = (double)ev[0].normal.y;
                pointY = ev[0].point.y;
            }
            totalHits += n;
        }
        if (mode == 0)
        {
            CHECK(totalHits == 0, "no opt-in, no hit events");
        }
        else if (mode == 1)
        {
            CHECK(totalHits == 1, "one impact, one hit event");
            CHECK(speed > 5.0 && speed < 6.0, "the hit speed is the analytic impact speed");
            CHECK(fabs(normalY) > 0.99, "the hit normal is the plane normal");
            CHECK(fabs(pointY) < 0.2, "the hit point sits at the floor");
        }
        else
        {
            CHECK(totalHits == 0, "a raised threshold silences the hit");
        }
        CHECK(m3World_HitEventsDropped(world) == 0, "nothing dropped");
        m3DestroyWorld(world);
    }
}

static void TestBodyMoveEvents(void)
{
    // The falling crate reports its transform every step; the step
    // it drops off to sleep is marked, then silence until disturbed.
    m3WorldId world = PlaneWorld(NULL);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 2.0, 0.0};
    m3BodyId crate = m3CreateBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3CreateBoxShape(crate, &sd, (m3Vec3){0.5f, 0.5f, 0.5f});

    int32_t sleepEdges = 0;
    int32_t silentSteps = 0;
    bool transformsMatch = true;
    for (int32_t i = 0; i < 400; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
        int32_t n = 0;
        const m3BodyMoveEvent* ev = m3World_BodyMoveEvents(world, &n);
        bool found = false;
        for (int32_t k = 0; k < n; ++k)
        {
            if (ev[k].body.index1 == crate.index1)
            {
                found = true;
                m3Pos3 p = m3Body_GetPosition(crate);
                if (fabs(ev[k].transform.p.y - p.y) > 1.0e-12)
                {
                    transformsMatch = false;
                }
                if (ev[k].fellAsleep)
                {
                    sleepEdges += 1;
                }
            }
        }
        if (!found)
        {
            silentSteps += 1;
        }
    }
    CHECK(transformsMatch, "move events mirror the body transform");
    CHECK(sleepEdges == 1, "exactly one fell-asleep edge");
    CHECK(silentSteps > 100, "a sleeping body emits nothing");
    CHECK(!m3Body_IsAwake(crate), "the crate did sleep");
    m3DestroyWorld(world);
}

// The one-way platform: pass upward, land coming down. Pure
// function of its arguments, as the loud contract demands.
static bool OneWayVeto(m3ShapeId shapeA, m3ShapeId shapeB, m3Pos3 point, m3Vec3 normal, void* ctx)
{
    (void)shapeA;
    (void)shapeB;
    (void)point;
    int* calls = (int*)ctx;
    *calls += 1;
    // The normal runs from shape A to shape B. Support only when it
    // carries an upward component: approaches from below veto out.
    return fabsf(normal.y) > 0.5f && normal.y > 0.0f ? true : false;
}

static void TestPreSolveOneWay(void)
{
    // A ball flung upward through a thin plate: the veto lets it
    // through on the way up, then it lands ON the plate.
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m3WorldId world = m3CreateWorld(&def);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane fl = {{0.0f, 1.0f, 0.0f}, 0.0f};
    m3CreatePlaneShape(ground, &sd, &fl);
    m3BodyDef pd = m3DefaultBodyDef();
    pd.position = (m3Pos3){0.0, 3.0, 0.0};
    m3BodyId plate = m3CreateBody(world, &pd);
    m3ShapeId plateShape = m3CreateBoxShape(plate, &sd, (m3Vec3){2.0f, 0.1f, 2.0f});
    m3Shape_EnablePreSolve(plateShape, true);
    CHECK(m3Shape_IsPreSolveEnabled(plateShape), "pre-solve flag reads back");

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 0.5, 0.0};
    m3BodyId ball = m3CreateBody(world, &bd);
    m3Sphere s = {{0.0f, 0.0f, 0.0f}, 0.4f};
    m3CreateSphereShape(ball, &sd, &s);

    int calls = 0;
    m3World_SetPreSolveCallback(world, OneWayVeto, &calls);
    m3Body_SetLinearVelocity(ball, (m3Vec3){0.0f, 12.0f, 0.0f});
    double peak = 0.0;
    for (int32_t i = 0; i < 300; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
        double y = m3Body_GetPosition(ball).y;
        if (y > peak)
        {
            peak = y;
        }
    }
    CHECK(peak > 4.0, "the ball passes the plate on the way up");
    double restY = m3Body_GetPosition(ball).y;
    CHECK(restY > 3.3 && restY < 3.8, "the ball lands ON the plate coming down");
    CHECK(calls > 0, "the veto was consulted");

    // Clearing the callback restores plain solid contact.
    m3World_SetPreSolveCallback(world, NULL, NULL);
    m3World_Step(world, 1.0f / 60.0f, 4);
    m3DestroyWorld(world);
}

static void TestEventTwinsAndJointStream(void)
{
    // Twin worlds running the same script produce byte-identical
    // hit streams; the joint break stream exists and stays empty
    // until joints learn to break (the next slice).
    uint64_t hitDigest[2] = {0, 0};
    for (int32_t run = 0; run < 2; ++run)
    {
        m3ShapeId floor;
        m3WorldId world = PlaneWorld(&floor);
        m3Shape_EnableHitEvents(floor, true);
        for (int32_t b = 0; b < 4; ++b)
        {
            m3BodyDef bd = m3DefaultBodyDef();
            bd.type = m3_dynamicBody;
            bd.position = (m3Pos3){(double)b * 1.5, 1.5 + (double)b * 0.5, 0.0};
            m3BodyId ball = m3CreateBody(world, &bd);
            m3ShapeDef sd = m3DefaultShapeDef();
            m3Sphere s = {{0.0f, 0.0f, 0.0f}, 0.4f};
            m3CreateSphereShape(ball, &sd, &s);
        }
        uint64_t digest = 1469598103934665603ull;
        for (int32_t i = 0; i < 120; ++i)
        {
            m3World_Step(world, 1.0f / 60.0f, 4);
            int32_t n = 0;
            const m3HitEvent* ev = m3World_HitEvents(world, &n);
            for (int32_t k = 0; k < n; ++k)
            {
                const uint8_t* bytes = (const uint8_t*)&ev[k];
                for (size_t j = 0; j < sizeof(m3HitEvent); ++j)
                {
                    digest = (digest ^ bytes[j]) * 1099511628211ull;
                }
            }
            int32_t jb = 0;
            m3World_JointBreakEvents(world, &jb);
            CHECK(jb == 0, "the joint break stream is armed and empty");
        }
        hitDigest[run] = digest;
        m3DestroyWorld(world);
    }
    CHECK(hitDigest[0] == hitDigest[1], "twin hit streams are byte-identical");
}

static void TestHostileEvents(void)
{
    m3ShapeId floor;
    m3WorldId world = PlaneWorld(&floor);
    m3World_SetHitEventThreshold(world, -1.0f);
    m3World_SetHitEventThreshold(world, NAN);
    m3ShapeId stale = floor;
    stale.index1 += 999;
    m3Shape_EnableHitEvents(stale, true);
    CHECK(!m3Shape_AreHitEventsEnabled(stale), "stale flag set refuses");
    int32_t n = -1;
    CHECK(m3World_HitEvents(world, &n) != NULL || n == 0, "empty stream reads clean");
    CHECK(n == 0, "no events before any step");
    m3DestroyWorld(world);
}

int main(void)
{
    TestHitEvents();
    TestBodyMoveEvents();
    TestPreSolveOneWay();
    TestEventTwinsAndJointStream();
    TestHostileEvents();
    if (s_failures == 0)
    {
        printf("test_events: all green\n");
        return 0;
    }
    printf("test_events: %d failure(s)\n", s_failures);
    return 1;
}
