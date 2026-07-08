// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The soft body gate (7-1): XPBD lattices under the four gates. A
// pinned rope hangs to its analytic length, a jelly cube rests on
// the plane at its geometric height, the whole session journals
// and replays bit-exact, and a mid-sag rollback re-sags onto
// identical bits. Hostile defs refuse loudly.

#include "maul3d/shape.h"
#include "maul3d/softbody.h"

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
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    m3WorldId world = m3CreateWorld(&def);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    m3CreatePlaneShape(ground, &sd, &floor);
    return world;
}

static void TestHangingRope(void)
{
    // Twenty particles on rigid rods, the top one pinned: the rope
    // hangs straight down to its analytic length.
    m3WorldId world = PlaneWorld();
    m3SoftBodyDef sb = m3DefaultSoftBodyDef();
    sb.position = (m3Pos3){0.0, 5.0, 0.0};
    sb.countX = 1;
    sb.countY = 20;
    sb.countZ = 1;
    sb.spacing = 0.25f;
    sb.compliance = 0.0f;
    m3SoftBodyId rope = m3CreateSoftBody(world, &sb);
    CHECK(m3SoftBody_IsValid(rope), "the rope creates");
    CHECK(m3SoftBody_GetParticleCount(rope) == 20, "twenty particles");
    m3SoftBody_PinParticle(rope, 19); // the top of the lattice

    for (int32_t i = 0; i < 300; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3Pos3 top = m3SoftBody_GetParticlePosition(rope, 19);
    m3Pos3 bottom = m3SoftBody_GetParticlePosition(rope, 0);
    CHECK(fabs(top.y - (5.0 + 19.0 * 0.25)) < 1.0e-9, "the pin never moves");
    CHECK(fabs((top.y - bottom.y) - 4.75) < 0.05, "the rope hangs to its analytic length");
    CHECK(fabs(bottom.x) < 0.01 && fabs(bottom.z) < 0.01, "the hang is plumb");
    m3DestroyWorld(world);
}

static void TestJellyRest(void)
{
    // A four-cube jelly dropped on the plane settles at its
    // geometric height: bottom layer at particle radius, top a
    // lattice above it, small squash tolerated.
    m3WorldId world = PlaneWorld();
    m3SoftBodyDef sb = m3DefaultSoftBodyDef();
    sb.position = (m3Pos3){0.0, 1.0, 0.0};
    sb.countX = 4;
    sb.countY = 4;
    sb.countZ = 4;
    sb.spacing = 0.3f;
    sb.compliance = 1.0e-4f;
    sb.radius = 0.1f;
    m3SoftBodyId jelly = m3CreateSoftBody(world, &sb);
    CHECK(m3SoftBody_IsValid(jelly), "the jelly creates");

    for (int32_t i = 0; i < 360; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    double minY = 1.0e30;
    double maxY = -1.0e30;
    for (int32_t p = 0; p < 64; ++p)
    {
        m3Pos3 q = m3SoftBody_GetParticlePosition(jelly, p);
        CHECK(isfinite(q.x) && isfinite(q.y) && isfinite(q.z), "the jelly stays finite");
        minY = q.y < minY ? q.y : minY;
        maxY = q.y > maxY ? q.y : maxY;
    }
    CHECK(minY > 0.05 && minY < 0.15, "the bottom layer rests at particle radius");
    CHECK(maxY - minY > 0.7 && maxY - minY < 1.0, "the cube keeps its height, squash tolerated");
    m3DestroyWorld(world);
}

static void TestSoftSessionAndRollback(void)
{
    // The full contract: journal from world birth, replay
    // bit-exact; snapshot mid-sag, re-sag onto the same bits.
    static uint8_t journal[262144];
    static uint8_t snap[393216];
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    m3WorldId world = m3CreateWorld(&def);
    CHECK(m3World_JournalBegin(world, journal, (int32_t)sizeof(journal)), "the session records");
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    m3CreatePlaneShape(ground, &sd, &floor);

    m3SoftBodyDef sb = m3DefaultSoftBodyDef();
    sb.position = (m3Pos3){0.0, 3.0, 0.0};
    sb.countX = 6;
    sb.countY = 6;
    sb.countZ = 1; // a cloth sheet
    sb.spacing = 0.2f;
    sb.compliance = 1.0e-5f;
    m3SoftBodyId cloth = m3CreateSoftBody(world, &sb);
    m3SoftBody_PinParticle(cloth, 30); // one top corner (x0, y5)
    m3SoftBody_PinParticle(cloth, 35); // the other top corner

    int32_t snapBytes = 0;
    for (int32_t i = 0; i < 120; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
        if (i == 60)
        {
            snapBytes = m3World_Snapshot(world, snap, (int32_t)sizeof(snap));
            CHECK(snapBytes > 0, "the mid-sag snapshot fits");
        }
    }
    uint64_t final = m3World_Hash(world);

    int32_t bytes = m3World_JournalEnd(world);
    CHECK(bytes > 0, "the session closes");
    m3WorldId fresh = m3CreateWorld(&def);
    CHECK(m3World_JournalReplay(fresh, journal, bytes), "the session replays");
    CHECK(m3World_Hash(fresh) == final, "the replayed sag is bit-identical");
    m3DestroyWorld(fresh);

    CHECK(m3World_Restore(world, snap, snapBytes), "the mid-sag restore lands");
    for (int32_t i = 61; i < 120; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m3World_Hash(world) == final, "the re-sag is bit-identical");
    m3DestroyWorld(world);
}

static void TestSoftContracts(void)
{
    m3WorldId world = PlaneWorld();
    m3SoftBodyDef sb = m3DefaultSoftBodyDef();
    sb.countX = 0;
    CHECK(!m3SoftBody_IsValid(m3CreateSoftBody(world, &sb)), "a zero count refuses");
    sb = m3DefaultSoftBodyDef();
    sb.countX = 9;
    sb.countY = 9;
    sb.countZ = 9; // 729 > 512
    CHECK(!m3SoftBody_IsValid(m3CreateSoftBody(world, &sb)), "an oversized lattice refuses");
    sb = m3DefaultSoftBodyDef();
    sb.spacing = 0.0f;
    CHECK(!m3SoftBody_IsValid(m3CreateSoftBody(world, &sb)), "a zero spacing refuses");
    sb = m3DefaultSoftBodyDef();
    float bad;
    uint32_t nanBits = 0x7FC00000u;
    memcpy(&bad, &nanBits, sizeof(bad));
    sb.compliance = bad;
    CHECK(!m3SoftBody_IsValid(m3CreateSoftBody(world, &sb)), "a NaN compliance refuses");

    sb = m3DefaultSoftBodyDef();
    m3SoftBodyId a = m3CreateSoftBody(world, &sb);
    sb.position = (m3Pos3){3.0, 1.0, 0.0};
    m3SoftBodyId b = m3CreateSoftBody(world, &sb);
    CHECK(m3SoftBody_IsValid(a) && m3SoftBody_IsValid(b), "the pool serves two");
    sb.position = (m3Pos3){6.0, 1.0, 0.0};
    CHECK(!m3SoftBody_IsValid(m3CreateSoftBody(world, &sb)), "the full pool refuses the third");

    m3SoftBody_PinParticle(a, 9999); // out of range: no-op
    m3DestroySoftBody(a);
    CHECK(!m3SoftBody_IsValid(a), "the destroyed soft body is stale");
    m3SoftBody_PinParticle(a, 0); // stale: no-op
    m3Pos3 p = m3SoftBody_GetParticlePosition(a, 0);
    CHECK(p.x == 0.0 && p.y == 0.0, "a stale getter returns zeros");
    m3DestroySoftBody(a); // stale: no-op
    m3World_Step(world, 1.0f / 60.0f, 4);
    CHECK(m3SoftBody_IsValid(b), "the survivor lives on");
    m3DestroyWorld(world);
}

int main(void)
{
    TestHangingRope();
    TestJellyRest();
    TestSoftSessionAndRollback();
    TestSoftContracts();
    if (s_failures == 0)
    {
        printf("test_softbody: all green\n");
        return 0;
    }
    printf("test_softbody: %d failure(s)\n", s_failures);
    return 1;
}
