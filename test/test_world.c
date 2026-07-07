// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// World gate: lifecycle, def-cookie validation, body identity (FIFO
// recycling, generation staleness, world pinning), and the loud
// failure paths. Black box: public headers only.

#include "maul3d/body.h"

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

static void TestWorldLifecycle(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    m3WorldId world = m3CreateWorld(&def);
    CHECK(m3World_IsValid(world), "world creation");

    m3WorldDef bad;
    memset(&bad, 0, sizeof(bad));
    CHECK(!m3World_IsValid(m3CreateWorld(&bad)), "a zeroed def is rejected loudly");

    m3DestroyWorld(world);
    CHECK(!m3World_IsValid(world), "a destroyed world id goes stale");

    // The recycled slot gets a new generation: the old id stays dead.
    m3WorldId again = m3CreateWorld(&def);
    CHECK(m3World_IsValid(again), "the slot recycles");
    CHECK(!m3World_IsValid(world), "the old world id is still stale after recycling");
    m3DestroyWorld(again);
}

static void TestBodies(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 4;
    m3WorldId world = m3CreateWorld(&def);

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){1.0, 2.0, 3.0};
    bd.linearVelocity = (m3Vec3){0.5f, 0.0f, -0.5f};
    bd.userData = 77;
    m3BodyId a = m3CreateBody(world, &bd);
    CHECK(m3Body_IsValid(a), "body creation");

    m3Pos3 p = m3Body_GetPosition(a);
    CHECK(p.x == 1.0 && p.y == 2.0 && p.z == 3.0, "position round-trips through the def");
    m3Vec3 v = m3Body_GetLinearVelocity(a);
    CHECK(v.x == 0.5f && v.z == -0.5f, "velocity round-trips");
    CHECK(m3Body_GetUserData(a) == 77, "user data is carried verbatim");
    CHECK(m3Body_GetType(a) == m3_dynamicBody, "type reads back");
    m3Quat q = m3Body_GetRotation(a);
    CHECK(q.w == 1.0f, "default rotation is identity");

    // Journaled setters mutate.
    m3Body_SetLinearVelocity(a, (m3Vec3){9.0f, 0.0f, 0.0f});
    CHECK(m3Body_GetLinearVelocity(a).x == 9.0f, "setter applies");

    // Destroy: the id goes stale, the slot recycles FIFO.
    m3BodyId b = m3CreateBody(world, &bd);
    m3DestroyBody(a);
    CHECK(!m3Body_IsValid(a), "a destroyed body id goes stale");
    CHECK(m3Body_IsValid(b), "other bodies are untouched");
    m3BodyId c = m3CreateBody(world, &bd);
    CHECK(c.index1 == a.index1 && c.generation != a.generation,
          "the slot recycles with a fresh generation");
    CHECK(!m3Body_IsValid(a), "the stale id still fails after recycling");

    // Capacity exhaustion returns the null id, never a hidden growth.
    m3BodyDef sd = m3DefaultBodyDef();
    m3CreateBody(world, &sd);
    m3CreateBody(world, &sd);
    m3BodyId overflow = m3CreateBody(world, &sd);
    CHECK(overflow.index1 == 0, "an exhausted pool returns the null id");

    m3DestroyWorld(world);
    CHECK(!m3Body_IsValid(b), "bodies die with their world");
}

static void TestTwoWorldsIsolation(void)
{
    // A body id is pinned to its world: handing it to another world's
    // bodies must fail validation, never alias.
    m3WorldDef def = m3DefaultWorldDef();
    m3WorldId w1 = m3CreateWorld(&def);
    m3WorldId w2 = m3CreateWorld(&def);
    m3BodyDef bd = m3DefaultBodyDef();
    m3BodyId b1 = m3CreateBody(w1, &bd);
    m3BodyId b2 = m3CreateBody(w2, &bd);
    CHECK(b1.world0 != b2.world0, "ids carry their world slot");
    CHECK(m3Body_IsValid(b1) && m3Body_IsValid(b2), "both bodies are live");
    m3DestroyWorld(w1);
    CHECK(!m3Body_IsValid(b1) && m3Body_IsValid(b2), "destroying one world spares the other");
    m3DestroyWorld(w2);
}

int main(void)
{
    TestWorldLifecycle();
    TestBodies();
    TestTwoWorldsIsolation();
    if (s_failures == 0)
    {
        printf("test_world: all checks passed\n");
    }
    return s_failures == 0 ? 0 : 1;
}
