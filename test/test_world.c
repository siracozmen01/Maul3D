// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// World gate: lifecycle, def-cookie validation, body identity (FIFO
// recycling, generation staleness, world pinning), and the loud
// failure paths. Black box: public headers only.

#include "maul3d/body.h"

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

static void TestSnapshot(void)
{
    // The day-one rule: if the empty world cannot snapshot and restore
    // byte-identically, the architecture is already wrong.
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 8;
    m3WorldId world = m3CreateWorld(&def);

    int32_t size = m3World_SnapshotSize(world);
    CHECK(size > 0, "the empty world has a snapshot size");
    void* snapA = malloc((size_t)size);
    void* snapB = malloc((size_t)size);
    CHECK(m3World_Snapshot(world, snapA, size) == size, "empty world snapshots");
    CHECK(m3World_Restore(world, snapA, size), "empty world restores into itself");
    CHECK(m3World_Snapshot(world, snapB, size) == size, "re-snapshot");
    CHECK(memcmp(snapA, snapB, (size_t)size) == 0, "restore then snapshot is byte-identical");

    // Populate, snapshot, mutate, restore: the mutation must vanish
    // completely, hash and bytes both.
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 3.0, 0.0};
    m3BodyId a = m3CreateBody(world, &bd);
    m3BodyId doomed = m3CreateBody(world, &bd);
    uint64_t hashBefore = m3World_Hash(world);
    CHECK(m3World_Snapshot(world, snapA, size) == size, "populated snapshot");

    m3Body_SetLinearVelocity(a, (m3Vec3){5.0f, 0.0f, 0.0f});
    m3DestroyBody(doomed);
    m3BodyId intruder = m3CreateBody(world, &bd); // reuses doomed's slot
    uint64_t hashMutated = m3World_Hash(world);
    CHECK(hashMutated != hashBefore, "mutations move the hash");
    CHECK(m3Body_IsValid(intruder), "the intruder is live before the rollback");

    CHECK(m3World_Restore(world, snapA, size), "restore rolls the world back");
    CHECK(m3World_Hash(world) == hashBefore, "the hash returns to the snapshot value");
    CHECK(m3World_Snapshot(world, snapB, size) == size, "post-restore snapshot");
    CHECK(memcmp(snapA, snapB, (size_t)size) == 0, "the bytes return too");
    CHECK(m3Body_IsValid(doomed), "the destroyed body lives again after rollback");
    CHECK(!m3Body_IsValid(intruder), "the post-snapshot body is stale after rollback");
    CHECK(m3Body_GetLinearVelocity(a).x == 0.0f, "the setter is rolled back");

    // The mini rollback-determinism proof: rerun the SAME mutations
    // and the timeline repeats exactly, minted id and hash included.
    m3Body_SetLinearVelocity(a, (m3Vec3){5.0f, 0.0f, 0.0f});
    m3DestroyBody(doomed);
    m3BodyId reMinted = m3CreateBody(world, &bd);
    CHECK(reMinted.index1 == intruder.index1 && reMinted.generation == intruder.generation,
          "rerunning the ops after rollback repeats the minted id");
    CHECK(m3World_Hash(world) == hashMutated, "and repeats the hash bit for bit");

    // Refusals are loud and total: corrupt config, corrupt magic,
    // truncated size, and a foreign capacity all fail without touching
    // the world.
    uint64_t untouched = m3World_Hash(world);
    uint8_t* corrupt = (uint8_t*)snapA;
    corrupt[8] ^= 0xFF; // configHash byte
    CHECK(!m3World_Restore(world, corrupt, size), "a corrupted config hash is refused");
    corrupt[8] ^= 0xFF;
    corrupt[0] ^= 0xFF; // magic byte
    CHECK(!m3World_Restore(world, corrupt, size), "a corrupted magic is refused");
    corrupt[0] ^= 0xFF;
    CHECK(!m3World_Restore(world, corrupt, size - 4), "a truncated snapshot is refused");
    m3WorldDef other = m3DefaultWorldDef();
    other.bodyCapacity = 16;
    m3WorldId foreign = m3CreateWorld(&other);
    CHECK(!m3World_Restore(foreign, snapA, size), "a foreign capacity is refused");
    CHECK(m3World_Hash(world) == untouched, "refused restores never touch the world");

    free(snapA);
    free(snapB);
    m3DestroyWorld(foreign);
    m3DestroyWorld(world);
}

static void TestWorldHashGate(void)
{
    // A fixed scene whose hash joins the cross-platform CI gate: every
    // cell must reproduce these bits.
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 16;
    m3WorldId world = m3CreateWorld(&def);
    m3BodyDef bd = m3DefaultBodyDef();
    for (int32_t i = 0; i < 5; ++i)
    {
        bd.type = (i & 1) ? m3_staticBody : m3_dynamicBody;
        bd.position = (m3Pos3){0.25 * (double)i, 1.0 + 0.5 * (double)i, -0.125 * (double)i};
        bd.linearVelocity = (m3Vec3){0.1f * (m3real)i, 0.0f, -0.2f * (m3real)i};
        bd.angularVelocity = (m3Vec3){0.0f, 0.3f * (m3real)i, 0.0f};
        bd.userData = (uint64_t)i;
        m3CreateBody(world, &bd);
    }
    printf("M3_WORLD_HASH=%016llx\n", (unsigned long long)m3World_Hash(world));
    m3DestroyWorld(world);
}

int main(void)
{
    TestWorldLifecycle();
    TestBodies();
    TestTwoWorldsIsolation();
    TestSnapshot();
    TestWorldHashGate();
    if (s_failures == 0)
    {
        printf("test_world: all checks passed\n");
    }
    return s_failures == 0 ? 0 : 1;
}
