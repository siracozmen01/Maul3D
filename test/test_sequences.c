// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Red-team round two (2d-2): hostile SEQUENCES. Round one poisoned
// the inputs; this round abuses the order of operations: stale ids
// into every entry, snapshots into short buffers, corrupted and
// truncated journals against the new atomic-replay guarantee,
// destroy cascades racing joint lists, and a pool churned all the
// way to generation retirement.

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

static m3WorldId SmallWorld(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    def.jointCapacity = 8;
    def.meshCapacity = 1;
    return m3CreateWorld(&def);
}

static void TestStaleIdsEverywhere(void)
{
    // Destroy an object, then feed its id to every entry that takes
    // one. The contract: getters return zeros, commands and destroys
    // no-op, creates refuse, and nothing crashes (the ASan cells
    // enforce the last clause with teeth).
    m3WorldId world = SmallWorld();
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 3.0, 0.0};
    m3BodyId body = m3CreateBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.4f};
    m3ShapeId shape = m3CreateSphereShape(body, &sd, &ball);
    m3BodyId other = m3CreateBody(world, &bd);
    m3JointDef jd = m3DefaultJointDef();
    jd.bodyA = body;
    jd.bodyB = other;
    m3JointId joint = m3CreateJoint(&jd);

    m3DestroyBody(body); // cascades the shape and the joint

    CHECK(!m3Body_IsValid(body), "the destroyed body id is stale");
    CHECK(!m3Shape_IsValid(shape), "the cascaded shape id is stale");
    CHECK(!m3Joint_IsValid(joint), "the cascaded joint id is stale");

    m3Pos3 p = m3Body_GetPosition(body);
    CHECK(p.x == 0.0 && p.y == 0.0 && p.z == 0.0, "a stale getter returns zeros");
    m3Vec3 v = m3Body_GetLinearVelocity(body);
    CHECK(v.x == 0.0f && v.y == 0.0f && v.z == 0.0f, "a stale velocity reads zero");
    m3Body_SetLinearVelocity(body, (m3Vec3){1.0f, 0.0f, 0.0f});  // no-op
    m3Body_SetAngularVelocity(body, (m3Vec3){0.0f, 1.0f, 0.0f}); // no-op
    m3DestroyBody(body);                                         // double destroy: no-op
    m3DestroyJoint(joint);                                       // stale: no-op

    CHECK(!m3Shape_IsValid(m3CreateSphereShape(body, &sd, &ball)),
          "a create on a stale body refuses");
    jd.bodyA = body;
    jd.bodyB = other;
    CHECK(!m3Joint_IsValid(m3CreateJoint(&jd)), "a joint on a stale body refuses");

    // A generation-stale id (slot recycled) must be just as dead.
    m3BodyId recycled = m3CreateBody(world, &bd);
    CHECK(recycled.index1 == body.index1, "the slot recycles");
    CHECK(!m3Body_IsValid(body), "the old generation stays stale after recycling");
    m3World_Step(world, 1.0f / 60.0f, 4);
    CHECK(m3World_Hash(world) != 0, "the world shrugs it all off");
    m3DestroyWorld(world);

    // World-level staleness: the world id carries a generation, so
    // every call through the dead world id refuses or no-ops.
    CHECK(m3World_SnapshotSize(world) == -1 || m3World_SnapshotSize(world) == 0,
          "a dead world refuses its snapshot size");
    m3DestroyWorld(world); // double destroy: no-op
}

static void TestSnapshotRefusals(void)
{
    m3WorldId world = SmallWorld();
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 2.0, 0.0};
    m3BodyId body = m3CreateBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.4f};
    m3CreateSphereShape(body, &sd, &ball);
    for (int32_t i = 0; i < 30; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }

    int32_t bytes = m3World_SnapshotSize(world);
    uint8_t* snap = (uint8_t*)malloc((size_t)bytes);
    CHECK(m3World_Snapshot(world, snap, bytes - 1) == -1, "a short buffer refuses, loudly");
    CHECK(m3World_Snapshot(world, snap, bytes) == bytes, "the right buffer writes");
    uint64_t hashBefore = m3World_Hash(world);

    // A corrupted header refuses and the world is untouched.
    uint8_t evil[8192];
    memcpy(evil, snap, (size_t)(bytes < 8192 ? bytes : 8192));
    evil[0] ^= 0xFF; // magic
    CHECK(!m3World_Restore(world, evil, bytes), "a corrupted magic refuses");
    memcpy(evil, snap, (size_t)(bytes < 8192 ? bytes : 8192));
    evil[4] ^= 0xFF; // version
    CHECK(!m3World_Restore(world, evil, bytes), "a corrupted version refuses");
    CHECK(!m3World_Restore(world, snap, bytes - 1), "a truncated snapshot refuses");
    CHECK(m3World_Hash(world) == hashBefore, "refused restores leave the world untouched");

    // A snapshot from a differently-shaped world refuses.
    m3WorldDef bigger = m3DefaultWorldDef();
    bigger.bodyCapacity = 16;
    bigger.shapeCapacity = 8;
    m3WorldId otherWorld = m3CreateWorld(&bigger);
    CHECK(!m3World_Restore(otherWorld, snap, bytes),
          "a snapshot never restores into a different world shape");
    m3DestroyWorld(otherWorld);

    free(snap);
    m3DestroyWorld(world);
}

// Record a small session with creates, commands, a mid-journal
// cascade destroy, and steps. Returns the byte count.
static int32_t RecordSession(m3WorldId world, uint8_t* journal, int32_t cap)
{
    CHECK(m3World_JournalBegin(world, journal, cap), "the journal arms");
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 2.0, 0.0};
    m3BodyId a = m3CreateBody(world, &bd);
    bd.position = (m3Pos3){0.0, 1.0, 0.0};
    m3BodyId b = m3CreateBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.3f};
    m3CreateSphereShape(a, &sd, &ball);
    m3CreateSphereShape(b, &sd, &ball);
    m3JointDef jd = m3DefaultJointDef();
    jd.bodyA = a;
    jd.bodyB = b;
    m3CreateJoint(&jd);
    m3Body_SetLinearVelocity(b, (m3Vec3){0.5f, 0.0f, 0.0f});
    // The mid-journal cascade: destroying `a` takes its shape and
    // the joint with it, all inside the recording.
    m3DestroyBody(a);
    return m3World_JournalEnd(world);
}

static void TestJournalCorruptionIsAtomic(void)
{
    static uint8_t journal[65536];
    m3WorldId source = SmallWorld();
    int32_t bytes = RecordSession(source, journal, (int32_t)sizeof(journal));
    CHECK(bytes > 0, "the cascade session records");
    uint64_t sourceHash = m3World_Hash(source);
    m3DestroyWorld(source);

    // The clean replay reproduces the cascade world bit for bit.
    m3WorldId clean = SmallWorld();
    CHECK(m3World_JournalReplay(clean, journal, bytes), "the clean session replays");
    CHECK(m3World_Hash(clean) == sourceHash, "the mid-journal cascade replays bit-exact");
    m3DestroyWorld(clean);

    // Corruption anywhere: replay refuses AND the target world is
    // exactly what it was before the call (the atomic guarantee).
    static uint8_t evil[65536];
    for (int32_t attack = 0; attack < 3; ++attack)
    {
        m3WorldId target = SmallWorld();
        m3BodyDef bd = m3DefaultBodyDef();
        m3BodyId resident = m3CreateBody(target, &bd); // pre-existing state
        CHECK(m3Body_IsValid(resident), "the resident body exists");
        uint64_t before = m3World_Hash(target);

        memcpy(evil, journal, (size_t)bytes);
        bool refused;
        if (attack == 0)
        {
            evil[bytes / 2] ^= 0xA5; // corrupt a middle byte
            refused = !m3World_JournalReplay(target, evil, bytes);
        }
        else if (attack == 1)
        {
            refused = !m3World_JournalReplay(target, evil, bytes - 7); // truncate
        }
        else
        {
            memset(evil, 0x7F, 8); // unknown op in the first header
            refused = !m3World_JournalReplay(target, evil, bytes);
        }
        CHECK(refused, "a damaged journal refuses");
        CHECK(m3World_Hash(target) == before, "the refused replay left no fingerprints");
        CHECK(m3Body_IsValid(resident), "the resident body survived the attack");
        m3World_Step(target, 1.0f / 60.0f, 4);
        m3DestroyWorld(target);
    }
}

static void TestCascadeRaces(void)
{
    // Joint lists under fire: one body in three joints, destroyed;
    // the survivor's list must be clean enough to joint again, and
    // twin worlds walking the same churn agree bit for bit.
    uint64_t hashes[2];
    for (int32_t run = 0; run < 2; ++run)
    {
        m3WorldId world = SmallWorld();
        m3BodyDef bd = m3DefaultBodyDef();
        bd.type = m3_dynamicBody;
        m3BodyId hub = m3CreateBody(world, &bd);
        m3BodyId spokes[3];
        m3JointId joints[3];
        m3JointDef jd = m3DefaultJointDef();
        for (int32_t i = 0; i < 3; ++i)
        {
            bd.position = (m3Pos3){(double)i - 1.0, 1.0, 0.0};
            spokes[i] = m3CreateBody(world, &bd);
            jd.bodyA = hub;
            jd.bodyB = spokes[i];
            joints[i] = m3CreateJoint(&jd);
        }
        m3DestroyJoint(joints[1]); // manual destroy first
        m3DestroyBody(hub);        // cascades the other two
        CHECK(!m3Joint_IsValid(joints[0]) && !m3Joint_IsValid(joints[2]),
              "the cascade takes the hub's remaining joints");
        // The survivors joint among themselves: their lists are clean.
        jd.bodyA = spokes[0];
        jd.bodyB = spokes[1];
        CHECK(m3Joint_IsValid(m3CreateJoint(&jd)), "survivors accept new joints");
        // Destroy in the OTHER order too: joint then body then body.
        jd.bodyA = spokes[1];
        jd.bodyB = spokes[2];
        m3JointId last = m3CreateJoint(&jd);
        m3DestroyJoint(last);
        m3DestroyBody(spokes[2]);
        for (int32_t i = 0; i < 30; ++i)
        {
            m3World_Step(world, 1.0f / 60.0f, 4);
        }
        hashes[run] = m3World_Hash(world);
        m3DestroyWorld(world);
    }
    CHECK(hashes[0] == hashes[1], "cascade churn is bit-deterministic");
}

static void TestGenerationRetirement(void)
{
    // Churn one slot to the generation ceiling: the slot retires at
    // 0xFFFF instead of wrapping, so no stale id can ever alias a
    // recycled object. With capacity 2 and a FIFO free queue both
    // slots retire; the pool then refuses, loudly, forever.
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 2;
    def.shapeCapacity = 2;
    m3WorldId world = m3CreateWorld(&def);
    m3BodyDef bd = m3DefaultBodyDef();

    m3BodyId early = m3CreateBody(world, &bd); // generation 1, slot 0
    m3DestroyBody(early);

    int32_t created = 0;
    for (;;)
    {
        m3BodyId body = m3CreateBody(world, &bd);
        if (!m3Body_IsValid(body))
        {
            break; // both slots retired
        }
        created += 1;
        m3DestroyBody(body);
        if (created > 300000)
        {
            break; // safety net: something is wrong
        }
    }
    // Two slots, generations 1..0xFFFE usable after which the free
    // bumps to 0xFFFF and retires: the exact count is an
    // implementation detail, the properties below are the contract.
    CHECK(created > 100000, "the pool survived deep churn");
    CHECK(created < 300000, "the pool retired instead of wrapping");
    CHECK(!m3Body_IsValid(m3CreateBody(world, &bd)), "a fully retired pool refuses forever");
    CHECK(!m3Body_IsValid(early), "the very first id is still stale after the churn");
    m3World_Step(world, 1.0f / 60.0f, 4);
    CHECK(m3World_Hash(world) != 0, "the retired world still steps");
    m3DestroyWorld(world);
}

int main(void)
{
    TestStaleIdsEverywhere();
    TestSnapshotRefusals();
    TestJournalCorruptionIsAtomic();
    TestCascadeRaces();
    TestGenerationRetirement();
    if (s_failures == 0)
    {
        printf("test_sequences: all green\n");
        return 0;
    }
    printf("test_sequences: %d failure(s)\n", s_failures);
    return 1;
}
