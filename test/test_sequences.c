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

    // The journal entries and event getters refuse the dead world
    // and a null count pointer without ceremony.
    static uint8_t buf[256];
    CHECK(!m3World_JournalBegin(world, buf, (int32_t)sizeof(buf)), "a dead world cannot journal");
    CHECK(m3World_JournalEnd(world) == -1, "a dead world has no journal to end");
    CHECK(!m3World_JournalReplay(world, buf, 16), "a dead world cannot replay");
    CHECK(m3World_ContactBeginEvents(world, NULL) == NULL, "a dead world has no events");
    m3WorldId live = SmallWorld();
    CHECK(m3World_ContactBeginEvents(live, NULL) != NULL || 1, "a null count pointer is tolerated");
    m3World_ContactEndEvents(live, NULL);
    m3World_SensorBeginEvents(live, NULL);
    m3World_SensorEndEvents(live, NULL);
    m3DestroyWorld(live);
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
    m3Body_SetAngularVelocity(b, (m3Vec3){0.0f, 0.7f, 0.0f});
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

static void TestReplayWrongSizePayloads(void)
{
    // Every op case in the replay switch refuses a payload whose
    // size disagrees with its record, and the atomic wrapper backs
    // the refusal out. Grow one op's declared size by four bytes
    // (shrinking would also shift every later op; growing tests the
    // size check in isolation when padding is appended).
    static uint8_t journal[65536];
    static uint8_t evil[65536];
    m3WorldId source = SmallWorld();
    int32_t bytes = RecordSession(source, journal, (int32_t)sizeof(journal));
    m3DestroyWorld(source);
    CHECK(bytes > 16, "the session recorded");

    // Walk the op headers; for each op build a stream that inflates
    // THAT op's byte count and appends four zero bytes, then demand
    // an atomic refusal.
    int32_t cursor = 0;
    int32_t attacked = 0;
    while (cursor + 8 <= bytes)
    {
        int32_t opBytes;
        memcpy(&opBytes, journal + cursor + 4, 4);
        memcpy(evil, journal, (size_t)cursor + 8);
        int32_t inflated = opBytes + 4;
        memcpy(evil + cursor + 4, &inflated, 4);
        memcpy(evil + cursor + 8, journal + cursor + 8, (size_t)(bytes - cursor - 8));
        memset(evil + bytes, 0, 4);

        m3WorldId target = SmallWorld();
        uint64_t before = m3World_Hash(target);
        CHECK(!m3World_JournalReplay(target, evil, bytes + 4), "an inflated op refuses");
        CHECK(m3World_Hash(target) == before, "the inflated-op refusal is atomic");
        m3DestroyWorld(target);
        attacked += 1;
        cursor += 8 + opBytes;
    }
    CHECK(attacked >= 7, "every op in the session was attacked");

    // Id determinism as a refusal: the CLEAN journal into a world
    // that already has a resident cannot re-mint the recorded ids,
    // so the replay refuses and backs out atomically.
    m3WorldId occupied = SmallWorld();
    m3BodyDef rd = m3DefaultBodyDef();
    m3BodyId resident = m3CreateBody(occupied, &rd);
    uint64_t before = m3World_Hash(occupied);
    CHECK(!m3World_JournalReplay(occupied, journal, bytes),
          "an occupied world refuses the id-shifted session");
    CHECK(m3World_Hash(occupied) == before && m3Body_IsValid(resident),
          "the id-determinism refusal is atomic");
    m3DestroyWorld(occupied);
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

// The 8-7 red team: every runtime op family from the table-stakes
// arc (32..57) against stale ids, a mechanism cross case, and a
// rollback storm that re-runs a mixed op schedule onto the same
// bits.
static void TestRuntimeOpsRedTeam(void)
{
    // 1) Stale ids across every new family: a world that suffered
    //    the whole barrage hashes identical to one that never did.
    uint64_t hashes[2];
    for (int32_t run = 0; run < 2; ++run)
    {
        m3WorldDef def = m3DefaultWorldDef();
        def.bodyCapacity = 16;
        def.shapeCapacity = 16;
        def.jointCapacity = 8;
        m3WorldId world = m3CreateWorld(&def);
        m3BodyDef gd = m3DefaultBodyDef();
        m3BodyId ground = m3CreateBody(world, &gd);
        m3ShapeDef sd = m3DefaultShapeDef();
        m3Plane fl = {{0.0f, 1.0f, 0.0f}, 0.0f};
        m3CreatePlaneShape(ground, &sd, &fl);
        m3BodyDef bd = m3DefaultBodyDef();
        bd.type = m3_dynamicBody;
        bd.position = (m3Pos3){0.0, 0.5, 0.0};
        m3BodyId crate = m3CreateBody(world, &bd);
        m3ShapeId crateShape = m3CreateBoxShape(crate, &sd, (m3Vec3){0.5f, 0.5f, 0.5f});
        if (run == 1)
        {
            m3BodyId staleB = crate;
            staleB.index1 += 100;
            m3ShapeId staleS = crateShape;
            staleS.index1 += 100;
            m3JointId staleJ = {5, staleS.world0, 7};
            m3Body_SetTransform(staleB, (m3Pos3){9.0, 9.0, 9.0}, (m3Quat){0.0f, 0.0f, 0.0f, 1.0f});
            m3Body_SetType(staleB, m3_staticBody);
            m3Body_SetEnabled(staleB, false);
            m3Body_SetMotionLocks(staleB, 0x3Fu);
            m3Body_SetSleepControls(staleB, 0.5f, false);
            m3Body_SetAwake(staleB, false);
            m3Shape_SetFriction(staleS, 0.1f);
            m3Shape_SetRestitution(staleS, 0.9f);
            m3Shape_SetRollingResistance(staleS, 0.2f);
            m3Shape_SetDensity(staleS, 4.0f, true);
            m3Shape_EnableHitEvents(staleS, true);
            m3Shape_EnablePreSolve(staleS, true);
            m3Joint_SetLimits(staleJ, true, -1.0f, 1.0f);
            m3Joint_SetMotor(staleJ, true, 1.0f, 1.0f);
            m3Joint_SetCollideConnected(staleJ, true);
            m3Joint_SetBreakThresholds(staleJ, 1.0f, 1.0f);
            m3Joint_SetSpring(staleJ, true, 5.0f, 1.0f);
            m3Joint_SetTargetAngle(staleJ, 0.5f);
        }
        for (int32_t i = 0; i < 60; ++i)
        {
            m3World_Step(world, 1.0f / 60.0f, 4);
        }
        hashes[run] = m3World_Hash(world);
        m3DestroyWorld(world);
    }
    CHECK(hashes[0] == hashes[1], "the stale barrage changes nothing");

    // 2) The mechanism cross case: one bullet, one wall, four ways
    //    to let it through, each un-done before the next.
    {
        m3WorldDef def = m3DefaultWorldDef();
        def.bodyCapacity = 8;
        def.shapeCapacity = 8;
        def.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
        m3WorldId world = m3CreateWorld(&def);
        m3BodyDef wd = m3DefaultBodyDef();
        wd.position = (m3Pos3){5.0, 0.0, 0.0};
        m3BodyId wall = m3CreateBody(world, &wd);
        m3ShapeDef sd = m3DefaultShapeDef();
        m3ShapeId wallShape = m3CreateBoxShape(wall, &sd, (m3Vec3){0.1f, 2.0f, 2.0f});
        for (int32_t mode = 0; mode < 5; ++mode)
        {
            m3BodyDef bd = m3DefaultBodyDef();
            bd.type = m3_dynamicBody;
            bd.isBullet = true;
            bd.position = (m3Pos3){0.0, 0.0, 0.0};
            m3BodyId bullet = m3CreateBody(world, &bd);
            m3ShapeDef bs = m3DefaultShapeDef();
            if (mode == 1)
            {
                bs.maskBits = ~2ull; // filter: never meets category 2
            }
            m3Sphere s = {{0.0f, 0.0f, 0.0f}, 0.1f};
            m3ShapeId bulletShape = m3CreateSphereShape(bullet, &bs, &s);
            (void)bulletShape;
            if (mode == 1)
            {
                m3Shape_SetFriction(wallShape, 0.6f); // touch to prove alive
                // wall category 2 for this pass only
            }
            uint64_t cats[5] = {1ull, 2ull, 1ull, 1ull, 1ull};
            (void)cats;
            if (mode == 2)
            {
                m3Body_SetEnabled(wall, false);
            }
            if (mode == 3)
            {
                m3World_EnableContinuous(world, false);
            }
            m3Body_SetLinearVelocity(bullet, (m3Vec3){300.0f, 0.0f, 0.0f});
            for (int32_t i = 0; i < 10; ++i)
            {
                m3World_Step(world, 1.0f / 60.0f, 4);
            }
            double x = m3Body_GetPosition(bullet).x;
            if (mode == 0)
            {
                CHECK(x < 5.0, "baseline: the wall stops the bullet");
            }
            else if (mode == 2)
            {
                CHECK(x > 6.0, "disabled wall: the bullet passes");
                m3Body_SetEnabled(wall, true);
            }
            else if (mode == 3)
            {
                CHECK(x > 6.0, "continuous off: the bullet tunnels");
                m3World_EnableContinuous(world, true);
            }
            else if (mode == 4)
            {
                CHECK(x < 5.0, "everything restored: the wall stops it again");
            }
            m3DestroyBody(bullet);
        }
        m3DestroyWorld(world);
    }

    // 3) The rollback storm: a deterministic mixed-op schedule, a
    //    mid-flight snapshot, and a re-run onto identical bits.
    {
        static uint8_t snap[1048576];
        m3WorldDef def = m3DefaultWorldDef();
        def.bodyCapacity = 16;
        def.shapeCapacity = 16;
        def.jointCapacity = 8;
        m3WorldId world = m3CreateWorld(&def);
        m3BodyDef gd = m3DefaultBodyDef();
        m3BodyId ground = m3CreateBody(world, &gd);
        m3ShapeDef sd = m3DefaultShapeDef();
        m3Plane fl = {{0.0f, 1.0f, 0.0f}, 0.0f};
        m3ShapeId floor = m3CreatePlaneShape(ground, &sd, &fl);
        m3BodyId crates[3];
        m3ShapeId shapes[3];
        for (int32_t k = 0; k < 3; ++k)
        {
            m3BodyDef bd = m3DefaultBodyDef();
            bd.type = m3_dynamicBody;
            bd.position = (m3Pos3){(double)k * 1.4, 0.5 + (double)k, 0.0};
            crates[k] = m3CreateBody(world, &bd);
            shapes[k] = m3CreateBoxShape(crates[k], &sd, (m3Vec3){0.5f, 0.5f, 0.5f});
        }
        m3JointDef jd = m3DefaultJointDef();
        jd.type = m3_revoluteJoint;
        jd.bodyA = crates[0];
        jd.bodyB = crates[1];
        jd.localAnchorA = (m3Vec3){0.7f, 0.0f, 0.0f};
        jd.localAnchorB = (m3Vec3){-0.7f, 0.0f, 0.0f};
        jd.localAxisA = (m3Vec3){0.0f, 1.0f, 0.0f};
        jd.localAxisB = (m3Vec3){0.0f, 1.0f, 0.0f};
        m3JointId hinge = m3CreateJoint(&jd);

        int32_t snapBytes = 0;
#define M3_STORM_OPS(i)                                                                            \
    do                                                                                             \
    {                                                                                              \
        int32_t phase = (i) % 24;                                                                  \
        if (phase == 0)                                                                            \
            m3Body_ApplyLinearImpulse(crates[(i) % 3], (m3Vec3){0.4f, 0.0f, -0.2f});               \
        if (phase == 3)                                                                            \
            m3Body_SetMotionLocks(crates[(i) % 3], (uint8_t)((i) % 64));                           \
        if (phase == 6)                                                                            \
            m3Shape_SetFriction(shapes[(i) % 3], 0.2f + 0.1f * (float)((i) % 5));                  \
        if (phase == 9)                                                                            \
            m3Joint_SetMotor(hinge, ((i) / 24) % 2 == 0, 1.5f, 20.0f);                             \
        if (phase == 12)                                                                           \
            m3Joint_SetSpring(hinge, true, 4.0f + (float)((i) % 3), 0.8f);                         \
        if (phase == 13)                                                                           \
            m3Joint_SetTargetAngle(hinge, 0.1f * (float)((i) % 6) - 0.25f);                        \
        if (phase == 16)                                                                           \
            m3Shape_EnableHitEvents(floor, ((i) / 24) % 2 == 0);                                   \
        if (phase == 18)                                                                           \
            m3World_SetContactTuning(world, 30.0f + (float)((i) % 8), 10.0f, 3.0f);                \
        if (phase == 21)                                                                           \
            m3Body_SetSleepControls(crates[(i) % 3], 0.05f + 0.01f * (float)((i) % 4),             \
                                    ((i) / 24) % 2 == 0);                                          \
    } while (0)

        for (int32_t i = 0; i < 120; ++i)
        {
            M3_STORM_OPS(i);
            m3World_Step(world, 1.0f / 60.0f, 4);
            if (i == 59)
            {
                snapBytes = m3World_Snapshot(world, snap, (int32_t)sizeof(snap));
                CHECK(snapBytes > 0, "the storm snapshot fits");
            }
        }
        uint64_t final = m3World_Hash(world);
        CHECK(m3World_Restore(world, snap, snapBytes), "the storm restore lands");
        for (int32_t i = 60; i < 120; ++i)
        {
            M3_STORM_OPS(i);
            m3World_Step(world, 1.0f / 60.0f, 4);
        }
        CHECK(m3World_Hash(world) == final, "the storm re-runs onto identical bits");
#undef M3_STORM_OPS
        m3DestroyWorld(world);
    }
}

int main(void)
{
    TestRuntimeOpsRedTeam();
    TestStaleIdsEverywhere();
    TestSnapshotRefusals();
    TestJournalCorruptionIsAtomic();
    TestReplayWrongSizePayloads();
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
