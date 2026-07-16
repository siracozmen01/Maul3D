// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The container gate (9-1): an M3J1 session round-trips in pure
// memory onto the recorder's hash, the header's counts match the
// stream, and every kind of corruption refuses loudly without
// touching a world.

#include "maul3d/character.h"
#include "maul3d/joint.h"
#include "maul3d/replay.h"
#include "maul3d/shape.h"
#include "maul3d/softbody.h"
#include "maul3d/vehicle.h"

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

static m3WorldDef Def(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    return def;
}

// Record a small session; returns malloc'd container + its size.
static uint8_t* RecordSession(int32_t* outBytes, uint64_t* outHash, int32_t* outOps,
                              int32_t* outSteps)
{
    m3WorldDef def = Def();
    m3WorldId world = m3CreateWorld(&def);
    int32_t snapBytes = m3World_SnapshotSize(world);
    uint8_t* snap = (uint8_t*)malloc((size_t)snapBytes);
    m3World_Snapshot(world, snap, snapBytes);

    static uint8_t journal[131072];
    m3World_JournalBegin(world, journal, (int32_t)sizeof(journal));
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane fl = {{0.0f, 1.0f, 0.0f}, 0.0f};
    m3CreatePlaneShape(ground, &sd, &fl);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 3.0, 0.0};
    m3BodyId crate = m3CreateBody(world, &bd);
    m3CreateBoxShape(crate, &sd, (m3Vec3){0.5f, 0.5f, 0.5f});
    for (int32_t i = 0; i < 90; ++i)
    {
        if (i == 30)
        {
            m3Body_ApplyLinearImpulse(crate, (m3Vec3){1.5f, 0.0f, 0.0f});
        }
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    int32_t journalBytes = m3World_JournalEnd(world);
    uint64_t final = m3World_Hash(world);

    // 4 creates + 1 impulse + 90 steps = 95 records.
    *outOps = 95;
    *outSteps = 90;
    int32_t need = m3ReplayEncodeSize(snapBytes, journalBytes);
    uint8_t* blob = (uint8_t*)malloc((size_t)need);
    int32_t wrote = m3ReplayEncode(snap, snapBytes, journal, journalBytes, final, blob, need);
    CHECK(wrote == need, "the encode fills its own quoted size");
    *outBytes = wrote;
    *outHash = final;
    free(snap);
    m3DestroyWorld(world);
    return blob;
}

static void TestRoundTrip(void)
{
    int32_t bytes;
    uint64_t recorded;
    int32_t ops;
    int32_t steps;
    uint8_t* blob = RecordSession(&bytes, &recorded, &ops, &steps);

    m3ReplayView view;
    CHECK(m3ReplayDecode(blob, bytes, &view), "the container decodes");
    CHECK(view.info.opCount == ops, "the op count matches the script");
    CHECK(view.info.stepCount == steps, "the step count matches the script");
    CHECK(view.finalHash == recorded, "the header carries the recorder's hash");

    m3WorldDef def = Def();
    m3WorldId world = m3CreateWorld(&def);
    CHECK(m3World_Restore(world, view.snapshot, view.snapshotBytes),
          "the embedded snapshot restores");
    CHECK(m3World_JournalReplay(world, view.journal, view.journalBytes),
          "the embedded journal replays");
    CHECK(m3World_Hash(world) == recorded, "the round trip lands on the recorder's hash");
    m3DestroyWorld(world);
    free(blob);
}

static void TestRefusals(void)
{
    int32_t bytes;
    uint64_t recorded;
    int32_t ops;
    int32_t steps;
    uint8_t* blob = RecordSession(&bytes, &recorded, &ops, &steps);
    m3ReplayView view;

    // Bad magic.
    uint8_t* bad = (uint8_t*)malloc((size_t)bytes);
    memcpy(bad, blob, (size_t)bytes);
    bad[0] ^= 0xFF;
    CHECK(!m3ReplayDecode(bad, bytes, &view), "bad magic refuses");

    // Truncation, three depths.
    memcpy(bad, blob, (size_t)bytes);
    CHECK(!m3ReplayDecode(bad, bytes - 1, &view), "truncated tail refuses");
    CHECK(!m3ReplayDecode(bad, 16, &view), "truncated header refuses");
    CHECK(!m3ReplayDecode(bad, 0, &view), "empty refuses");

    // A corrupted record length inside the journal.
    memcpy(bad, blob, (size_t)bytes);
    m3ReplayDecode(blob, bytes, &view);
    int32_t journalOffset = (int32_t)((const uint8_t*)view.journal - blob);
    int32_t huge = 1 << 30;
    memcpy(bad + journalOffset + 4, &huge, 4); // first record's length
    CHECK(!m3ReplayDecode(bad, bytes, &view), "a corrupt record length refuses");

    // A header that lies about its counts.
    memcpy(bad, blob, (size_t)bytes);
    int32_t lie = 9999;
    memcpy(bad + 16, &lie, 4);
    CHECK(!m3ReplayDecode(bad, bytes, &view), "a lying step count refuses");

    // Describe on garbage.
    m3JournalInfo info;
    uint8_t garbage[16] = {7, 0, 0, 0, 99, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8};
    CHECK(!m3JournalDescribe(garbage, (int32_t)sizeof(garbage), &info),
          "an overlong payload refuses");
    CHECK(m3JournalDescribe(NULL, 0, &info) == false, "null journal refuses");
    CHECK(m3ReplayEncode(blob, 10, NULL, 4, 0, bad, bytes) == 0,
          "a null journal with bytes refuses");
    CHECK(m3ReplayEncode(NULL, 10, NULL, 0, 0, bad, bytes) == 0, "a null snapshot refuses");
    CHECK(m3ReplayEncodeSize(-5, 10) == 0, "negative sizes refuse");

    free(bad);
    free(blob);
}

static void TestSmallCapacity(void)
{
    int32_t bytes;
    uint64_t recorded;
    int32_t ops;
    int32_t steps;
    uint8_t* blob = RecordSession(&bytes, &recorded, &ops, &steps);
    m3ReplayView view;
    CHECK(m3ReplayDecode(blob, bytes, &view), "decode for the capacity test");
    uint8_t* out = (uint8_t*)malloc((size_t)bytes);
    CHECK(m3ReplayEncode(view.snapshot, view.snapshotBytes, view.journal, view.journalBytes,
                         view.finalHash, out, bytes - 1) == 0,
          "one byte short refuses");
    free(out);
    free(blob);
}

static void TestFuzz(void)
{
    // The 9-5 fuzz law: 200 LCG bit-mutations of a valid container
    // must never crash. Decode either refuses or yields a view; a
    // view that decodes must survive restore + replay attempts on a
    // fresh world (the atomic-replay guarantee backs out partial
    // application). The sanitizer cell turns any slip into a
    // failure.
    int32_t bytes;
    uint64_t recorded;
    int32_t ops;
    int32_t steps;
    uint8_t* blob = RecordSession(&bytes, &recorded, &ops, &steps);
    uint8_t* mutant = (uint8_t*)malloc((size_t)bytes);
    // The container is snapshot-dominated, so pure-random offsets
    // almost never touch the 32-byte header or the journal tail
    // (the first draft proved it: 200 draws, zero framing hits).
    // Half the mutations aim at the framing on purpose; the other
    // half roam the whole container.
    m3ReplayView valid;
    CHECK(m3ReplayDecode(blob, bytes, &valid), "decode for the fuzz split");
    int32_t journalStart = (int32_t)((const uint8_t*)valid.journal - blob);
    uint32_t rng = 87654321u;
    int32_t refused = 0;
    int32_t survived = 0;
    for (int32_t t = 0; t < 200; ++t)
    {
        memcpy(mutant, blob, (size_t)bytes);
        rng = rng * 1664525u + 1013904223u;
        int32_t where;
        if (t % 2 == 0)
        {
            int32_t zone = 32 + (bytes - journalStart);
            int32_t pick = (int32_t)(rng % (uint32_t)zone);
            where = pick < 32 ? pick : journalStart + (pick - 32);
        }
        else
        {
            where = (int32_t)(rng % (uint32_t)bytes);
        }
        rng = rng * 1664525u + 1013904223u;
        mutant[where] ^= (uint8_t)(1u << (rng % 8));
        m3ReplayView view;
        if (!m3ReplayDecode(mutant, bytes, &view))
        {
            refused += 1;
            continue;
        }
        m3WorldDef def = Def();
        m3WorldId world = m3CreateWorld(&def);
        if (m3World_Restore(world, view.snapshot, view.snapshotBytes))
        {
            m3World_JournalReplay(world, view.journal, view.journalBytes);
        }
        m3DestroyWorld(world);
        survived += 1;
    }
    // Every mutation lands somewhere; the split just has to add up
    // and nothing above may crash or trip the sanitizers.
    CHECK(refused + survived == 200, "every mutant either refused or survived");
    CHECK(refused > 0, "at least some mutations hit the framing");
    free(mutant);
    free(blob);
}

static void TestFuzzPhase12Ops(void)
{
    // The 12-4 red team: a session DENSE in the phase 12 ops (62
    // drivetrain def, 63 gear, 64 stance, plus a wheel joint and
    // its runtime control) so the mutation storm actually lands on
    // their payloads. Every mutation aims at the journal region on
    // purpose. The law is the 9-5 law: never crash, decode refuses
    // or the atomic replay survives; the sanitizer cell converts
    // any slip into a failure.
    m3WorldDef def = Def();
    def.vehicleCapacity = 2;
    def.characterCapacity = 2;
    def.jointCapacity = 8;
    m3WorldId world = m3CreateWorld(&def);
    int32_t snapBytes = m3World_SnapshotSize(world);
    uint8_t* snap = (uint8_t*)malloc((size_t)snapBytes);
    m3World_Snapshot(world, snap, snapBytes);
    static uint8_t journal[131072];
    m3World_JournalBegin(world, journal, (int32_t)sizeof(journal));

    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane fl = {{0.0f, 1.0f, 0.0f}, 0.0f};
    m3CreatePlaneShape(ground, &sd, &fl);

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 0.7, 0.0};
    m3BodyId chassis = m3CreateBody(world, &bd);
    sd.density = 300.0f;
    m3CreateBoxShape(chassis, &sd, (m3Vec3){1.0f, 0.25f, 0.5f});
    m3VehicleDef vd = m3DefaultVehicleDef();
    vd.chassis = chassis;
    vd.wheelCount = 4;
    for (int32_t w = 0; w < 4; ++w)
    {
        vd.wheels[w].anchor =
            (m3Vec3){(w & 1) != 0 ? 0.8f : -0.8f, -0.25f, (w & 2) != 0 ? 0.45f : -0.45f};
        vd.wheels[w].driven = true;
    }
    m3VehicleId car = m3CreateVehicle(world, &vd);
    m3DrivetrainDef dt = m3DefaultDrivetrainDef();
    dt.autoShift = false;
    m3Vehicle_SetDrivetrain(car, &dt);

    bd.position = (m3Pos3){4.0, 0.7, 3.0};
    m3BodyId hub = m3CreateBody(world, &bd);
    sd.density = 100.0f;
    m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.3f};
    m3CreateSphereShape(hub, &sd, &ball);
    bd.position = (m3Pos3){4.0, 0.3, 3.0};
    m3BodyId rim = m3CreateBody(world, &bd);
    m3CreateSphereShape(rim, &sd, &ball);
    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_wheelJoint;
    jd.bodyA = hub;
    jd.bodyB = rim;
    jd.localAxisA = (m3Vec3){0.0f, -1.0f, 0.0f};
    jd.localAxisB = (m3Vec3){0.0f, 0.0f, 1.0f};
    m3JointId axle = m3CreateJoint(&jd);
    m3Joint_SetMotor(axle, true, -15.0f, 40.0f);
    m3Joint_SetBreakThresholds(axle, 0.0f, 30.0f);

    m3CharacterDef cd = m3DefaultCharacterDef();
    cd.position = (m3Pos3){-4.0, 1.0, 0.0};
    m3CharacterId hero = m3CreateCharacter(world, &cd);

    for (int32_t i = 0; i < 120; ++i)
    {
        if (i == 10)
        {
            m3Vehicle_SetCommands(car, 1.0f, 0.0f, 0.0f);
        }
        if (i % 20 == 5)
        {
            m3Vehicle_SelectGear(car, 1 + (i / 20) % 5); // op 63 spray
        }
        if (i % 15 == 7)
        {
            m3Character_SetStance(hero, i % 30 == 7 ? 0.2f : 0.5f, 0.4f); // op 64 spray
        }
        m3Character_Move(hero, (m3Vec3){0.02f, -0.1f, 0.0f});
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    int32_t journalBytes = m3World_JournalEnd(world);
    uint64_t final = m3World_Hash(world);
    int32_t need = m3ReplayEncodeSize(snapBytes, journalBytes);
    uint8_t* blob = (uint8_t*)malloc((size_t)need);
    CHECK(m3ReplayEncode(snap, snapBytes, journal, journalBytes, final, blob, need) == need,
          "the phase 12 session encodes");
    m3DestroyWorld(world);

    m3ReplayView valid;
    CHECK(m3ReplayDecode(blob, need, &valid), "the phase 12 session decodes");
    int32_t journalStart = (int32_t)((const uint8_t*)valid.journal - blob);
    uint8_t* mutant = (uint8_t*)malloc((size_t)need);
    uint32_t rng = 246813579u;
    int32_t refused = 0;
    int32_t survived = 0;
    for (int32_t t = 0; t < 300; ++t)
    {
        memcpy(mutant, blob, (size_t)need);
        rng = rng * 1664525u + 1013904223u;
        int32_t where = journalStart + (int32_t)(rng % (uint32_t)(need - journalStart));
        rng = rng * 1664525u + 1013904223u;
        mutant[where] ^= (uint8_t)(1u << (rng % 8));
        m3ReplayView view;
        if (!m3ReplayDecode(mutant, need, &view))
        {
            refused += 1;
            continue;
        }
        m3WorldDef fresh = Def();
        fresh.vehicleCapacity = 2;
        fresh.characterCapacity = 2;
        fresh.jointCapacity = 8;
        m3WorldId probe = m3CreateWorld(&fresh);
        if (m3World_Restore(probe, view.snapshot, view.snapshotBytes))
        {
            m3World_JournalReplay(probe, view.journal, view.journalBytes);
        }
        m3DestroyWorld(probe);
        survived += 1;
    }
    CHECK(refused + survived == 300, "every phase 12 mutant either refused or survived");
    free(mutant);
    free(blob);
    free(snap);
}

static void TestFuzzPhase13Ops(void)
{
    // The 13-4 red team: a session DENSE in the phase 13 ops (65
    // allowFastRotation, 66 angular cap, 67 explode with carve and
    // soft push) so the mutation storm lands on their payloads.
    // Every mutation aims at the journal region on purpose; the 9-5
    // law holds: never crash, decode refuses or the atomic replay
    // survives, sanitizers convert any slip into a failure.
    m3WorldDef def = Def();
    def.bodyCapacity = 32;
    def.shapeCapacity = 32;
    def.voxelCapacity = 2;
    def.softBodyCapacity = 2;
    m3WorldId world = m3CreateWorld(&def);
    int32_t snapBytes = m3World_SnapshotSize(world);
    uint8_t* snap = (uint8_t*)malloc((size_t)snapBytes);
    m3World_Snapshot(world, snap, snapBytes);
    static uint8_t journal[131072];
    m3World_JournalBegin(world, journal, (int32_t)sizeof(journal));

    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane fl = {{0.0f, 1.0f, 0.0f}, 0.0f};
    m3CreatePlaneShape(ground, &sd, &fl);

    static uint8_t voxels[16 * 16 * 16];
    memset(voxels, 0, sizeof(voxels));
    for (int32_t y = 0; y < 12; ++y)
    {
        voxels[8 + 16 * (y + 16 * 8)] = 1;
        voxels[7 + 16 * (y + 16 * 8)] = 1;
    }
    m3BodyDef cd = m3DefaultBodyDef();
    cd.position = (m3Pos3){-8.0, 0.0, -8.0};
    m3BodyId keep = m3CreateBody(world, &cd);
    m3CreateVoxelChunkShape(keep, &sd, voxels, NULL, 1.0f);

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){2.0, 1.0, 2.0};
    bd.angularVelocity = (m3Vec3){0.0f, 900.0f, 0.0f};
    m3BodyId spinner = m3CreateBody(world, &bd);
    m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.4f};
    m3CreateSphereShape(spinner, &sd, &ball);
    for (int32_t i = 0; i < 4; ++i)
    {
        m3BodyDef crate = m3DefaultBodyDef();
        crate.type = m3_dynamicBody;
        crate.position = (m3Pos3){-2.0 + (double)i, 0.5, 3.0};
        m3CreateBoxShape(m3CreateBody(world, &crate), &sd, (m3Vec3){0.4f, 0.4f, 0.4f});
    }
    m3SoftBodyDef sbd = m3DefaultSoftBodyDef();
    sbd.position = (m3Pos3){4.0, 1.0, -2.0};
    sbd.countX = 2;
    sbd.countY = 2;
    sbd.countZ = 2;
    sbd.spacing = 0.6f;
    m3CreateSoftBody(world, &sbd);

    for (int32_t i = 0; i < 120; ++i)
    {
        if (i % 10 == 3)
        {
            m3Body_SetAllowFastRotation(spinner, (i / 10) % 2 == 0); // op 65 spray
        }
        if (i % 25 == 7)
        {
            m3World_SetMaximumAngularSpeed(world, i % 50 == 7 ? 40.0f : 200.0f); // op 66
        }
        if (i % 30 == 11)
        {
            m3ExplosionDef boom = m3DefaultExplosionDef();
            boom.position = (m3Pos3){0.5 * (double)(i % 8), 4.0, 0.0};
            boom.radius = 4.0f;
            boom.falloff = 2.0f;
            boom.impulsePerArea = (i % 60 == 11) ? 3.0f : -2.0f;
            boom.voxelCarve = 1.5f;
            m3World_Explode(world, &boom); // op 67 spray
        }
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    int32_t journalBytes = m3World_JournalEnd(world);
    uint64_t final = m3World_Hash(world);
    int32_t need = m3ReplayEncodeSize(snapBytes, journalBytes);
    uint8_t* blob = (uint8_t*)malloc((size_t)need);
    CHECK(m3ReplayEncode(snap, snapBytes, journal, journalBytes, final, blob, need) == need,
          "the phase 13 session encodes");
    m3DestroyWorld(world);

    m3ReplayView valid;
    CHECK(m3ReplayDecode(blob, need, &valid), "the phase 13 session decodes");
    int32_t journalStart = (int32_t)((const uint8_t*)valid.journal - blob);
    uint8_t* mutant = (uint8_t*)malloc((size_t)need);
    uint32_t rng = 135792468u;
    int32_t refused = 0;
    int32_t survived = 0;
    for (int32_t t = 0; t < 300; ++t)
    {
        memcpy(mutant, blob, (size_t)need);
        rng = rng * 1664525u + 1013904223u;
        int32_t where = journalStart + (int32_t)(rng % (uint32_t)(need - journalStart));
        rng = rng * 1664525u + 1013904223u;
        mutant[where] ^= (uint8_t)(1u << (rng % 8));
        m3ReplayView view;
        if (!m3ReplayDecode(mutant, need, &view))
        {
            refused += 1;
            continue;
        }
        m3WorldDef fresh = Def();
        fresh.bodyCapacity = 32;
        fresh.shapeCapacity = 32;
        fresh.voxelCapacity = 2;
        fresh.softBodyCapacity = 2;
        m3WorldId probe = m3CreateWorld(&fresh);
        if (m3World_Restore(probe, view.snapshot, view.snapshotBytes))
        {
            m3World_JournalReplay(probe, view.journal, view.journalBytes);
        }
        m3DestroyWorld(probe);
        survived += 1;
    }
    CHECK(refused + survived == 300, "every phase 13 mutant either refused or survived");
    free(mutant);
    free(blob);
    free(snap);
}

static void TestFuzzPhase15Ops(void)
{
    // The 15-4 red team: a session dense in op 69 (geometry swaps)
    // over a rolling cylinder (op 7 recipe) and a morphing sphere,
    // then 300 journal-aimed mutations under the 9-5 law.
    m3WorldDef def = Def();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    m3WorldId world = m3CreateWorld(&def);
    int32_t snapBytes = m3World_SnapshotSize(world);
    uint8_t* snap = (uint8_t*)malloc((size_t)snapBytes);
    m3World_Snapshot(world, snap, snapBytes);
    static uint8_t journal[131072];
    m3World_JournalBegin(world, journal, (int32_t)sizeof(journal));

    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane fl = {{0.0f, 1.0f, 0.0f}, 0.0f};
    m3CreatePlaneShape(ground, &sd, &fl);

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 0.6, 0.0};
    bd.angularVelocity = (m3Vec3){0.0f, 0.0f, -6.0f};
    m3BodyId roller = m3CreateBody(world, &bd);
    m3Cylinder keg = {{0.0f, 0.0f, -0.4f}, {0.0f, 0.0f, 0.4f}, 0.5f};
    m3CreateCylinderShape(roller, &sd, &keg, 16);

    bd.position = (m3Pos3){3.0, 1.0, 0.0};
    bd.angularVelocity = (m3Vec3){0.0f, 0.0f, 0.0f};
    m3BodyId morpher = m3CreateBody(world, &bd);
    m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.5f};
    m3ShapeId orb = m3CreateSphereShape(morpher, &sd, &ball);

    for (int32_t i = 0; i < 120; ++i)
    {
        if (i % 20 == 5)
        {
            if ((i / 20) % 2 == 0)
            {
                m3Capsule pill = {{0.0f, -0.3f, 0.0f}, {0.0f, 0.3f, 0.0f}, 0.3f};
                m3Shape_SetCapsule(orb, &pill); // op 69 spray
            }
            else
            {
                m3Shape_SetSphere(orb, &ball); // op 69 spray
            }
        }
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    int32_t journalBytes = m3World_JournalEnd(world);
    uint64_t final = m3World_Hash(world);
    int32_t need = m3ReplayEncodeSize(snapBytes, journalBytes);
    uint8_t* blob = (uint8_t*)malloc((size_t)need);
    CHECK(m3ReplayEncode(snap, snapBytes, journal, journalBytes, final, blob, need) == need,
          "the phase 15 session encodes");
    m3DestroyWorld(world);

    m3ReplayView valid;
    CHECK(m3ReplayDecode(blob, need, &valid), "the phase 15 session decodes");
    int32_t journalStart = (int32_t)((const uint8_t*)valid.journal - blob);
    uint8_t* mutant = (uint8_t*)malloc((size_t)need);
    uint32_t rng = 468135792u;
    int32_t refused = 0;
    int32_t survived = 0;
    for (int32_t t = 0; t < 300; ++t)
    {
        memcpy(mutant, blob, (size_t)need);
        rng = rng * 1664525u + 1013904223u;
        int32_t where = journalStart + (int32_t)(rng % (uint32_t)(need - journalStart));
        rng = rng * 1664525u + 1013904223u;
        mutant[where] ^= (uint8_t)(1u << (rng % 8));
        m3ReplayView view;
        if (!m3ReplayDecode(mutant, need, &view))
        {
            refused += 1;
            continue;
        }
        m3WorldDef fresh = Def();
        fresh.bodyCapacity = 16;
        fresh.shapeCapacity = 16;
        m3WorldId probe = m3CreateWorld(&fresh);
        if (m3World_Restore(probe, view.snapshot, view.snapshotBytes))
        {
            m3World_JournalReplay(probe, view.journal, view.journalBytes);
        }
        m3DestroyWorld(probe);
        survived += 1;
    }
    CHECK(refused + survived == 300, "every phase 15 mutant either refused or survived");
    free(mutant);
    free(blob);
    free(snap);
}

static void TestFuzzPhase16Ops(void)
{
    // The 16-7 red team: a session dense in the whole phase-16
    // surface (op 70 steering, op 71 servo aims, motor budgets,
    // gear and pulley creates on top of hinges) plus the setters
    // the 16-7 wall sweep hardened (velocities, impulses), then
    // 300 journal-aimed mutations under the 9-5 law.
    m3WorldDef def = Def();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    def.jointCapacity = 8;
    m3WorldId world = m3CreateWorld(&def);
    int32_t snapBytes = m3World_SnapshotSize(world);
    uint8_t* snap = (uint8_t*)malloc((size_t)snapBytes);
    m3World_Snapshot(world, snap, snapBytes);
    static uint8_t journal[131072];
    m3World_JournalBegin(world, journal, (int32_t)sizeof(journal));

    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane fl = {{0.0f, 1.0f, 0.0f}, 0.0f};
    m3CreatePlaneShape(ground, &sd, &fl);

    // The steerable wheel (op 70 sprayer).
    m3BodyDef bd = m3DefaultBodyDef();
    bd.position = (m3Pos3){0.0, 2.0, 0.0};
    m3BodyId hub = m3CreateBody(world, &bd);
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 1.4, 0.0};
    m3BodyId rim = m3CreateBody(world, &bd);
    m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.3f};
    m3CreateSphereShape(rim, &sd, &ball);
    m3JointDef wj = m3DefaultJointDef();
    wj.type = m3_wheelJoint;
    wj.bodyA = hub;
    wj.bodyB = rim;
    wj.localAxisA = (m3Vec3){0.0f, -1.0f, 0.0f};
    wj.localAxisB = (m3Vec3){0.0f, 0.0f, 1.0f};
    m3JointId wheel = m3CreateJoint(&wj);

    // The servo weld (op 71 sprayer) with budgets (motor SetLimits).
    bd.position = (m3Pos3){2.0, 1.5, 0.0};
    m3BodyId cube = m3CreateBody(world, &bd);
    m3CreateBoxShape(cube, &sd, (m3Vec3){0.3f, 0.3f, 0.3f});
    m3JointDef mj = m3DefaultJointDef();
    mj.type = m3_motorJoint;
    mj.bodyA = hub;
    mj.bodyB = cube;
    m3JointId servo = m3CreateJoint(&mj);
    m3Joint_SetSpring(servo, true, 6.0f, 1.0f);
    m3Joint_SetLimits(servo, true, 40.0f, 5.0f);

    // The gear pair on hinges and the pulley (the new creates).
    bd.position = (m3Pos3){-2.5, 2.0, 0.0};
    m3BodyId ga = m3CreateBody(world, &bd);
    m3CreateBoxShape(ga, &sd, (m3Vec3){0.3f, 0.3f, 0.1f});
    bd.position = (m3Pos3){-1.5, 2.0, 0.0};
    m3BodyId gb = m3CreateBody(world, &bd);
    m3CreateBoxShape(gb, &sd, (m3Vec3){0.3f, 0.3f, 0.1f});
    m3JointDef hj = m3DefaultJointDef();
    hj.type = m3_revoluteJoint;
    hj.bodyA = ground;
    hj.bodyB = ga;
    hj.localAnchorA = (m3Vec3){-2.5f, 2.0f, 0.0f};
    hj.localAxisA = (m3Vec3){0.0f, 0.0f, 1.0f};
    hj.localAxisB = (m3Vec3){0.0f, 0.0f, 1.0f};
    m3CreateJoint(&hj);
    hj.bodyB = gb;
    hj.localAnchorA = (m3Vec3){-1.5f, 2.0f, 0.0f};
    m3CreateJoint(&hj);
    m3JointDef gj = m3DefaultJointDef();
    gj.type = m3_gearJoint;
    gj.bodyA = ga;
    gj.bodyB = gb;
    gj.localAxisA = (m3Vec3){0.0f, 0.0f, 1.0f};
    gj.localAxisB = (m3Vec3){0.0f, 0.0f, 1.0f};
    gj.ratio = 2.0f;
    m3CreateJoint(&gj);
    m3Body_ApplyAngularImpulse(ga, (m3Vec3){0.0f, 0.0f, 0.05f});
    bd.position = (m3Pos3){4.0, 2.0, 0.0};
    m3BodyId crateA = m3CreateBody(world, &bd);
    m3CreateBoxShape(crateA, &sd, (m3Vec3){0.4f, 0.4f, 0.4f});
    bd.position = (m3Pos3){6.0, 2.0, 0.0};
    m3BodyId crateB = m3CreateBody(world, &bd);
    m3CreateBoxShape(crateB, &sd, (m3Vec3){0.2f, 0.2f, 0.2f});
    m3JointDef pj = m3DefaultJointDef();
    pj.type = m3_pulleyJoint;
    pj.bodyA = crateA;
    pj.bodyB = crateB;
    pj.groundAnchorA = (m3Pos3){4.0, 4.5, 0.0};
    pj.groundAnchorB = (m3Pos3){6.0, 4.5, 0.0};
    pj.ratio = 1.5f;
    m3CreateJoint(&pj);

    for (int32_t i = 0; i < 120; ++i)
    {
        if (i % 24 == 6)
        {
            float phase = (float)((i / 24) % 3) - 1.0f;
            m3Joint_SetSteer(wheel, true, 0.4f * phase, 8.0f, 1.0f, 0.0f); // op 70
            m3Quat aim = {0.0f, 0.19f * phase, 0.0f, 0.981f};
            m3Joint_SetMotorPose(servo, (m3Vec3){0.3f * phase, -0.5f, 2.0f}, aim); // op 71
            m3Body_SetLinearVelocity(crateB, (m3Vec3){0.0f, 0.5f * phase, 0.0f});
        }
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    int32_t journalBytes = m3World_JournalEnd(world);
    uint64_t final = m3World_Hash(world);
    int32_t need = m3ReplayEncodeSize(snapBytes, journalBytes);
    uint8_t* blob = (uint8_t*)malloc((size_t)need);
    CHECK(m3ReplayEncode(snap, snapBytes, journal, journalBytes, final, blob, need) == need,
          "the phase 16 session encodes");
    m3DestroyWorld(world);

    m3ReplayView valid;
    CHECK(m3ReplayDecode(blob, need, &valid), "the phase 16 session decodes");
    int32_t journalStart = (int32_t)((const uint8_t*)valid.journal - blob);
    uint8_t* mutant = (uint8_t*)malloc((size_t)need);
    uint32_t rng = 579246813u;
    int32_t refused = 0;
    int32_t survived = 0;
    for (int32_t t = 0; t < 300; ++t)
    {
        memcpy(mutant, blob, (size_t)need);
        rng = rng * 1664525u + 1013904223u;
        int32_t where = journalStart + (int32_t)(rng % (uint32_t)(need - journalStart));
        rng = rng * 1664525u + 1013904223u;
        mutant[where] ^= (uint8_t)(1u << (rng % 8));
        m3ReplayView view;
        if (!m3ReplayDecode(mutant, need, &view))
        {
            refused += 1;
            continue;
        }
        m3WorldDef fresh = Def();
        fresh.bodyCapacity = 16;
        fresh.shapeCapacity = 16;
        fresh.jointCapacity = 8;
        m3WorldId probe = m3CreateWorld(&fresh);
        if (m3World_Restore(probe, view.snapshot, view.snapshotBytes))
        {
            m3World_JournalReplay(probe, view.journal, view.journalBytes);
        }
        m3DestroyWorld(probe);
        survived += 1;
    }
    CHECK(refused + survived == 300, "every phase 16 mutant either refused or survived");
    free(mutant);
    free(blob);
    free(snap);
}

static void TestFuzzPhase17Ops(void)
{
    // The 17-5 red team: a session dense in op 72 (material paint,
    // variable payload) and op 73 (broadphase rebuilds) over a
    // painted mesh floor with a conveyor lane, then 300
    // journal-aimed mutations under the 9-5 law.
    m3WorldDef def = Def();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    m3WorldId world = m3CreateWorld(&def);
    int32_t snapBytes = m3World_SnapshotSize(world);
    uint8_t* snap = (uint8_t*)malloc((size_t)snapBytes);
    m3World_Snapshot(world, snap, snapBytes);
    static uint8_t journal[131072];
    m3World_JournalBegin(world, journal, (int32_t)sizeof(journal));

    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    static const m3Vec3 verts[4] = {
        {-4.0f, 0.0f, -4.0f}, {4.0f, 0.0f, -4.0f}, {4.0f, 0.0f, 4.0f}, {-4.0f, 0.0f, 4.0f}};
    static const uint16_t tris[6] = {0, 2, 1, 0, 3, 2};
    m3ShapeId floor = m3CreateMeshShape(ground, &sd, verts, 4, tris, 2);

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    for (int32_t i = 0; i < 6; ++i)
    {
        bd.position = (m3Pos3){-2.0 + 0.8 * (double)i, 0.5 + 0.4 * (double)(i % 2), 0.0};
        m3BodyId crate = m3CreateBody(world, &bd);
        m3CreateBoxShape(crate, &sd, (m3Vec3){0.25f, 0.25f, 0.25f});
    }

    for (int32_t i = 0; i < 120; ++i)
    {
        if (i % 30 == 4)
        {
            m3MeshSurfaceMaterial mats[2];
            memset(mats, 0, sizeof(mats));
            mats[0].friction = 0.9f;
            mats[1].friction = 0.05f + 0.1f * (float)((i / 30) % 3);
            mats[1].surfaceVelocity = (m3Vec3){0.5f * (float)((i / 30) % 2), 0.0f, 0.0f};
            uint8_t tri[2] = {0, 1};
            m3Shape_SetMeshMaterials(floor, mats, 2, tri); // op 72 spray
        }
        if (i % 40 == 15)
        {
            m3World_RebuildBroadphase(world); // op 73 spray
        }
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    int32_t journalBytes = m3World_JournalEnd(world);
    uint64_t final = m3World_Hash(world);
    int32_t need = m3ReplayEncodeSize(snapBytes, journalBytes);
    uint8_t* blob = (uint8_t*)malloc((size_t)need);
    CHECK(m3ReplayEncode(snap, snapBytes, journal, journalBytes, final, blob, need) == need,
          "the phase 17 session encodes");
    m3DestroyWorld(world);

    m3ReplayView valid;
    CHECK(m3ReplayDecode(blob, need, &valid), "the phase 17 session decodes");
    int32_t journalStart = (int32_t)((const uint8_t*)valid.journal - blob);
    uint8_t* mutant = (uint8_t*)malloc((size_t)need);
    uint32_t rng = 691358247u;
    int32_t refused = 0;
    int32_t survived = 0;
    for (int32_t t = 0; t < 300; ++t)
    {
        memcpy(mutant, blob, (size_t)need);
        rng = rng * 1664525u + 1013904223u;
        int32_t where = journalStart + (int32_t)(rng % (uint32_t)(need - journalStart));
        rng = rng * 1664525u + 1013904223u;
        mutant[where] ^= (uint8_t)(1u << (rng % 8));
        m3ReplayView view;
        if (!m3ReplayDecode(mutant, need, &view))
        {
            refused += 1;
            continue;
        }
        m3WorldDef fresh = Def();
        fresh.bodyCapacity = 16;
        fresh.shapeCapacity = 16;
        m3WorldId probe = m3CreateWorld(&fresh);
        if (m3World_Restore(probe, view.snapshot, view.snapshotBytes))
        {
            m3World_JournalReplay(probe, view.journal, view.journalBytes);
        }
        m3DestroyWorld(probe);
        survived += 1;
    }
    CHECK(refused + survived == 300, "every phase 17 mutant either refused or survived");
    free(mutant);
    free(blob);
    free(snap);
}

static void TestFuzzPhase18Ops(void)
{
    // The 18-3 red team: a session dense in ops 74/75 (tides in
    // and out) over floating crates, then 300 journal-aimed
    // mutations under the 9-5 law.
    m3WorldDef def = Def();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    m3WorldId world = m3CreateWorld(&def);
    int32_t snapBytes = m3World_SnapshotSize(world);
    uint8_t* snap = (uint8_t*)malloc((size_t)snapBytes);
    m3World_Snapshot(world, snap, snapBytes);
    static uint8_t journal[131072];
    m3World_JournalBegin(world, journal, (int32_t)sizeof(journal));

    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane fl = {{0.0f, 1.0f, 0.0f}, 0.0f};
    m3CreatePlaneShape(ground, &sd, &fl);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    for (int32_t i = 0; i < 5; ++i)
    {
        bd.position = (m3Pos3){-2.0 + (double)i, 2.0 + 0.4 * (double)i, 0.0};
        m3BodyId crate = m3CreateBody(world, &bd);
        m3ShapeDef cs = m3DefaultShapeDef();
        cs.density = 300.0f + 400.0f * (float)(i % 3);
        m3CreateBoxShape(crate, &cs, (m3Vec3){0.25f, 0.25f, 0.25f});
    }

    m3WaterVolumeId tide = {0, 0, 0};
    for (int32_t i = 0; i < 120; ++i)
    {
        if (i % 40 == 5)
        {
            m3WaterVolumeDef wdf = m3DefaultWaterVolumeDef();
            wdf.lo = (m3Pos3){-6.0, 0.0, -6.0};
            wdf.hi = (m3Pos3){6.0, 2.5 + 0.5 * (double)((i / 40) % 2), 6.0};
            wdf.flow = (m3Vec3){0.3f * (float)((i / 40) % 3), 0.0f, 0.0f};
            tide = m3CreateWaterVolume(world, &wdf); // op 74 spray
        }
        if (i % 40 == 30 && m3WaterVolume_IsValid(tide))
        {
            m3DestroyWaterVolume(tide); // op 75 spray
        }
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    int32_t journalBytes = m3World_JournalEnd(world);
    uint64_t final = m3World_Hash(world);
    int32_t need = m3ReplayEncodeSize(snapBytes, journalBytes);
    uint8_t* blob = (uint8_t*)malloc((size_t)need);
    CHECK(m3ReplayEncode(snap, snapBytes, journal, journalBytes, final, blob, need) == need,
          "the phase 18 session encodes");
    m3DestroyWorld(world);

    m3ReplayView valid;
    CHECK(m3ReplayDecode(blob, need, &valid), "the phase 18 session decodes");
    int32_t journalStart = (int32_t)((const uint8_t*)valid.journal - blob);
    uint8_t* mutant = (uint8_t*)malloc((size_t)need);
    uint32_t rng = 802470369u;
    int32_t refused = 0;
    int32_t survived = 0;
    for (int32_t t = 0; t < 300; ++t)
    {
        memcpy(mutant, blob, (size_t)need);
        rng = rng * 1664525u + 1013904223u;
        int32_t where = journalStart + (int32_t)(rng % (uint32_t)(need - journalStart));
        rng = rng * 1664525u + 1013904223u;
        mutant[where] ^= (uint8_t)(1u << (rng % 8));
        m3ReplayView view;
        if (!m3ReplayDecode(mutant, need, &view))
        {
            refused += 1;
            continue;
        }
        m3WorldDef fresh = Def();
        fresh.bodyCapacity = 16;
        fresh.shapeCapacity = 16;
        m3WorldId probe = m3CreateWorld(&fresh);
        if (m3World_Restore(probe, view.snapshot, view.snapshotBytes))
        {
            m3World_JournalReplay(probe, view.journal, view.journalBytes);
        }
        m3DestroyWorld(probe);
        survived += 1;
    }
    CHECK(refused + survived == 300, "every phase 18 mutant either refused or survived");
    free(mutant);
    free(blob);
    free(snap);
}

static void TestFuzzPhase19Ops(void)
{
    // The 19-3 red team: a session whose floor IS a native grid
    // (op 76, variable payload) under a crate rain, then 300
    // journal-aimed mutations under the 9-5 law.
    m3WorldDef def = Def();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    m3WorldId world = m3CreateWorld(&def);
    int32_t snapBytes = m3World_SnapshotSize(world);
    uint8_t* snap = (uint8_t*)malloc((size_t)snapBytes);
    m3World_Snapshot(world, snap, snapBytes);
    static uint8_t journal[131072];
    m3World_JournalBegin(world, journal, (int32_t)sizeof(journal));

    m3BodyDef gd = m3DefaultBodyDef();
    gd.position = (m3Pos3){-4.0, 0.0, -4.0};
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    static float bumps[17 * 17];
    for (int32_t i = 0; i < 17 * 17; ++i)
    {
        bumps[i] = 0.1f * (float)((i * 11) % 5);
    }
    m3CreateHeightFieldGridShape(ground, &sd, bumps, 17, 17, 0.5f);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    for (int32_t i = 0; i < 6; ++i)
    {
        bd.position =
            (m3Pos3){-2.0 + 0.8 * (double)i, 2.0 + 0.5 * (double)(i % 2), 0.3 * (double)i};
        m3BodyId crate = m3CreateBody(world, &bd);
        m3CreateBoxShape(crate, &sd, (m3Vec3){0.25f, 0.25f, 0.25f});
    }
    for (int32_t i = 0; i < 120; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    int32_t journalBytes = m3World_JournalEnd(world);
    uint64_t final = m3World_Hash(world);
    int32_t need = m3ReplayEncodeSize(snapBytes, journalBytes);
    uint8_t* blob = (uint8_t*)malloc((size_t)need);
    CHECK(m3ReplayEncode(snap, snapBytes, journal, journalBytes, final, blob, need) == need,
          "the phase 19 session encodes");
    m3DestroyWorld(world);

    m3ReplayView valid;
    CHECK(m3ReplayDecode(blob, need, &valid), "the phase 19 session decodes");
    int32_t journalStart = (int32_t)((const uint8_t*)valid.journal - blob);
    uint8_t* mutant = (uint8_t*)malloc((size_t)need);
    uint32_t rng = 913582460u;
    int32_t refused = 0;
    int32_t survived = 0;
    for (int32_t t = 0; t < 300; ++t)
    {
        memcpy(mutant, blob, (size_t)need);
        rng = rng * 1664525u + 1013904223u;
        int32_t where = journalStart + (int32_t)(rng % (uint32_t)(need - journalStart));
        rng = rng * 1664525u + 1013904223u;
        mutant[where] ^= (uint8_t)(1u << (rng % 8));
        m3ReplayView view;
        if (!m3ReplayDecode(mutant, need, &view))
        {
            refused += 1;
            continue;
        }
        m3WorldDef fresh = Def();
        fresh.bodyCapacity = 16;
        fresh.shapeCapacity = 16;
        m3WorldId probe = m3CreateWorld(&fresh);
        if (m3World_Restore(probe, view.snapshot, view.snapshotBytes))
        {
            m3World_JournalReplay(probe, view.journal, view.journalBytes);
        }
        m3DestroyWorld(probe);
        survived += 1;
    }
    CHECK(refused + survived == 300, "every phase 19 mutant either refused or survived");
    free(mutant);
    free(blob);
    free(snap);
}

static void TestFuzzPhase20Ops(void)
{
    // The 20-5 red team: a session dense in op 77 (tet creates)
    // beside bend ropes, pressurized cubes, and tethered cloth
    // (the whole phase-20 def surface), then 300 journal-aimed
    // mutations under the 9-5 law.
    m3WorldDef def = Def();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    m3WorldId world = m3CreateWorld(&def);
    int32_t snapBytes = m3World_SnapshotSize(world);
    uint8_t* snap = (uint8_t*)malloc((size_t)snapBytes);
    m3World_Snapshot(world, snap, snapBytes);
    static uint8_t journal[131072];
    m3World_JournalBegin(world, journal, (int32_t)sizeof(journal));

    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane fl = {{0.0f, 1.0f, 0.0f}, 0.0f};
    m3CreatePlaneShape(ground, &sd, &fl);

    static const m3Vec3 pts[8] = {{0.0f, 0.0f, 0.0f}, {0.5f, 0.0f, 0.0f}, {0.5f, 0.5f, 0.0f},
                                  {0.0f, 0.5f, 0.0f}, {0.0f, 0.0f, 0.5f}, {0.5f, 0.0f, 0.5f},
                                  {0.5f, 0.5f, 0.5f}, {0.0f, 0.5f, 0.5f}};
    static const uint16_t tets[20] = {0, 1, 2, 5, 0, 2, 3, 7, 0, 5, 2, 7, 0, 5, 7, 4, 2, 5, 6, 7};
    m3SoftBodyDef sb = m3DefaultSoftBodyDef();
    sb.position = (m3Pos3){-2.0, 1.5, 0.0};
    sb.compliance = 1.0e-4f;
    m3CreateSoftBodyTet(world, &sb, pts, 8, tets, 5); // op 77

    sb = m3DefaultSoftBodyDef();
    sb.position = (m3Pos3){2.0, 2.0, 0.0};
    sb.countX = 3;
    sb.countY = 3;
    sb.countZ = 3;
    sb.pressure = 1.5f;
    sb.compliance = 1.0e-4f;
    m3CreateSoftBody(world, &sb);

    sb = m3DefaultSoftBodyDef();
    sb.position = (m3Pos3){0.0, 2.5, 2.0};
    sb.countX = 8;
    sb.countY = 1;
    sb.countZ = 1;
    sb.bendCompliance = 1.0e-6f;
    sb.maxDeviation = 0.6f;
    m3CreateSoftBody(world, &sb);

    for (int32_t i = 0; i < 120; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    int32_t journalBytes = m3World_JournalEnd(world);
    uint64_t final = m3World_Hash(world);
    int32_t need = m3ReplayEncodeSize(snapBytes, journalBytes);
    uint8_t* blob = (uint8_t*)malloc((size_t)need);
    CHECK(m3ReplayEncode(snap, snapBytes, journal, journalBytes, final, blob, need) == need,
          "the phase 20 session encodes");
    m3DestroyWorld(world);

    m3ReplayView valid;
    CHECK(m3ReplayDecode(blob, need, &valid), "the phase 20 session decodes");
    int32_t journalStart = (int32_t)((const uint8_t*)valid.journal - blob);
    uint8_t* mutant = (uint8_t*)malloc((size_t)need);
    uint32_t rng = 246813579u;
    int32_t refused = 0;
    int32_t survived = 0;
    for (int32_t t = 0; t < 300; ++t)
    {
        memcpy(mutant, blob, (size_t)need);
        rng = rng * 1664525u + 1013904223u;
        int32_t where = journalStart + (int32_t)(rng % (uint32_t)(need - journalStart));
        rng = rng * 1664525u + 1013904223u;
        mutant[where] ^= (uint8_t)(1u << (rng % 8));
        m3ReplayView view;
        if (!m3ReplayDecode(mutant, need, &view))
        {
            refused += 1;
            continue;
        }
        m3WorldDef fresh = Def();
        fresh.bodyCapacity = 16;
        fresh.shapeCapacity = 16;
        m3WorldId probe = m3CreateWorld(&fresh);
        if (m3World_Restore(probe, view.snapshot, view.snapshotBytes))
        {
            m3World_JournalReplay(probe, view.journal, view.journalBytes);
        }
        m3DestroyWorld(probe);
        survived += 1;
    }
    CHECK(refused + survived == 300, "every phase 20 mutant either refused or survived");
    free(mutant);
    free(blob);
    free(snap);
}

// R5-4 (audit B3): a session recorded WITH a vetoing pre-solve
// callback must replay to the same bits WITHOUT it: the veto
// annex (op 79) makes the tape self-sufficient.
static bool VetoAll(m3ShapeId a, m3ShapeId b, m3Pos3 point, m3Vec3 normal, void* context)
{
    (void)a;
    (void)b;
    (void)point;
    (void)normal;
    int* calls = (int*)context;
    *calls += 1;
    return false; // veto every flagged contact
}

static void TestPreSolveReplay(void)
{
    m3WorldDef def = Def();
    m3WorldId world = m3CreateWorld(&def);
    int32_t snapBytes = m3World_SnapshotSize(world);
    uint8_t* snap = (uint8_t*)malloc((size_t)snapBytes);
    m3World_Snapshot(world, snap, snapBytes);
    static uint8_t journal[262144];
    m3World_JournalBegin(world, journal, (int32_t)sizeof(journal));

    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane fl = {{0.0f, 1.0f, 0.0f}, 0.0f};
    m3ShapeId floor = m3CreatePlaneShape(ground, &sd, &fl);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 2.0, 0.0};
    m3BodyId faller = m3CreateBody(world, &bd);
    m3CreateBoxShape(faller, &sd, (m3Vec3){0.4f, 0.4f, 0.4f});
    m3Shape_EnablePreSolve(floor, true);
    int calls = 0;
    m3World_SetPreSolveCallback(world, VetoAll, &calls);
    for (int32_t i = 0; i < 120; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    int32_t journalBytes = m3World_JournalEnd(world);
    uint64_t recorded = m3World_Hash(world);
    CHECK(calls > 0, "the callback actually ran");
    CHECK(m3Body_GetPosition(faller).y < -1.0, "the veto let the box fall through the floor");
    m3DestroyWorld(world);

    // The bare replay: no callback anywhere, the annex drives.
    m3WorldId replay = m3CreateWorld(&def);
    CHECK(m3World_Restore(replay, snap, snapBytes), "the base snapshot restores");
    CHECK(m3World_JournalReplay(replay, journal, journalBytes), "the veto tape replays");
    CHECK(m3World_Hash(replay) == recorded, "a bare replay lands on the recorded bits");
    m3DestroyWorld(replay);
    free(snap);
}

int main(void)
{
    TestPreSolveReplay();
    TestRoundTrip();
    TestRefusals();
    TestSmallCapacity();
    TestFuzz();
    TestFuzzPhase12Ops();
    TestFuzzPhase13Ops();
    TestFuzzPhase15Ops();
    TestFuzzPhase16Ops();
    TestFuzzPhase17Ops();
    TestFuzzPhase18Ops();
    TestFuzzPhase19Ops();
    TestFuzzPhase20Ops();
    if (s_failures == 0)
    {
        printf("test_replayfile: all green\n");
        return 0;
    }
    printf("test_replayfile: %d failure(s)\n", s_failures);
    return 1;
}
