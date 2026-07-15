// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The angular cap gate (13-1): the reference spin clamp with the
// allowFastRotation escape hatch. A legal tumbler keeps its spin, a
// pathological spinner clamps to the cap, a flagged wheel sails past
// it, the cap tunes at runtime through the journaled setter, the
// flag survives motion-lock writes and rollback, twins and replays
// land on identical bits, and hostile tape bytes bounce loudly.

#include "maul3d/body.h"
#include "maul3d/shape.h"

#include <math.h>
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

// Zero gravity, no contacts: spin evolves through damping, gyro,
// and the cap alone. Cubes have isotropic inertia, so a clean spin
// about any axis is gyroscopically silent.
static m3WorldId SpinWorld(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    return m3CreateWorld(&def);
}

static m3BodyId Spinner(m3WorldId world, double x, float spinY)
{
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){x, 0.0, 0.0};
    bd.angularVelocity = (m3Vec3){0.0f, spinY, 0.0f};
    m3BodyId body = m3CreateBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3CreateBoxShape(body, &sd, (m3Vec3){0.5f, 0.5f, 0.5f});
    return body;
}

static float SpinOf(m3BodyId body)
{
    m3Vec3 w = m3Body_GetAngularVelocity(body);
    return sqrtf(w.x * w.x + w.y * w.y + w.z * w.z);
}

static void TestCapAndEscape(void)
{
    m3WorldId world = SpinWorld();
    m3BodyId legal = Spinner(world, -4.0, 50.0f);
    m3BodyId wild = Spinner(world, 0.0, 2000.0f);
    m3BodyId wheel = Spinner(world, 4.0, 2000.0f);
    m3Body_SetAllowFastRotation(wheel, true);
    CHECK(m3Body_GetAllowFastRotation(wheel), "the flag reads back");
    CHECK(!m3Body_GetAllowFastRotation(wild), "the default is off");
    CHECK(m3Body_GetMotionLocks(wheel) == 0u, "the flag is not a motion lock");
    for (int32_t i = 0; i < 60; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(SpinOf(legal) > 49.9f, "a legal tumbler keeps its spin");
    CHECK(SpinOf(wild) < 800.5f, "the wild spinner is capped");
    CHECK(SpinOf(wild) > 799.0f, "the cap clamps, it does not kill");
    CHECK(SpinOf(wheel) > 1999.0f, "the flagged wheel sails past the cap");
    m3DestroyWorld(world);
}

static void TestRuntimeCapAndLockInterplay(void)
{
    m3WorldId world = SpinWorld();
    m3BodyId top = Spinner(world, 0.0, 50.0f);
    m3World_SetMaximumAngularSpeed(world, 10.0f);
    // Hostile knob values bounce at the public wall and change
    // nothing: the spinner still lands on the 10 rad/s cap.
    float bad;
    uint32_t nanBits = 0x7FC00000u;
    memcpy(&bad, &nanBits, sizeof(bad));
    m3World_SetMaximumAngularSpeed(world, bad);
    m3World_SetMaximumAngularSpeed(world, -5.0f);
    m3World_SetMaximumAngularSpeed(world, 0.0f);
    for (int32_t i = 0; i < 10; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(SpinOf(top) < 10.5f && SpinOf(top) > 9.5f, "the runtime cap applies");
    // The flag and the locks share a byte but never each other's
    // bits: lock writes preserve the flag, the flag write preserves
    // the locks, and the locks getter never leaks bit 6.
    m3Body_SetAllowFastRotation(top, true);
    m3Body_SetMotionLocks(top, 0x3Fu);
    CHECK(m3Body_GetAllowFastRotation(top), "locks do not clobber the flag");
    CHECK(m3Body_GetMotionLocks(top) == 0x3Fu, "the locks read back clean");
    m3Body_SetMotionLocks(top, 0u);
    CHECK(m3Body_GetAllowFastRotation(top), "clearing locks keeps the flag");
    m3Body_SetAllowFastRotation(top, false);
    CHECK(!m3Body_GetAllowFastRotation(top), "the flag clears");
    CHECK(m3Body_GetMotionLocks(top) == 0u, "the flag write left the locks alone");
    m3BodyId stale = {99, top.world0, 7};
    m3Body_SetAllowFastRotation(stale, true);
    CHECK(!m3Body_GetAllowFastRotation(stale), "a stale id bounces");
    m3DestroyWorld(world);
}

static void TestHashFoldsOffDefaultOnly(void)
{
    m3WorldId a = SpinWorld();
    m3WorldId b = SpinWorld();
    CHECK(m3World_Hash(a) == m3World_Hash(b), "twin worlds start identical");
    m3World_SetMaximumAngularSpeed(b, 799.0f);
    CHECK(m3World_Hash(a) != m3World_Hash(b), "an off-default cap folds");
    m3World_SetMaximumAngularSpeed(b, 800.0f);
    CHECK(m3World_Hash(a) == m3World_Hash(b), "the exact default does not fold");
    m3DestroyWorld(a);
    m3DestroyWorld(b);
}

static void TestTwinsAndReplay(void)
{
    static uint8_t journal[65536];
    uint64_t hashes[2];
    for (int32_t run = 0; run < 2; ++run)
    {
        m3WorldId world = SpinWorld();
        bool recording =
            run == 0 && m3World_JournalBegin(world, journal, (int32_t)sizeof(journal));
        m3BodyId rotor = Spinner(world, -3.0, 2000.0f);
        m3BodyId flywheel = Spinner(world, 3.0, 2000.0f);
        for (int32_t i = 0; i < 240; ++i)
        {
            if (i == 30)
            {
                m3Body_SetAllowFastRotation(flywheel, true);
            }
            if (i == 60)
            {
                m3World_SetMaximumAngularSpeed(world, 40.0f);
            }
            m3World_Step(world, 1.0f / 60.0f, 4);
        }
        CHECK(SpinOf(rotor) < 40.5f, "the tightened cap catches the rotor");
        // The default cap already ate the flywheel's excess in the 30
        // unflagged ticks; the flag stops FUTURE clamping (it rides
        // through the 40 rad/s tightening), it does not refund spin.
        CHECK(SpinOf(flywheel) > 799.0f, "the flagged flywheel ignores the tightening");
        hashes[run] = m3World_Hash(world);
        if (recording)
        {
            int32_t bytes = m3World_JournalEnd(world);
            CHECK(bytes > 0, "the spin session records");
            m3WorldId replayed = SpinWorld();
            CHECK(m3World_JournalReplay(replayed, journal, bytes), "the session replays");
            CHECK(m3World_Hash(replayed) == hashes[0], "the replay is bit-identical");
            m3DestroyWorld(replayed);
        }
        m3DestroyWorld(world);
    }
    CHECK(hashes[0] == hashes[1], "twin spin sessions are bit-identical");
}

static void TestRollbackAcrossTheClamp(void)
{
    static uint8_t snap[2097152];
    m3WorldId world = SpinWorld();
    m3BodyId rotor = Spinner(world, -3.0, 2000.0f);
    m3BodyId flywheel = Spinner(world, 3.0, 2000.0f);
    m3Body_SetAllowFastRotation(flywheel, true);
    m3World_SetMaximumAngularSpeed(world, 100.0f);
    int32_t snapBytes = 0;
    for (int32_t i = 0; i < 120; ++i)
    {
        if (i == 30)
        {
            snapBytes = m3World_Snapshot(world, snap, (int32_t)sizeof(snap));
            CHECK(snapBytes > 0, "the mid-spin snapshot fits");
        }
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    uint64_t final = m3World_Hash(world);
    CHECK(m3World_Restore(world, snap, snapBytes), "the mid-spin restore lands");
    CHECK(m3Body_GetAllowFastRotation(flywheel), "the flag survives the restore");
    CHECK(!m3Body_GetAllowFastRotation(rotor), "the rotor stays unflagged");
    for (int32_t i = 30; i < 120; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m3World_Hash(world) == final, "the re-run spin is bit-identical");
    m3DestroyWorld(world);
}

static void TestHostileTape(void)
{
    // The tape is [op:4][bytes:4][payload], no prologue. One op per
    // tape, then surgical byte damage: every mutation must fail the
    // replay loudly and a clean tape must land.
    static uint8_t tape[64];
    m3WorldId world = SpinWorld();
    m3BodyId body = Spinner(world, 0.0, 10.0f);
    CHECK(m3World_JournalBegin(world, tape, (int32_t)sizeof(tape)), "the tape opens");
    m3Body_SetAllowFastRotation(body, true);
    int32_t bytes = m3World_JournalEnd(world);
    CHECK(bytes == 20, "one flag op is 8 header + 12 payload");
    m3DestroyWorld(world);

    m3WorldId twin = SpinWorld();
    m3BodyId twinBody = Spinner(twin, 0.0, 10.0f);
    uint8_t hostile[64];
    memcpy(hostile, tape, (size_t)bytes);
    hostile[16] = 7; // allow must be 0 or 1
    CHECK(!m3World_JournalReplay(twin, hostile, bytes), "allow=7 bounces");
    CHECK(!m3World_JournalReplay(twin, tape, bytes - 1), "a truncated tape bounces");
    CHECK(!m3Body_GetAllowFastRotation(twinBody), "the bounced tapes changed nothing");
    CHECK(m3World_JournalReplay(twin, tape, bytes), "the clean tape lands");
    CHECK(m3Body_GetAllowFastRotation(twinBody), "the replayed flag landed on the twin body");
    m3DestroyWorld(twin);

    m3WorldId world2 = SpinWorld();
    CHECK(m3World_JournalBegin(world2, tape, (int32_t)sizeof(tape)), "the cap tape opens");
    m3World_SetMaximumAngularSpeed(world2, 50.0f);
    bytes = m3World_JournalEnd(world2);
    CHECK(bytes == 12, "one cap op is 8 header + 4 payload");
    m3DestroyWorld(world2);

    m3WorldId twin2 = SpinWorld();
    memcpy(hostile, tape, (size_t)bytes);
    uint32_t nanBits = 0x7FC00000u;
    memcpy(hostile + 8, &nanBits, 4);
    CHECK(!m3World_JournalReplay(twin2, hostile, bytes), "a NaN cap bounces");
    float negative = -5.0f;
    memcpy(hostile + 8, &negative, 4);
    CHECK(!m3World_JournalReplay(twin2, hostile, bytes), "a negative cap bounces");
    CHECK(m3World_JournalReplay(twin2, tape, bytes), "the clean cap tape lands");
    m3DestroyWorld(twin2);
}

int main(void)
{
    TestCapAndEscape();
    TestRuntimeCapAndLockInterplay();
    TestHashFoldsOffDefaultOnly();
    TestTwinsAndReplay();
    TestRollbackAcrossTheClamp();
    TestHostileTape();
    if (s_failures == 0)
    {
        printf("test_angularcap: all passed\n");
        return 0;
    }
    return 1;
}
