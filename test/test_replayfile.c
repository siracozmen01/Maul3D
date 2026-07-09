// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The container gate (9-1): an M3J1 session round-trips in pure
// memory onto the recorder's hash, the header's counts match the
// stream, and every kind of corruption refuses loudly without
// touching a world.

#include "maul3d/replay.h"
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

int main(void)
{
    TestRoundTrip();
    TestRefusals();
    TestSmallCapacity();
    TestFuzz();
    if (s_failures == 0)
    {
        printf("test_replayfile: all green\n");
        return 0;
    }
    printf("test_replayfile: %d failure(s)\n", s_failures);
    return 1;
}
