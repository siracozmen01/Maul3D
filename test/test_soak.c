// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The soak (2d-6): one long life, no drift, no growth. A busy zoo
// runs more than twenty thousand steps in segments; every segment
// is journaled and replayed against a snapshot twin (the replay
// must land on the live world's exact hash), every segment ends in
// a rolling rollback (snapshot, run ahead, restore, rerun, same
// bits), and the engine's own allocator must report ZERO net
// allocation across the steady state (stepping never allocates;
// restore and replay balance their scratch exactly). The sanitizer
// cells run this whole life under ASan and UBSan, and their leak
// check has teeth at exit.

#include "allocator.h"
#include "maul3d/joint.h"
#include "maul3d/shape.h"

#include <stdio.h>
#include <stdlib.h>

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

#define SEGMENTS      28
#define SEGMENT_STEPS 500
#define ROLL_STEPS    100
#define WARMUP_STEPS  500
#define JOURNAL_CAP   (1 << 20)

static m3WorldDef SoakDef(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 64;
    def.shapeCapacity = 64;
    def.jointCapacity = 8;
    def.meshCapacity = 2;
    return def;
}

// The soak zoo: bumpy terrain, a sensor curtain, a swinging chain,
// and a mixed rain. Everything the engine has, kept small enough
// that a sanitizer cell can afford a long life.
static m3WorldId BuildSoakZoo(m3BodyId* kicker)
{
    m3WorldDef def = SoakDef();
    m3WorldId world = m3CreateWorld(&def);

    m3BodyDef gd = m3DefaultBodyDef();
    gd.position = (m3Pos3){-8.0, 0.0, -8.0};
    m3BodyId terrain = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.friction = 0.5f;
    float heights[16 * 16];
    for (int32_t i = 0; i < 16 * 16; ++i)
    {
        heights[i] = 0.1f * (float)((i * 7) % 5);
    }
    m3CreateHeightFieldShape(terrain, &sd, heights, 16, 16, 1.0f);

    m3BodyDef cd = m3DefaultBodyDef();
    cd.position = (m3Pos3){0.0, 2.5, 0.0};
    m3BodyId curtain = m3CreateBody(world, &cd);
    m3ShapeDef sensor = m3DefaultShapeDef();
    sensor.isSensor = true;
    m3CreateBoxShape(curtain, &sensor, (m3Vec3){6.0f, 0.3f, 6.0f});

    m3BodyDef postDef = m3DefaultBodyDef();
    postDef.position = (m3Pos3){5.0, 3.5, 5.0};
    m3BodyId post = m3CreateBody(world, &postDef);
    m3BodyDef linkDef = m3DefaultBodyDef();
    linkDef.type = m3_dynamicBody;
    linkDef.position = (m3Pos3){5.0, 2.9, 5.0};
    m3BodyId link = m3CreateBody(world, &linkDef);
    m3CreateCapsuleShape(link, &sd, &(m3Capsule){{0.0f, 0.3f, 0.0f}, {0.0f, -0.3f, 0.0f}, 0.1f});
    m3JointDef jd = m3DefaultJointDef();
    jd.bodyA = post;
    jd.bodyB = link;
    jd.localAnchorB = (m3Vec3){0.0f, 0.35f, 0.0f};
    m3CreateJoint(&jd);

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    for (int32_t k = 0; k < 20; ++k)
    {
        bd.position = (m3Pos3){-3.0 + 0.7 * (double)(k % 6), 1.5 + 0.6 * (double)k,
                               -3.0 + 0.7 * (double)(k / 6)};
        m3BodyId body = m3CreateBody(world, &bd);
        if (k % 3 == 0)
        {
            m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.3f};
            m3CreateSphereShape(body, &sd, &ball);
        }
        else if (k % 3 == 1)
        {
            m3CreateBoxShape(body, &sd, (m3Vec3){0.25f, 0.25f, 0.25f});
        }
        else
        {
            m3CreateCapsuleShape(body, &sd,
                                 &(m3Capsule){{-0.2f, 0.0f, 0.0f}, {0.2f, 0.0f, 0.0f}, 0.15f});
        }
        if (k == 0 && kicker != NULL)
        {
            *kicker = body;
        }
    }
    return world;
}

int main(void)
{
    static uint8_t journal[JOURNAL_CAP];
    m3BodyId kicker;
    m3WorldId live = BuildSoakZoo(&kicker);
    m3WorldDef def = SoakDef();
    m3WorldId scratch = m3CreateWorld(&def); // the replay twin, reused

    int32_t snapBytes = m3World_SnapshotSize(live);
    uint8_t* snap = (uint8_t*)malloc((size_t)snapBytes);
    uint8_t* roll = (uint8_t*)malloc((size_t)snapBytes);

    for (int32_t i = 0; i < WARMUP_STEPS; ++i)
    {
        m3World_Step(live, 1.0f / 60.0f, 4);
    }

    // Prime the reused twin once: its first restore buys the
    // count-derived mesh arrays (10-3), a one-time capacity
    // purchase exactly like world creation, not steady state.
    CHECK(m3World_Snapshot(live, snap, snapBytes) == snapBytes, "the priming snapshot writes");
    CHECK(m3World_Restore(scratch, snap, snapBytes), "the priming restore lands");

    // Steady state starts here: the engine may not allocate again
    // on the live path (snapshot/restore/replay scratch must
    // balance to zero within every segment).
    int64_t allocs0;
    int64_t frees0;
    m3DebugAllocCounts(&allocs0, &frees0);
    int64_t liveAllocs0 = allocs0 - frees0;

    int64_t totalSteps = WARMUP_STEPS;
    for (int32_t segment = 0; segment < SEGMENTS; ++segment)
    {
        // Segment: snapshot the start, journal the life.
        CHECK(m3World_Snapshot(live, snap, snapBytes) == snapBytes, "segment snapshot writes");
        CHECK(m3World_JournalBegin(live, journal, JOURNAL_CAP), "segment journal arms");
        for (int32_t i = 0; i < SEGMENT_STEPS; ++i)
        {
            if (i % 125 == 0)
            {
                // A deterministic kick keeps the zoo from settling
                // into an eternal sleep (a soak of sleepers proves
                // nothing).
                float side = (segment + i) % 2 == 0 ? 1.0f : -1.0f;
                m3Body_SetLinearVelocity(kicker, (m3Vec3){2.0f * side, 3.0f, -1.5f * side});
            }
            m3World_Step(live, 1.0f / 60.0f, 4);
        }
        int32_t journalBytes = m3World_JournalEnd(live);
        CHECK(journalBytes > 0, "the segment fits its journal");
        totalSteps += SEGMENT_STEPS;

        // The replay twin: restore the segment start, replay the
        // recorded life, land on the live world's exact bits.
        CHECK(m3World_Restore(scratch, snap, snapBytes), "the twin restores the segment start");
        CHECK(m3World_JournalReplay(scratch, journal, journalBytes), "the segment replays");
        CHECK(m3World_Hash(scratch) == m3World_Hash(live),
              "the replayed segment lands on the live hash");

        // The rolling rollback: run ahead, remember, rewind, rerun.
        CHECK(m3World_Snapshot(live, roll, snapBytes) == snapBytes, "roll snapshot writes");
        for (int32_t i = 0; i < ROLL_STEPS; ++i)
        {
            m3World_Step(live, 1.0f / 60.0f, 4);
        }
        uint64_t ahead = m3World_Hash(live);
        CHECK(m3World_Restore(live, roll, snapBytes), "the rolling restore lands");
        for (int32_t i = 0; i < ROLL_STEPS; ++i)
        {
            m3World_Step(live, 1.0f / 60.0f, 4);
        }
        CHECK(m3World_Hash(live) == ahead, "the rerun timeline is bit-identical");
        totalSteps += 2 * ROLL_STEPS;

        // The no-growth law, checked every segment.
        int64_t allocs;
        int64_t frees;
        m3DebugAllocCounts(&allocs, &frees);
        CHECK(allocs - frees == liveAllocs0, "steady state performs zero net allocation");
    }

    printf("M3_SOAK steps=%lld segments=%d journalCap=%d\n", (long long)totalSteps, SEGMENTS,
           JOURNAL_CAP);
    CHECK(totalSteps >= 20000, "the soak lived past twenty thousand steps");

    free(snap);
    free(roll);
    m3DestroyWorld(scratch);
    m3DestroyWorld(live);
    if (s_failures == 0)
    {
        printf("test_soak: all green\n");
        return 0;
    }
    printf("test_soak: %d failure(s)\n", s_failures);
    return 1;
}
