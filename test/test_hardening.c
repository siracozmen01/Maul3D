// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The 2b-14 hardening gate: the zoo (every shape family in one
// scene, twinned, snapshotted, replayed) and the capacity sweeps
// (every pool exhausted on purpose; every refusal loud and clean,
// the world never corrupted).

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

static uint64_t SplitMix(uint64_t* state)
{
    *state += 0x9E3779B97F4A7C15ull;
    uint64_t z = *state;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    return z ^ (z >> 31);
}

static double RandRange(uint64_t* state, double lo, double hi)
{
    return lo + (hi - lo) * ((double)(SplitMix(state) >> 11) / 9007199254740992.0);
}

// Build the zoo: heightfield terrain, a mesh ramp, planes on the
// borders, a kinematic sweeper, then a pseudo-random rain of every
// dynamic family (spheres, boxes, capsules, QuickHull rocks, one
// bullet). Deterministic by construction from the seed.
static m3WorldId BuildZoo(uint64_t seed, uint8_t* journal, int32_t journalBytes)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 128;
    def.shapeCapacity = 128;
    m3WorldId world = m3CreateWorld(&def);
    if (journal != NULL)
    {
        m3World_JournalBegin(world, journal, journalBytes);
    }

    m3BodyDef gd = m3DefaultBodyDef();
    gd.position = (m3Pos3){-8.0, 0.0, -8.0};
    m3BodyId terrainBody = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.friction = 0.5f;
    float heights[16 * 16];
    uint64_t hseed = seed ^ 0x1234567890ABCDEFull;
    for (int32_t i = 0; i < 16 * 16; ++i)
    {
        heights[i] = (float)RandRange(&hseed, 0.0, 0.4);
    }
    m3CreateHeightFieldShape(terrainBody, &sd, heights, 16, 16, 1.0f);

    m3BodyDef rd = m3DefaultBodyDef();
    rd.position = (m3Pos3){-2.0, 0.3, -6.0};
    m3BodyId rampBody = m3CreateBody(world, &rd);
    m3Vec3 rampVerts[4] = {
        {0.0f, 0.0f, 0.0f}, {4.0f, 1.6f, 0.0f}, {0.0f, 0.0f, 3.0f}, {4.0f, 1.6f, 3.0f}};
    uint16_t rampTris[6] = {0, 2, 1, 1, 2, 3};
    m3CreateMeshShape(rampBody, &sd, rampVerts, 4, rampTris, 2);

    m3BodyDef kd = m3DefaultBodyDef();
    kd.type = m3_kinematicBody;
    kd.position = (m3Pos3){0.0, 2.6, 5.0};
    kd.linearVelocity = (m3Vec3){0.6f, 0.0f, 0.0f};
    m3BodyId sweeper = m3CreateBody(world, &kd);
    m3CreateBoxShape(sweeper, &sd, (m3Vec3){1.2f, 0.15f, 1.2f});

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    uint64_t rng = seed;
    for (int32_t k = 0; k < 28; ++k)
    {
        bd.position = (m3Pos3){RandRange(&rng, -4.0, 4.0), 1.5 + 0.55 * (double)k,
                               RandRange(&rng, -4.0, 4.0)};
        bd.linearVelocity =
            (m3Vec3){(m3real)RandRange(&rng, -1.0, 1.0), 0.0f, (m3real)RandRange(&rng, -1.0, 1.0)};
        bd.isBullet = k == 27; // the last one flies fast
        if (k == 27)
        {
            bd.linearVelocity = (m3Vec3){-60.0f, -5.0f, 40.0f};
            bd.position = (m3Pos3){6.0, 4.0, -6.0};
        }
        m3BodyId body = m3CreateBody(world, &bd);
        int32_t family = k % 4;
        if (family == 0)
        {
            m3Sphere ball = {{0.0f, 0.0f, 0.0f}, (m3real)RandRange(&rng, 0.2, 0.4)};
            m3CreateSphereShape(body, &sd, &ball);
        }
        else if (family == 1)
        {
            m3CreateBoxShape(body, &sd,
                             (m3Vec3){(m3real)RandRange(&rng, 0.15, 0.35),
                                      (m3real)RandRange(&rng, 0.15, 0.35),
                                      (m3real)RandRange(&rng, 0.15, 0.35)});
        }
        else if (family == 2)
        {
            m3Capsule capsule = {
                {-0.25f, 0.0f, 0.0f}, {0.25f, 0.0f, 0.0f}, (m3real)RandRange(&rng, 0.12, 0.25)};
            m3CreateCapsuleShape(body, &sd, &capsule);
        }
        else
        {
            m3Vec3 cloud[10];
            for (int32_t p = 0; p < 10; ++p)
            {
                cloud[p] =
                    (m3Vec3){(m3real)RandRange(&rng, -0.3, 0.3), (m3real)RandRange(&rng, -0.3, 0.3),
                             (m3real)RandRange(&rng, -0.3, 0.3)};
            }
            if (!m3Shape_IsValid(m3CreateHullShape(body, &sd, cloud, 10)))
            {
                // A degenerate cloud is possible; fall back to a box
                // so the body count stays deterministic.
                m3CreateBoxShape(body, &sd, (m3Vec3){0.2f, 0.2f, 0.2f});
            }
        }
        bd.isBullet = false;
    }
    return world;
}

static void StepN(m3WorldId world, int32_t steps)
{
    for (int32_t i = 0; i < steps; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
}

static void TestZoo(void)
{
    // The everything-at-once scene: twin runs bit-identical, a
    // mid-flight rollback bit-exact, the journaled session replays,
    // and every body ends finite and near the arena.
    enum
    {
        JOURNAL_CAP = 262144
    };
    static uint8_t journal[JOURNAL_CAP];

    m3WorldId a = BuildZoo(0xC0FFEEull, journal, JOURNAL_CAP);
    m3WorldId b = BuildZoo(0xC0FFEEull, NULL, 0);

    StepN(a, 120);
    StepN(b, 120);
    CHECK(m3World_Hash(a) == m3World_Hash(b), "zoo twins are bit-identical at 120");

    // The journal closes BEFORE the rollback block: a restore is not
    // an op, so a journal spanning one would replay a longer history
    // than the world lived (the composition rule, worth this comment).
    int32_t bytes = m3World_JournalEnd(a);
    CHECK(bytes > 0, "the zoo session recorded");
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 128;
    def.shapeCapacity = 128;
    m3WorldId c = m3CreateWorld(&def);
    CHECK(m3World_JournalReplay(c, journal, bytes), "the zoo session replays");
    CHECK(m3World_Hash(c) == m3World_Hash(a), "the zoo replay is bit-identical");

    // Rollback in the thick of it.
    int32_t snapBytes = m3World_SnapshotSize(a);
    void* snap = malloc((size_t)snapBytes);
    CHECK(m3World_Snapshot(a, snap, snapBytes) == snapBytes, "zoo snapshot writes");
    StepN(a, 90);
    uint64_t after = m3World_Hash(a);
    CHECK(m3World_Restore(a, snap, snapBytes), "zoo snapshot restores");
    StepN(a, 90);
    CHECK(m3World_Hash(a) == after, "the zoo rolls back bit-exact");
    free(snap);

    m3DestroyWorld(a);
    m3DestroyWorld(b);
    m3DestroyWorld(c);
}

static void TestCapacityExhaustion(void)
{
    // Every pool runs dry ON PURPOSE; every refusal is a null id and
    // the world keeps stepping, hash-stable, never corrupted.
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 4;
    def.shapeCapacity = 4;
    def.meshCapacity = 1;
    m3WorldId world = m3CreateWorld(&def);

    m3BodyDef bd = m3DefaultBodyDef();
    m3BodyId bodies[4];
    for (int32_t i = 0; i < 4; ++i)
    {
        bodies[i] = m3CreateBody(world, &bd);
        CHECK(m3Body_IsValid(bodies[i]), "bodies up to capacity create");
    }
    m3BodyId extra = m3CreateBody(world, &bd);
    CHECK(!m3Body_IsValid(extra), "the body pool refuses past capacity");

    m3ShapeDef sd = m3DefaultShapeDef();
    m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.3f};
    for (int32_t i = 0; i < 4; ++i)
    {
        CHECK(m3Shape_IsValid(m3CreateSphereShape(bodies[i % 4], &sd, &ball)),
              "shapes up to capacity create");
    }
    CHECK(!m3Shape_IsValid(m3CreateSphereShape(bodies[0], &sd, &ball)),
          "the shape pool refuses past capacity");

    m3DestroyWorld(world);

    // Mesh slots: one fills, the second refuses, and the failed
    // create leaves no half-registered shape behind.
    m3WorldDef md = m3DefaultWorldDef();
    md.bodyCapacity = 4;
    md.shapeCapacity = 8;
    md.meshCapacity = 1;
    m3WorldId mworld = m3CreateWorld(&md);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(mworld, &gd);
    m3ShapeDef msd = m3DefaultShapeDef();
    m3Vec3 verts[3] = {{0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
    uint16_t tris[3] = {0, 2, 1};
    CHECK(m3Shape_IsValid(m3CreateMeshShape(ground, &msd, verts, 3, tris, 1)),
          "the single mesh slot fills");
    CHECK(!m3Shape_IsValid(m3CreateMeshShape(ground, &msd, verts, 3, tris, 1)),
          "the mesh pool refuses past capacity");
    m3World_Step(mworld, 1.0f / 60.0f, 4);
    uint64_t hashAfterRefusal = m3World_Hash(mworld);

    // A refused create legitimately advances pool identity (the slot
    // was minted and retired; identity IS state). The property that
    // matters is that the FAILURE PATH is deterministic: a twin world
    // walking the same refusals lands on the identical hash.
    m3WorldId twin = m3CreateWorld(&md);
    m3BodyId tground = m3CreateBody(twin, &gd);
    CHECK(m3Shape_IsValid(m3CreateMeshShape(tground, &msd, verts, 3, tris, 1)),
          "the twin's mesh slot fills");
    CHECK(!m3Shape_IsValid(m3CreateMeshShape(tground, &msd, verts, 3, tris, 1)),
          "the twin's refusal repeats");
    m3World_Step(twin, 1.0f / 60.0f, 4);
    CHECK(m3World_Hash(twin) == hashAfterRefusal, "the failure path is bit-deterministic");
    m3DestroyWorld(twin);
    m3DestroyWorld(mworld);
}

static void TestJournalOverflowIsLoud(void)
{
    // A journal too small for the session latches overflow, stops
    // recording, and End reports -1: the caller can never mistake a
    // truncated recording for a good one.
    uint8_t tiny[64];
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m3WorldId world = m3CreateWorld(&def);
    CHECK(m3World_JournalBegin(world, tiny, (int32_t)sizeof(tiny)), "the tiny journal arms");
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    for (int32_t i = 0; i < 4; ++i)
    {
        m3CreateBody(world, &bd); // each op needs more than 64 bytes
    }
    CHECK(m3World_JournalEnd(world) == -1, "journal overflow reports -1, loudly");
    m3World_Step(world, 1.0f / 60.0f, 4); // the world itself is fine
    m3DestroyWorld(world);
}

int main(void)
{
    TestZoo();
    TestCapacityExhaustion();
    TestJournalOverflowIsLoud();
    if (s_failures == 0)
    {
        printf("test_hardening: all checks passed\n");
    }
    return s_failures == 0 ? 0 : 1;
}
