// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The scale gate (6-4): the 5k-body claims, proven at CI size. A
// thousand mixed bodies rain into a voxel block; worker-count twins
// (serial versus a real four-thread host pool) land on identical
// bits; the whole session records, replays, and rolls back
// bit-exact mid-storm; capacity sweeps grow to the bench's 8192
// without a creak. The full-size numbers (5k rollback cost, 10k
// smoke) live in the bench binary where wall time belongs.

#include "maul3d/shape.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <pthread.h>
#endif

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

// The test-grade host pool, the determinism suite's pattern: real
// threads, spawn per enqueue, any split must produce identical bits.
#define POOL_WORKERS 4

typedef struct PoolRange
{
    m3TaskFn* task;
    void* taskContext;
    int32_t start;
    int32_t end;
} PoolRange;

typedef struct PoolTask
{
    PoolRange ranges[POOL_WORKERS];
    int32_t rangeCount;
#ifdef _WIN32
    HANDLE threads[POOL_WORKERS];
#else
    pthread_t threads[POOL_WORKERS];
#endif
    int32_t threadCount;
} PoolTask;

#ifdef _WIN32
static DWORD WINAPI PoolMain(LPVOID arg)
{
    PoolRange* r = (PoolRange*)arg;
    r->task(r->start, r->end, r->taskContext);
    return 0;
}
#else
static void* PoolMain(void* arg)
{
    PoolRange* r = (PoolRange*)arg;
    r->task(r->start, r->end, r->taskContext);
    return NULL;
}
#endif

static void* PoolEnqueue(m3TaskFn* task, int32_t itemCount, int32_t minRange, void* taskContext,
                         void* userContext)
{
    (void)userContext;
    static PoolTask taskSlot;
    PoolTask* t = &taskSlot;
    int32_t workers = POOL_WORKERS;
    int32_t grain = (itemCount + workers - 1) / workers;
    grain = grain < minRange ? minRange : grain;
    t->rangeCount = 0;
    t->threadCount = 0;
    for (int32_t start = 0; start < itemCount; start += grain)
    {
        int32_t end = start + grain < itemCount ? start + grain : itemCount;
        PoolRange* r = &t->ranges[t->rangeCount++];
        r->task = task;
        r->taskContext = taskContext;
        r->start = start;
        r->end = end;
        if (t->rangeCount == POOL_WORKERS)
        {
            r->end = itemCount;
            break;
        }
    }
    for (int32_t k = 1; k < t->rangeCount; ++k)
    {
#ifdef _WIN32
        t->threads[t->threadCount] = CreateThread(NULL, 0, PoolMain, &t->ranges[k], 0, NULL);
#else
        pthread_create(&t->threads[t->threadCount], NULL, PoolMain, &t->ranges[k]);
#endif
        t->threadCount += 1;
    }
    t->ranges[0].task(t->ranges[0].start, t->ranges[0].end, taskContext);
    return t;
}

static void PoolFinish(void* userTask, void* userContext)
{
    (void)userContext;
    PoolTask* t = (PoolTask*)userTask;
    for (int32_t k = 0; k < t->threadCount; ++k)
    {
#ifdef _WIN32
        WaitForSingleObject(t->threads[k], INFINITE);
        CloseHandle(t->threads[k]);
#else
        pthread_join(t->threads[k], NULL);
#endif
    }
}

static uint64_t SplitMix(uint64_t* s)
{
    *s += 0x9E3779B97F4A7C15ull;
    uint64_t z = *s;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    return z ^ (z >> 31);
}

static double RandRange(uint64_t* s, double lo, double hi)
{
    return lo + (hi - lo) * ((double)(SplitMix(s) >> 11) / 9007199254740992.0);
}

// The mini block: two welded tower chunks and a thousand mixed
// bodies over a 20 x 20 yard. The city block's little sibling,
// small enough for every CI cell, big enough to mean it.
static m3ShapeId BuildMiniBlock(m3WorldId world)
{
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef pd = m3DefaultShapeDef();
    pd.friction = 0.6f;
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    m3CreatePlaneShape(ground, &pd, &floor);

    static uint8_t tower[16 * 16 * 16];
    memset(tower, 0, sizeof(tower));
    for (int32_t z = 0; z < 16; ++z)
    {
        for (int32_t x = 0; x < 16; ++x)
        {
            tower[x + 16 * (0 + 16 * z)] = 1;
        }
    }
    for (int32_t y = 1; y < 16; ++y)
    {
        for (int32_t z = 5; z <= 10; ++z)
        {
            for (int32_t x = 5; x <= 10; ++x)
            {
                tower[x + 16 * (y + 16 * z)] = 1;
            }
        }
    }
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.friction = 0.6f;
    m3BodyDef td = m3DefaultBodyDef();
    td.position = (m3Pos3){-4.0, 0.0, -4.0};
    m3ShapeId lower = m3CreateVoxelChunkShape(m3CreateBody(world, &td), &sd, tower, NULL, 0.5f);
    td.position = (m3Pos3){-4.0, 8.0, -4.0}; // welded stack
    m3CreateVoxelChunkShape(m3CreateBody(world, &td), &sd, tower, NULL, 0.5f);

    m3ShapeDef bs = m3DefaultShapeDef();
    bs.friction = 0.6f;
    bs.rollingResistance = 0.05f;
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    uint64_t rng = 0x5CA1Eull;
    for (int32_t k = 0; k < 1000; ++k)
    {
        bd.position = (m3Pos3){RandRange(&rng, -10.0, 10.0), RandRange(&rng, 6.0, 26.0),
                               RandRange(&rng, -10.0, 10.0)};
        m3BodyId body = m3CreateBody(world, &bd);
        int32_t f = k % 4;
        if (f == 0)
        {
            m3Sphere ball = {{0.0f, 0.0f, 0.0f}, (m3real)RandRange(&rng, 0.2, 0.4)};
            m3CreateSphereShape(body, &bs, &ball);
        }
        else if (f == 1)
        {
            m3CreateBoxShape(body, &bs,
                             (m3Vec3){(m3real)RandRange(&rng, 0.15, 0.35),
                                      (m3real)RandRange(&rng, 0.15, 0.35),
                                      (m3real)RandRange(&rng, 0.15, 0.35)});
        }
        else if (f == 2)
        {
            m3Capsule c = {
                {-0.2f, 0.0f, 0.0f}, {0.2f, 0.0f, 0.0f}, (m3real)RandRange(&rng, 0.12, 0.2)};
            m3CreateCapsuleShape(body, &bs, &c);
        }
        else
        {
            m3Vec3 cl[8];
            for (int32_t c = 0; c < 8; ++c)
            {
                cl[c] = (m3Vec3){(m3real)RandRange(&rng, -0.25, 0.25),
                                 (m3real)RandRange(&rng, -0.25, 0.25),
                                 (m3real)RandRange(&rng, -0.25, 0.25)};
            }
            if (!m3Shape_IsValid(m3CreateHullShape(body, &bs, cl, 8)))
            {
                m3CreateBoxShape(body, &bs, (m3Vec3){0.2f, 0.2f, 0.2f});
            }
        }
    }
    return lower;
}

static m3WorldDef ScaleDef(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 2048;
    def.shapeCapacity = 2048;
    def.voxelCapacity = 4;
    return def;
}

static void TestWorkerTwinsAtScale(void)
{
    // Serial versus a real four-thread pool: identical bits after
    // 120 storm steps with mid-run carves. The worker count is a
    // performance knob, never a state input: the scale gate's core
    // promise.
    uint64_t hashes[2];
    for (int32_t run = 0; run < 2; ++run)
    {
        m3WorldDef def = ScaleDef();
        if (run == 1)
        {
            def.workerCount = POOL_WORKERS;
            def.enqueueTask = PoolEnqueue;
            def.finishTask = PoolFinish;
        }
        m3WorldId world = m3CreateWorld(&def);
        m3ShapeId chunk = BuildMiniBlock(world);
        for (int32_t i = 0; i < 120; ++i)
        {
            if (i >= 40 && i % 10 == 0)
            {
                int32_t cx = 5 + (i / 10) % 6;
                int32_t lo[3] = {cx, 2, 5};
                int32_t hi[3] = {cx, 2, 10};
                m3VoxelChunk_ClearBox(chunk, lo, hi);
            }
            m3World_Step(world, 1.0f / 60.0f, 4);
        }
        hashes[run] = m3World_Hash(world);
        m3DestroyWorld(world);
    }
    CHECK(hashes[0] == hashes[1], "serial and four-thread scale twins are bit-identical");
}

static void TestScaleSessionAndRollback(void)
{
    // The full netcode loop at a thousand bodies: the session
    // journals from world birth, replays bit-exact into a fresh
    // world, and a mid-storm snapshot re-fights the same tail onto
    // the same bits.
    static uint8_t journal[4 * 1024 * 1024];
    m3WorldDef def = ScaleDef();
    m3WorldId world = m3CreateWorld(&def);
    CHECK(m3World_JournalBegin(world, journal, (int32_t)sizeof(journal)), "the session records");
    m3ShapeId chunk = BuildMiniBlock(world);

    int32_t snapBytes = m3World_SnapshotSize(world);
    uint8_t* snap = (uint8_t*)malloc((size_t)snapBytes);
    CHECK(snap != NULL, "the snapshot buffer allocates");
    int32_t written = 0;

    for (int32_t i = 0; i < 100; ++i)
    {
        if (i >= 30 && i % 10 == 0)
        {
            int32_t cx = 5 + (i / 10) % 6;
            int32_t lo[3] = {cx, 3, 5};
            int32_t hi[3] = {cx, 3, 10};
            m3VoxelChunk_ClearBox(chunk, lo, hi);
        }
        m3World_Step(world, 1.0f / 60.0f, 4);
        if (i == 50)
        {
            written = m3World_Snapshot(world, snap, snapBytes);
            CHECK(written == snapBytes, "the mid-storm snapshot fits its own size");
        }
    }
    uint64_t final = m3World_Hash(world);

    int32_t bytes = m3World_JournalEnd(world);
    CHECK(bytes > 0, "the session closes");
    m3WorldId fresh = m3CreateWorld(&def);
    CHECK(m3World_JournalReplay(fresh, journal, bytes), "the session replays");
    CHECK(m3World_Hash(fresh) == final, "the replayed storm is bit-identical");
    m3DestroyWorld(fresh);

    CHECK(m3World_Restore(world, snap, written), "the mid-storm restore lands");
    for (int32_t i = 51; i < 100; ++i)
    {
        if (i >= 30 && i % 10 == 0)
        {
            int32_t cx = 5 + (i / 10) % 6;
            int32_t lo[3] = {cx, 3, 5};
            int32_t hi[3] = {cx, 3, 10};
            m3VoxelChunk_ClearBox(chunk, lo, hi);
        }
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m3World_Hash(world) == final, "the re-fought storm is bit-identical");
    free(snap);
    m3DestroyWorld(world);
}

static void TestCapacityGrowthSweep(void)
{
    // Worlds from small to bench-sized: creation fills to the brim,
    // refuses one past it, and tears down clean at every size.
    int32_t caps[3] = {256, 2048, 8192};
    for (int32_t c = 0; c < 3; ++c)
    {
        m3WorldDef def = m3DefaultWorldDef();
        def.bodyCapacity = caps[c];
        def.shapeCapacity = caps[c];
        m3WorldId world = m3CreateWorld(&def);
        CHECK(m3World_IsValid(world), "the sized world creates");
        m3BodyDef bd = m3DefaultBodyDef();
        bd.type = m3_dynamicBody;
        int32_t made = 0;
        for (int32_t k = 0; k < caps[c] + 8; ++k)
        {
            bd.position = (m3Pos3){(double)(k % 64), 1.0 + (double)(k / 64), 0.0};
            if (m3Body_IsValid(m3CreateBody(world, &bd)))
            {
                made += 1;
            }
        }
        CHECK(made == caps[c], "creation fills exactly to capacity");
        m3World_Step(world, 1.0f / 60.0f, 1);
        m3DestroyWorld(world);
    }
}

int main(void)
{
    TestWorkerTwinsAtScale();
    TestScaleSessionAndRollback();
    TestCapacityGrowthSweep();
    if (s_failures == 0)
    {
        printf("test_scale: all green\n");
        return 0;
    }
    printf("test_scale: %d failure(s)\n", s_failures);
    return 1;
}
