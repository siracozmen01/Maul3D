// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The four permanent determinism gates of phase 2a, replacing the
// bootstrap seed test:
//   1. The golden scene: a 30-sphere pyramid plus droppers, 300 steps;
//      every CI cell must print the identical M3_DET_HASH.
//   2. Replay: the same run twice is bit-identical, and a journaled
//      session (creates plus steps) replays bit for bit.
//   3. Rollback round-trip, THE flagship: snapshot at step 100, run to
//      200, restore, rerun to 200, bit-identical; and a divergent
//      continuation after a second restore proves restore is total.
//   4. Worker twins: worker counts 1 and 4 produce identical bits
//      (trivially serial today; the gate is armed for 2b threading).

#include "maul3d/shape.h"

#include <stdio.h>
#include <stdlib.h>

// ---------------------------------------------------------------
// A test-grade host thread pool for the worker-twin gate: real
// threads, spawn-per-enqueue (fine for tests). The library never
// owns threads; it only describes ranges. Any split and any
// schedule must produce identical bits, and gate 4 now proves that
// with actual concurrency instead of a vacuous serial pass.
// ---------------------------------------------------------------
#ifdef _WIN32
#include <windows.h>
#else
#include <pthread.h>
#endif

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
    static PoolTask taskSlot; // one in-flight task (the library's contract)
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
            r->end = itemCount; // last range absorbs the tail
            break;
        }
    }
    // Run range 0 on this thread, the rest on spawned threads.
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

// The golden scene: a plane, a 4-layer sphere pyramid (16+9+4+1 = 30),
// and five offset droppers. Every def is pinned; nothing is random.
static m3BodyId BuildGoldenScene(m3WorldId world)
{
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef gs = m3DefaultShapeDef();
    gs.friction = 0.8f;
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    m3CreatePlaneShape(ground, &gs, &floor);

    m3ShapeDef sd = m3DefaultShapeDef();
    sd.friction = 0.6f;
    sd.restitution = 0.1f;
    m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.5f};

    for (int32_t layer = 0; layer < 4; ++layer)
    {
        int32_t size = 4 - layer;
        double base = 0.55 * (double)layer;
        for (int32_t i = 0; i < size; ++i)
        {
            for (int32_t k = 0; k < size; ++k)
            {
                m3BodyDef bd = m3DefaultBodyDef();
                bd.type = m3_dynamicBody;
                bd.position = (m3Pos3){base + 1.02 * (double)i, 0.5 + 0.95 * (double)layer,
                                       base + 1.02 * (double)k};
                m3BodyId body = m3CreateBody(world, &bd);
                m3CreateSphereShape(body, &sd, &ball);
            }
        }
    }

    m3BodyId lastDropper = m3_nullBodyId;
    for (int32_t d = 0; d < 5; ++d)
    {
        m3BodyDef bd = m3DefaultBodyDef();
        bd.type = m3_dynamicBody;
        bd.position = (m3Pos3){0.4 * (double)d, 7.0 + 0.8 * (double)d, 1.3 - 0.3 * (double)d};
        m3BodyId body = m3CreateBody(world, &bd);
        m3CreateSphereShape(body, &sd, &ball);
        lastDropper = body;
    }
    return lastDropper;
}

static m3WorldId MakeGoldenWorld(int32_t workerCount, int threaded)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 64;
    def.shapeCapacity = 64;
    def.workerCount = workerCount;
    if (threaded)
    {
        def.enqueueTask = PoolEnqueue;
        def.finishTask = PoolFinish;
    }
    return m3CreateWorld(&def);
}

static void StepN(m3WorldId world, int32_t steps)
{
    for (int32_t i = 0; i < steps; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
}

static uint64_t RunGolden(int32_t steps, int32_t workerCount)
{
    m3WorldId world = MakeGoldenWorld(workerCount, workerCount > 1);
    BuildGoldenScene(world);
    StepN(world, steps);
    uint64_t h = m3World_Hash(world);
    m3DestroyWorld(world);
    return h;
}

static void GateGoldenAndReplay(void)
{
    // Gate 1 and the first half of gate 2: the same run twice.
    uint64_t h1 = RunGolden(300, 1);
    uint64_t h2 = RunGolden(300, 1);
    CHECK(h1 == h2, "gate 2a: the same run twice is bit-identical");
    printf("M3_DET_HASH=%016llx\n", (unsigned long long)h1);

    // Gate 2b: a journaled session replays bit for bit.
    m3WorldId a = MakeGoldenWorld(1, 0);
    uint8_t* journal = (uint8_t*)malloc(1 << 20);
    CHECK(m3World_JournalBegin(a, journal, 1 << 20), "journal begins");
    BuildGoldenScene(a);
    StepN(a, 150);
    int32_t bytes = m3World_JournalEnd(a);
    CHECK(bytes > 0, "the session fits the journal");
    m3WorldId b = MakeGoldenWorld(1, 0);
    CHECK(m3World_JournalReplay(b, journal, bytes), "the session replays");
    CHECK(m3World_Hash(a) == m3World_Hash(b), "gate 2b: journal replay is bit-identical");
    free(journal);
    m3DestroyWorld(a);
    m3DestroyWorld(b);
}

static void GateRollback(void)
{
    // Gate 3, the flagship: what our base engine cannot do.
    m3WorldId w = MakeGoldenWorld(1, 0);
    m3BodyId dropper = BuildGoldenScene(w);
    StepN(w, 100);

    int32_t size = m3World_SnapshotSize(w);
    void* snap = malloc((size_t)size);
    CHECK(m3World_Snapshot(w, snap, size) == size, "mid-flight snapshot");

    StepN(w, 100);
    uint64_t h1 = m3World_Hash(w);

    CHECK(m3World_Restore(w, snap, size), "mid-flight restore");
    StepN(w, 100);
    uint64_t h2 = m3World_Hash(w);
    CHECK(h1 == h2, "gate 3: rollback round-trip is bit-identical");

    // A different continuation must diverge: restore is total, resume
    // is real simulation, not a cached tail.
    CHECK(m3World_Restore(w, snap, size), "second restore");
    m3Body_SetLinearVelocity(dropper, (m3Vec3){3.0f, 1.0f, -2.0f});
    StepN(w, 100);
    uint64_t h3 = m3World_Hash(w);
    CHECK(h3 != h1, "a divergent continuation diverges");

    free(snap);
    m3DestroyWorld(w);
}

static void GateWorkerTwins(void)
{
    // Gate 4, and since 2b-11 it BITES: the four-worker run executes
    // the narrowphase on a real thread pool (spawned threads, any
    // scheduling the OS feels like) and must produce the exact bits
    // of the serial run. Twice, so the pool run is also self-stable.
    uint64_t h1 = RunGolden(200, 1);
    uint64_t h4 = RunGolden(200, 4);
    uint64_t h4b = RunGolden(200, 4);
    CHECK(h1 == h4, "gate 4: worker twins are bit-identical");
    CHECK(h4 == h4b, "gate 4: the threaded run is self-stable");
}

int main(void)
{
    GateGoldenAndReplay();
    GateRollback();
    GateWorkerTwins();
    if (s_failures == 0)
    {
        printf("test_determinism: all four gates hold\n");
    }
    return s_failures == 0 ? 0 : 1;
}
