// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The benchmark harness (2c-1). NOT a CI gate: wall time is not
// deterministic and never will be. CI builds this binary so it
// cannot rot; profiles run locally in Release. Every performance
// slice in 2c must print its before-and-after from here in the
// commit message, and the corpus plan carries the baselines. Each
// scene also prints its state hash: the scenes themselves obey the
// determinism contract, and a moved hash means the scene changed,
// not just the clock.

#ifndef _WIN32
#define _POSIX_C_SOURCE 199309L // clock_gettime under strict c17
#endif

#include "maul3d/shape.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#include <windows.h>
static double NowMs(void)
{
    LARGE_INTEGER freq;
    LARGE_INTEGER now;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&now);
    return 1000.0 * (double)now.QuadPart / (double)freq.QuadPart;
}
#else
#include <time.h>
static double NowMs(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return 1000.0 * (double)ts.tv_sec + (double)ts.tv_nsec / 1.0e6;
}
#endif

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

static void AddPlane(m3WorldId world)
{
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.friction = 0.6f;
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    m3CreatePlaneShape(ground, &sd, &floor);
}

// Scene 1: the sphere pyramid, the golden scene's big brother.
static m3WorldId ScenePyramid(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 256;
    def.shapeCapacity = 256;
    m3WorldId world = m3CreateWorld(&def);
    AddPlane(world);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.friction = 0.6f;
    for (int32_t layer = 0; layer < 6; ++layer)
    {
        int32_t n = 6 - layer;
        for (int32_t i = 0; i < n; ++i)
        {
            for (int32_t j = 0; j < n; ++j)
            {
                bd.position = (m3Pos3){(double)i - 0.5 * (double)n + 0.05 * (double)layer,
                                       0.5 + 1.02 * (double)layer, (double)j - 0.5 * (double)n};
                m3BodyId body = m3CreateBody(world, &bd);
                m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.5f};
                m3CreateSphereShape(body, &sd, &ball);
            }
        }
    }
    return world;
}

// Scene 2: the hull jam, SAT-heavy boxes and rocks in a pit.
static m3WorldId SceneHullJam(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 256;
    def.shapeCapacity = 256;
    m3WorldId world = m3CreateWorld(&def);
    AddPlane(world);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.friction = 0.5f;
    uint64_t rng = 0xBEEFull;
    for (int32_t k = 0; k < 80; ++k)
    {
        bd.position =
            (m3Pos3){RandRange(&rng, -2.0, 2.0), 0.6 + 0.5 * (double)k, RandRange(&rng, -2.0, 2.0)};
        m3BodyId body = m3CreateBody(world, &bd);
        if (k % 3 == 0)
        {
            m3Vec3 cloud[12];
            for (int32_t p = 0; p < 12; ++p)
            {
                cloud[p] =
                    (m3Vec3){(m3real)RandRange(&rng, -0.3, 0.3), (m3real)RandRange(&rng, -0.3, 0.3),
                             (m3real)RandRange(&rng, -0.3, 0.3)};
            }
            if (!m3Shape_IsValid(m3CreateHullShape(body, &sd, cloud, 12)))
            {
                m3CreateBoxShape(body, &sd, (m3Vec3){0.25f, 0.25f, 0.25f});
            }
        }
        else
        {
            m3CreateBoxShape(body, &sd, (m3Vec3){0.3f, 0.25f, 0.35f});
        }
    }
    return world;
}

// Scene 3: the mesh field, the midphase workout (2c-10's target).
static m3WorldId SceneMeshField(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 256;
    def.shapeCapacity = 256;
    m3WorldId world = m3CreateWorld(&def);
    m3BodyDef gd = m3DefaultBodyDef();
    gd.position = (m3Pos3){-16.0, 0.0, -16.0};
    m3BodyId terrain = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.friction = 0.5f;
    float heights[32 * 32];
    uint64_t hseed = 0xFEEDull;
    for (int32_t i = 0; i < 32 * 32; ++i)
    {
        heights[i] = (float)RandRange(&hseed, 0.0, 0.8);
    }
    m3CreateHeightFieldShape(terrain, &sd, heights, 32, 32, 1.0f);

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    uint64_t rng = 0xFACEull;
    for (int32_t k = 0; k < 60; ++k)
    {
        bd.position = (m3Pos3){RandRange(&rng, -10.0, 10.0), 2.0 + 0.4 * (double)k,
                               RandRange(&rng, -10.0, 10.0)};
        m3BodyId body = m3CreateBody(world, &bd);
        int32_t family = k % 3;
        if (family == 0)
        {
            m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.35f};
            m3CreateSphereShape(body, &sd, &ball);
        }
        else if (family == 1)
        {
            m3CreateBoxShape(body, &sd, (m3Vec3){0.3f, 0.3f, 0.3f});
        }
        else
        {
            m3Capsule capsule = {{-0.3f, 0.0f, 0.0f}, {0.3f, 0.0f, 0.0f}, 0.2f};
            m3CreateCapsuleShape(body, &sd, &capsule);
        }
    }
    return world;
}

// Scene 4 (3-8): voxfort, the destruction demo and the
// constitution's consumer. A hollow voxel fort is shelled by
// deterministic charges; the HOST reads fragment events and turns
// recipes into dynamic bodies (small islands become QuickHull
// bodies from their voxel corner clouds, large ones become their
// bounds boxes with density matched to the event mass), then
// bullets rake the breach. Public API only, front to back.
static m3WorldId SceneVoxfort(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 512;
    def.shapeCapacity = 512;
    def.voxelCapacity = 2;
    m3WorldId world = m3CreateWorld(&def);
    AddPlane(world);

    static uint8_t voxels[16 * 16 * 16];
    memset(voxels, 0, sizeof(voxels));
    // A hollow keep: footprint 2..13, walls one voxel thick, eight
    // voxels tall, on a solid base layer (the anchor).
    for (int32_t z = 0; z < 16; ++z)
    {
        for (int32_t x = 0; x < 16; ++x)
        {
            voxels[x + 16 * (0 + 16 * z)] = 1;
        }
    }
    for (int32_t y = 1; y <= 8; ++y)
    {
        for (int32_t i = 2; i <= 13; ++i)
        {
            voxels[i + 16 * (y + 16 * 2)] = 1;
            voxels[i + 16 * (y + 16 * 13)] = 1;
            voxels[2 + 16 * (y + 16 * i)] = 1;
            voxels[13 + 16 * (y + 16 * i)] = 1;
        }
    }
    m3BodyDef gd = m3DefaultBodyDef();
    gd.position = (m3Pos3){-8.0, 0.0, -8.0};
    m3BodyId keepBody = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.friction = 0.6f;
    m3CreateVoxelChunkShape(keepBody, &sd, voxels, NULL, 1.0f);
    return world;
}

// The host recipe: an island becomes a dynamic body. Small islands
// get the full QuickHull treatment from their voxel corner clouds;
// larger ones get their bounds box; density matches the event mass.
static void SpawnFragment(m3WorldId world, const m3FragmentEvent* ev, const uint16_t* recipe)
{
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = ev->comWorld;
    m3BodyId body = m3CreateBody(world, &bd);
    m3Vec3 half = {0.5f * (m3real)(ev->boundsHi[0] - ev->boundsLo[0] + 1),
                   0.5f * (m3real)(ev->boundsHi[1] - ev->boundsLo[1] + 1),
                   0.5f * (m3real)(ev->boundsHi[2] - ev->boundsLo[2] + 1)};
    m3ShapeDef fd = m3DefaultShapeDef();
    fd.friction = 0.6f;
    fd.density = ev->mass / (8.0f * half.x * half.y * half.z);
    if (ev->voxelCount <= 8 && ev->recipeStart >= 0)
    {
        m3Vec3 cloud[64];
        int32_t points = 0;
        for (int32_t k = 0; k < ev->recipeCount; ++k)
        {
            uint16_t v = recipe[ev->recipeStart + k];
            m3real x = (m3real)(v % 16);
            m3real y = (m3real)((v / 16) % 16);
            m3real z = (m3real)(v / 256);
            for (int32_t c = 0; c < 8; ++c)
            {
                cloud[points++] = (m3Vec3){x + ((c & 1) != 0 ? 1.0f : 0.0f) - ev->comChunk.x,
                                           y + ((c & 2) != 0 ? 1.0f : 0.0f) - ev->comChunk.y,
                                           z + ((c & 4) != 0 ? 1.0f : 0.0f) - ev->comChunk.z};
            }
        }
        if (m3Shape_IsValid(m3CreateHullShape(body, &fd, cloud, points)))
        {
            return;
        }
        // A degenerate cloud (a lone flat island) falls back to the
        // bounds box below, deterministically.
    }
    m3CreateBoxShape(body, &fd, half);
}

static void RunVoxfort(int32_t steps)
{
    m3WorldId world = SceneVoxfort();
    // The chunk shape id is deterministic: the first shape after the
    // plane (index1 = 2 in this scene).
    m3ShapeId keep = {2, 0, 1};
    double t0 = NowMs();
    uint64_t rng = 0xF0F7ull;
    int32_t shells = 0;
    for (int32_t i = 0; i < steps; ++i)
    {
        if (i % 15 == 5)
        {
            // A shell: carve a deterministic 2x2x2 charge out of the
            // south wall, marching along it, then spawn what fell.
            int32_t sx = 2 + (shells * 3) % 11;
            int32_t sy = 1 + (shells * 2) % 6;
            int32_t lo[3] = {sx, sy, 2};
            int32_t hi[3] = {sx + 1 < 16 ? sx + 1 : 15, sy + 1, 2};
            m3VoxelChunk_ClearBox(keep, lo, hi);
            shells += 1;
            int32_t count = 0;
            const m3FragmentEvent* events = m3World_FragmentEvents(world, &count);
            int32_t recipeCount = 0;
            const uint16_t* recipe = m3World_FragmentRecipe(world, &recipeCount);
            for (int32_t e = 0; e < count && e < 16; ++e)
            {
                SpawnFragment(world, &events[e], recipe);
            }
        }
        if (i % 45 == 20)
        {
            // Bullets rake the wall: stopped by standing voxels,
            // through the breaches once the charges open them.
            m3BodyDef bd = m3DefaultBodyDef();
            bd.type = m3_dynamicBody;
            bd.isBullet = true;
            bd.position = (m3Pos3){RandRange(&rng, -4.0, 4.0), 2.5, -20.0};
            bd.linearVelocity = (m3Vec3){0.0f, 0.0f, 150.0f};
            m3BodyId bullet = m3CreateBody(world, &bd);
            m3ShapeDef sd = m3DefaultShapeDef();
            m3Sphere slug = {{0.0f, 0.0f, 0.0f}, 0.15f};
            m3CreateSphereShape(bullet, &sd, &slug);
        }
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    double t1 = NowMs();
    double total = t1 - t0;
    printf("M3_BENCH %-10s steps=%d totalMs=%9.2f perStepMs=%7.4f hash=%016llx\n", "voxfort", steps,
           total, total / (double)steps, (unsigned long long)m3World_Hash(world));
    m3DestroyWorld(world);
}

typedef m3WorldId (*SceneFn)(void);

static void RunScene(const char* name, SceneFn build, int32_t steps)
{
    m3WorldId world = build();
    double t0 = NowMs();
    for (int32_t i = 0; i < steps; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    double t1 = NowMs();
    double total = t1 - t0;
    printf("M3_BENCH %-10s steps=%d totalMs=%9.2f perStepMs=%7.4f hash=%016llx\n", name, steps,
           total, total / (double)steps, (unsigned long long)m3World_Hash(world));
    m3DestroyWorld(world);
}

int main(int argc, char** argv)
{
    int32_t steps = 300;
    if (argc > 1)
    {
        int parsed = atoi(argv[1]);
        if (parsed > 0)
        {
            steps = parsed;
        }
    }
    RunScene("pyramid", ScenePyramid, steps);
    RunScene("hulljam", SceneHullJam, steps);
    RunScene("meshfield", SceneMeshField, steps);
    RunVoxfort(steps);
    return 0;
}
