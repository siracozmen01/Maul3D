// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Snapshot v1: the portable half of the rollback spine. The format is
// a padding-free little-endian header plus the persistent arrays as
// canonical field blocks, walked by ONE function in ONE order (the
// Maul2D single-source-of-truth rule: the size IS the walk). This is
// the deliberate opposite of a build-locked raw world image: no
// pointers, no layout hash, no SIMD width anywhere in the bytes. The
// header's config hash covers exactly the things that change the
// MEANING of the bytes (engine version, solver revision, precision,
// FP policy) and restore refuses a mismatch loudly.

#include "world_internal.h"

#include <string.h>

// Little-endian only until a big-endian CI cell exists to prove the
// swap path; the walker is the single place a swap would live.
#if defined(__BYTE_ORDER__) && (__BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__)
#error "maul3d snapshot v1 is little-endian only (add the swap in WalkBlocks first)"
#endif

#define M3_SNAPSHOT_MAGIC   0x4D33534Eu // 'M3SN'
#define M3_SNAPSHOT_VERSION 4u          // v4: inertia tensor and center of mass

// The math types are canonical field data only because they are
// provably padding-free; a change here is a format version bump.
_Static_assert(sizeof(m3Vec3) == 12, "m3Vec3 must be padding-free");
_Static_assert(sizeof(m3Quat) == 16, "m3Quat must be padding-free");
_Static_assert(sizeof(m3Pos3) == 24, "m3Pos3 must be padding-free");
_Static_assert(sizeof(m3Transform) == 40, "m3Transform must be padding-free");
_Static_assert(sizeof(m3Mat3) == 36, "m3Mat3 must be padding-free");

typedef struct m3SnapshotHeader
{
    uint32_t magic;
    uint32_t formatVersion;
    uint64_t configHash;
    uint64_t stepCount;
    int32_t bodyCapacity;
    int32_t maxIndex;
    int32_t freeHead;
    int32_t freeCount;
    int32_t retiredCount;
    int32_t shapeCapacity;
    int32_t shapeMaxIndex;
    int32_t shapeFreeHead;
    int32_t shapeFreeCount;
    int32_t shapeRetiredCount;
    int32_t pairCount;
    m3Vec3 gravity;
} m3SnapshotHeader;

_Static_assert(sizeof(m3SnapshotHeader) == 80, "snapshot header must be padding-free");

static uint64_t ConfigHash(void)
{
    // Everything that changes what the serialized bytes MEAN, and
    // nothing that does not (SIMD backend and worker count are
    // deliberately absent: the format is portable across them).
    uint64_t h = M3_HASH_INIT;
    int32_t version = m3GetVersion();
    int32_t solverRev = M3_SOLVER_REV;
    int32_t realSize = (int32_t)sizeof(m3real);
    int32_t posSize = (int32_t)sizeof(double);
    const char* fpPolicy = "contract-off;no-fast-math;no-fuse-4wide";
    h = m3Hash64(h, &version, 4);
    h = m3Hash64(h, &solverRev, 4);
    h = m3Hash64(h, &realSize, 4);
    h = m3Hash64(h, &posSize, 4);
    h = m3Hash64(h, fpPolicy, (int32_t)strlen(fpPolicy));
    return h;
}

typedef enum m3WalkMode
{
    m3_walkMeasure = 0,
    m3_walkWrite = 1,
    m3_walkRead = 2,
} m3WalkMode;

// The single source of truth for what persistent state IS. Every
// persistent array appears here exactly once, in canonical order.
// Measure returns the byte total; write and read stream against the
// caller's buffer at the returned running offset.
static int32_t WalkBlocks(m3World* world, uint8_t* out, const uint8_t* in, m3WalkMode mode)
{
    int32_t cursor = 0;
    int32_t cap = world->bodyCapacity;

#define M3_BLOCK(ptr, bytes)                                                                       \
    do                                                                                             \
    {                                                                                              \
        int32_t n = (int32_t)(bytes);                                                              \
        if (mode == m3_walkWrite)                                                                  \
        {                                                                                          \
            memcpy(out + cursor, (ptr), (size_t)n);                                                \
        }                                                                                          \
        else if (mode == m3_walkRead)                                                              \
        {                                                                                          \
            memcpy((ptr), in + cursor, (size_t)n);                                                 \
        }                                                                                          \
        cursor += n;                                                                               \
    } while (0)

    M3_BLOCK(world->transforms, cap * (int32_t)sizeof(m3Transform));
    M3_BLOCK(world->linearVelocities, cap * (int32_t)sizeof(m3Vec3));
    M3_BLOCK(world->angularVelocities, cap * (int32_t)sizeof(m3Vec3));
    M3_BLOCK(world->invMass, cap * (int32_t)sizeof(m3real));
    M3_BLOCK(world->invInertiaLocal, cap * (int32_t)sizeof(m3Mat3));
    M3_BLOCK(world->localCenters, cap * (int32_t)sizeof(m3Vec3));
    M3_BLOCK(world->gravityScales, cap * (int32_t)sizeof(m3real));
    M3_BLOCK(world->linearDamping, cap * (int32_t)sizeof(m3real));
    M3_BLOCK(world->angularDamping, cap * (int32_t)sizeof(m3real));
    M3_BLOCK(world->types, cap * (int32_t)sizeof(uint8_t));
    M3_BLOCK(world->userData, cap * (int32_t)sizeof(uint64_t));
    // Identity is state: generations, liveness, and the FIFO queue
    // restore exactly, so post-rollback id minting cannot diverge.
    M3_BLOCK(world->bodyPool.generations, cap * (int32_t)sizeof(uint16_t));
    M3_BLOCK(world->bodyPool.alive, cap * (int32_t)sizeof(uint8_t));
    M3_BLOCK(world->bodyPool.freeQueue, cap * (int32_t)sizeof(int32_t));
    M3_BLOCK(world->bodyShapeHead, cap * (int32_t)sizeof(int32_t));

    int32_t shapeCap = world->shapeCapacity;
    M3_BLOCK(world->shapeBody, shapeCap * (int32_t)sizeof(int32_t));
    M3_BLOCK(world->shapeType, shapeCap * (int32_t)sizeof(uint8_t));
    M3_BLOCK(world->shapeGeom, shapeCap * (int32_t)sizeof(m3ShapeGeom));
    M3_BLOCK(world->shapeDensity, shapeCap * (int32_t)sizeof(float));
    M3_BLOCK(world->shapeFriction, shapeCap * (int32_t)sizeof(float));
    M3_BLOCK(world->shapeRestitution, shapeCap * (int32_t)sizeof(float));
    M3_BLOCK(world->shapeUserData, shapeCap * (int32_t)sizeof(uint64_t));
    M3_BLOCK(world->shapeNext, shapeCap * (int32_t)sizeof(int32_t));
    M3_BLOCK(world->shapePool.generations, shapeCap * (int32_t)sizeof(uint16_t));
    M3_BLOCK(world->shapePool.alive, shapeCap * (int32_t)sizeof(uint8_t));
    M3_BLOCK(world->shapePool.freeQueue, shapeCap * (int32_t)sizeof(int32_t));

    M3_BLOCK(world->pairKeys, world->pairCapacity * (int32_t)sizeof(uint64_t));
    M3_BLOCK(world->manifolds, world->pairCapacity * (int32_t)sizeof(m3Manifold));

#undef M3_BLOCK
    return cursor;
}

int32_t m3World_SnapshotSize(m3WorldId worldId)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL)
    {
        return -1;
    }
    return (int32_t)sizeof(m3SnapshotHeader) + WalkBlocks(world, NULL, NULL, m3_walkMeasure);
}

int32_t m3World_Snapshot(m3WorldId worldId, void* out, int32_t capacity)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || out == NULL)
    {
        return -1;
    }
    int32_t size =
        (int32_t)sizeof(m3SnapshotHeader) + WalkBlocks(world, NULL, NULL, m3_walkMeasure);
    if (capacity < size)
    {
        return -1; // loud: the caller sized with m3World_SnapshotSize
    }

    m3SnapshotHeader header;
    memset(&header, 0, sizeof(header));
    header.magic = M3_SNAPSHOT_MAGIC;
    header.formatVersion = M3_SNAPSHOT_VERSION;
    header.configHash = ConfigHash();
    header.stepCount = world->stepCount;
    header.bodyCapacity = world->bodyCapacity;
    header.maxIndex = world->bodyPool.maxIndex;
    header.freeHead = world->bodyPool.freeHead;
    header.freeCount = world->bodyPool.freeCount;
    header.retiredCount = world->bodyPool.retiredCount;
    header.shapeCapacity = world->shapeCapacity;
    header.shapeMaxIndex = world->shapePool.maxIndex;
    header.shapeFreeHead = world->shapePool.freeHead;
    header.shapeFreeCount = world->shapePool.freeCount;
    header.shapeRetiredCount = world->shapePool.retiredCount;
    header.pairCount = world->pairCount;
    header.gravity = world->gravity;

    uint8_t* bytes = (uint8_t*)out;
    memcpy(bytes, &header, sizeof(header));
    WalkBlocks(world, bytes + sizeof(header), NULL, m3_walkWrite);
    return size;
}

bool m3World_Restore(m3WorldId worldId, const void* data, int32_t size)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || data == NULL || size < (int32_t)sizeof(m3SnapshotHeader))
    {
        return false;
    }
    m3SnapshotHeader header;
    memcpy(&header, data, sizeof(header));
    if (header.magic != M3_SNAPSHOT_MAGIC || header.formatVersion != M3_SNAPSHOT_VERSION ||
        header.configHash != ConfigHash() || header.bodyCapacity != world->bodyCapacity ||
        header.shapeCapacity != world->shapeCapacity)
    {
        // Wrong world shape or wrong build semantics: refuse loudly,
        // never a partial restore.
        return false;
    }
    int32_t expected =
        (int32_t)sizeof(m3SnapshotHeader) + WalkBlocks(world, NULL, NULL, m3_walkMeasure);
    if (size != expected)
    {
        return false;
    }

    world->stepCount = header.stepCount;
    world->gravity = header.gravity;
    world->bodyPool.maxIndex = header.maxIndex;
    world->bodyPool.freeHead = header.freeHead;
    world->bodyPool.freeCount = header.freeCount;
    world->bodyPool.retiredCount = header.retiredCount;
    world->shapePool.maxIndex = header.shapeMaxIndex;
    world->shapePool.freeHead = header.shapeFreeHead;
    world->shapePool.freeCount = header.shapeFreeCount;
    world->shapePool.retiredCount = header.shapeRetiredCount;
    world->pairCount = header.pairCount;
    WalkBlocks(world, NULL, (const uint8_t*)data + sizeof(header), m3_walkRead);
    return true;
}

uint64_t m3World_Hash(m3WorldId worldId)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL)
    {
        return 0;
    }
    // Curated deterministic state in canonical slot order: what the
    // simulation IS, not how it is stored. Dead slots contribute only
    // their liveness byte (destroy zeroes state, but the hash must not
    // depend on that coincidence).
    uint64_t h = M3_HASH_INIT;
    h = m3Hash64(h, &world->stepCount, 8);
    h = m3Hash64(h, &world->gravity, (int32_t)sizeof(m3Vec3));
    int32_t maxIndex = world->bodyPool.maxIndex;
    for (int32_t i = 0; i < maxIndex; ++i)
    {
        uint8_t alive = world->bodyPool.alive[i];
        h = m3Hash64(h, &alive, 1);
        if (alive == 0)
        {
            continue;
        }
        h = m3Hash64(h, &world->transforms[i], (int32_t)sizeof(m3Transform));
        h = m3Hash64(h, &world->linearVelocities[i], (int32_t)sizeof(m3Vec3));
        h = m3Hash64(h, &world->angularVelocities[i], (int32_t)sizeof(m3Vec3));
        h = m3Hash64(h, &world->invMass[i], (int32_t)sizeof(m3real));
        h = m3Hash64(h, &world->invInertiaLocal[i], (int32_t)sizeof(m3Mat3));
        h = m3Hash64(h, &world->localCenters[i], (int32_t)sizeof(m3Vec3));
        h = m3Hash64(h, &world->types[i], 1);
    }
    int32_t maxShape = world->shapePool.maxIndex;
    for (int32_t i = 0; i < maxShape; ++i)
    {
        uint8_t alive = world->shapePool.alive[i];
        h = m3Hash64(h, &alive, 1);
        if (alive == 0)
        {
            continue;
        }
        h = m3Hash64(h, &world->shapeBody[i], 4);
        h = m3Hash64(h, &world->shapeType[i], 1);
        h = m3Hash64(h, &world->shapeGeom[i], (int32_t)sizeof(m3ShapeGeom));
        h = m3Hash64(h, &world->shapeDensity[i], 4);
        h = m3Hash64(h, &world->shapeFriction[i], 4);
        h = m3Hash64(h, &world->shapeRestitution[i], 4);
    }
    // Pairs and manifolds: warm-start impulses are simulation state
    // (they steer the next solve), so they are part of what the world
    // IS.
    h = m3Hash64(h, &world->pairCount, 4);
    for (int32_t i = 0; i < world->pairCount; ++i)
    {
        h = m3Hash64(h, &world->pairKeys[i], 8);
        h = m3Hash64(h, &world->manifolds[i], (int32_t)sizeof(m3Manifold));
    }
    return h;
}
