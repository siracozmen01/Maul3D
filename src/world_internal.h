// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The world: SoA body state in persistent arrays (every one of them a
// future M3_BLOCK in the snapshot walker), the id pool behind body
// handles, the per-step scratch stack, and the journal cursor. No
// pointers inside persistent state; slots cross-reference by index.

#ifndef MAUL3D_WORLD_INTERNAL_H
#define MAUL3D_WORLD_INTERNAL_H

#include "allocator.h"

#include "maul3d/body.h"
#include "maul3d/world.h"

#define M3_MAX_WORLDS 64

// Bumped on ANY change that alters simulation behavior (solver math,
// integration order, constants). Part of the snapshot config hash, so
// a snapshot from a different behavior revision is refused loudly
// instead of silently diverging (the Jolt friction-model lesson).
#define M3_SOLVER_REV 1

// Def cookies: a def that did not come from its m3Default*Def factory
// is rejected loudly (the Maul2D pattern).
#define M3_COOKIE       0x4D33u // 'M3'
#define M3_WORLD_COOKIE ((int32_t)(M3_COOKIE ^ ((int32_t)sizeof(m3WorldDef) << 8)))
#define M3_BODY_COOKIE  ((int32_t)(M3_COOKIE ^ ((int32_t)sizeof(m3BodyDef) << 8) ^ 1))

// Journal ops. The stream is [i32 op][i32 size][payload], replayed
// through the same internal functions the public API uses.
typedef enum m3Op
{
    m3_opStep = 1, // reserved for task 9
    m3_opCreateBody = 2,
    m3_opDestroyBody = 3,
    m3_opSetLinearVelocity = 4,
    m3_opSetAngularVelocity = 5,
    m3_opCreateShape = 6, // reserved for task 7
} m3Op;

typedef struct m3World
{
    // World-global state (snapshot header material, task 6).
    m3Vec3 gravity;
    uint64_t stepCount;
    int32_t bodyCapacity;
    int32_t shapeCapacity;
    int32_t workerCount;
    uint16_t generation;  // this world slot's generation
    uint16_t worldIndex0; // 0-based slot in the world table

    // Body identity.
    m3IdPool bodyPool;

    // SoA body state, hot fields first. All persistent, all walked by
    // the snapshot in task 6.
    m3Transform* transforms;
    m3Vec3* linearVelocities;
    m3Vec3* angularVelocities;
    m3real* invMass;
    m3real* invInertia; // scalar: 2a bodies are spheres (isotropic)
    m3real* gravityScales;
    m3real* linearDamping;
    m3real* angularDamping;
    uint8_t* types;
    uint64_t* userData;

    // Per-step scratch (lifetime 2: never snapshotted).
    m3Stack scratch;

    // Journal cursor over a caller-owned buffer.
    uint8_t* journalBuffer;
    int32_t journalCapacity;
    int32_t journalCursor;
    int32_t journalActive;
    int32_t journalOverflow;
} m3World;

// Registry lookup: NULL for a stale or null id.
m3World* m3WorldFromId(m3WorldId worldId);

// Registry lookup by slot only (no generation check): body and shape
// ids carry their own generation, so their world reference resolves by
// slot, the Maul2D FromIndex0 pattern.
m3World* m3WorldFromIndex0(uint16_t index0);

// Slot lookup with generation check: -1 for a stale or foreign id.
int32_t m3BodySlot(const m3World* world, m3BodyId bodyId);

// Internal mutation functions: the ONLY paths that change state. The
// public API validates, journals, then calls these; replay calls them
// directly, so a replayed world takes the identical code path.
int32_t m3CreateBodyInternal(m3World* world, const m3BodyDef* def);
void m3DestroyBodyInternal(m3World* world, int32_t index);
void m3SetLinearVelocityInternal(m3World* world, int32_t index, m3Vec3 velocity);
void m3SetAngularVelocityInternal(m3World* world, int32_t index, m3Vec3 velocity);

void m3JournalRecord(m3World* world, int32_t op, const void* payload, int32_t bytes);

#endif // MAUL3D_WORLD_INTERNAL_H
