// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen

#ifndef MAUL3D_WORLD_H
#define MAUL3D_WORLD_H

#include "maul3d/math.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /// The task interface (2b-11): the HOST owns the threads, the
    /// library only describes work. enqueueTask receives a task
    /// function, the item count, and a minimum grain; the host splits
    /// [0, itemCount) into subranges, runs each range exactly once on
    /// any thread in any order, and returns an opaque handle that
    /// finishTask must block on. Determinism holds for ANY split and
    /// ANY schedule because parallel phases only ever write disjoint
    /// slots; the worker-twin gate proves it on every commit. Leave
    /// both hooks null for serial execution.
    typedef void m3TaskFn(int32_t startIndex, int32_t endIndex, void* taskContext);
    typedef void* m3EnqueueTaskFn(m3TaskFn* task, int32_t itemCount, int32_t minRange,
                                  void* taskContext, void* userContext);
    typedef void m3FinishTaskFn(void* userTask, void* userContext);

    /// World definition. Build with m3DefaultWorldDef so the cookie is
    /// valid; a zeroed or hand-rolled def is rejected loudly.
    typedef struct m3WorldDef
    {
        m3Vec3 gravity;
        int32_t bodyCapacity;
        int32_t shapeCapacity;
        int32_t meshCapacity;         // static triangle-mesh slots (24 KB each)
        int32_t workerCount;          // twin worlds with different counts must hash equal
        m3EnqueueTaskFn* enqueueTask; // both null = serial (the default)
        m3FinishTaskFn* finishTask;
        void* userTaskContext;
        int32_t internalValue;
    } m3WorldDef;

    M3_API m3WorldDef m3DefaultWorldDef(void);

    /// Create a world. Returns the null id on an invalid def or an
    /// exhausted world table (diagnostic in debug builds).
    M3_API m3WorldId m3CreateWorld(const m3WorldDef* def);
    M3_API void m3DestroyWorld(m3WorldId worldId);
    M3_API bool m3World_IsValid(m3WorldId worldId);

    /// Advance the simulation: collide, then the Soft Step solver
    /// with the given substep count. Deterministic: same inputs, same
    /// bits, on every platform and backend. Journaled.
    M3_API void m3World_Step(m3WorldId worldId, float dt, int32_t substeps);

    /// Snapshot and rollback, first-class from day one. The format is
    /// portable and versioned: field blocks in little-endian order with
    /// a header carrying a config hash (engine version, solver
    /// revision, precision, FP policy). Restore refuses a mismatched
    /// config or format loudly, restores in place, and the restored
    /// world resimulates bit-exactly (the rollback gate, task 10).
    M3_API int32_t m3World_SnapshotSize(m3WorldId worldId);
    M3_API int32_t m3World_Snapshot(m3WorldId worldId, void* out, int32_t capacity);
    M3_API bool m3World_Restore(m3WorldId worldId, const void* data, int32_t size);

    /// FNV-1a 64 over the curated deterministic state (transforms,
    /// velocities, mass, types, step count, gravity) in canonical slot
    /// order. The value every gate compares.
    M3_API uint64_t m3World_Hash(m3WorldId worldId);

    /// Journal: every mutation of the world is a discrete recorded op,
    /// and replaying the stream through the same internal functions
    /// reproduces the world bit for bit. Begin hands the world a
    /// caller-owned buffer; End returns the bytes written (or -1 after
    /// an overflow, loudly); Replay applies a stream to this world.
    M3_API bool m3World_JournalBegin(m3WorldId worldId, void* buffer, int32_t capacity);
    M3_API int32_t m3World_JournalEnd(m3WorldId worldId);
    M3_API bool m3World_JournalReplay(m3WorldId worldId, const void* data, int32_t size);

#ifdef __cplusplus
}
#endif

#endif // MAUL3D_WORLD_H
