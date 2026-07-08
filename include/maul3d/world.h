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
        int32_t jointCapacity;        // articulation slots (2c-2)
        int32_t workerCount;          // twin worlds with different counts must hash equal
        m3EnqueueTaskFn* enqueueTask; // both null = serial (the default)
        m3FinishTaskFn* finishTask;
        void* userTaskContext;
        int32_t internalValue;
    } m3WorldDef;

    /// The pinned world defaults: gravity (0, -10, 0), 1024 bodies,
    /// 2048 shapes, 64 joints, 4 meshes, one worker, no task hooks,
    /// and the def cookie every create demands.
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

    /// Contact events (2b-13): begin fires on the step two shapes
    /// first touch, end fires on the step they separate. Derived from
    /// the canonical pair walk, so replay produces the identical
    /// stream. Pointers are valid until the next step or restore.
    /// Destroying a shape emits no end event (the id would be stale);
    /// events are transient observers and never snapshot state.
    typedef struct m3ContactEvent
    {
        m3ShapeId shapeA; // the lower shape index of the pair
        m3ShapeId shapeB;
    } m3ContactEvent;

    /// Contact begin and end streams for the LAST step, in canonical
    /// deterministic order. Valid until the next step, restore, or
    /// world destruction (restore clears them: events are
    /// observations, not state). Pass a non-null count.
    M3_API const m3ContactEvent* m3World_ContactBeginEvents(m3WorldId worldId, int32_t* count);
    M3_API const m3ContactEvent* m3World_ContactEndEvents(m3WorldId worldId, int32_t* count);

    /// Sensor overlap events, the same law as contact events but in
    /// their own streams (a sensor touch is not a contact). shapeA
    /// is the lower shape index; either side may be the sensor.
    M3_API const m3ContactEvent* m3World_SensorBeginEvents(m3WorldId worldId, int32_t* count);
    M3_API const m3ContactEvent* m3World_SensorEndEvents(m3WorldId worldId, int32_t* count);

    /// Closest-hit ray cast: origin in world doubles, translation =
    /// direction times reach. fraction in [0, 1] along the
    /// translation; front faces only (winding is a contract). Ties
    /// break to the lower shape index. A zero translation misses.
    typedef struct m3RayHit
    {
        m3ShapeId shape;
        m3Pos3 point;
        m3Vec3 normal;
        m3real fraction;
        bool hit;
    } m3RayHit;

    /// The closest front-face hit along origin + t * translation for
    /// t in [0, 1]. Rays MISS shapes they start inside or exactly on
    /// (front faces only); ask m3World_PointInside for containment.
    M3_API m3RayHit m3World_CastRayClosest(m3WorldId worldId, m3Pos3 origin, m3Vec3 translation);

    /// Every ray hit along the translation, one entry point per
    /// shape, sorted by fraction (ties to the lower shape index).
    /// Returns the count written (never more than capacity; excess
    /// hits are dropped from the FAR end, deterministically).
    M3_API int32_t m3World_CastRayAll(m3WorldId worldId, m3Pos3 origin, m3Vec3 translation,
                                      m3RayHit* hits, int32_t capacity);

    /// Closest-hit shape casts: sweep a sphere or a capsule along a
    /// translation. fraction is the earliest touch in [0, 1]; a cast
    /// that STARTS overlapped hits at fraction zero with a zero
    /// normal (the documented start-inside contract; rays instead
    /// MISS shapes they start inside, front faces only).
    M3_API m3RayHit m3World_CastSphereClosest(m3WorldId worldId, m3Pos3 center, m3real radius,
                                              m3Vec3 translation);
    M3_API m3RayHit m3World_CastCapsuleClosest(m3WorldId worldId, m3Pos3 center, m3Vec3 point1,
                                               m3Vec3 point2, m3real radius, m3Vec3 translation);

    /// The first shape (lowest index) whose volume contains the
    /// point, or the null id. Meshes are open surfaces and never
    /// contain points; planes are solid half spaces.
    M3_API m3ShapeId m3World_PointInside(m3WorldId worldId, m3Pos3 point);

    /// Shapes whose tight bounds overlap the box, in ascending shape
    /// index order. Returns the count written.
    M3_API int32_t m3World_OverlapAabb(m3WorldId worldId, m3Pos3 lo, m3Pos3 hi, m3ShapeId* shapes,
                                       int32_t capacity);

    /// Shapes within reach of the sphere (exact per family), in
    /// ascending shape index order. Returns the count written.
    M3_API int32_t m3World_OverlapSphere(m3WorldId worldId, m3Pos3 center, m3real radius,
                                         m3ShapeId* shapes, int32_t capacity);

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
    /// Replay a recorded session into this world. ATOMIC: on any
    /// refusal (truncation, corruption, an op that cannot re-mint
    /// its recorded id) the world is restored to its pre-call state
    /// and false returns; a half-applied session is impossible.
    M3_API bool m3World_JournalReplay(m3WorldId worldId, const void* data, int32_t size);

#ifdef __cplusplus
}
#endif

#endif // MAUL3D_WORLD_H
