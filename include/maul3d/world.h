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

    /// Query-side filter (8-1): a query behaves like a shape with
    /// these bits. A shape is visible to the query when the
    /// query's category intersects the shape's mask AND the
    /// shape's category intersects the query's mask. The default
    /// sees everything.
    typedef struct m3QueryFilter
    {
        uint64_t categoryBits;
        uint64_t maskBits;
    } m3QueryFilter;

    M3_API m3QueryFilter m3DefaultQueryFilter(void);

    /// World definition. Build with m3DefaultWorldDef so the cookie is
    /// valid; a zeroed or hand-rolled def is rejected loudly.
    typedef struct m3WorldDef
    {
        m3Vec3 gravity;
        int32_t bodyCapacity;
        int32_t shapeCapacity;
        int32_t meshCapacity; // static triangle-mesh slots (24 KB each)
        int32_t jointCapacity;
        int32_t voxelCapacity;        // voxel chunk slots (3-1)
        int32_t characterCapacity;    // character controllers (4-4)
        int32_t vehicleCapacity;      // raycast vehicles (5-1)
        int32_t softBodyCapacity;     // XPBD lattices (7-1)
        int32_t workerCount;          // twin worlds with different counts must hash equal
        m3EnqueueTaskFn* enqueueTask; // both null = serial (the default)
        m3FinishTaskFn* finishTask;
        void* userTaskContext;
        // Tuning knobs (8-4). All journaled-settable at runtime too;
        // they are engine STATE: snapshots carry them and replays
        // reproduce them. Defaults are the reference values.
        float contactHertz;         // contact softness frequency (30)
        float contactDampingRatio;  // contact softness damping (10)
        float contactPushMaxSpeed;  // max depenetration speed (3)
        float restitutionThreshold; // impact speed under it: no bounce (1)
        float maximumLinearSpeed;   // hard velocity cap (400)
        int32_t enableSleeping;     // 1 = islands may sleep (default 1)
        int32_t enableContinuous;   // 1 = CCD phase runs (default 1)
        float hitEventThreshold;    // hit events need approach speed
                                    // above this (default 1) (8-5)
        int32_t internalValue;
    } m3WorldDef;

    /// The pinned world defaults: gravity (0, -10, 0), 1024 bodies,
    /// 2048 shapes, 64 joints, 4 meshes, 4 voxel chunks, 4
    /// characters, 2 vehicles, one worker, no task hooks, and the
    /// def cookie every create demands.
    M3_API m3WorldDef m3DefaultWorldDef(void);

    /// Create a world. Returns the null id on an invalid def or an
    /// exhausted world table (diagnostic in debug builds).
    M3_API m3WorldId m3CreateWorld(const m3WorldDef* def);
    M3_API void m3DestroyWorld(m3WorldId worldId);
    M3_API bool m3World_IsValid(m3WorldId worldId);

    /// Set the gravity vector. Journaled; sleeping islands stay
    /// asleep until disturbed (the reference behavior).
    M3_API void m3World_SetGravity(m3WorldId worldId, m3Vec3 gravity);

    /// Current gravity.
    M3_API m3Vec3 m3World_GetGravity(m3WorldId worldId);

    /// Contact softness tuning: frequency (hertz), damping ratio and
    /// the maximum depenetration speed. Journaled. Static contacts
    /// use twice the frequency, like the reference.
    M3_API void m3World_SetContactTuning(m3WorldId worldId, float hertz, float dampingRatio,
                                         float pushMaxSpeed);

    /// Impact speed below the threshold does not bounce. Journaled.
    M3_API void m3World_SetRestitutionThreshold(m3WorldId worldId, float value);

    /// Hard cap on any body's linear speed, applied every substep.
    /// Journaled.
    M3_API void m3World_SetMaximumLinearSpeed(m3WorldId worldId, float value);

    /// Hard cap on any body's angular speed in rad/s, applied every
    /// substep. The default (800) is a catastrophe guard, not a
    /// gameplay clamp; bodies flagged with
    /// m3Body_SetAllowFastRotation bypass it. Journaled.
    M3_API void m3World_SetMaximumAngularSpeed(m3WorldId worldId, float value);

    /// Turn island sleeping on or off. Turning it OFF wakes every
    /// sleeping body (the reference behavior). Journaled.
    M3_API void m3World_EnableSleeping(m3WorldId worldId, bool flag);

    /// Whether island sleeping is enabled.
    M3_API bool m3World_IsSleepingEnabled(m3WorldId worldId);

    /// Turn the continuous phase (CCD) on or off. Journaled.
    M3_API void m3World_EnableContinuous(m3WorldId worldId, bool flag);

    /// Whether the continuous phase is enabled.
    M3_API bool m3World_IsContinuousEnabled(m3WorldId worldId);

    /// Wind (11-3): a deterministic field applied to soft-body
    /// particles as proportional drag toward the wind velocity.
    /// The gust is a sine of an ACCUMULATED phase (state, in the
    /// snapshot), so rollbacks resume the exact same wave: no host
    /// randomness exists anywhere in it. speed zero disables.
    /// direction must be near-unit when speed is nonzero. Rigid
    /// bodies are not wind-blown in v1 (documented: the force API
    /// already serves them); journaled.
    M3_API void m3World_SetWind(m3WorldId worldId, m3Vec3 direction, float speed, float gustHertz,
                                float gustScale);

    /// Advance the simulation: collide, then the Soft Step solver
    /// with the given substep count (1..256; out of range refuses
    /// loudly). Deterministic: same inputs, same bits, on every
    /// platform and backend. Journaled.
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

    /// A fragment-spawn event (3-3): a voxel edit disconnected an
    /// island from its chunk's base layer (y = 0 voxels anchor a
    /// chunk; an island with no path to the base cannot stand). The
    /// engine REMOVES the island from the grid as part of the edit's
    /// state transition and emits this event; it never spawns bodies
    /// (what a fragment becomes is the host's decision; the
    /// reference recipe feeds the island's voxels to the built-in
    /// QuickHull and creates a hull body). Events and recipes are
    /// valid until the next step, restore, or edit-free frame
    /// boundary of your choosing; the next m3World_Step clears them.
    typedef struct m3FragmentEvent
    {
        m3ShapeId chunkShape;
        int32_t voxelCount;
        /// Range into m3World_FragmentRecipe: chunk-local linear
        /// voxel indices (v = x + 16 * (y + 16 * z)). recipeStart is
        /// -1 when the recipe buffer overflowed (the event and the
        /// removal still happened; the bounds below still describe
        /// the island).
        int32_t recipeStart;
        int32_t recipeCount;
        m3Vec3 comChunk;     /// island center of mass, chunk frame
        m3Pos3 comWorld;     /// the same point in world space at emit time
        m3real mass;         /// at density one (cell volume times count)
        uint8_t boundsLo[3]; /// island voxel bounds, inclusive
        uint8_t boundsHi[3];
        uint8_t pad[2];
    } m3FragmentEvent;

    M3_API const m3FragmentEvent* m3World_FragmentEvents(m3WorldId worldId, int32_t* count);
    M3_API const uint16_t* m3World_FragmentRecipe(m3WorldId worldId, int32_t* count);
    /// Islands beyond the event capacity are still removed from the
    /// grid (state transitions stay pure); only their EVENTS drop,
    /// and this counter says how many, loudly.
    M3_API int32_t m3World_FragmentEventsDropped(m3WorldId worldId);

    /// Sensor overlap events, the same law as contact events but in
    /// their own streams (a sensor touch is not a contact). shapeA
    /// is the lower shape index; either side may be the sensor.
    M3_API const m3ContactEvent* m3World_SensorBeginEvents(m3WorldId worldId, int32_t* count);
    M3_API const m3ContactEvent* m3World_SensorEndEvents(m3WorldId worldId, int32_t* count);

    /// A hit event (8-5): two shapes collided with approach speed
    /// above the world threshold. Emitted at most once per contact
    /// per step, for the fastest-approaching manifold point, and
    /// only when either shape opted in (m3Shape_EnableHitEvents).
    /// Streams are transient observers like contact events.
    typedef struct m3HitEvent
    {
        m3ShapeId shapeA;
        m3ShapeId shapeB;
        m3Pos3 point;         /// approximate contact point, world space
        m3Vec3 normal;        /// from shape A to shape B
        m3real approachSpeed; /// relative normal speed at impact, > 0
    } m3HitEvent;

    M3_API const m3HitEvent* m3World_HitEvents(m3WorldId worldId, int32_t* count);
    /// Hits beyond capacity still simulate; only their events drop,
    /// and this counter says how many, loudly.
    M3_API int32_t m3World_HitEventsDropped(m3WorldId worldId);

    /// Hit events require approach speed above this. Journaled.
    M3_API void m3World_SetHitEventThreshold(m3WorldId worldId, float value);

    /// A body move event (8-5): one per body that MOVED this step
    /// (every mover), in ascending body order, carrying the post-step
    /// transform. fellAsleep marks the step a body drops off; a
    /// sleeping body emits nothing until it wakes. Render sync reads
    /// this instead of polling every body.
    typedef struct m3BodyMoveEvent
    {
        m3BodyId body;
        m3Transform transform;
        bool fellAsleep;
    } m3BodyMoveEvent;

    M3_API const m3BodyMoveEvent* m3World_BodyMoveEvents(m3WorldId worldId, int32_t* count);

    /// A joint break event (8-5 machinery; joints learn to break in
    /// the joint runtime slice). The id is already stale when the
    /// event is read: the joint destroyed itself. Use it as a key,
    /// not a handle.
    typedef struct m3JointBreakEvent
    {
        m3JointId joint;
    } m3JointBreakEvent;

    M3_API const m3JointBreakEvent* m3World_JointBreakEvents(m3WorldId worldId, int32_t* count);

    /// Pre-solve veto (8-5): called during contact preparation, in
    /// canonical pair order, for pairs where either shape opted in
    /// (m3Shape_EnablePreSolve). Return false to disable the contact
    /// for THIS step (the one-way platform tool). point is the
    /// deepest contact point, normal points from shape A to shape B.
    ///
    /// THE DETERMINISM CONTRACT, LOUD: the journal records inputs,
    /// never host whims. The callback MUST be a pure function of its
    /// arguments (and of host state that is itself bit-identical on
    /// replay). Register the SAME pure callback before replaying a
    /// journal or resimulating from a snapshot; a callback that
    /// consults a clock, a random source, or mutable host state
    /// breaks bit-exactness and that breakage is YOURS. The engine
    /// calls it from the serial prepare pass, never from workers.
    typedef bool m3PreSolveFn(m3ShapeId shapeA, m3ShapeId shapeB, m3Pos3 point, m3Vec3 normal,
                              void* context);

    /// Register (or clear with NULL) the pre-solve callback. Host
    /// wiring like the task hooks: never journaled, never snapshot
    /// state; pair it with the contract above.
    M3_API void m3World_SetPreSolveCallback(m3WorldId worldId, m3PreSolveFn* fn, void* context);

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
    /// Filtered variants (8-1): the query carries an m3QueryFilter
    /// (defined in shape.h) and behaves like a shape with those
    /// bits; the unfiltered forms see everything.
    M3_API m3RayHit m3World_CastRayClosestEx(m3WorldId worldId, m3Pos3 origin, m3Vec3 translation,
                                             m3QueryFilter filter);

    /// Every ray hit along the translation, one entry point per
    /// shape, sorted by fraction (ties to the lower shape index).
    /// Returns the count written (never more than capacity; excess
    /// hits are dropped from the FAR end, deterministically).
    M3_API int32_t m3World_CastRayAll(m3WorldId worldId, m3Pos3 origin, m3Vec3 translation,
                                      m3RayHit* hits, int32_t capacity);
    M3_API int32_t m3World_CastRayAllEx(m3WorldId worldId, m3Pos3 origin, m3Vec3 translation,
                                        m3RayHit* hits, int32_t capacity, m3QueryFilter filter);

    /// Closest-hit shape casts: sweep a sphere or a capsule along a
    /// translation. fraction is the earliest touch in [0, 1]; a cast
    /// that STARTS overlapped hits at fraction zero with a zero
    /// normal (the documented start-inside contract; rays instead
    /// MISS shapes they start inside, front faces only).
    /// Generic convex casts (4-1): a box (with orientation) or a
    /// caller point cloud (2..24 points, base-relative) swept along
    /// a translation, skinless. Same contracts as every cast: the
    /// earliest touch in [0, 1], start-overlapped reports fraction
    /// zero with a zero normal, hostile inputs miss. A single point
    /// is a ray: use the ray casts.
    M3_API m3RayHit m3World_CastBoxClosest(m3WorldId worldId, m3Pos3 center, m3Vec3 halfExtents,
                                           m3Quat rotation, m3Vec3 translation);
    M3_API m3RayHit m3World_CastBoxClosestEx(m3WorldId worldId, m3Pos3 center, m3Vec3 halfExtents,
                                             m3Quat rotation, m3Vec3 translation,
                                             m3QueryFilter filter);
    M3_API m3RayHit m3World_CastHullClosest(m3WorldId worldId, m3Pos3 base, const m3Vec3* points,
                                            int32_t count, m3Vec3 translation);
    M3_API m3RayHit m3World_CastHullClosestEx(m3WorldId worldId, m3Pos3 base, const m3Vec3* points,
                                              int32_t count, m3Vec3 translation,
                                              m3QueryFilter filter);
    M3_API m3RayHit m3World_CastSphereClosest(m3WorldId worldId, m3Pos3 center, m3real radius,
                                              m3Vec3 translation);
    M3_API m3RayHit m3World_CastSphereClosestEx(m3WorldId worldId, m3Pos3 center, m3real radius,
                                                m3Vec3 translation, m3QueryFilter filter);
    M3_API m3RayHit m3World_CastCapsuleClosest(m3WorldId worldId, m3Pos3 center, m3Vec3 point1,
                                               m3Vec3 point2, m3real radius, m3Vec3 translation);
    M3_API m3RayHit m3World_CastCapsuleClosestEx(m3WorldId worldId, m3Pos3 center, m3Vec3 point1,
                                                 m3Vec3 point2, m3real radius, m3Vec3 translation,
                                                 m3QueryFilter filter);

    /// The first shape (lowest index) whose volume contains the
    /// point, or the null id. Meshes are open surfaces and never
    /// contain points; planes are solid half spaces.
    M3_API m3ShapeId m3World_PointInside(m3WorldId worldId, m3Pos3 point);

    /// Shapes whose tight bounds overlap the box, in ascending shape
    /// index order. Returns the count written.
    M3_API int32_t m3World_OverlapAabb(m3WorldId worldId, m3Pos3 lo, m3Pos3 hi, m3ShapeId* shapes,
                                       int32_t capacity);
    M3_API int32_t m3World_OverlapAabbEx(m3WorldId worldId, m3Pos3 lo, m3Pos3 hi, m3ShapeId* shapes,
                                         int32_t capacity, m3QueryFilter filter);

    /// Shapes within reach of the sphere (exact per family), in
    /// ascending shape index order. Returns the count written.
    M3_API int32_t m3World_OverlapSphere(m3WorldId worldId, m3Pos3 center, m3real radius,
                                         m3ShapeId* shapes, int32_t capacity);
    M3_API int32_t m3World_OverlapSphereEx(m3WorldId worldId, m3Pos3 center, m3real radius,
                                           m3ShapeId* shapes, int32_t capacity,
                                           m3QueryFilter filter);

    /// Explosion definition (13-2): one journaled call pushes every
    /// dynamic convex shape in range. The impulse scales with the
    /// area the shape shows to the blast (the reference model),
    /// fades linearly to zero across the falloff band past the
    /// radius, wakes everything it reaches, and may be negative for
    /// an implosion. Spheres, capsules, and hulls respond; meshes
    /// and planes are static; voxel chunks couple through charges.
    typedef struct m3ExplosionDef
    {
        m3Pos3 position;      // world-space center
        m3QueryFilter filter; // which shapes the blast may touch
        float radius;         // full-impulse radius, >= 0
        float falloff;        // fade band past the radius, >= 0
        float impulsePerArea; // impulse per m^2 of facing area
        float voxelCarve;     // carve radius into voxel chunks, 0 = off
        float softPush;       // soft particle push scale, default 1
        int32_t internalValue;
    } m3ExplosionDef;

    /// Returns a def with pinned defaults (10 m radius, 5 m falloff,
    /// zero impulse, no carve, soft push 1, open filter) and a valid
    /// cookie.
    M3_API m3ExplosionDef m3DefaultExplosionDef(void);

    /// Apply the explosion now. Journaled; hostile defs apply
    /// nothing and journal nothing.
    M3_API void m3World_Explode(m3WorldId worldId, const m3ExplosionDef* def);

    /// FNV-1a 64 over the curated deterministic state (transforms,
    /// velocities, mass, types, step count, gravity) in canonical slot
    /// order. The value every gate compares.
    M3_API uint64_t m3World_Hash(m3WorldId worldId);

    /// Live contact readback (14-3): one entry per manifold touching
    /// the queried body or shape, in canonical pair order. Pure
    /// observer data; points are world space at read time.
    typedef struct m3ContactData
    {
        m3ShapeId shapeA;
        m3ShapeId shapeB;
        m3Vec3 normal; // from shape A toward shape B
        int32_t pointCount;
        m3Pos3 points[4];
        float separations[4];    // negative = penetration
        float normalImpulses[4]; // last step's warm-start payload
    } m3ContactData;

    /// Live-world counters (14-1): pure observer data. Reading them
    /// never touches the simulation, the snapshot, or the hash. The
    /// count fields are themselves deterministic (twins report
    /// identical counts); islandCount, colorCount, and scratchPeak
    /// describe the last completed step and are zero before it.
    typedef struct m3Counters
    {
        int32_t bodyCount;
        int32_t shapeCount;
        int32_t jointCount;
        int32_t contactCount; // live broadphase pairs with manifolds
        int32_t awakeCount;   // awake dynamic bodies
        int32_t islandCount;  // dynamic islands in the last step
        int32_t colorCount;   // solver graph colors in the last step
        int32_t characterCount;
        int32_t vehicleCount;
        int32_t softBodyCount;
        int32_t voxelChunkCount;
        int32_t hullCount; // interned hull slabs
        int32_t meshCount; // interned mesh slabs
        int32_t treeHeight;
        int32_t scratchPeak;     // step scratch high water, bytes
        int32_t scratchCapacity; // step scratch reserve, bytes
        int32_t snapshotBytes;   // exact m3World_Snapshot size now
        int64_t allocCalls;      // process-global persistent allocs
        int64_t freeCalls;
    } m3Counters;

    M3_API m3Counters m3World_GetCounters(m3WorldId worldId);

    /// Wall-clock milliseconds per phase of the LAST COMPLETED step
    /// (14-1): honest observer timing from a monotonic clock. Never
    /// deterministic, never hashed, never serialized; zero before
    /// the first step, and a step that stalls on scratch growth
    /// keeps the previous profile.
    typedef struct m3Profile
    {
        float step;        // the whole of m3World_Step
        float vehicles;    // suspension and drivetrain pass
        float broadphase;  // pair update
        float narrowphase; // manifold generation
        float events;      // contact/sensor event merge walk
        float prepare;     // sizing, sweep capture, island wake
        float solve;       // constraint preparation and substeps
        float continuous;  // CCD sweeps including voxel TOI
        float sleep;       // island sleep pass
        float characters;  // rider carry
        float softBodies;  // the soft body pass
    } m3Profile;

    M3_API m3Profile m3World_GetProfile(m3WorldId worldId);

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
