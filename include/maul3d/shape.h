// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen

#ifndef MAUL3D_SHAPE_H
#define MAUL3D_SHAPE_H

#include "maul3d/body.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum m3ShapeType
    {
        m3_sphereShape = 0,
        m3_planeShape = 1,   // static bodies only: an infinite half-space
        m3_hullShape = 2,    // convex hull (boxes in 2b-3; point clouds later)
        m3_capsuleShape = 3, // a segment with a radius
        m3_meshShape = 4,    // static triangle soup (2b-9)
        m3_voxelShape = 5,   // dense 16^3 voxel chunk (3-1)
    } m3ShapeType;

    typedef struct m3Sphere
    {
        m3Vec3 center; // body frame; must be the origin on dynamic bodies in 2a
        m3real radius;
    } m3Sphere;

    /// Half-space: points p with dot(normal, p) <= offset are solid.
    /// A native plane is a deliberate addition over the reference
    /// (which fakes one with a huge box or a height field).
    typedef struct m3Plane
    {
        m3Vec3 normal; // normalized on create
        m3real offset;
    } m3Plane;

    typedef struct m3ShapeDef
    {
        float density; // kg/m^3
        float friction;
        float restitution;
        /// Collision filtering (8-1). A pair collides when each
        /// side's category intersects the other's mask, unless a
        /// shared nonzero groupIndex overrides: positive forces
        /// collision, negative forbids it (the reference rule).
        /// Defaults: category 1, mask all bits, group 0.
        uint64_t categoryBits;
        uint64_t maskBits;
        int32_t groupIndex;
        /// Rolling resistance (6-3): a dimensionless material knob
        /// braking relative rotation at contacts, mixed by maximum
        /// across the pair and scaled by the pair's extent (the
        /// reference recipe). Zero (the default) rolls free; around
        /// 0.01 to 0.1 reads as soft ground. Without it a sphere
        /// pile literally never stops rolling or sleeping.
        float rollingResistance;
        uint64_t userData;
        /// A sensor detects overlap and fires its own begin and end
        /// events but never produces contact response: bodies pass
        /// through, bullets do not stop, sleepers are not woken.
        /// Sensors do not sense other sensors.
        bool isSensor;
        bool enableHitEvents;      // 8-5: default false, streams cost
        bool enablePreSolveEvents; // 8-5: default false, veto calls cost
        /// Compound offset (10-1): the shape's transform relative to
        /// its body. Identity by default; a near-unit rotation is
        /// demanded, garbage refuses at create.
        m3Vec3 localPosition;
        m3Quat localRotation;
        int32_t internalValue;
    } m3ShapeDef;

    /// Returns a def with pinned defaults (density 1, friction 0.6,
    /// restitution 0) and a valid cookie.
    M3_API m3ShapeDef m3DefaultShapeDef(void);

    /// Create a sphere on a body and recompute the body's mass from
    /// its shapes (sphere mass = density * 4/3 pi r^3, inertia
    /// 0.4 m r^2). Returns the null id on an invalid def, a stale
    /// body, an exhausted pool, or an off-origin center on a dynamic
    /// body (the 2a rule: scalar inertia stays exact; the full tensor
    /// arrives in 2b).
    M3_API m3ShapeId m3CreateSphereShape(m3BodyId bodyId, const m3ShapeDef* def,
                                         const m3Sphere* sphere);

    /// Create a half-space on a STATIC body (a plane on a dynamic body
    /// returns the null id loudly). The normal is normalized on create.
    M3_API m3ShapeId m3CreatePlaneShape(m3BodyId bodyId, const m3ShapeDef* def,
                                        const m3Plane* plane);

    /// Create a box (a convex hull with analytic mass properties:
    /// m = 8 rho hx hy hz, I = m/3 diag(hy^2+hz^2, ...) about the
    /// center). Hull collision lands in the GJK and SAT slices; until
    /// then a box participates in the broadphase and in mass only.
    M3_API m3ShapeId m3CreateBoxShape(m3BodyId bodyId, const m3ShapeDef* def, m3Vec3 halfExtents);

    /// A capsule: the segment p1-p2 swept by a radius. Mass is the
    /// closed-form cylinder plus two hemispheres about the midpoint.
    typedef struct m3Capsule
    {
        m3Vec3 point1;
        m3Vec3 point2;
        m3real radius;
    } m3Capsule;

    /// A capsule: the segment point1..point2 inflated by radius. A
    /// zero-length segment is refused (that shape is a sphere; use
    /// m3CreateSphereShape).
    M3_API m3ShapeId m3CreateCapsuleShape(m3BodyId bodyId, const m3ShapeDef* def,
                                          const m3Capsule* capsule);

    /// A cylinder, honestly faceted (15-1): the factory mints a
    /// 2N-vertex prism through the interned hull path (N = segments,
    /// clamped to [3, 32] so 2N fits the 64-vertex hull), so mass,
    /// contacts, casts, carving, and CCD all inherit exact hull math
    /// for the prism. The documented trade: the side is an N-gon,
    /// not a curve; 24 segments roll visually smooth at meter
    /// scales. The analytic round cylinder stays on the ledger for
    /// the day a consumer needs it. Degenerate axes and radii refuse
    /// loudly like every hull.
    typedef struct m3Cylinder
    {
        m3Vec3 point1; // center of one cap
        m3Vec3 point2; // center of the other
        m3real radius;
    } m3Cylinder;

    M3_API m3ShapeId m3CreateCylinderShape(m3BodyId bodyId, const m3ShapeDef* def,
                                           const m3Cylinder* cylinder, int32_t segments);

    /// Runtime geometry replacement (15-2): swap a sphere or capsule
    /// shape's geometry in place; conversions between the two are
    /// legal. Hulls, meshes, voxels, and planes refuse loudly
    /// (interned slabs are immutable by law). Journaled; mass,
    /// extents, and the broadphase follow, and sleepers around both
    /// the old and the new bounds wake.
    M3_API bool m3Shape_SetSphere(m3ShapeId shapeId, const m3Sphere* sphere);
    M3_API bool m3Shape_SetCapsule(m3ShapeId shapeId, const m3Capsule* capsule);

    /// A convex hull built from a point cloud (QuickHull, coplanar
    /// faces merged, mass integrated). Between 4 and 64 finite points;
    /// degenerate clouds (coplanar, collinear) and hulls that exceed
    /// the 24-vertex budget return the null id, loudly.
    M3_API m3ShapeId m3CreateHullShape(m3BodyId bodyId, const m3ShapeDef* def, const m3Vec3* points,
                                       int32_t count);

    /// A static triangle mesh (level geometry, voxel chunk surfaces).
    /// Static bodies only; up to 1024 vertices and 2048 triangles per
    /// mesh (three indices each, counter-clockwise seen from outside:
    /// contacts cull the back side). Ghost collisions on shared edges
    /// are filtered by feature welding. Refused loudly on a dynamic
    /// body, out-of-cap counts, or out-of-range indices.
    M3_API m3ShapeId m3CreateMeshShape(m3BodyId bodyId, const m3ShapeDef* def,
                                       const m3Vec3* vertices, int32_t vertexCount,
                                       const uint16_t* indices, int32_t triangleCount);

    /// A per-triangle surface material for mesh shapes (17-2): the
    /// struck triangle's entry replaces the SHAPE's friction,
    /// restitution, and rolling resistance in the contact mix, and
    /// its surface velocity feeds the conveyor row (a moving
    /// walkway per triangle). All finite, frictions and resistances
    /// nonnegative.
    typedef struct m3MeshSurfaceMaterial
    {
        float friction;
        float restitution;
        float rollingResistance;
        m3Vec3 surfaceVelocity; // world frame, the conveyor tangent
    } m3MeshSurfaceMaterial;

    /// Paint material groups onto a mesh shape: up to 8 entries and
    /// one group index byte per triangle (triangleMaterials, length
    /// = the mesh's triangle count, every byte < materialCount).
    /// Journaled; refuses loudly on a non-mesh shape, hostile
    /// values, or out-of-range group bytes. A mesh without painted
    /// materials keeps using its shape material everywhere.
    M3_API void m3Shape_SetMeshMaterials(m3ShapeId shapeId, const m3MeshSurfaceMaterial* materials,
                                         int32_t materialCount, const uint8_t* triangleMaterials);

    /// A heightfield chunk: an nx by nz grid of heights (row-major,
    /// x fastest) spaced `cellSize` apart in x and z, triangulated
    /// into a static mesh through the same welded triangle path.
    /// Grid limits per chunk: 2..32 in each direction (the mesh caps);
    /// larger terrain tiles as chunks, the voxel-world model. The
    /// grid's minimum corner sits at the body origin.
    M3_API m3ShapeId m3CreateHeightFieldShape(m3BodyId bodyId, const m3ShapeDef* def,
                                              const float* heights, int32_t nx, int32_t nz,
                                              m3real cellSize);

    /// True while the id names a live shape in a live world; false
    /// for the null id, stale generations, and destroyed worlds.
    /// A voxel chunk: a dense 16x16x16 grid anchored at the body
    /// origin, cells of `cellSize` meters. `voxels` is one byte per
    /// voxel (zero = empty), x fastest then y then z; `payload` is
    /// an optional uint16 per voxel (material, health, type),
    /// carried as state and hashed. Static bodies only; sensors
    /// refused (sensor volumes are convex by contract); an empty
    /// grid refused. Solid voxels are CLOSED volumes: point-inside
    /// answers true inside filled cells. Rays, contacts, shape
    /// casts, and continuous collision all see voxel chunks; fast
    /// bodies and bullets sweep the merged surface, so a
    /// one-voxel-thick wall stops a bullet, and a wall whose voxel
    /// was cleared last step is a REAL hole.
    ///
    /// Seam welding: chunks weld automatically when both bodies
    /// carry the exact identity rotation, cell sizes match, and
    /// world positions differ by exactly one chunk extent along one
    /// axis (grid-laid level geometry). Welded borders are interior
    /// geometry and never produce contact features, so bodies roll
    /// and slide across chunk boundaries without seam impulses.
    /// Rotated or misaligned chunks still collide correctly; they
    /// just do not weld.
    M3_API m3ShapeId m3CreateVoxelChunkShape(m3BodyId bodyId, const m3ShapeDef* def,
                                             const uint8_t* voxels, const uint16_t* payload,
                                             m3real cellSize);

    /// Voxel edits (3-2): deterministic state transitions, journaled
    /// and replayed like every other mutation, fully inside the
    /// rollback delta. The collision surface rebuilds as a pure
    /// function of the grid after any occupancy change (a
    /// payload-only set touches no geometry), and dynamic bodies
    /// whose bounds touch the edited region are woken (a floor
    /// vanishing under a sleeper is a disturbance). Coordinates are
    /// voxel indices in [0, 15]; out-of-range and inverted regions
    /// refuse loudly. A chunk edited down to empty stays a valid
    /// shape that collides with nothing (the natural end state of
    /// destruction).
    ///
    /// The anchor convention: an island is anchored through the
    /// CHUNK's y = 0 base layer, not through world ground. Build
    /// structures at their chunk's base; a floating platform is a
    /// chunk whose BODY sits in the air with the platform at local
    /// y = 0. Voxels with no path to the base layer survive
    /// creation but fragment on the first clearing edit anywhere in
    /// the chunk (the sweep is chunk-wide by design).
    M3_API bool m3VoxelChunk_SetVoxel(m3ShapeId shapeId, int32_t x, int32_t y, int32_t z,
                                      uint16_t payload);
    M3_API bool m3VoxelChunk_ClearVoxel(m3ShapeId shapeId, int32_t x, int32_t y, int32_t z);
    /// Clears the inclusive box [lo, hi] per axis. Returns the
    /// number of voxels that were actually cleared (zero is legal),
    /// or -1 on a stale id, a non-voxel shape, or a bad region.
    M3_API int32_t m3VoxelChunk_ClearBox(m3ShapeId shapeId, const int32_t lo[3],
                                         const int32_t hi[3]);
    /// Fill fraction (3-6): 255 is a whole voxel, 1 a sliver. A
    /// mass and destruction property, never geometry: the voxel
    /// still collides as a full box, but its fragment mass scales
    /// by fill / 255. Refuses zero fill (that is a clear: use
    /// m3VoxelChunk_ClearVoxel) and empty voxels. Journaled state,
    /// hashed, inside the rollback delta like every edit.
    M3_API bool m3VoxelChunk_SetFill(m3ShapeId shapeId, int32_t x, int32_t y, int32_t z,
                                     uint8_t fill);

    M3_API bool m3Shape_IsValid(m3ShapeId shapeId);
    /// Who touches this shape now (14-3): fills up to capacity
    /// entries and returns the count written. See m3ContactData.
    M3_API int32_t m3Shape_GetContactData(m3ShapeId shapeId, m3ContactData* out, int32_t capacity);

    /// Destroy one shape and rebuild the owner's mass books (10-4).
    /// Journaled; contacts involving the shape dissolve at the next
    /// step. The last shape leaves a shapeless dynamic body at unit
    /// mass (the reference convention).
    M3_API void m3DestroyShape(m3ShapeId shapeId);

    /// Runtime material setters (8-4). Journaled; contacts read
    /// materials at prepare, so changes bind from the next step. A
    /// sleeping stack keeps its old mix until something wakes it
    /// (the reference behavior, documented).
    M3_API void m3Shape_SetFriction(m3ShapeId shapeId, float friction);
    M3_API float m3Shape_GetFriction(m3ShapeId shapeId);
    M3_API void m3Shape_SetRestitution(m3ShapeId shapeId, float restitution);
    M3_API float m3Shape_GetRestitution(m3ShapeId shapeId);
    M3_API void m3Shape_SetRollingResistance(m3ShapeId shapeId, float value);
    M3_API float m3Shape_GetRollingResistance(m3ShapeId shapeId);

    /// Set the density and optionally rebuild the owning body's
    /// mass, center and inertia from all its shapes. Journaled.
    M3_API void m3Shape_SetDensity(m3ShapeId shapeId, float density, bool updateBodyMass);
    M3_API float m3Shape_GetDensity(m3ShapeId shapeId);

    /// Opt a shape into hit events / the pre-solve veto (8-5).
    /// Journaled; the flags are state and snapshot with the world.
    M3_API void m3Shape_EnableHitEvents(m3ShapeId shapeId, bool flag);
    M3_API bool m3Shape_AreHitEventsEnabled(m3ShapeId shapeId);
    M3_API void m3Shape_EnablePreSolve(m3ShapeId shapeId, bool flag);
    M3_API bool m3Shape_IsPreSolveEnabled(m3ShapeId shapeId);

    /// Conveyor (11-3): a world-frame surface velocity on the
    /// shape. Contacts drive the tangential target toward it, the
    /// reference tangentVelocity semantic the central-friction port
    /// carried at zero until now. Journaled; state, hashed when
    /// nonzero.
    M3_API void m3Shape_SetSurfaceVelocity(m3ShapeId shapeId, m3Vec3 velocity);

    static const m3ShapeId m3_nullShapeId = {0, 0, 0};

#ifdef __cplusplus
}
#endif

#endif // MAUL3D_SHAPE_H
