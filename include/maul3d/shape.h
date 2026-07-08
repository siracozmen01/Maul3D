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
        uint64_t userData;
        /// A sensor detects overlap and fires its own begin and end
        /// events but never produces contact response: bodies pass
        /// through, bullets do not stop, sleepers are not woken.
        /// Sensors do not sense other sensors.
        bool isSensor;
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
    /// answers true inside filled cells. Shape casts and continuous
    /// collision do not see voxel chunks until the voxel CCD slice
    /// (3-5); rays and contacts do.
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
    M3_API bool m3VoxelChunk_SetVoxel(m3ShapeId shapeId, int32_t x, int32_t y, int32_t z,
                                      uint16_t payload);
    M3_API bool m3VoxelChunk_ClearVoxel(m3ShapeId shapeId, int32_t x, int32_t y, int32_t z);
    /// Clears the inclusive box [lo, hi] per axis. Returns the
    /// number of voxels that were actually cleared (zero is legal),
    /// or -1 on a stale id, a non-voxel shape, or a bad region.
    M3_API int32_t m3VoxelChunk_ClearBox(m3ShapeId shapeId, const int32_t lo[3],
                                         const int32_t hi[3]);

    M3_API bool m3Shape_IsValid(m3ShapeId shapeId);

    static const m3ShapeId m3_nullShapeId = {0, 0, 0};

#ifdef __cplusplus
}
#endif

#endif // MAUL3D_SHAPE_H
