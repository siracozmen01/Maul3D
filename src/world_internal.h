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
#include "dynamic_tree.h"

#include "maul3d/body.h"
#include "maul3d/shape.h"
#include "maul3d/world.h"

#define M3_MAX_WORLDS 64

// Bumped on ANY change that alters simulation behavior (solver math,
// integration order, constants). Part of the snapshot config hash, so
// a snapshot from a different behavior revision is refused loudly
// instead of silently diverging (the Jolt friction-model lesson).
#define M3_SOLVER_REV 2 // rev 2: full inertia tensor and center of mass

// Def cookies: a def that did not come from its m3Default*Def factory
// is rejected loudly (the Maul2D pattern).
#define M3_COOKIE       0x4D33u // 'M3'
#define M3_WORLD_COOKIE ((int32_t)(M3_COOKIE ^ ((int32_t)sizeof(m3WorldDef) << 8)))
#define M3_BODY_COOKIE  ((int32_t)(M3_COOKIE ^ ((int32_t)sizeof(m3BodyDef) << 8) ^ 1))
#define M3_SHAPE_COOKIE ((int32_t)(M3_COOKIE ^ ((int32_t)sizeof(m3ShapeDef) << 8) ^ 2))

// Fat AABB margin: pairs exist slightly before touch so speculative
// contacts (task 8) have something to work with.
#define M3_AABB_MARGIN (4.0f * 0.005f)

// Journal ops. The stream is [i32 op][i32 size][payload], replayed
// through the same internal functions the public API uses.
typedef enum m3Op
{
    m3_opStep = 1, // reserved for task 9
    m3_opCreateBody = 2,
    m3_opDestroyBody = 3,
    m3_opSetLinearVelocity = 4,
    m3_opSetAngularVelocity = 5,
    m3_opCreateShape = 6,
} m3Op;

// Immutable interned hull data (lifetime 3): vertices, face planes,
// and face vertex loops for the SAT (2b-5), plus unit-density mass
// properties. Content-deduplicated on intern; shapes reference by
// index and refcount. Fixed arrays keep it one snapshot block.
#define M3_HULL_MAX_VERTS        24
#define M3_HULL_MAX_FACES        16
#define M3_HULL_MAX_FACE_INDICES 64

typedef struct m3HullData
{
    int32_t vertexCount;
    int32_t faceCount;
    int32_t indexCount;
    float unitMass;        // at density one
    m3Vec3 unitCom;        // body-local centroid
    m3Mat3 unitInertiaCom; // at density one, about the centroid
    m3Vec3 vertices[M3_HULL_MAX_VERTS];
    m3Vec3 faceNormals[M3_HULL_MAX_FACES];
    m3real faceOffsets[M3_HULL_MAX_FACES];
    uint8_t faceVertCounts[M3_HULL_MAX_FACES];
    uint8_t faceVertStart[M3_HULL_MAX_FACES];
    uint8_t faceIndices[M3_HULL_MAX_FACE_INDICES];
} m3HullData;

_Static_assert(sizeof(m3HullData) == 704, "hull data must be padding-free");

// Geometry is one padding-free 16-byte record per shape, interpreted
// by type: sphere {v=center, s=radius}, plane {v=normal, s=offset},
// hull {v=box half extents for the journaled rebuild, s unused}.
typedef struct m3ShapeGeom
{
    m3Vec3 v;
    m3real s;
} m3ShapeGeom;

// One contact point: anchors are measured from each body's center in
// world orientation (float is exact enough near contact), impulses are
// the warm-start payload that persists across steps and through the
// snapshot. Padding-free by construction.
typedef struct m3ManifoldPoint
{
    m3Vec3 anchorA;
    m3Vec3 anchorB;
    m3real separation; // negative = penetration, positive = speculative
    m3real normalImpulse;
    m3real tangentImpulse1;
    m3real tangentImpulse2;
    uint16_t id;    // feature id (spheres have exactly one feature: 0)
    uint16_t flags; // bit 0: persisted (impulses carried this rebuild)
} m3ManifoldPoint;

_Static_assert(sizeof(m3ManifoldPoint) == 44, "manifold point must be padding-free");

// Sphere and plane contacts have at most one point; the array widens
// in 2b (hulls) with a snapshot format bump.
typedef struct m3Manifold
{
    m3Vec3 normal; // from shape A to shape B, world frame
    int32_t pointCount;
    m3ManifoldPoint points[1];
} m3Manifold;

_Static_assert(sizeof(m3Manifold) == 60, "manifold must be padding-free");

// Journal payload for shape creation (replay re-derives mass).
typedef struct m3CreateShapeOp
{
    m3ShapeDef def;
    m3ShapeGeom geom;
    m3BodyId body;
    m3ShapeId expected;
    uint8_t type;
    uint8_t pad[7];
} m3CreateShapeOp;

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
    m3Mat3* invInertiaLocal; // inverse inertia about the COM, body frame
    m3Vec3* localCenters;    // center of mass in the body frame
    m3real* gravityScales;
    m3real* linearDamping;
    m3real* angularDamping;
    uint8_t* types;
    uint64_t* userData;
    int32_t* bodyShapeHead; // head of each body's shape list, -1 none

    // Shape identity and SoA shape state (persistent, walked).
    m3IdPool shapePool;
    int32_t* shapeBody;
    uint8_t* shapeType;
    m3ShapeGeom* shapeGeom;
    float* shapeDensity;
    float* shapeFriction;
    float* shapeRestitution;
    uint64_t* shapeUserData;
    int32_t* shapeNext;      // next shape on the same body, -1 end
    int32_t* shapeHullIndex; // interned hull slot, -1 for non-hulls

    // The interned hull pool (immutable content, refcounted).
    m3IdPool hullPool;
    m3HullData* hullData;
    int32_t* hullRefCounts;

    // Broadphase: the fat-AABB tree (spheres only; infinite planes
    // stay out and take a dedicated pass), per-shape proxy ids, both
    // persistent snapshot state.
    m3Tree tree;
    int32_t* proxyIds; // M3_TREE_NULL for planes and dead shapes

    // Candidate pairs in canonical ascending key order, and their
    // manifolds (persistent: warm-start impulses live here and ride
    // the snapshot).
    uint64_t* pairKeys;
    m3Manifold* manifolds;
    int32_t pairCount;
    int32_t pairCapacity;

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

int32_t m3ShapeSlot(const m3World* world, m3ShapeId shapeId);

// Analytic box hull (canonical vertex and face order) and the intern
// machinery. Interning dedupes by content in ascending slot order.
void m3BuildBoxHull(m3HullData* out, m3Vec3 halfExtents);
int32_t m3InternHull(m3World* world, const m3HullData* data); // -1 = pool exhausted
void m3ReleaseHull(m3World* world, int32_t hullIndex);
int32_t m3CreateShapeInternal(m3World* world, int32_t bodyIndex, uint8_t type,
                              const m3ShapeGeom* geom, const m3ShapeDef* def);
void m3DestroyShapeInternal(m3World* world, int32_t index);
void m3RecomputeMass(m3World* world, int32_t bodyIndex);

// Broadphase v1 (the swappable seam): fills pairKeys in canonical
// ascending key order from fat AABBs; the dynamic tree replaces the
// scan in 2b behind this same contract.
m3Result m3UpdatePairs(m3World* world);

// The 2a brute-force scan, kept as the referee: on any scene the tree
// path must produce the identical pair list (a test gate).
m3Result m3UpdatePairsBruteForce(m3World* world);

// Fat world bounds of a sphere shape (double, margin included).
void m3ShapeFatAabb(const m3World* world, int32_t shape, double lo[3], double hi[3]);

// Narrowphase v1: rebuild manifolds for the current pairs in pair
// order, carrying warm-start impulses forward by feature id from the
// PREVIOUS step's stash (the caller copies keys and manifolds to step
// scratch BEFORE m3UpdatePairs overwrites them; oldKeys are sorted).
m3Result m3UpdateContacts(m3World* world, const uint64_t* oldKeys, const m3Manifold* oldManifolds,
                          int32_t oldCount);

// Deterministic tangent basis: ONE fixed rule (the world axis with the
// smallest absolute normal component, ties broken x before y before
// z), because the friction rows are order-sensitive downstream.
void m3MakeTangentBasis(m3Vec3 normal, m3Vec3* t1, m3Vec3* t2);

// GJK distance (2b-4), adapted from the reference distance.c: convex
// proxies, a warm-startable simplex cache, results in frame A. The
// SAT manifolds (2b-5) and the TOI (2b-8) build on this kernel.
#define M3_MAX_GJK_ITERATIONS 32

typedef struct m3DistanceProxy
{
    const m3Vec3* points;
    int32_t count;
    m3real radius;
} m3DistanceProxy;

typedef struct m3SimplexCache
{
    m3real metric;
    uint16_t count;
    uint8_t indexA[4];
    uint8_t indexB[4];
} m3SimplexCache;

typedef struct m3DistanceInput
{
    m3DistanceProxy proxyA;
    m3DistanceProxy proxyB;
    m3Quat q; // rotation of B in A's frame
    m3Vec3 p; // position of B in A's frame (caller localizes doubles)
    bool useRadii;
} m3DistanceInput;

typedef struct m3DistanceOutput
{
    m3Vec3 pointA; // frame A
    m3Vec3 pointB;
    m3Vec3 normal;   // A toward B (zero on overlap)
    m3real distance; // zero on overlap
    int32_t iterations;
} m3DistanceOutput;

m3DistanceOutput m3ShapeDistance(const m3DistanceInput* input, m3SimplexCache* cache);

// Pure collide kernels (world-independent, tested in isolation).
// Normals point from A to B. d is the center offset B minus A in
// floats (exact enough near contact).
m3Manifold m3CollideSpheres(m3Vec3 d, m3real radiusA, m3real radiusB);
// Plane (A) versus sphere (B): dist is the signed distance of the
// sphere center above the plane, computed in double by the caller.
m3Manifold m3CollidePlaneSphere(m3Vec3 planeNormal, m3real dist, m3real radius);

// The step body: the journal replays through this exact path.
void m3StepInternal(m3World* world, float dt, int32_t substeps);

void m3JournalRecord(m3World* world, int32_t op, const void* payload, int32_t bytes);

#endif // MAUL3D_WORLD_INTERNAL_H
