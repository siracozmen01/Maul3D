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
#include "maul3d/joint.h"
#include "maul3d/shape.h"
#include "maul3d/world.h"

#define M3_MAX_WORLDS 64

// Bumped on ANY change that alters simulation behavior (solver math,
// integration order, constants). Part of the snapshot config hash, so
// a snapshot from a different behavior revision is refused loudly
// instead of silently diverging (the Jolt friction-model lesson).
// NOTE: rev 17 was skipped by a slipped edit in the 2c-5 slice (the
// cone and twist limits shipped without their bump; same format,
// new-capability-only, harmless in effect but a discipline miss,
// recorded here so the ledger stays honest).
#define M3_SOLVER_REV 18 // rev 18: spherical limits (late) + sensors

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
    m3_opCreateHullShape = 7, // carries the input points (the recipe)
    m3_opCreateMeshShape = 8, // header + exact-size vertex/index payload
    m3_opCreateJoint = 9,
    m3_opDestroyJoint = 10,
} m3Op;

// Immutable interned hull data (lifetime 3): vertices, face planes,
// and face vertex loops for the SAT (2b-5), plus unit-density mass
// properties. Content-deduplicated on intern; shapes reference by
// index and refcount. Fixed arrays keep it one snapshot block.
// Euler-consistent worst case for 24 vertices: a simplicial hull has
// F = 2V - 4 = 44 faces and E = 3V - 6 = 66 edges (132 half edges).
// Sized so no valid 24-vertex hull can ever overflow the fixed block.
#define M3_HULL_MAX_VERTS        24
#define M3_HULL_MAX_FACES        44
#define M3_HULL_MAX_FACE_INDICES 132
#define M3_HULL_MAX_HALF_EDGES   132
#define M3_HULL_MAX_INPUT        64 // QuickHull input point cap

// Half-edge adjacency (the Gauss-map edge query reads the two faces
// flanking every edge). Twins sit at 2k and 2k+1 by construction.
typedef struct m3HullHalfEdge
{
    uint8_t origin;
    uint8_t twin;
    uint8_t next;
    uint8_t face;
} m3HullHalfEdge;

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
    int32_t edgeCount; // half-edge count (twice the undirected edges)
    m3HullHalfEdge edges[M3_HULL_MAX_HALF_EDGES];
    m3Vec3 center; // vertex centroid, orients the edge separation
} m3HullData;

_Static_assert(sizeof(m3HullData) == 1820, "hull data must be padding-free");

// Static triangle mesh content (lifetime 3): fixed caps keep it one
// snapshot block per slot, so the snapshot law stays uniform (no
// variable-size special case; a deliberate deviation from the
// reference's variable allocations, argued in the 2b-9 slice). The
// midphase in 2b-9a is a bounded per-triangle scan; the static BVH
// arrives in 2b-9b behind the same query contract.
#define M3_MESH_MAX_VERTS 1024
#define M3_MESH_MAX_TRIS  2048

typedef struct m3MeshData
{
    int32_t vertexCount;
    int32_t triangleCount;
    m3Vec3 vertices[M3_MESH_MAX_VERTS];
    uint16_t indices[3 * M3_MESH_MAX_TRIS]; // CCW from outside
    // Bit k set = triangle edge k (vertex k to k+1) is CONVEX or a
    // boundary: a REAL contact feature. Clear = flat or concave: a
    // ghost candidate the welding filter may silence. Baked at
    // create time (2b-9d), deterministic.
    uint8_t edgeFlags[M3_MESH_MAX_TRIS];
} m3MeshData;

_Static_assert(sizeof(m3MeshData) == 26632, "mesh data must be padding-free");

// Bake the edge-convexity flags (a bounded pair scan; the BVH slice
// will speed it up if profiles ever ask).
void m3BakeMeshEdgeFlags(m3MeshData* mesh);

// Geometry is one padding-free 32-byte record per shape, interpreted
// by type: sphere {v=center, s=radius}, plane {v=normal, s=offset},
// hull {v=box half extents for the journaled rebuild, s unused},
// capsule {v=point1, v2=point2, s=radius}.
typedef struct m3ShapeGeom
{
    m3Vec3 v;  // sphere center | plane normal | box half extents | capsule p1
    m3real s;  // radius | offset | unused | radius
    m3Vec3 v2; // capsule p2
    m3real s2; // reserved
} m3ShapeGeom;

_Static_assert(sizeof(m3ShapeGeom) == 32, "shape geom must be padding-free");

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

// Up to four contact points (2b-5): a hull face on a plane or a face
// pair needs four; spheres use one. Feature ids identify points across
// rebuilds for the warm-start carry.
#define M3_MANIFOLD_MAX_POINTS 4

typedef struct m3Manifold
{
    m3Vec3 normal; // from shape A to shape B, world frame
    int32_t pointCount;
    m3ManifoldPoint points[M3_MANIFOLD_MAX_POINTS];
} m3Manifold;

_Static_assert(sizeof(m3Manifold) == 192, "manifold must be padding-free");

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

// Journal payload for general hull shapes: the raw input points are
// the recipe; replay rebuilds through the same QuickHull, so the
// derived hull data never has to ride the journal.
// Journal header for mesh creation; the exact-size vertex and index
// arrays follow it in the payload (a full-cap struct would bloat the
// journal by 24 KB per mesh).
typedef struct m3CreateMeshShapeOp
{
    m3ShapeDef def;
    m3BodyId body;
    m3ShapeId expected;
    int32_t vertexCount;
    int32_t triangleCount;
} m3CreateMeshShapeOp;

typedef struct m3CreateJointOp
{
    m3JointDef def;
    m3JointId expected;
} m3CreateJointOp;

typedef struct m3CreateHullShapeOp
{
    m3ShapeDef def;
    m3BodyId body;
    m3ShapeId expected;
    int32_t count;
    m3Vec3 points[M3_HULL_MAX_INPUT];
} m3CreateHullShapeOp;

typedef struct m3World
{
    // World-global state (snapshot header material, task 6).
    m3Vec3 gravity;
    uint64_t stepCount;
    int32_t bodyCapacity;
    int32_t shapeCapacity;
    int32_t workerCount;
    m3EnqueueTaskFn* enqueueTask; // host threading hooks (never state)
    m3FinishTaskFn* finishTask;
    void* userTaskContext;
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
    m3Mat3* inertiaLocal;    // forward tensor (the gyroscopic solve reads it)
    m3Vec3* localCenters;    // center of mass in the body frame
    m3real* gravityScales;
    m3real* linearDamping;
    m3real* angularDamping;
    uint8_t* types;
    uint8_t* awake;       // 0 = sleeping (dynamic bodies only), frozen solid
    float* sleepTimes;    // seconds below the sleep threshold (2b-10)
    uint8_t* bulletFlags; // 1 = full continuous vs dynamics (2b-8)
    float* minExtents;    // per body: thinnest shape measure (CCD trigger)
    float* maxExtents;    // per body: farthest point from the COM (CCD arc bound)
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
    int32_t* shapeMeshIndex; // mesh slot, -1 for non-meshes
    uint8_t* shapeSensor;    // 1 = overlap detector, never contact response

    // The interned hull pool (immutable content, refcounted).
    m3IdPool hullPool;
    m3HullData* hullData;
    int32_t* hullRefCounts;

    // Static mesh slots (immutable content, refcounted, no content
    // dedupe: meshes are big and user-authored).
    int32_t meshCapacity;
    m3IdPool meshPool;
    m3MeshData* meshData;
    int32_t* meshRefCounts;

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

    // Joints (2c-2): SoA arenas, all persistent snapshot state. The
    // warm-start impulse is simulation state (the architecture doc
    // names it); the body lists drive the jointed-pair contact
    // filter, the destroy cascade, and island coupling.
    m3IdPool jointPool;
    uint8_t* jointType;
    int32_t* jointBodyA;
    int32_t* jointBodyB;
    m3Vec3* jointLocalA;
    m3Vec3* jointLocalB;
    uint8_t* jointCollide;
    m3Vec3* jointImpulse;        // warm-start linear impulse
    m3Vec3* jointPerpImpulse;    // x, y = collinearity rows; z = motor
    m3Vec3* jointLimitImpulse;   // x = lower, y = upper, z unused
    m3Vec3* jointAngularImpulse; // prismatic 3-DOF rotation lock
    m3Quat* jointFrameQA;        // joint frame in body A (axis = local z)
    m3Quat* jointFrameQB;
    uint8_t* jointFlags; // bit0 limit, bit1 motor
    m3Vec3* jointMotor;  // x = motorSpeed, y = maxMotorTorque, z unused
    m3Vec3* jointLimits; // x = lower angle, y = upper angle, z unused
    int32_t* bodyJointHead;
    int32_t* jointNextA; // next joint in body A's list
    int32_t* jointNextB; // next joint in body B's list
    int32_t jointCapacity;

    // Contact events (transient observers, never snapshotted;
    // cleared on step and on restore).
    m3ContactEvent* beginEvents;
    m3ContactEvent* endEvents;
    m3ContactEvent* sensorBeginEvents;
    m3ContactEvent* sensorEndEvents;
    int32_t beginEventCount;
    int32_t endEventCount;
    int32_t sensorBeginEventCount;
    int32_t sensorEndEventCount;

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

// Half-edge adjacency from the face loops (twins at 2k and 2k+1 by
// construction, the invariant the Gauss-map edge query leans on).
// One law for every hull source: the box builder and QuickHull both
// finish through this.
void m3HullBuildHalfEdges(m3HullData* hull);

// QuickHull (2b-3b), adapted from the reference hull.c (Erin Catto,
// with portions contributed by Dirk Gregorius): builds the convex
// hull of up to M3_HULL_MAX_INPUT points into the fixed m3HullData
// block, faces merged coplanar, unit-density mass properties
// integrated. Returns false loudly on degenerate input (fewer than
// four points, coplanar clouds, non-finite coordinates) or if the
// result exceeds the fixed caps.
bool m3ComputeHull(const m3Vec3* points, int32_t count, m3HullData* out);
int32_t m3InternHull(m3World* world, const m3HullData* data); // -1 = pool exhausted
void m3ReleaseHull(m3World* world, int32_t hullIndex);
int32_t m3CreateShapeInternal(m3World* world, int32_t bodyIndex, uint8_t type,
                              const m3ShapeGeom* geom, const m3ShapeDef* def,
                              const m3HullData* prebuilt, const m3MeshData* meshPrebuilt);
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

// The narrowphase over one pair range: every pair writes only its own
// manifold slot, so any partition of [0, pairCount) is bit-identical
// to the serial run (the parallel contract).
void m3UpdateContactsRange(m3World* world, int32_t start, int32_t end, const uint64_t* oldKeys,
                           const m3Manifold* oldManifolds, int32_t oldCount);

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

// Sweep of one body's COM and rotation across a step, in a float
// frame the caller re-centered (the reference precision trick: TOI
// runs relative to the fast body's begin COM so doubles never enter
// the kernel).
typedef struct m3Sweep
{
    m3Vec3 localCenter; // COM in the body frame
    m3Vec3 c1;          // begin COM, re-centered
    m3Vec3 c2;          // end COM, re-centered
    m3Quat q1;
    m3Quat q2;
} m3Sweep;

m3Transform m3GetSweepTransform(const m3Sweep* sweep, m3real time);

typedef struct m3TOIInput
{
    m3DistanceProxy proxyA; // the target shape
    m3DistanceProxy proxyB; // the fast shape
    m3Sweep sweepA;
    m3Sweep sweepB;
    m3real maxFraction;
} m3TOIInput;

typedef enum m3TOIState
{
    m3_toiStateUnknown = 0,
    m3_toiStateFailed,
    m3_toiStateOverlapped,
    m3_toiStateHit,
    m3_toiStateSeparated,
} m3TOIState;

typedef struct m3TOIOutput
{
    m3TOIState state;
    m3real fraction;
    m3Vec3 normal; // A toward B at the hit (valid on hit only)
} m3TOIOutput;

// Time of impact by conservative advancement with root finding
// (2b-8), adapted from the reference distance.c: separation
// functions built from the GJK simplex cache (vertices, edge pairs,
// faces), deepest-point push-back, mixed false-position and
// bisection roots. Both sweeps participate: dynamic versus dynamic
// is a first-class citizen.
m3TOIOutput m3TimeOfImpact(const m3TOIInput* input);

// GJK proxy for one shape in its local frame (spheres and capsules
// borrow the caller's scratch for their point storage).
m3DistanceProxy m3MakeShapeProxy(const m3World* world, int32_t shape, m3Vec3 scratch[2]);

// Hull-versus-hull SAT (2b-5b): face queries both ways, the Gauss-map
// edge query, face clipping or the edge closest-point contact. B is
// given in A's frame; the manifold is in A's frame with the A-to-B
// normal. Reduction reuses the deepest-four canonical rule.
m3Manifold m3CollideHulls(const m3HullData* hullA, const m3HullData* hullB, m3Quat q, m3Vec3 p);

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

int32_t m3JointSlot(const m3World* world, m3JointId jointId);
int32_t m3CreateJointInternal(m3World* world, const m3JointDef* def, int32_t bodyA, int32_t bodyB);
void m3DestroyJointInternal(m3World* world, int32_t index);

#endif // MAUL3D_WORLD_INTERNAL_H
