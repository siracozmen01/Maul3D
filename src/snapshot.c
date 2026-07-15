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
#define M3_SNAPSHOT_VERSION 47u
// v47: per-triangle mesh materials (17-2).
// v46: count-derived hull content (17-1): the 5808-byte fixed
//      slabs leave the fixed prefix; an empty slot costs 16 bytes.
// v45: pulley world anchors (16-6).
// v44: drivetrain differentials and wheel contact speeds (16-4);
//      body debug names and soft explosion kicks rode v43's bump
//      window in the same release train.
// v43: angular speed cap (13-1).
// v42: vehicle drivetrain, the engine curve and gearbox (12-1).
// v41: wind field and conveyor surface velocities (11-3).
// v40: soft-to-soft anchors (11-2).
// v39: mesh content went count-derived (10-3): variable blocks
//      replace the fixed mesh slab, reversing the 2b-9 deviation.
// v38: hull capacity 64 (10-2): bigger hull blocks, same law.
// v37: compound shape offsets (10-1).
// v36: joint drive springs and targets (8-6b).
// v35: joint break thresholds (8-6a).
// v34: hit threshold + shape event flags (8-5).
// v33: world tuning knobs (8-4).
// v32: central friction manifold payload replaces per-point
//      tangent impulses (solver rev 21).
// v31: runtime body control state (8-3).
// v30: host force and torque accumulators (8-2).
// v29: collision filters (8-1).
// v28: soft body anchors (7-3 coupling).
// v27: the soft body pool (7-1 XPBD lattices).
// v26: shape rolling resistance (6-3).
// v25: tire grip, drive commands, and wheel spin (5-2).
// v24: the vehicle pool and wheel arrays (5-1 raycast vehicles).
// v23: character mass, push ratio, and the ground-body reference
// (4-6 riders). v22: stepHeight. v21: the character pool.
// v20: generic joint state. NOTE for the ledger: v19 (voxel fill
// fractions, 3-6) shipped MISLABELED as 18: the bump script died
// after a partial edit, the same failure mode as the rev-17 skip
// in 2c-5. No released pairing could misread a snapshot (the
// config hash embeds the library version and every release differs
// there), but the label was wrong and this comment is the honest
// record. The 2c-8 lesson now has a second clause: after a partial
// script failure, diff EVERY intended edit, not just the one that
// raised.

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
    int32_t treeRoot;
    int32_t treeFreeList;
    int32_t hullMaxIndex;
    int32_t hullFreeHead;
    int32_t hullFreeCount;
    int32_t hullRetiredCount;
    int32_t meshCapacity;
    int32_t meshMaxIndex;
    int32_t meshFreeHead;
    int32_t meshFreeCount;
    int32_t meshRetiredCount;
    int32_t jointCapacity;
    int32_t jointMaxIndex;
    int32_t jointFreeHead;
    int32_t jointFreeCount;
    int32_t jointRetiredCount;
    int32_t voxelCapacity;
    int32_t voxelMaxIndex;
    int32_t voxelFreeHead;
    int32_t voxelFreeCount;
    int32_t voxelRetiredCount;
    int32_t charCapacity;
    int32_t charMaxIndex;
    int32_t charFreeHead;
    int32_t charFreeCount;
    int32_t charRetiredCount;
    int32_t reserved[2]; // keeps the 8-byte-aligned header padding-free
    m3Vec3 gravity;
} m3SnapshotHeader;

_Static_assert(sizeof(m3SnapshotHeader) == 192, "snapshot header must be padding-free");

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
static int32_t WalkBlocks(m3World* world, uint8_t* out, const uint8_t* in, m3WalkMode mode,
                          int includeMesh)
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
    M3_BLOCK(world->inertiaLocal, cap * (int32_t)sizeof(m3Mat3));
    M3_BLOCK(world->localCenters, cap * (int32_t)sizeof(m3Vec3));
    M3_BLOCK(world->gravityScales, cap * (int32_t)sizeof(m3real));
    M3_BLOCK(world->linearDamping, cap * (int32_t)sizeof(m3real));
    M3_BLOCK(world->angularDamping, cap * (int32_t)sizeof(m3real));
    M3_BLOCK(world->types, cap * (int32_t)sizeof(uint8_t));
    M3_BLOCK(world->awake, cap * (int32_t)sizeof(uint8_t));
    M3_BLOCK(world->sleepTimes, cap * (int32_t)sizeof(float));
    M3_BLOCK(world->bulletFlags, cap * (int32_t)sizeof(uint8_t));
    M3_BLOCK(world->minExtents, cap * (int32_t)sizeof(float));
    M3_BLOCK(world->maxExtents, cap * (int32_t)sizeof(float));
    M3_BLOCK(world->userData, cap * (int32_t)sizeof(uint64_t));
    M3_BLOCK(world->bodyNames,
             cap * M3_BODY_NAME_CAPACITY); // v43: debug names (14-3), never hashed
    M3_BLOCK(world->bodyForce, cap * (int32_t)sizeof(m3Vec3));
    M3_BLOCK(world->bodyTorque, cap * (int32_t)sizeof(m3Vec3));
    M3_BLOCK(world->bodyEnabled, cap * (int32_t)sizeof(uint8_t));
    M3_BLOCK(world->bodyLocks, cap * (int32_t)sizeof(uint8_t));
    M3_BLOCK(world->bodySleepThreshold, cap * (int32_t)sizeof(float));
    M3_BLOCK(world->bodyCanSleep, cap * (int32_t)sizeof(uint8_t));
    M3_BLOCK(world->bodyHasTarget, cap * (int32_t)sizeof(uint8_t));
    M3_BLOCK(world->bodyTarget, cap * (int32_t)sizeof(m3Transform));
    M3_BLOCK(&world->contactHertz, (int32_t)sizeof(float));
    M3_BLOCK(&world->contactDampingRatio, (int32_t)sizeof(float));
    M3_BLOCK(&world->contactPushMaxSpeed, (int32_t)sizeof(float));
    M3_BLOCK(&world->restitutionThreshold, (int32_t)sizeof(float));
    M3_BLOCK(&world->maximumLinearSpeed, (int32_t)sizeof(float));
    M3_BLOCK(&world->maximumAngularSpeed, (int32_t)sizeof(float)); // v43
    M3_BLOCK(&world->sleepEnabled, 1);
    M3_BLOCK(&world->continuousEnabled, 1);
    M3_BLOCK(&world->hitEventThreshold, (int32_t)sizeof(float));
    M3_BLOCK(&world->windDir, (int32_t)sizeof(m3Vec3));
    M3_BLOCK(&world->windSpeed, (int32_t)sizeof(float));
    M3_BLOCK(&world->windGustHertz, (int32_t)sizeof(float));
    M3_BLOCK(&world->windGustScale, (int32_t)sizeof(float));
    M3_BLOCK(&world->windPhase, (int32_t)sizeof(float));
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
    // The broadphase is state: proxies and the tree restore exactly,
    // so a rolled-back world continues on the identical tree shape.
    M3_BLOCK(world->proxyIds, shapeCap * (int32_t)sizeof(int32_t));
    M3_BLOCK(world->tree.nodes, world->tree.capacity * (int32_t)sizeof(m3TreeNode));
    M3_BLOCK(world->shapeHullIndex, shapeCap * (int32_t)sizeof(int32_t));
    // Hull CONTENT moved to the count-derived tail (17-1): the
    // 5808-byte fixed slabs cost more snapshot than every joint
    // in the world combined, per EMPTY slot.
    M3_BLOCK(world->hullRefCounts, shapeCap * (int32_t)sizeof(int32_t));
    M3_BLOCK(world->hullPool.generations, shapeCap * (int32_t)sizeof(uint16_t));
    M3_BLOCK(world->hullPool.alive, shapeCap * (int32_t)sizeof(uint8_t));
    M3_BLOCK(world->hullPool.freeQueue, shapeCap * (int32_t)sizeof(int32_t));
    M3_BLOCK(world->shapeMeshIndex, shapeCap * (int32_t)sizeof(int32_t));
    M3_BLOCK(world->shapeSensor, shapeCap * (int32_t)sizeof(uint8_t));
    M3_BLOCK(world->shapeRollingResistance, shapeCap * (int32_t)sizeof(float));
    M3_BLOCK(world->shapeHitEvents, shapeCap * (int32_t)sizeof(uint8_t));
    M3_BLOCK(world->shapePreSolve, shapeCap * (int32_t)sizeof(uint8_t));
    M3_BLOCK(world->shapeSurfaceVel, shapeCap * (int32_t)sizeof(m3Vec3));
    M3_BLOCK(world->shapeLocalPos, shapeCap * (int32_t)sizeof(m3Vec3));
    M3_BLOCK(world->shapeLocalRot, shapeCap * (int32_t)sizeof(m3Quat));
    M3_BLOCK(world->shapeHasOffset, shapeCap * (int32_t)sizeof(uint8_t));
    M3_BLOCK(world->shapeCategory, shapeCap * (int32_t)sizeof(uint64_t));
    M3_BLOCK(world->shapeMask, shapeCap * (int32_t)sizeof(uint64_t));
    M3_BLOCK(world->shapeGroup, shapeCap * (int32_t)sizeof(int32_t));

    M3_BLOCK(world->voxelData, world->voxelCapacity * (int32_t)sizeof(m3VoxelChunkData));
    M3_BLOCK(world->voxelRefCounts, world->voxelCapacity * (int32_t)sizeof(int32_t));
    M3_BLOCK(world->voxelPool.generations, world->voxelCapacity * (int32_t)sizeof(uint16_t));
    M3_BLOCK(world->voxelPool.alive, world->voxelCapacity * (int32_t)sizeof(uint8_t));
    M3_BLOCK(world->voxelPool.freeQueue, world->voxelCapacity * (int32_t)sizeof(int32_t));
    M3_BLOCK(world->shapeVoxelIndex, world->shapeCapacity * (int32_t)sizeof(int32_t));
    M3_BLOCK(world->charBody, world->characterCapacity * (int32_t)sizeof(int32_t));
    M3_BLOCK(world->charRadius, world->characterCapacity * (int32_t)sizeof(m3real));
    M3_BLOCK(world->charHalfHeight, world->characterCapacity * (int32_t)sizeof(m3real));
    M3_BLOCK(world->charCosSlope, world->characterCapacity * (int32_t)sizeof(m3real));
    M3_BLOCK(world->charSnap, world->characterCapacity * (int32_t)sizeof(m3real));
    M3_BLOCK(world->charSkin, world->characterCapacity * (int32_t)sizeof(m3real));
    M3_BLOCK(world->charStepHeight, world->characterCapacity * (int32_t)sizeof(m3real));
    M3_BLOCK(world->charGrounded, world->characterCapacity * (int32_t)sizeof(uint8_t));
    M3_BLOCK(world->charGroundNormal, world->characterCapacity * (int32_t)sizeof(m3Vec3));
    M3_BLOCK(world->charMass, world->characterCapacity * (int32_t)sizeof(m3real));
    M3_BLOCK(world->charPushMax, world->characterCapacity * (int32_t)sizeof(m3real));
    M3_BLOCK(world->charGroundBody, world->characterCapacity * (int32_t)sizeof(int32_t));
    M3_BLOCK(world->charGroundGen, world->characterCapacity * (int32_t)sizeof(uint16_t));
    M3_BLOCK(world->charPool.generations, world->characterCapacity * (int32_t)sizeof(uint16_t));
    M3_BLOCK(world->charPool.alive, world->characterCapacity * (int32_t)sizeof(uint8_t));
    M3_BLOCK(world->charPool.freeQueue, world->characterCapacity * (int32_t)sizeof(int32_t));
    M3_BLOCK(world->vehChassis, world->vehicleCapacity * (int32_t)sizeof(int32_t));
    M3_BLOCK(world->vehChassisGen, world->vehicleCapacity * (int32_t)sizeof(uint16_t));
    M3_BLOCK(world->vehWheelCount, world->vehicleCapacity * (int32_t)sizeof(int32_t));
    M3_BLOCK(world->vehMaxSteer, world->vehicleCapacity * (int32_t)sizeof(m3real));
    M3_BLOCK(world->vehDriveForce, world->vehicleCapacity * (int32_t)sizeof(m3real));
    M3_BLOCK(world->vehBrakeForce, world->vehicleCapacity * (int32_t)sizeof(m3real));
    M3_BLOCK(world->vehUserData, world->vehicleCapacity * (int32_t)sizeof(uint64_t));
    M3_BLOCK(world->vehWheelAnchor,
             world->vehicleCapacity * M3_VEHICLE_MAX_WHEELS * (int32_t)sizeof(m3Vec3));
    M3_BLOCK(world->vehWheelDir,
             world->vehicleCapacity * M3_VEHICLE_MAX_WHEELS * (int32_t)sizeof(m3Vec3));
    M3_BLOCK(world->vehWheelRest,
             world->vehicleCapacity * M3_VEHICLE_MAX_WHEELS * (int32_t)sizeof(m3real));
    M3_BLOCK(world->vehWheelTravel,
             world->vehicleCapacity * M3_VEHICLE_MAX_WHEELS * (int32_t)sizeof(m3real));
    M3_BLOCK(world->vehWheelHertz,
             world->vehicleCapacity * M3_VEHICLE_MAX_WHEELS * (int32_t)sizeof(m3real));
    M3_BLOCK(world->vehWheelZeta,
             world->vehicleCapacity * M3_VEHICLE_MAX_WHEELS * (int32_t)sizeof(m3real));
    M3_BLOCK(world->vehWheelRadius,
             world->vehicleCapacity * M3_VEHICLE_MAX_WHEELS * (int32_t)sizeof(m3real));
    M3_BLOCK(world->vehWheelFlags,
             world->vehicleCapacity * M3_VEHICLE_MAX_WHEELS * (int32_t)sizeof(uint8_t));
    M3_BLOCK(world->vehWheelBrake,
             world->vehicleCapacity * M3_VEHICLE_MAX_WHEELS * (int32_t)sizeof(m3real));
    M3_BLOCK(world->vehWheelCompression,
             world->vehicleCapacity * M3_VEHICLE_MAX_WHEELS * (int32_t)sizeof(m3real));
    M3_BLOCK(world->vehWheelContact,
             world->vehicleCapacity * M3_VEHICLE_MAX_WHEELS * (int32_t)sizeof(uint8_t));
    M3_BLOCK(world->vehTireGrip, world->vehicleCapacity * (int32_t)sizeof(m3real));
    M3_BLOCK(world->vehThrottle, world->vehicleCapacity * (int32_t)sizeof(m3real));
    M3_BLOCK(world->vehSteer, world->vehicleCapacity * (int32_t)sizeof(m3real));
    M3_BLOCK(world->vehBrake, world->vehicleCapacity * (int32_t)sizeof(m3real));
    M3_BLOCK(world->vehWheelSpin,
             world->vehicleCapacity * M3_VEHICLE_MAX_WHEELS * (int32_t)sizeof(m3real));
    M3_BLOCK(world->vehDtActive, world->vehicleCapacity * (int32_t)sizeof(uint8_t));
    M3_BLOCK(world->vehDtCurveCount, world->vehicleCapacity * (int32_t)sizeof(int32_t));
    M3_BLOCK(world->vehDtCurveRpm,
             world->vehicleCapacity * M3_DRIVETRAIN_MAX_CURVE * (int32_t)sizeof(m3real));
    M3_BLOCK(world->vehDtCurveTorque,
             world->vehicleCapacity * M3_DRIVETRAIN_MAX_CURVE * (int32_t)sizeof(m3real));
    M3_BLOCK(world->vehDtGearCount, world->vehicleCapacity * (int32_t)sizeof(int32_t));
    M3_BLOCK(world->vehDtGearRatio,
             world->vehicleCapacity * M3_DRIVETRAIN_MAX_GEARS * (int32_t)sizeof(m3real));
    M3_BLOCK(world->vehDtReverse, world->vehicleCapacity * (int32_t)sizeof(m3real));
    M3_BLOCK(world->vehDtFinal, world->vehicleCapacity * (int32_t)sizeof(m3real));
    M3_BLOCK(world->vehDtDiffMode, world->vehicleCapacity * (int32_t)sizeof(int32_t));  // v44
    M3_BLOCK(world->vehDtDiffCouple, world->vehicleCapacity * (int32_t)sizeof(m3real)); // v44
    M3_BLOCK(world->vehWheelLon,
             world->vehicleCapacity * M3_VEHICLE_MAX_WHEELS * (int32_t)sizeof(m3real)); // v44
    M3_BLOCK(world->vehDtShiftUp, world->vehicleCapacity * (int32_t)sizeof(m3real));
    M3_BLOCK(world->vehDtShiftDown, world->vehicleCapacity * (int32_t)sizeof(m3real));
    M3_BLOCK(world->vehDtClutchSteps, world->vehicleCapacity * (int32_t)sizeof(int32_t));
    M3_BLOCK(world->vehDtAutoShift, world->vehicleCapacity * (int32_t)sizeof(uint8_t));
    M3_BLOCK(world->vehDtGear, world->vehicleCapacity * (int32_t)sizeof(int8_t));
    M3_BLOCK(world->vehDtClutch, world->vehicleCapacity * (int32_t)sizeof(int32_t));
    M3_BLOCK(world->vehDtRpm, world->vehicleCapacity * (int32_t)sizeof(m3real));
    M3_BLOCK(world->vehPool.generations, world->vehicleCapacity * (int32_t)sizeof(uint16_t));
    M3_BLOCK(world->vehPool.alive, world->vehicleCapacity * (int32_t)sizeof(uint8_t));
    M3_BLOCK(world->vehPool.freeQueue, world->vehicleCapacity * (int32_t)sizeof(int32_t));
    M3_BLOCK(world->softParticleCount, world->softBodyCapacity * (int32_t)sizeof(int32_t));
    M3_BLOCK(world->softEdgeCount, world->softBodyCapacity * (int32_t)sizeof(int32_t));
    M3_BLOCK(world->softCompliance, world->softBodyCapacity * (int32_t)sizeof(m3real));
    M3_BLOCK(world->softRadius, world->softBodyCapacity * (int32_t)sizeof(m3real));
    M3_BLOCK(world->softGravityScale, world->softBodyCapacity * (int32_t)sizeof(m3real));
    M3_BLOCK(world->softUserData, world->softBodyCapacity * (int32_t)sizeof(uint64_t));
    M3_BLOCK(world->softPos,
             world->softBodyCapacity * M3_SOFTBODY_MAX_PARTICLES * (int32_t)sizeof(m3Pos3));
    M3_BLOCK(world->softPrev,
             world->softBodyCapacity * M3_SOFTBODY_MAX_PARTICLES * (int32_t)sizeof(m3Pos3));
    M3_BLOCK(world->softInvMass,
             world->softBodyCapacity * M3_SOFTBODY_MAX_PARTICLES * (int32_t)sizeof(m3real));
    M3_BLOCK(world->softKick, // v43: pending explosion kicks (13-3)
             world->softBodyCapacity * M3_SOFTBODY_MAX_PARTICLES * (int32_t)sizeof(m3Vec3));
    M3_BLOCK(world->softEdgeA,
             world->softBodyCapacity * M3_SOFTBODY_MAX_EDGES * (int32_t)sizeof(uint16_t));
    M3_BLOCK(world->softEdgeB,
             world->softBodyCapacity * M3_SOFTBODY_MAX_EDGES * (int32_t)sizeof(uint16_t));
    M3_BLOCK(world->softEdgeRest,
             world->softBodyCapacity * M3_SOFTBODY_MAX_EDGES * (int32_t)sizeof(m3real));
    M3_BLOCK(world->softAnchorCount, world->softBodyCapacity * (int32_t)sizeof(int32_t));
    M3_BLOCK(world->softAnchorParticle,
             world->softBodyCapacity * M3_SOFTBODY_MAX_ANCHORS * (int32_t)sizeof(int32_t));
    M3_BLOCK(world->softAnchorBody,
             world->softBodyCapacity * M3_SOFTBODY_MAX_ANCHORS * (int32_t)sizeof(int32_t));
    M3_BLOCK(world->softAnchorGen,
             world->softBodyCapacity * M3_SOFTBODY_MAX_ANCHORS * (int32_t)sizeof(uint16_t));
    M3_BLOCK(world->softAnchorLocal,
             world->softBodyCapacity * M3_SOFTBODY_MAX_ANCHORS * (int32_t)sizeof(m3Vec3));
    M3_BLOCK(world->softSoftCount, world->softBodyCapacity * (int32_t)sizeof(int32_t));
    M3_BLOCK(world->softSoftParticleA,
             world->softBodyCapacity * M3_SOFTBODY_MAX_ANCHORS * (int32_t)sizeof(int32_t));
    M3_BLOCK(world->softSoftSlotB,
             world->softBodyCapacity * M3_SOFTBODY_MAX_ANCHORS * (int32_t)sizeof(int32_t));
    M3_BLOCK(world->softSoftGenB,
             world->softBodyCapacity * M3_SOFTBODY_MAX_ANCHORS * (int32_t)sizeof(uint16_t));
    M3_BLOCK(world->softSoftParticleB,
             world->softBodyCapacity * M3_SOFTBODY_MAX_ANCHORS * (int32_t)sizeof(int32_t));
    M3_BLOCK(world->softPool.generations, world->softBodyCapacity * (int32_t)sizeof(uint16_t));
    M3_BLOCK(world->softPool.alive, world->softBodyCapacity * (int32_t)sizeof(uint8_t));
    M3_BLOCK(world->softPool.freeQueue, world->softBodyCapacity * (int32_t)sizeof(int32_t));
    M3_BLOCK(world->meshRefCounts, world->meshCapacity * (int32_t)sizeof(int32_t));
    M3_BLOCK(world->meshPool.generations, world->meshCapacity * (int32_t)sizeof(uint16_t));
    M3_BLOCK(world->meshPool.alive, world->meshCapacity * (int32_t)sizeof(uint8_t));
    M3_BLOCK(world->meshPool.freeQueue, world->meshCapacity * (int32_t)sizeof(int32_t));
    M3_BLOCK(world->jointType, world->jointCapacity * (int32_t)sizeof(uint8_t));
    M3_BLOCK(world->jointBodyA, world->jointCapacity * (int32_t)sizeof(int32_t));
    M3_BLOCK(world->jointBodyB, world->jointCapacity * (int32_t)sizeof(int32_t));
    M3_BLOCK(world->jointLocalA, world->jointCapacity * (int32_t)sizeof(m3Vec3));
    M3_BLOCK(world->jointLocalB, world->jointCapacity * (int32_t)sizeof(m3Vec3));
    M3_BLOCK(world->jointCollide, world->jointCapacity * (int32_t)sizeof(uint8_t));
    M3_BLOCK(world->jointImpulse, world->jointCapacity * (int32_t)sizeof(m3Vec3));
    M3_BLOCK(world->jointPerpImpulse, world->jointCapacity * (int32_t)sizeof(m3Vec3));
    M3_BLOCK(world->jointLimitImpulse, world->jointCapacity * (int32_t)sizeof(m3Vec3));
    M3_BLOCK(world->jointAngularImpulse, world->jointCapacity * (int32_t)sizeof(m3Vec3));
    M3_BLOCK(world->jointFrameQA, world->jointCapacity * (int32_t)sizeof(m3Quat));
    M3_BLOCK(world->jointFrameQB, world->jointCapacity * (int32_t)sizeof(m3Quat));
    M3_BLOCK(world->jointFlags, world->jointCapacity * (int32_t)sizeof(uint8_t));
    M3_BLOCK(world->jointMotor, world->jointCapacity * (int32_t)sizeof(m3Vec3));
    M3_BLOCK(world->jointBreak, world->jointCapacity * (int32_t)sizeof(m3Vec3));
    M3_BLOCK(world->jointSpring, world->jointCapacity * (int32_t)sizeof(m3Vec3));
    M3_BLOCK(world->jointTargetScalar, world->jointCapacity * (int32_t)sizeof(float));
    M3_BLOCK(world->jointTargetQ, world->jointCapacity * (int32_t)sizeof(m3Quat));
    M3_BLOCK(world->jointSpringImpulse, world->jointCapacity * (int32_t)sizeof(m3Vec3));
    M3_BLOCK(world->jointLimits, world->jointCapacity * (int32_t)sizeof(m3Vec3));
    M3_BLOCK(world->jointGenericModes, world->jointCapacity * (int32_t)sizeof(uint16_t));
    M3_BLOCK(world->jointGenLinLower, world->jointCapacity * (int32_t)sizeof(m3Vec3));
    M3_BLOCK(world->jointGenLinUpper, world->jointCapacity * (int32_t)sizeof(m3Vec3));
    M3_BLOCK(world->jointGenAngLower, world->jointCapacity * (int32_t)sizeof(m3Vec3));
    M3_BLOCK(world->jointGenAngUpper, world->jointCapacity * (int32_t)sizeof(m3Vec3));
    M3_BLOCK(world->jointGroundA, world->jointCapacity * (int32_t)sizeof(m3Pos3));
    M3_BLOCK(world->jointGroundB, world->jointCapacity * (int32_t)sizeof(m3Pos3));
    M3_BLOCK(world->jointNextA, world->jointCapacity * (int32_t)sizeof(int32_t));
    M3_BLOCK(world->jointNextB, world->jointCapacity * (int32_t)sizeof(int32_t));
    M3_BLOCK(world->bodyJointHead, cap * (int32_t)sizeof(int32_t));
    M3_BLOCK(world->jointPool.generations, world->jointCapacity * (int32_t)sizeof(uint16_t));
    M3_BLOCK(world->jointPool.alive, world->jointCapacity * (int32_t)sizeof(uint8_t));
    M3_BLOCK(world->jointPool.freeQueue, world->jointCapacity * (int32_t)sizeof(int32_t));
    M3_BLOCK(world->manifolds, world->pairCapacity * (int32_t)sizeof(m3Manifold));

    // Mesh content LAST (10-3): the only variable-size section, so
    // the fixed prefix above stays state-independent and the
    // restore can pre-validate sizes straight from the buffer.
    // Counts land first; the read pass sizes the slot through the
    // alloc gate before its content arrives. The BVH stays derived
    // and rebuilds after restore.
    if (includeMesh)
    {
        for (int32_t m = 0; m < world->meshCapacity; ++m)
        {
            m3MeshData* mesh = &world->meshData[m];
            M3_BLOCK(&mesh->vertexCount, 4);
            M3_BLOCK(&mesh->triangleCount, 4);
            if (mode == m3_walkRead)
            {
                if (!m3MeshDataAlloc(mesh))
                {
                    return -1; // corrupt counts or no memory: refuse
                }
            }
            if (mesh->triangleCount > 0)
            {
                M3_BLOCK(mesh->vertices, mesh->vertexCount * (int32_t)sizeof(m3Vec3));
                M3_BLOCK(mesh->indices, 3 * mesh->triangleCount * (int32_t)sizeof(uint16_t));
                M3_BLOCK(mesh->edgeFlags, mesh->triangleCount);
                // Material groups (17-2): the count, the fixed
                // table, and one group byte per triangle. A
                // material-free mesh writes zeros throughout.
                M3_BLOCK(&mesh->materialCount, 4);
                if (mode == m3_walkRead &&
                    (mesh->materialCount < 0 || mesh->materialCount > M3_MESH_MAX_MATERIALS))
                {
                    return -1; // corrupt count: refuse
                }
                M3_BLOCK(mesh->materials, (int32_t)sizeof(mesh->materials));
                M3_BLOCK(mesh->triMaterials, mesh->triangleCount);
            }
        }
        // Hull content, count-derived (17-1): counts land first,
        // the read pass validates them against the compile-time
        // caps and zeroes the slab so the unused tail is canonical
        // (a released slot already zeroes itself; this guards the
        // restore-over-a-lived-in-world path). Only the used
        // prefixes of each array travel; an empty slot costs 16
        // bytes instead of 5808.
        for (int32_t hIdx = 0; hIdx < world->shapeCapacity; ++hIdx)
        {
            m3HullData* hull = &world->hullData[hIdx];
            M3_BLOCK(&hull->vertexCount, 4);
            M3_BLOCK(&hull->faceCount, 4);
            M3_BLOCK(&hull->indexCount, 4);
            M3_BLOCK(&hull->edgeCount, 4);
            if (mode == m3_walkRead)
            {
                if (hull->vertexCount < 0 || hull->vertexCount > M3_HULL_MAX_VERTS ||
                    hull->faceCount < 0 || hull->faceCount > M3_HULL_MAX_FACES ||
                    hull->indexCount < 0 || hull->indexCount > M3_HULL_MAX_FACE_INDICES ||
                    hull->edgeCount < 0 || hull->edgeCount > M3_HULL_MAX_HALF_EDGES ||
                    (hull->vertexCount == 0 &&
                     (hull->faceCount | hull->indexCount | hull->edgeCount) != 0))
                {
                    return -1; // corrupt counts: refuse
                }
                int32_t vc = hull->vertexCount;
                int32_t fc = hull->faceCount;
                int32_t ic = hull->indexCount;
                int32_t ec = hull->edgeCount;
                memset(hull, 0, sizeof(*hull));
                hull->vertexCount = vc;
                hull->faceCount = fc;
                hull->indexCount = ic;
                hull->edgeCount = ec;
            }
            if (hull->vertexCount > 0)
            {
                M3_BLOCK(&hull->unitMass, 4);
                M3_BLOCK(&hull->unitCom, (int32_t)sizeof(m3Vec3));
                M3_BLOCK(&hull->unitInertiaCom, (int32_t)sizeof(m3Mat3));
                M3_BLOCK(&hull->center, (int32_t)sizeof(m3Vec3));
                M3_BLOCK(hull->vertices, hull->vertexCount * (int32_t)sizeof(m3Vec3));
                M3_BLOCK(hull->faceNormals, hull->faceCount * (int32_t)sizeof(m3Vec3));
                M3_BLOCK(hull->faceOffsets, hull->faceCount * (int32_t)sizeof(m3real));
                M3_BLOCK(hull->faceVertCounts, hull->faceCount);
                M3_BLOCK(hull->faceVertStart, hull->faceCount * (int32_t)sizeof(uint16_t));
                M3_BLOCK(hull->faceIndices, hull->indexCount);
                M3_BLOCK(hull->edges, hull->edgeCount * (int32_t)sizeof(m3HullHalfEdge));
            }
        }
    }

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
    return (int32_t)sizeof(m3SnapshotHeader) + WalkBlocks(world, NULL, NULL, m3_walkMeasure, 1);
}

int32_t m3World_Snapshot(m3WorldId worldId, void* out, int32_t capacity)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || out == NULL)
    {
        return -1;
    }
    int32_t size =
        (int32_t)sizeof(m3SnapshotHeader) + WalkBlocks(world, NULL, NULL, m3_walkMeasure, 1);
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
    header.treeRoot = world->tree.root;
    header.treeFreeList = world->tree.freeList;
    header.hullMaxIndex = world->hullPool.maxIndex;
    header.hullFreeHead = world->hullPool.freeHead;
    header.hullFreeCount = world->hullPool.freeCount;
    header.hullRetiredCount = world->hullPool.retiredCount;
    header.reserved[0] = 0;
    header.reserved[1] = 0;
    header.meshCapacity = world->meshCapacity;
    header.jointCapacity = world->jointCapacity;
    header.jointMaxIndex = world->jointPool.maxIndex;
    header.jointFreeHead = world->jointPool.freeHead;
    header.jointFreeCount = world->jointPool.freeCount;
    header.jointRetiredCount = world->jointPool.retiredCount;
    header.charCapacity = world->characterCapacity;
    header.charMaxIndex = world->charPool.maxIndex;
    header.charFreeHead = world->charPool.freeHead;
    header.charFreeCount = world->charPool.freeCount;
    header.charRetiredCount = world->charPool.retiredCount;
    header.voxelCapacity = world->voxelCapacity;
    header.voxelMaxIndex = world->voxelPool.maxIndex;
    header.voxelFreeHead = world->voxelPool.freeHead;
    header.voxelFreeCount = world->voxelPool.freeCount;
    header.voxelRetiredCount = world->voxelPool.retiredCount;
    header.meshMaxIndex = world->meshPool.maxIndex;
    header.meshFreeHead = world->meshPool.freeHead;
    header.meshFreeCount = world->meshPool.freeCount;
    header.meshRetiredCount = world->meshPool.retiredCount;
    header.gravity = world->gravity;

    uint8_t* bytes = (uint8_t*)out;
    memcpy(bytes, &header, sizeof(header));
    WalkBlocks(world, bytes + sizeof(header), NULL, m3_walkWrite, 1);
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
        header.shapeCapacity != world->shapeCapacity ||
        header.meshCapacity != world->meshCapacity ||
        header.jointCapacity != world->jointCapacity ||
        header.voxelCapacity != world->voxelCapacity ||
        header.charCapacity != world->characterCapacity)
    {
        // Wrong world shape or wrong build semantics: refuse loudly,
        // never a partial restore.
        return false;
    }
    // Two-phase size validation (10-3): the fixed prefix is
    // state-independent, and the variable mesh tail is parsed
    // straight from the buffer BEFORE any byte lands in the world,
    // so refusal stays atomic.
    int32_t fixed =
        (int32_t)sizeof(m3SnapshotHeader) + WalkBlocks(world, NULL, NULL, m3_walkMeasure, 0);
    int32_t cursor = fixed;
    const uint8_t* raw = (const uint8_t*)data;
    for (int32_t m = 0; m < world->meshCapacity; ++m)
    {
        if (size - cursor < 8)
        {
            return false;
        }
        int32_t vc;
        int32_t tc;
        memcpy(&vc, raw + cursor, 4);
        memcpy(&tc, raw + cursor + 4, 4);
        cursor += 8;
        if (vc < 0 || vc > M3_MESH_MAX_VERTS || tc < 0 || tc > M3_MESH_MAX_TRIS ||
            (vc == 0) != (tc == 0))
        {
            return false; // corrupt counts refuse before any write
        }
        if (tc > 0)
        {
            // Content (10-3) plus the material section (17-2): the
            // count word, the fixed eight-entry table, and a group
            // byte per triangle.
            int64_t content = (int64_t)vc * (int64_t)sizeof(m3Vec3) + 6LL * tc + (int64_t)tc + 4 +
                              (int64_t)(M3_MESH_MAX_MATERIALS * sizeof(m3MeshSurfaceMaterial)) +
                              (int64_t)tc;
            if ((int64_t)size - cursor < content)
            {
                return false;
            }
            cursor += (int32_t)content;
        }
    }
    // The hull tail (17-1): same pre-validation, four counts per
    // slot, content only where vertices exist.
    for (int32_t hIdx = 0; hIdx < world->shapeCapacity; ++hIdx)
    {
        if (size - cursor < 16)
        {
            return false;
        }
        int32_t vc;
        int32_t fc;
        int32_t ic;
        int32_t ec;
        memcpy(&vc, raw + cursor, 4);
        memcpy(&fc, raw + cursor + 4, 4);
        memcpy(&ic, raw + cursor + 8, 4);
        memcpy(&ec, raw + cursor + 12, 4);
        cursor += 16;
        if (vc < 0 || vc > M3_HULL_MAX_VERTS || fc < 0 || fc > M3_HULL_MAX_FACES || ic < 0 ||
            ic > M3_HULL_MAX_FACE_INDICES || ec < 0 || ec > M3_HULL_MAX_HALF_EDGES ||
            (vc == 0 && (fc | ic | ec) != 0))
        {
            return false; // corrupt counts refuse before any write
        }
        if (vc > 0)
        {
            int64_t content = 4 + 2LL * (int64_t)sizeof(m3Vec3) + (int64_t)sizeof(m3Mat3) +
                              (int64_t)vc * (int64_t)sizeof(m3Vec3) +
                              (int64_t)fc * ((int64_t)sizeof(m3Vec3) + 4 + 1 + 2) + (int64_t)ic +
                              (int64_t)ec * (int64_t)sizeof(m3HullHalfEdge);
            if ((int64_t)size - cursor < content)
            {
                return false;
            }
            cursor += (int32_t)content;
        }
    }
    if (size != cursor)
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
    world->tree.root = header.treeRoot;
    world->tree.freeList = header.treeFreeList;
    world->jointPool.maxIndex = header.jointMaxIndex;
    world->jointPool.freeHead = header.jointFreeHead;
    world->jointPool.freeCount = header.jointFreeCount;
    world->jointPool.retiredCount = header.jointRetiredCount;
    world->charPool.maxIndex = header.charMaxIndex;
    world->charPool.freeHead = header.charFreeHead;
    world->charPool.freeCount = header.charFreeCount;
    world->charPool.retiredCount = header.charRetiredCount;
    world->voxelPool.maxIndex = header.voxelMaxIndex;
    world->voxelPool.freeHead = header.voxelFreeHead;
    world->voxelPool.freeCount = header.voxelFreeCount;
    world->voxelPool.retiredCount = header.voxelRetiredCount;
    world->meshPool.maxIndex = header.meshMaxIndex;
    world->meshPool.freeHead = header.meshFreeHead;
    world->meshPool.freeCount = header.meshFreeCount;
    world->meshPool.retiredCount = header.meshRetiredCount;
    // Events are transient observers: a restore clears them.
    world->beginEventCount = 0;
    world->endEventCount = 0;
    world->sensorBeginEventCount = 0;
    world->sensorEndEventCount = 0;
    world->fragmentEventCount = 0;
    world->fragmentRecipeCount = 0;
    world->fragmentDropped = 0;
    world->hitEventCount = 0;
    world->hitEventsDropped = 0;
    world->moveEventCount = 0;
    world->jointBreakEventCount = 0;
    world->lastInvH = 0.0f; // readback reads 0 until the next step
    world->hullPool.maxIndex = header.hullMaxIndex;
    world->hullPool.freeHead = header.hullFreeHead;
    world->hullPool.freeCount = header.hullFreeCount;
    world->hullPool.retiredCount = header.hullRetiredCount;
    if (WalkBlocks(world, NULL, (const uint8_t*)data + sizeof(header), m3_walkRead, 1) < 0)
    {
        return false; // the pre-parse above makes this unreachable,
                      // except for exhausted memory in the alloc gate
    }
    // Derived data follows content: the per-mesh BVHs are rebuilt
    // from the restored triangle sets (a pure function, so the tree
    // a restore produces is byte-identical to the one the original
    // create produced).
    for (int32_t m = 0; m < world->meshPool.maxIndex; ++m)
    {
        if (world->meshPool.alive[m] != 0)
        {
            m3MeshBvhBuild(&world->meshBvh[m], &world->meshData[m]);
        }
    }
    for (int32_t v = 0; v < world->voxelPool.maxIndex; ++v)
    {
        world->voxelShape[v] = -1;
        if (world->voxelPool.alive[v] != 0)
        {
            m3VoxelSurfaceBuild(&world->voxelSurface[v], &world->voxelData[v]);
        }
    }
    for (int32_t s2 = 0; s2 < world->shapePool.maxIndex; ++s2)
    {
        if (world->shapePool.alive[s2] != 0 && world->shapeVoxelIndex[s2] >= 0)
        {
            world->voxelShape[world->shapeVoxelIndex[s2]] = s2;
        }
    }
    m3VoxelRebuildLinks(world);
    for (int32_t v = 0; v < world->voxelPool.maxIndex; ++v)
    {
        if (world->voxelPool.alive[v] != 0)
        {
            m3VoxelCoverageBuild(world, v);
        }
    }
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
    if (world->contactHertz != M3_CONTACT_HERTZ_DEFAULT ||
        world->contactDampingRatio != M3_CONTACT_DAMPING_RATIO_DEFAULT ||
        world->contactPushMaxSpeed != M3_CONTACT_PUSH_MAX_SPEED_DEFAULT ||
        world->restitutionThreshold != M3_RESTITUTION_THRESHOLD_DEFAULT ||
        world->maximumLinearSpeed != M3_MAX_LINEAR_SPEED_DEFAULT || world->sleepEnabled == 0 ||
        world->continuousEnabled == 0 || world->hitEventThreshold != M3_HIT_EVENT_THRESHOLD_DEFAULT)
    {
        // Additive-state golden rule, sixth use: tuning knobs fold
        // only off their defaults.
        h = m3Hash64(h, &world->contactHertz, 4);
        h = m3Hash64(h, &world->contactDampingRatio, 4);
        h = m3Hash64(h, &world->contactPushMaxSpeed, 4);
        h = m3Hash64(h, &world->restitutionThreshold, 4);
        h = m3Hash64(h, &world->maximumLinearSpeed, 4);
        h = m3Hash64(h, &world->sleepEnabled, 1);
        h = m3Hash64(h, &world->continuousEnabled, 1);
        h = m3Hash64(h, &world->hitEventThreshold, 4);
    }
    if (world->maximumAngularSpeed != M3_MAX_ANGULAR_SPEED_DEFAULT)
    {
        // The angular cap folds in its OWN block (13-1), not the 8-4
        // knob block above: appending there would move the hash of
        // every world already off-default on an older knob.
        h = m3Hash64(h, &world->maximumAngularSpeed, 4);
    }
    if (world->windSpeed != 0.0f)
    {
        // Wind folds only when it blows (additive rule), phase
        // included: the gust wave is trajectory-shaping state.
        h = m3Hash64(h, &world->windDir, (int32_t)sizeof(m3Vec3));
        h = m3Hash64(h, &world->windSpeed, 4);
        h = m3Hash64(h, &world->windGustHertz, 4);
        h = m3Hash64(h, &world->windGustScale, 4);
        h = m3Hash64(h, &world->windPhase, 4);
    }
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
        if (world->bodyForce[i].x != 0.0f || world->bodyForce[i].y != 0.0f ||
            world->bodyForce[i].z != 0.0f || world->bodyTorque[i].x != 0.0f ||
            world->bodyTorque[i].y != 0.0f || world->bodyTorque[i].z != 0.0f)
        {
            // Additive-state golden rule: pending host forces fold
            // only when present (a mid-step snapshot must carry
            // them; every force-free scene keeps its hash).
            h = m3Hash64(h, &world->bodyForce[i], (int32_t)sizeof(m3Vec3));
            h = m3Hash64(h, &world->bodyTorque[i], (int32_t)sizeof(m3Vec3));
        }
        if (world->bodyEnabled[i] == 0 || world->bodyLocks[i] != 0 ||
            world->bodySleepThreshold[i] != M3_SLEEP_VELOCITY_DEFAULT ||
            world->bodyCanSleep[i] == 0 || world->bodyHasTarget[i] != 0)
        {
            // Additive-state golden rule, fifth use: control state
            // folds only off its defaults.
            h = m3Hash64(h, &world->bodyEnabled[i], 1);
            h = m3Hash64(h, &world->bodyLocks[i], 1);
            h = m3Hash64(h, &world->bodySleepThreshold[i], 4);
            h = m3Hash64(h, &world->bodyCanSleep[i], 1);
            h = m3Hash64(h, &world->bodyHasTarget[i], 1);
            h = m3Hash64(h, &world->bodyTarget[i], (int32_t)sizeof(m3Transform));
        }
        h = m3Hash64(h, &world->angularVelocities[i], (int32_t)sizeof(m3Vec3));
        h = m3Hash64(h, &world->invMass[i], (int32_t)sizeof(m3real));
        h = m3Hash64(h, &world->invInertiaLocal[i], (int32_t)sizeof(m3Mat3));
        h = m3Hash64(h, &world->localCenters[i], (int32_t)sizeof(m3Vec3));
        h = m3Hash64(h, &world->types[i], 1);
        h = m3Hash64(h, &world->bulletFlags[i], 1);
        h = m3Hash64(h, &world->awake[i], 1);
        h = m3Hash64(h, &world->sleepTimes[i], 4);
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
        if (world->shapeHitEvents[i] != 0 || world->shapePreSolve[i] != 0)
        {
            // Event flags are observable shape state: they fold
            // off-default (additive rule, seventh use).
            h = m3Hash64(h, &world->shapeHitEvents[i], 1);
            h = m3Hash64(h, &world->shapePreSolve[i], 1);
        }
        if (world->shapeSurfaceVel[i].x != 0.0f || world->shapeSurfaceVel[i].y != 0.0f ||
            world->shapeSurfaceVel[i].z != 0.0f)
        {
            h = m3Hash64(h, &world->shapeSurfaceVel[i], (int32_t)sizeof(m3Vec3));
        }
        if (world->shapeHasOffset[i] != 0)
        {
            // Compound offsets fold off-identity (additive rule,
            // tenth use).
            h = m3Hash64(h, &world->shapeLocalPos[i], (int32_t)sizeof(m3Vec3));
            h = m3Hash64(h, &world->shapeLocalRot[i], (int32_t)sizeof(m3Quat));
        }
        if (world->shapeRollingResistance[i] != 0.0f)
        {
            // The additive-state golden rule: the new field folds
            // only where it is set, so every pre-existing scene
            // (rolling resistance zero everywhere) keeps its hash.
            h = m3Hash64(h, &world->shapeRollingResistance[i], 4);
        }
        if (world->shapeCategory[i] != 1ull || world->shapeMask[i] != ~0ull ||
            world->shapeGroup[i] != 0)
        {
            // Same rule for filters (8-1): default-filtered shapes
            // keep every pre-existing hash still.
            h = m3Hash64(h, &world->shapeCategory[i], 8);
            h = m3Hash64(h, &world->shapeMask[i], 8);
            h = m3Hash64(h, &world->shapeGroup[i], 4);
        }
        h = m3Hash64(h, &world->shapeHullIndex[i], 4);
        h = m3Hash64(h, &world->shapeMeshIndex[i], 4);
        h = m3Hash64(h, &world->shapeSensor[i], 1);
        if (world->shapeType[i] == (uint8_t)m3_voxelShape)
        {
            // Folded only for the new type: pre-voxel scenes keep
            // their exact hash input set (the golden must not move
            // for worlds that never touch voxels).
            h = m3Hash64(h, &world->shapeVoxelIndex[i], 4);
        }
    }

    // Character state: live slots only, the additive-state rule.
    int32_t maxChar = world->charPool.maxIndex;
    for (int32_t i = 0; i < maxChar; ++i)
    {
        uint8_t alive = world->charPool.alive[i];
        h = m3Hash64(h, &alive, 1);
        if (alive == 0)
        {
            continue;
        }
        h = m3Hash64(h, &world->charBody[i], 4);
        h = m3Hash64(h, &world->charRadius[i], 4);
        h = m3Hash64(h, &world->charHalfHeight[i], 4);
        h = m3Hash64(h, &world->charCosSlope[i], 4);
        h = m3Hash64(h, &world->charSnap[i], 4);
        h = m3Hash64(h, &world->charSkin[i], 4);
        h = m3Hash64(h, &world->charStepHeight[i], 4);
        h = m3Hash64(h, &world->charGrounded[i], 1);
        h = m3Hash64(h, &world->charGroundNormal[i], (int32_t)sizeof(m3Vec3));
        h = m3Hash64(h, &world->charMass[i], 4);
        h = m3Hash64(h, &world->charPushMax[i], 4);
        h = m3Hash64(h, &world->charGroundBody[i], 4);
        h = m3Hash64(h, &world->charGroundGen[i], 2);
    }
    for (int32_t i = 0; i < world->softPool.maxIndex; ++i)
    {
        if (world->softPool.alive[i] == 0)
        {
            continue; // the additive-state golden rule: live slots only
        }
        h = m3Hash64(h, &world->softParticleCount[i], 4);
        h = m3Hash64(h, &world->softCompliance[i], 4);
        h = m3Hash64(h, &world->softRadius[i], 4);
        h = m3Hash64(h, &world->softGravityScale[i], 4);
        int32_t pc = world->softParticleCount[i];
        for (int32_t p = 0; p < pc; ++p)
        {
            int32_t k = i * M3_SOFTBODY_MAX_PARTICLES + p;
            h = m3Hash64(h, &world->softPos[k], (int32_t)sizeof(m3Pos3));
            h = m3Hash64(h, &world->softPrev[k], (int32_t)sizeof(m3Pos3));
            h = m3Hash64(h, &world->softInvMass[k], 4);
            if (world->softKick[k].x != 0.0f || world->softKick[k].y != 0.0f ||
                world->softKick[k].z != 0.0f)
            {
                // Pending kicks fold only while they exist (13-3):
                // the window between a blast and its next step is
                // real rollback state, everything else is silence.
                h = m3Hash64(h, &world->softKick[k], (int32_t)sizeof(m3Vec3));
            }
        }
        int32_t ec = world->softEdgeCount[i];
        for (int32_t e = 0; e < ec; ++e)
        {
            int32_t k = i * M3_SOFTBODY_MAX_EDGES + e;
            h = m3Hash64(h, &world->softEdgeA[k], 2);
            h = m3Hash64(h, &world->softEdgeB[k], 2);
            h = m3Hash64(h, &world->softEdgeRest[k], 4);
        }
        int32_t ac = world->softAnchorCount[i];
        for (int32_t a = 0; a < ac; ++a)
        {
            int32_t k = i * M3_SOFTBODY_MAX_ANCHORS + a;
            h = m3Hash64(h, &world->softAnchorParticle[k], 4);
            h = m3Hash64(h, &world->softAnchorBody[k], 4);
            h = m3Hash64(h, &world->softAnchorGen[k], 2);
            h = m3Hash64(h, &world->softAnchorLocal[k], (int32_t)sizeof(m3Vec3));
        }
        // Soft-to-soft anchors fold off-empty (additive rule).
        int32_t sc = world->softSoftCount[i];
        for (int32_t a = 0; a < sc; ++a)
        {
            int32_t k = i * M3_SOFTBODY_MAX_ANCHORS + a;
            h = m3Hash64(h, &world->softSoftParticleA[k], 4);
            h = m3Hash64(h, &world->softSoftSlotB[k], 4);
            h = m3Hash64(h, &world->softSoftGenB[k], 2);
            h = m3Hash64(h, &world->softSoftParticleB[k], 4);
        }
    }
    for (int32_t i = 0; i < world->vehPool.maxIndex; ++i)
    {
        if (world->vehPool.alive[i] == 0)
        {
            continue; // the additive-state golden rule: live slots only
        }
        h = m3Hash64(h, &world->vehChassis[i], 4);
        h = m3Hash64(h, &world->vehChassisGen[i], 2);
        h = m3Hash64(h, &world->vehWheelCount[i], 4);
        h = m3Hash64(h, &world->vehMaxSteer[i], 4);
        h = m3Hash64(h, &world->vehDriveForce[i], 4);
        h = m3Hash64(h, &world->vehBrakeForce[i], 4);
        h = m3Hash64(h, &world->vehTireGrip[i], 4);
        h = m3Hash64(h, &world->vehThrottle[i], 4);
        h = m3Hash64(h, &world->vehSteer[i], 4);
        h = m3Hash64(h, &world->vehBrake[i], 4);
        for (int32_t w = 0; w < world->vehWheelCount[i]; ++w)
        {
            int32_t k = i * M3_VEHICLE_MAX_WHEELS + w;
            h = m3Hash64(h, &world->vehWheelAnchor[k], (int32_t)sizeof(m3Vec3));
            h = m3Hash64(h, &world->vehWheelDir[k], (int32_t)sizeof(m3Vec3));
            h = m3Hash64(h, &world->vehWheelRest[k], 4);
            h = m3Hash64(h, &world->vehWheelTravel[k], 4);
            h = m3Hash64(h, &world->vehWheelHertz[k], 4);
            h = m3Hash64(h, &world->vehWheelZeta[k], 4);
            h = m3Hash64(h, &world->vehWheelRadius[k], 4);
            h = m3Hash64(h, &world->vehWheelFlags[k], 1);
            h = m3Hash64(h, &world->vehWheelBrake[k], 4);
            h = m3Hash64(h, &world->vehWheelCompression[k], 4);
            h = m3Hash64(h, &world->vehWheelContact[k], 1);
            h = m3Hash64(h, &world->vehWheelSpin[k], 4);
        }
        if (world->vehDtActive[i] != 0)
        {
            // The drivetrain hashes only when attached: vehicles on
            // the flat force model keep their pre-12-1 hashes (the
            // additive-state golden rule).
            h = m3Hash64(h, &world->vehDtCurveCount[i], 4);
            for (int32_t c = 0; c < world->vehDtCurveCount[i]; ++c)
            {
                h = m3Hash64(h, &world->vehDtCurveRpm[i * M3_DRIVETRAIN_MAX_CURVE + c], 4);
                h = m3Hash64(h, &world->vehDtCurveTorque[i * M3_DRIVETRAIN_MAX_CURVE + c], 4);
            }
            h = m3Hash64(h, &world->vehDtGearCount[i], 4);
            for (int32_t g = 0; g < world->vehDtGearCount[i]; ++g)
            {
                h = m3Hash64(h, &world->vehDtGearRatio[i * M3_DRIVETRAIN_MAX_GEARS + g], 4);
            }
            h = m3Hash64(h, &world->vehDtReverse[i], 4);
            h = m3Hash64(h, &world->vehDtFinal[i], 4);
            if (world->vehDtDiffMode[i] != 0)
            {
                // The diff folds only when engaged (16-4), wheel
                // speeds included: they steer forces only then.
                h = m3Hash64(h, &world->vehDtDiffMode[i], 4);
                h = m3Hash64(h, &world->vehDtDiffCouple[i], 4);
                for (int32_t wq = 0; wq < world->vehWheelCount[i]; ++wq)
                {
                    h = m3Hash64(h, &world->vehWheelLon[i * M3_VEHICLE_MAX_WHEELS + wq], 4);
                }
            }
            h = m3Hash64(h, &world->vehDtShiftUp[i], 4);
            h = m3Hash64(h, &world->vehDtShiftDown[i], 4);
            h = m3Hash64(h, &world->vehDtClutchSteps[i], 4);
            h = m3Hash64(h, &world->vehDtAutoShift[i], 1);
            h = m3Hash64(h, &world->vehDtGear[i], 1);
            h = m3Hash64(h, &world->vehDtClutch[i], 4);
            h = m3Hash64(h, &world->vehDtRpm[i], 4);
        }
    }

    // Voxel chunk content is simulation state (destruction edits it
    // and rollback must cover it); live slots only, same law as
    // everything above.
    int32_t maxVoxel = world->voxelPool.maxIndex;
    for (int32_t i = 0; i < maxVoxel; ++i)
    {
        uint8_t alive = world->voxelPool.alive[i];
        h = m3Hash64(h, &alive, 1);
        if (alive == 0)
        {
            continue;
        }
        h = m3Hash64(h, &world->voxelData[i].cellSize, 4);
        h = m3Hash64(h, &world->voxelData[i].filledCount, 4);
        h = m3Hash64(h, world->voxelData[i].occupancy,
                     (int32_t)sizeof(world->voxelData[i].occupancy));
        h = m3Hash64(h, world->voxelData[i].payload, (int32_t)sizeof(world->voxelData[i].payload));
        h = m3Hash64(h, world->voxelData[i].fill, (int32_t)sizeof(world->voxelData[i].fill));
    }
    int32_t maxJoint = world->jointPool.maxIndex;
    for (int32_t i = 0; i < maxJoint; ++i)
    {
        uint8_t alive = world->jointPool.alive[i];
        h = m3Hash64(h, &alive, 1);
        if (alive == 0)
        {
            continue;
        }
        h = m3Hash64(h, &world->jointType[i], 1);
        h = m3Hash64(h, &world->jointBodyA[i], 4);
        h = m3Hash64(h, &world->jointBodyB[i], 4);
        h = m3Hash64(h, &world->jointLocalA[i], (int32_t)sizeof(m3Vec3));
        h = m3Hash64(h, &world->jointLocalB[i], (int32_t)sizeof(m3Vec3));
        h = m3Hash64(h, &world->jointImpulse[i], (int32_t)sizeof(m3Vec3));
        if (world->jointBreak[i].x != 0.0f || world->jointBreak[i].y != 0.0f)
        {
            // Break thresholds fold off-default (additive rule,
            // eighth use).
            h = m3Hash64(h, &world->jointBreak[i], (int32_t)sizeof(m3Vec3));
        }
        if ((world->jointFlags[i] & 8) != 0 || world->jointTargetScalar[i] != 0.0f ||
            world->jointTargetQ[i].x != 0.0f || world->jointTargetQ[i].y != 0.0f ||
            world->jointTargetQ[i].z != 0.0f || world->jointTargetQ[i].w != 1.0f ||
            world->jointSpringImpulse[i].x != 0.0f || world->jointSpringImpulse[i].y != 0.0f ||
            world->jointSpringImpulse[i].z != 0.0f)
        {
            // Drive springs fold off-default (additive rule, ninth
            // use). The impulse joins: it is dynamics state.
            h = m3Hash64(h, &world->jointSpring[i], (int32_t)sizeof(m3Vec3));
            h = m3Hash64(h, &world->jointTargetScalar[i], 4);
            h = m3Hash64(h, &world->jointTargetQ[i], (int32_t)sizeof(m3Quat));
            h = m3Hash64(h, &world->jointSpringImpulse[i], (int32_t)sizeof(m3Vec3));
        }
        h = m3Hash64(h, &world->jointPerpImpulse[i], (int32_t)sizeof(m3Vec3));
        h = m3Hash64(h, &world->jointLimitImpulse[i], (int32_t)sizeof(m3Vec3));
        h = m3Hash64(h, &world->jointAngularImpulse[i], (int32_t)sizeof(m3Vec3));
        h = m3Hash64(h, &world->jointFrameQA[i], (int32_t)sizeof(m3Quat));
        h = m3Hash64(h, &world->jointFrameQB[i], (int32_t)sizeof(m3Quat));
        h = m3Hash64(h, &world->jointFlags[i], 1);
        if (world->jointType[i] == (uint8_t)m3_genericJoint)
        {
            // Additive-state golden rule: new fields fold only for
            // the new type.
            h = m3Hash64(h, &world->jointGenericModes[i], 2);
            h = m3Hash64(h, &world->jointGenLinLower[i], (int32_t)sizeof(m3Vec3));
            h = m3Hash64(h, &world->jointGenLinUpper[i], (int32_t)sizeof(m3Vec3));
            h = m3Hash64(h, &world->jointGenAngLower[i], (int32_t)sizeof(m3Vec3));
            h = m3Hash64(h, &world->jointGenAngUpper[i], (int32_t)sizeof(m3Vec3));
        }
        if (world->jointType[i] == (uint8_t)m3_pulleyJoint)
        {
            // The same golden rule, tenth use: the rope's world
            // anchors exist only under a pulley.
            h = m3Hash64(h, &world->jointGroundA[i], (int32_t)sizeof(m3Pos3));
            h = m3Hash64(h, &world->jointGroundB[i], (int32_t)sizeof(m3Pos3));
        }
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
