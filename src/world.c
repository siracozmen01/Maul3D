// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// World lifecycle and the journal. Structure follows Maul2D's proven
// world.c (itself adapted from Box2D v3, MIT, Erin Catto): a static
// world table with generations, def-cookie validation, SoA arrays
// allocated per field, and a journal whose replay goes through the
// same internal functions the public API uses.

#include "world_internal.h"

#include <stddef.h>
#include <string.h>

static m3World* s_worlds[M3_MAX_WORLDS];
static uint16_t s_worldGenerations[M3_MAX_WORLDS];

// Journaled defs are untrusted bytes (the fuzz law, 9-5): a
// flipped bit in an embedded bool field is undefined even to LOAD
// as _Bool, so every replay handler normalizes bool bytes through
// uint8_t before the def is used as its C type. UBSAN convicted
// the raw load on the container fuzzer's first day.
static void NormalizeBoolByte(void* base, size_t offset)
{
    uint8_t* b = (uint8_t*)base + offset;
    *b = *b != 0 ? 1 : 0;
}

static void NormalizeShapeDefBools(m3ShapeDef* def)
{
    NormalizeBoolByte(def, offsetof(m3ShapeDef, isSensor));
    NormalizeBoolByte(def, offsetof(m3ShapeDef, enableHitEvents));
    NormalizeBoolByte(def, offsetof(m3ShapeDef, enablePreSolveEvents));
}

m3World* m3WorldFromId(m3WorldId worldId)
{
    int32_t index = worldId.index1 - 1;
    if (index < 0 || index >= M3_MAX_WORLDS || s_worlds[index] == NULL ||
        s_worldGenerations[index] != worldId.generation)
    {
        return NULL;
    }
    return s_worlds[index];
}

m3World* m3WorldFromIndex0(uint16_t index0)
{
    return index0 < M3_MAX_WORLDS ? s_worlds[index0] : NULL;
}

m3WorldDef m3DefaultWorldDef(void)
{
    m3WorldDef def;
    memset(&def, 0, sizeof(def));
    def.gravity = (m3Vec3){0.0f, -10.0f, 0.0f};
    def.bodyCapacity = 1024;
    def.shapeCapacity = 2048;
    def.meshCapacity = 4;
    def.jointCapacity = 64;
    def.voxelCapacity = 4;
    def.characterCapacity = 4;
    def.vehicleCapacity = 2;
    def.softBodyCapacity = 2;
    def.workerCount = 1;
    def.contactHertz = M3_CONTACT_HERTZ_DEFAULT;
    def.contactDampingRatio = M3_CONTACT_DAMPING_RATIO_DEFAULT;
    def.contactPushMaxSpeed = M3_CONTACT_PUSH_MAX_SPEED_DEFAULT;
    def.restitutionThreshold = M3_RESTITUTION_THRESHOLD_DEFAULT;
    def.maximumLinearSpeed = M3_MAX_LINEAR_SPEED_DEFAULT;
    def.enableSleeping = 1;
    def.enableContinuous = 1;
    def.hitEventThreshold = M3_HIT_EVENT_THRESHOLD_DEFAULT;
    def.internalValue = M3_WORLD_COOKIE;
    return def;
}

m3WorldId m3CreateWorld(const m3WorldDef* def)
{
    m3WorldId nullId = {0, 0};
    if (def == NULL || def->internalValue != M3_WORLD_COOKIE || def->bodyCapacity <= 0 ||
        def->shapeCapacity <= 0 || def->meshCapacity <= 0 || def->jointCapacity <= 0 ||
        def->voxelCapacity <= 0 || def->characterCapacity <= 0 || def->vehicleCapacity <= 0 ||
        def->softBodyCapacity <= 0 || def->workerCount <= 0 ||
        (def->enqueueTask == NULL) != (def->finishTask == NULL) || !m3FiniteV3(def->gravity) ||
        !m3FiniteF(def->contactHertz) || def->contactHertz <= 0.0f ||
        !m3FiniteF(def->contactDampingRatio) || def->contactDampingRatio <= 0.0f ||
        !m3FiniteF(def->contactPushMaxSpeed) || def->contactPushMaxSpeed <= 0.0f ||
        !m3FiniteF(def->restitutionThreshold) || def->restitutionThreshold < 0.0f ||
        !m3FiniteF(def->maximumLinearSpeed) || def->maximumLinearSpeed <= 0.0f ||
        !m3FiniteF(def->hitEventThreshold) || def->hitEventThreshold < 0.0f)
    {
        // User-input validation is contract, not invariant: the API
        // promises a null id for a bad def (tests exercise this), so
        // no assert here. Asserts guard states that cannot happen.
        return nullId;
    }

    int32_t slot = -1;
    for (int32_t i = 0; i < M3_MAX_WORLDS; ++i)
    {
        if (s_worlds[i] == NULL)
        {
            slot = i;
            break;
        }
    }
    if (slot < 0)
    {
        return nullId; // table exhausted: loud, never silent, and a
                       // capacity refusal is contract, not invariant
    }

    m3World* world = (m3World*)m3AllocZeroed((int32_t)sizeof(m3World));
    int32_t cap = def->bodyCapacity;
    world->gravity = def->gravity;
    world->contactHertz = def->contactHertz;
    world->contactDampingRatio = def->contactDampingRatio;
    world->contactPushMaxSpeed = def->contactPushMaxSpeed;
    world->restitutionThreshold = def->restitutionThreshold;
    world->maximumLinearSpeed = def->maximumLinearSpeed;
    // Not a def field (13-1): the def cookie stays put under 1.x.
    // Hosts tune it through the journaled setter.
    world->maximumAngularSpeed = M3_MAX_ANGULAR_SPEED_DEFAULT;
    world->sleepEnabled = def->enableSleeping != 0 ? 1 : 0;
    world->continuousEnabled = def->enableContinuous != 0 ? 1 : 0;
    world->hitEventThreshold = def->hitEventThreshold;
    world->preSolveFn = NULL;
    world->preSolveContext = NULL;
    world->lastInvH = 0.0f;
    world->windDir = (m3Vec3){0.0f, 0.0f, 0.0f};
    world->windSpeed = 0.0f;
    world->windGustHertz = 0.0f;
    world->windGustScale = 0.0f;
    world->windPhase = 0.0f;
    world->bodyCapacity = cap;
    world->shapeCapacity = def->shapeCapacity;
    world->meshCapacity = def->meshCapacity;
    world->voxelCapacity = def->voxelCapacity;
    world->characterCapacity = def->characterCapacity;
    world->vehicleCapacity = def->vehicleCapacity;
    world->softBodyCapacity = def->softBodyCapacity;
    world->jointCapacity = def->jointCapacity;
    world->workerCount = def->workerCount;
    world->enqueueTask = def->enqueueTask;
    world->finishTask = def->finishTask;
    world->userTaskContext = def->userTaskContext;
    world->generation = s_worldGenerations[slot];
    world->worldIndex0 = (uint16_t)slot;
    world->bodyPool = m3IdPoolCreate(cap);

    M3_ALLOC(world->transforms, cap, m3Transform);
    M3_ALLOC(world->linearVelocities, cap, m3Vec3);
    M3_ALLOC(world->angularVelocities, cap, m3Vec3);
    M3_ALLOC(world->invMass, cap, m3real);
    M3_ALLOC(world->invInertiaLocal, cap, m3Mat3);
    M3_ALLOC(world->inertiaLocal, cap, m3Mat3);
    M3_ALLOC(world->localCenters, cap, m3Vec3);
    M3_ALLOC(world->gravityScales, cap, m3real);
    M3_ALLOC(world->linearDamping, cap, m3real);
    M3_ALLOC(world->angularDamping, cap, m3real);
    M3_ALLOC(world->types, cap, uint8_t);
    M3_ALLOC(world->awake, cap, uint8_t);
    M3_ALLOC(world->sleepTimes, cap, float);
    M3_ALLOC(world->bulletFlags, cap, uint8_t);
    M3_ALLOC(world->minExtents, cap, float);
    M3_ALLOC(world->maxExtents, cap, float);
    M3_ALLOC(world->userData, cap, uint64_t);
    M3_ALLOC(world->bodyForce, cap, m3Vec3);
    M3_ALLOC(world->bodyTorque, cap, m3Vec3);
    M3_ALLOC(world->bodyEnabled, cap, uint8_t);
    M3_ALLOC(world->bodyLocks, cap, uint8_t);
    M3_ALLOC(world->bodySleepThreshold, cap, float);
    M3_ALLOC(world->bodyCanSleep, cap, uint8_t);
    M3_ALLOC(world->bodyHasTarget, cap, uint8_t);
    M3_ALLOC(world->bodyTarget, cap, m3Transform);
    M3_ALLOC(world->bodyIsland, cap, int32_t);
    M3_ALLOC(world->bodyNames, cap * M3_BODY_NAME_CAPACITY, char);
    M3_ALLOC(world->bodyShapeHead, cap, int32_t);
    for (int32_t i = 0; i < cap; ++i)
    {
        world->bodyIsland[i] = -1; // observer label, no island yet
        world->bodyShapeHead[i] = -1;
    }

    int32_t shapeCap = def->shapeCapacity;
    world->shapePool = m3IdPoolCreate(shapeCap);
    M3_ALLOC(world->shapeBody, shapeCap, int32_t);
    M3_ALLOC(world->shapeType, shapeCap, uint8_t);
    M3_ALLOC(world->shapeGeom, shapeCap, m3ShapeGeom);
    M3_ALLOC(world->shapeDensity, shapeCap, float);
    M3_ALLOC(world->shapeFriction, shapeCap, float);
    M3_ALLOC(world->shapeRestitution, shapeCap, float);
    M3_ALLOC(world->shapeRollingResistance, shapeCap, float);
    M3_ALLOC(world->shapeCategory, shapeCap, uint64_t);
    M3_ALLOC(world->shapeMask, shapeCap, uint64_t);
    M3_ALLOC(world->shapeGroup, shapeCap, int32_t);
    M3_ALLOC(world->shapeUserData, shapeCap, uint64_t);
    M3_ALLOC(world->shapeNext, shapeCap, int32_t);
    for (int32_t i = 0; i < shapeCap; ++i)
    {
        world->shapeBody[i] = -1;
        world->shapeNext[i] = -1;
    }

    M3_ALLOC(world->shapeHullIndex, shapeCap, int32_t);
    for (int32_t i = 0; i < shapeCap; ++i)
    {
        world->shapeHullIndex[i] = -1;
    }
    world->hullPool = m3IdPoolCreate(shapeCap);
    M3_ALLOC(world->hullData, shapeCap, m3HullData);
    M3_ALLOC(world->hullRefCounts, shapeCap, int32_t);
    world->jointPool = m3IdPoolCreate(def->jointCapacity);
    M3_ALLOC(world->jointType, def->jointCapacity, uint8_t);
    M3_ALLOC(world->jointBodyA, def->jointCapacity, int32_t);
    M3_ALLOC(world->jointBodyB, def->jointCapacity, int32_t);
    M3_ALLOC(world->jointLocalA, def->jointCapacity, m3Vec3);
    M3_ALLOC(world->jointLocalB, def->jointCapacity, m3Vec3);
    M3_ALLOC(world->jointCollide, def->jointCapacity, uint8_t);
    M3_ALLOC(world->jointImpulse, def->jointCapacity, m3Vec3);
    M3_ALLOC(world->jointPerpImpulse, def->jointCapacity, m3Vec3);
    M3_ALLOC(world->jointLimitImpulse, def->jointCapacity, m3Vec3);
    M3_ALLOC(world->jointAngularImpulse, def->jointCapacity, m3Vec3);
    M3_ALLOC(world->jointFrameQA, def->jointCapacity, m3Quat);
    M3_ALLOC(world->jointFrameQB, def->jointCapacity, m3Quat);
    M3_ALLOC(world->jointFlags, def->jointCapacity, uint8_t);
    M3_ALLOC(world->jointBreak, def->jointCapacity, m3Vec3);
    M3_ALLOC(world->jointSpring, def->jointCapacity, m3Vec3);
    M3_ALLOC(world->jointTargetScalar, def->jointCapacity, float);
    M3_ALLOC(world->jointTargetQ, def->jointCapacity, m3Quat);
    M3_ALLOC(world->jointSpringImpulse, def->jointCapacity, m3Vec3);
    M3_ALLOC(world->jointMotor, def->jointCapacity, m3Vec3);
    M3_ALLOC(world->jointLimits, def->jointCapacity, m3Vec3);
    M3_ALLOC(world->jointGenericModes, def->jointCapacity, uint16_t);
    M3_ALLOC(world->jointGenLinLower, def->jointCapacity, m3Vec3);
    M3_ALLOC(world->jointGenLinUpper, def->jointCapacity, m3Vec3);
    M3_ALLOC(world->jointGenAngLower, def->jointCapacity, m3Vec3);
    M3_ALLOC(world->jointGenAngUpper, def->jointCapacity, m3Vec3);
    M3_ALLOC(world->jointGroundA, def->jointCapacity, m3Pos3);
    M3_ALLOC(world->jointGroundB, def->jointCapacity, m3Pos3);
    world->hfPool = m3IdPoolCreate(def->shapeCapacity);
    M3_ALLOC(world->hfData, def->shapeCapacity, m3HeightFieldData);
    M3_ALLOC(world->hfRefCounts, def->shapeCapacity, int32_t);
    M3_ALLOC(world->shapeHfIndex, def->shapeCapacity, int32_t);
    for (int32_t hf = 0; hf < def->shapeCapacity; ++hf)
    {
        world->shapeHfIndex[hf] = -1;
    }
    world->waterPool = m3IdPoolCreate(M3_MAX_WATER_VOLUMES);
    world->charPool = m3IdPoolCreate(def->characterCapacity);
    M3_ALLOC(world->charBody, def->characterCapacity, int32_t);
    M3_ALLOC(world->charRadius, def->characterCapacity, m3real);
    M3_ALLOC(world->charHalfHeight, def->characterCapacity, m3real);
    M3_ALLOC(world->charCosSlope, def->characterCapacity, m3real);
    M3_ALLOC(world->charSnap, def->characterCapacity, m3real);
    M3_ALLOC(world->charSkin, def->characterCapacity, m3real);
    M3_ALLOC(world->charStepHeight, def->characterCapacity, m3real);
    M3_ALLOC(world->charMass, def->characterCapacity, m3real);
    M3_ALLOC(world->charPushMax, def->characterCapacity, m3real);
    M3_ALLOC(world->charGroundBody, def->characterCapacity, int32_t);
    M3_ALLOC(world->charGroundGen, def->characterCapacity, uint16_t);
    world->vehPool = m3IdPoolCreate(def->vehicleCapacity);
    M3_ALLOC(world->vehChassis, def->vehicleCapacity, int32_t);
    M3_ALLOC(world->vehChassisGen, def->vehicleCapacity, uint16_t);
    M3_ALLOC(world->vehWheelCount, def->vehicleCapacity, int32_t);
    M3_ALLOC(world->vehMaxSteer, def->vehicleCapacity, m3real);
    M3_ALLOC(world->vehDriveForce, def->vehicleCapacity, m3real);
    M3_ALLOC(world->vehBrakeForce, def->vehicleCapacity, m3real);
    M3_ALLOC(world->vehUserData, def->vehicleCapacity, uint64_t);
    M3_ALLOC(world->vehWheelAnchor, def->vehicleCapacity * M3_VEHICLE_MAX_WHEELS, m3Vec3);
    M3_ALLOC(world->vehWheelDir, def->vehicleCapacity * M3_VEHICLE_MAX_WHEELS, m3Vec3);
    M3_ALLOC(world->vehWheelRest, def->vehicleCapacity * M3_VEHICLE_MAX_WHEELS, m3real);
    M3_ALLOC(world->vehWheelTravel, def->vehicleCapacity * M3_VEHICLE_MAX_WHEELS, m3real);
    M3_ALLOC(world->vehWheelHertz, def->vehicleCapacity * M3_VEHICLE_MAX_WHEELS, m3real);
    M3_ALLOC(world->vehWheelZeta, def->vehicleCapacity * M3_VEHICLE_MAX_WHEELS, m3real);
    M3_ALLOC(world->vehWheelRadius, def->vehicleCapacity * M3_VEHICLE_MAX_WHEELS, m3real);
    M3_ALLOC(world->vehWheelFlags, def->vehicleCapacity * M3_VEHICLE_MAX_WHEELS, uint8_t);
    M3_ALLOC(world->vehWheelBrake, def->vehicleCapacity * M3_VEHICLE_MAX_WHEELS, m3real);
    M3_ALLOC(world->vehTrackMode, def->vehicleCapacity, uint8_t);
    M3_ALLOC(world->vehTrackLeft, def->vehicleCapacity, m3real);
    M3_ALLOC(world->vehTrackRight, def->vehicleCapacity, m3real);
    M3_ALLOC(world->vehLeanGain, def->vehicleCapacity, m3real);
    M3_ALLOC(world->vehWheelCompression, def->vehicleCapacity * M3_VEHICLE_MAX_WHEELS, m3real);
    M3_ALLOC(world->vehWheelContact, def->vehicleCapacity * M3_VEHICLE_MAX_WHEELS, uint8_t);
    M3_ALLOC(world->vehTireGrip, def->vehicleCapacity, m3real);
    M3_ALLOC(world->vehThrottle, def->vehicleCapacity, m3real);
    M3_ALLOC(world->vehSteer, def->vehicleCapacity, m3real);
    M3_ALLOC(world->vehBrake, def->vehicleCapacity, m3real);
    M3_ALLOC(world->vehWheelSpin, def->vehicleCapacity * M3_VEHICLE_MAX_WHEELS, m3real);
    M3_ALLOC(world->vehDtActive, def->vehicleCapacity, uint8_t);
    M3_ALLOC(world->vehDtCurveCount, def->vehicleCapacity, int32_t);
    M3_ALLOC(world->vehDtCurveRpm, def->vehicleCapacity * M3_DRIVETRAIN_MAX_CURVE, m3real);
    M3_ALLOC(world->vehDtCurveTorque, def->vehicleCapacity * M3_DRIVETRAIN_MAX_CURVE, m3real);
    M3_ALLOC(world->vehDtGearCount, def->vehicleCapacity, int32_t);
    M3_ALLOC(world->vehDtGearRatio, def->vehicleCapacity * M3_DRIVETRAIN_MAX_GEARS, m3real);
    M3_ALLOC(world->vehDtReverse, def->vehicleCapacity, m3real);
    M3_ALLOC(world->vehDtFinal, def->vehicleCapacity, m3real);
    M3_ALLOC(world->vehDtDiffMode, def->vehicleCapacity, int32_t);
    M3_ALLOC(world->vehDtDiffCouple, def->vehicleCapacity, m3real);
    M3_ALLOC(world->vehWheelLon, def->vehicleCapacity * M3_VEHICLE_MAX_WHEELS, m3real);
    M3_ALLOC(world->vehDtShiftUp, def->vehicleCapacity, m3real);
    M3_ALLOC(world->vehDtShiftDown, def->vehicleCapacity, m3real);
    M3_ALLOC(world->vehDtClutchSteps, def->vehicleCapacity, int32_t);
    M3_ALLOC(world->vehDtAutoShift, def->vehicleCapacity, uint8_t);
    M3_ALLOC(world->vehDtGear, def->vehicleCapacity, int8_t);
    M3_ALLOC(world->vehDtClutch, def->vehicleCapacity, int32_t);
    M3_ALLOC(world->vehDtRpm, def->vehicleCapacity, m3real);
    world->softPool = m3IdPoolCreate(def->softBodyCapacity);
    M3_ALLOC(world->softParticleCount, def->softBodyCapacity, int32_t);
    M3_ALLOC(world->softEdgeCount, def->softBodyCapacity, int32_t);
    M3_ALLOC(world->softCompliance, def->softBodyCapacity, m3real);
    M3_ALLOC(world->softBendStart, def->softBodyCapacity, int32_t);
    M3_ALLOC(world->softBendCompliance, def->softBodyCapacity, m3real);
    M3_ALLOC(world->softDimX, def->softBodyCapacity, uint16_t);
    M3_ALLOC(world->softDimY, def->softBodyCapacity, uint16_t);
    M3_ALLOC(world->softDimZ, def->softBodyCapacity, uint16_t);
    M3_ALLOC(world->softRestVolume, def->softBodyCapacity, m3real);
    M3_ALLOC(world->softPressure, def->softBodyCapacity, m3real);
    M3_ALLOC(world->softTetCount, def->softBodyCapacity, int32_t);
    M3_ALLOC(world->softTetA, def->softBodyCapacity * M3_SOFTBODY_MAX_TETS, uint16_t);
    M3_ALLOC(world->softTetB, def->softBodyCapacity * M3_SOFTBODY_MAX_TETS, uint16_t);
    M3_ALLOC(world->softTetC, def->softBodyCapacity * M3_SOFTBODY_MAX_TETS, uint16_t);
    M3_ALLOC(world->softTetD, def->softBodyCapacity * M3_SOFTBODY_MAX_TETS, uint16_t);
    M3_ALLOC(world->softTetRestV6, def->softBodyCapacity * M3_SOFTBODY_MAX_TETS, m3real);
    M3_ALLOC(world->softBindPos, def->softBodyCapacity * M3_SOFTBODY_MAX_PARTICLES, m3Pos3);
    M3_ALLOC(world->softMaxDeviation, def->softBodyCapacity, m3real);
    M3_ALLOC(world->softRadius, def->softBodyCapacity, m3real);
    M3_ALLOC(world->softGravityScale, def->softBodyCapacity, m3real);
    M3_ALLOC(world->softUserData, def->softBodyCapacity, uint64_t);
    M3_ALLOC(world->softPos, def->softBodyCapacity * M3_SOFTBODY_MAX_PARTICLES, m3Pos3);
    M3_ALLOC(world->softPrev, def->softBodyCapacity * M3_SOFTBODY_MAX_PARTICLES, m3Pos3);
    M3_ALLOC(world->softInvMass, def->softBodyCapacity * M3_SOFTBODY_MAX_PARTICLES, m3real);
    M3_ALLOC(world->softKick, def->softBodyCapacity * M3_SOFTBODY_MAX_PARTICLES, m3Vec3);
    M3_ALLOC(world->softEdgeA, def->softBodyCapacity * M3_SOFTBODY_MAX_EDGES, uint16_t);
    M3_ALLOC(world->softEdgeB, def->softBodyCapacity * M3_SOFTBODY_MAX_EDGES, uint16_t);
    M3_ALLOC(world->softEdgeRest, def->softBodyCapacity * M3_SOFTBODY_MAX_EDGES, m3real);
    M3_ALLOC(world->softAnchorCount, def->softBodyCapacity, int32_t);
    M3_ALLOC(world->softAnchorParticle, def->softBodyCapacity * M3_SOFTBODY_MAX_ANCHORS, int32_t);
    M3_ALLOC(world->softAnchorBody, def->softBodyCapacity * M3_SOFTBODY_MAX_ANCHORS, int32_t);
    M3_ALLOC(world->softAnchorGen, def->softBodyCapacity * M3_SOFTBODY_MAX_ANCHORS, uint16_t);
    M3_ALLOC(world->softAnchorLocal, def->softBodyCapacity * M3_SOFTBODY_MAX_ANCHORS, m3Vec3);
    M3_ALLOC(world->softSoftCount, def->softBodyCapacity, int32_t);
    M3_ALLOC(world->softSoftParticleA, def->softBodyCapacity * M3_SOFTBODY_MAX_ANCHORS, int32_t);
    M3_ALLOC(world->softSoftSlotB, def->softBodyCapacity * M3_SOFTBODY_MAX_ANCHORS, int32_t);
    M3_ALLOC(world->softSoftGenB, def->softBodyCapacity * M3_SOFTBODY_MAX_ANCHORS, uint16_t);
    M3_ALLOC(world->softSoftParticleB, def->softBodyCapacity * M3_SOFTBODY_MAX_ANCHORS, int32_t);
    for (int32_t v = 0; v < def->vehicleCapacity; ++v)
    {
        world->vehChassis[v] = -1;
    }
    M3_ALLOC(world->charGrounded, def->characterCapacity, uint8_t);
    M3_ALLOC(world->charGroundNormal, def->characterCapacity, m3Vec3);
    for (int32_t i = 0; i < def->characterCapacity; ++i)
    {
        world->charBody[i] = -1;
    }
    M3_ALLOC(world->jointNextA, def->jointCapacity, int32_t);
    M3_ALLOC(world->jointNextB, def->jointCapacity, int32_t);
    M3_ALLOC(world->bodyJointHead, cap, int32_t);
    for (int32_t i = 0; i < def->jointCapacity; ++i)
    {
        world->jointBodyA[i] = -1;
        world->jointBodyB[i] = -1;
        world->jointNextA[i] = -1;
        world->jointNextB[i] = -1;
    }
    for (int32_t i = 0; i < cap; ++i)
    {
        world->bodyJointHead[i] = -1;
    }
    world->meshPool = m3IdPoolCreate(def->meshCapacity);
    M3_ALLOC(world->meshData, def->meshCapacity, m3MeshData);
    M3_ALLOC(world->meshRefCounts, def->meshCapacity, int32_t);
    M3_ALLOC(world->meshBvh, def->meshCapacity, m3MeshBvh);
    world->voxelPool = m3IdPoolCreate(def->voxelCapacity);
    M3_ALLOC(world->voxelData, def->voxelCapacity, m3VoxelChunkData);
    M3_ALLOC(world->voxelRefCounts, def->voxelCapacity, int32_t);
    M3_ALLOC(world->voxelSurface, def->voxelCapacity, m3VoxelSurface);
    M3_ALLOC(world->voxelShape, def->voxelCapacity, int32_t);
    M3_ALLOC(world->voxelNeighbors, def->voxelCapacity * 6, int32_t);
    for (int32_t i = 0; i < def->voxelCapacity; ++i)
    {
        world->voxelShape[i] = -1;
    }
    for (int32_t i = 0; i < def->voxelCapacity * 6; ++i)
    {
        world->voxelNeighbors[i] = -1;
    }
    M3_ALLOC(world->shapeVoxelIndex, shapeCap, int32_t);
    M3_ALLOC(world->fragmentEvents, M3_FRAGMENT_EVENT_CAP, m3FragmentEvent);
    M3_ALLOC(world->fragmentRecipe, M3_FRAGMENT_RECIPE_CAP, uint16_t);
    for (int32_t i = 0; i < shapeCap; ++i)
    {
        world->shapeVoxelIndex[i] = -1;
    }
    M3_ALLOC(world->shapeMeshIndex, shapeCap, int32_t);
    M3_ALLOC(world->shapeSensor, shapeCap, uint8_t);
    M3_ALLOC(world->shapeHitEvents, shapeCap, uint8_t);
    M3_ALLOC(world->shapePreSolve, shapeCap, uint8_t);
    M3_ALLOC(world->shapeLocalPos, shapeCap, m3Vec3);
    M3_ALLOC(world->shapeLocalRot, shapeCap, m3Quat);
    M3_ALLOC(world->shapeHasOffset, shapeCap, uint8_t);
    M3_ALLOC(world->shapeSurfaceVel, shapeCap, m3Vec3);
    for (int32_t i = 0; i < shapeCap; ++i)
    {
        world->shapeMeshIndex[i] = -1;
    }

    world->tree = m3TreeCreate(2 * shapeCap);
    M3_ALLOC(world->proxyIds, shapeCap, int32_t);
    for (int32_t i = 0; i < shapeCap; ++i)
    {
        world->proxyIds[i] = M3_TREE_NULL;
    }

    world->pairCapacity = 8 * shapeCap;
    M3_ALLOC(world->beginEvents, world->pairCapacity, m3ContactEvent);
    M3_ALLOC(world->endEvents, world->pairCapacity, m3ContactEvent);
    M3_ALLOC(world->sensorBeginEvents, world->pairCapacity, m3ContactEvent);
    M3_ALLOC(world->sensorEndEvents, world->pairCapacity, m3ContactEvent);
    M3_ALLOC(world->hitEvents, world->pairCapacity, m3HitEvent);
    M3_ALLOC(world->moveEvents, def->bodyCapacity, m3BodyMoveEvent);
    M3_ALLOC(world->jointBreakEvents, def->jointCapacity, m3JointBreakEvent);
    world->hitEventCount = 0;
    world->hitEventsDropped = 0;
    world->moveEventCount = 0;
    world->jointBreakEventCount = 0;
    world->beginEventCount = 0;
    world->endEventCount = 0;
    M3_ALLOC(world->pairKeys, world->pairCapacity, uint64_t);
    M3_ALLOC(world->manifolds, world->pairCapacity, m3Manifold);
    world->pairCount = 0;

    // Step scratch: grows between steps on m3_errorCapacity, never
    // mid-step. 256 KiB is generous for the 2a sphere world.
    world->scratch = m3StackCreate(256 * 1024);

    s_worlds[slot] = world;
    m3WorldId id = {slot + 1, world->generation};
    return id;
}

void m3DestroyWorld(m3WorldId worldId)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL)
    {
        return; // stale or foreign id: contract, not invariant
    }
    int32_t slot = world->worldIndex0;

    m3IdPoolDestroy(&world->bodyPool);
    m3Free(world->transforms);
    m3Free(world->linearVelocities);
    m3Free(world->angularVelocities);
    m3Free(world->invMass);
    m3Free(world->invInertiaLocal);
    m3Free(world->inertiaLocal);
    m3Free(world->localCenters);
    m3Free(world->gravityScales);
    m3Free(world->linearDamping);
    m3Free(world->angularDamping);
    m3Free(world->types);
    m3Free(world->awake);
    m3Free(world->sleepTimes);
    m3Free(world->bulletFlags);
    m3Free(world->minExtents);
    m3Free(world->maxExtents);
    m3Free(world->userData);
    m3Free(world->bodyForce);
    m3Free(world->bodyTorque);
    m3Free(world->bodyEnabled);
    m3Free(world->bodyLocks);
    m3Free(world->bodySleepThreshold);
    m3Free(world->bodyCanSleep);
    m3Free(world->bodyHasTarget);
    m3Free(world->bodyTarget);
    m3Free(world->bodyIsland);
    m3Free(world->bodyNames);
    m3Free(world->bodyShapeHead);
    m3IdPoolDestroy(&world->shapePool);
    m3Free(world->shapeBody);
    m3Free(world->shapeType);
    m3Free(world->shapeGeom);
    m3Free(world->shapeDensity);
    m3Free(world->shapeFriction);
    m3Free(world->shapeRestitution);
    m3Free(world->shapeRollingResistance);
    m3Free(world->shapeCategory);
    m3Free(world->shapeMask);
    m3Free(world->shapeGroup);
    m3Free(world->shapeUserData);
    m3Free(world->shapeNext);
    m3Free(world->shapeHullIndex);
    m3IdPoolDestroy(&world->hullPool);
    m3Free(world->hullData);
    m3Free(world->hullRefCounts);
    m3IdPoolDestroy(&world->jointPool);
    m3Free(world->jointType);
    m3Free(world->jointBodyA);
    m3Free(world->jointBodyB);
    m3Free(world->jointLocalA);
    m3Free(world->jointLocalB);
    m3Free(world->jointCollide);
    m3Free(world->jointImpulse);
    m3Free(world->jointPerpImpulse);
    m3Free(world->jointLimitImpulse);
    m3Free(world->jointAngularImpulse);
    m3Free(world->jointFrameQA);
    m3Free(world->jointFrameQB);
    m3Free(world->jointFlags);
    m3Free(world->jointBreak);
    m3Free(world->jointSpring);
    m3Free(world->jointTargetScalar);
    m3Free(world->jointTargetQ);
    m3Free(world->jointSpringImpulse);
    m3Free(world->jointMotor);
    m3Free(world->jointLimits);
    m3Free(world->jointGenericModes);
    m3Free(world->jointGenLinLower);
    m3Free(world->jointGenLinUpper);
    m3Free(world->jointGenAngLower);
    m3Free(world->jointGenAngUpper);
    m3Free(world->jointGroundA);
    m3Free(world->jointGroundB);
    for (int32_t hf = 0; hf < world->shapeCapacity; ++hf)
    {
        m3HeightFieldDataFree(&world->hfData[hf]);
    }
    m3IdPoolDestroy(&world->hfPool);
    m3Free(world->hfData);
    m3Free(world->hfRefCounts);
    m3Free(world->shapeHfIndex);
    m3IdPoolDestroy(&world->waterPool);
    m3IdPoolDestroy(&world->charPool);
    m3Free(world->charBody);
    m3Free(world->charRadius);
    m3Free(world->charHalfHeight);
    m3Free(world->charCosSlope);
    m3Free(world->charSnap);
    m3Free(world->charSkin);
    m3Free(world->charStepHeight);
    m3Free(world->charMass);
    m3Free(world->charPushMax);
    m3Free(world->charGroundBody);
    m3Free(world->charGroundGen);
    m3IdPoolDestroy(&world->vehPool);
    m3Free(world->vehChassis);
    m3Free(world->vehChassisGen);
    m3Free(world->vehWheelCount);
    m3Free(world->vehMaxSteer);
    m3Free(world->vehDriveForce);
    m3Free(world->vehBrakeForce);
    m3Free(world->vehUserData);
    m3Free(world->vehWheelAnchor);
    m3Free(world->vehWheelDir);
    m3Free(world->vehWheelRest);
    m3Free(world->vehWheelTravel);
    m3Free(world->vehWheelHertz);
    m3Free(world->vehWheelZeta);
    m3Free(world->vehWheelRadius);
    m3Free(world->vehWheelFlags);
    m3Free(world->vehWheelBrake);
    m3Free(world->vehTrackMode);
    m3Free(world->vehTrackLeft);
    m3Free(world->vehTrackRight);
    m3Free(world->vehLeanGain);
    m3Free(world->vehWheelCompression);
    m3Free(world->vehWheelContact);
    m3Free(world->vehTireGrip);
    m3Free(world->vehThrottle);
    m3Free(world->vehSteer);
    m3Free(world->vehBrake);
    m3Free(world->vehWheelSpin);
    m3Free(world->vehDtActive);
    m3Free(world->vehDtCurveCount);
    m3Free(world->vehDtCurveRpm);
    m3Free(world->vehDtCurveTorque);
    m3Free(world->vehDtGearCount);
    m3Free(world->vehDtGearRatio);
    m3Free(world->vehDtReverse);
    m3Free(world->vehDtFinal);
    m3Free(world->vehDtDiffMode);
    m3Free(world->vehDtDiffCouple);
    m3Free(world->vehWheelLon);
    m3Free(world->vehDtShiftUp);
    m3Free(world->vehDtShiftDown);
    m3Free(world->vehDtClutchSteps);
    m3Free(world->vehDtAutoShift);
    m3Free(world->vehDtGear);
    m3Free(world->vehDtClutch);
    m3Free(world->vehDtRpm);
    m3IdPoolDestroy(&world->softPool);
    m3Free(world->softParticleCount);
    m3Free(world->softEdgeCount);
    m3Free(world->softCompliance);
    m3Free(world->softBendStart);
    m3Free(world->softBendCompliance);
    m3Free(world->softDimX);
    m3Free(world->softDimY);
    m3Free(world->softDimZ);
    m3Free(world->softRestVolume);
    m3Free(world->softPressure);
    m3Free(world->softTetCount);
    m3Free(world->softTetA);
    m3Free(world->softTetB);
    m3Free(world->softTetC);
    m3Free(world->softTetD);
    m3Free(world->softTetRestV6);
    m3Free(world->softBindPos);
    m3Free(world->softMaxDeviation);
    m3Free(world->softRadius);
    m3Free(world->softGravityScale);
    m3Free(world->softUserData);
    m3Free(world->softPos);
    m3Free(world->softPrev);
    m3Free(world->softInvMass);
    m3Free(world->softKick);
    m3Free(world->softEdgeA);
    m3Free(world->softEdgeB);
    m3Free(world->softEdgeRest);
    m3Free(world->softAnchorCount);
    m3Free(world->softAnchorParticle);
    m3Free(world->softAnchorBody);
    m3Free(world->softAnchorGen);
    m3Free(world->softAnchorLocal);
    m3Free(world->softSoftCount);
    m3Free(world->softSoftParticleA);
    m3Free(world->softSoftSlotB);
    m3Free(world->softSoftGenB);
    m3Free(world->softSoftParticleB);
    m3Free(world->charGrounded);
    m3Free(world->charGroundNormal);
    m3Free(world->jointNextA);
    m3Free(world->jointNextB);
    m3Free(world->bodyJointHead);
    m3IdPoolDestroy(&world->meshPool);
    for (int32_t m = 0; m < world->meshCapacity; ++m)
    {
        m3MeshDataFree(&world->meshData[m]);
        m3MeshBvhFree(&world->meshBvh[m]);
    }
    for (int32_t v = 0; v < world->voxelCapacity; ++v)
    {
        m3MeshBvhFree(&world->voxelSurface[v].bvh);
    }
    m3Free(world->meshData);
    m3Free(world->meshRefCounts);
    m3Free(world->meshBvh);
    m3IdPoolDestroy(&world->voxelPool);
    m3Free(world->voxelData);
    m3Free(world->voxelRefCounts);
    m3Free(world->voxelSurface);
    m3Free(world->voxelShape);
    m3Free(world->voxelNeighbors);
    m3Free(world->shapeVoxelIndex);
    m3Free(world->fragmentEvents);
    m3Free(world->fragmentRecipe);
    m3Free(world->shapeMeshIndex);
    m3Free(world->shapeSensor);
    m3Free(world->shapeHitEvents);
    m3Free(world->shapePreSolve);
    m3Free(world->shapeLocalPos);
    m3Free(world->shapeLocalRot);
    m3Free(world->shapeHasOffset);
    m3Free(world->shapeSurfaceVel);
    m3Free(world->hitEvents);
    m3Free(world->moveEvents);
    m3Free(world->jointBreakEvents);
    m3Free(world->beginEvents);
    m3Free(world->endEvents);
    m3Free(world->sensorBeginEvents);
    m3Free(world->sensorEndEvents);
    m3TreeDestroy(&world->tree);
    m3Free(world->proxyIds);
    m3Free(world->pairKeys);
    m3Free(world->manifolds);
    m3StackDestroy(&world->scratch);
    m3Free(world);

    s_worlds[slot] = NULL;
    s_worldGenerations[slot] += 1;
}

bool m3World_IsValid(m3WorldId worldId)
{
    return m3WorldFromId(worldId) != NULL;
}

// --- Tuning knobs (8-4) -----------------------------------------------------

void m3SetGravityInternal(m3World* world, m3Vec3 gravity)
{
    world->gravity = gravity;
}

void m3SetContactTuningInternal(m3World* world, float hertz, float dampingRatio, float pushSpeed)
{
    world->contactHertz = hertz;
    world->contactDampingRatio = dampingRatio;
    world->contactPushMaxSpeed = pushSpeed;
}

void m3SetRestitutionThresholdInternal(m3World* world, float value)
{
    world->restitutionThreshold = value;
}

void m3SetMaximumLinearSpeedInternal(m3World* world, float value)
{
    world->maximumLinearSpeed = value;
}

void m3SetMaximumAngularSpeedInternal(m3World* world, float value)
{
    world->maximumAngularSpeed = value;
}

void m3EnableSleepingInternal(m3World* world, int32_t on)
{
    world->sleepEnabled = on != 0 ? 1 : 0;
    if (on == 0)
    {
        // The reference wakes every sleeping set when sleeping turns
        // off; nothing may keep napping through the new regime.
        int32_t maxBody = world->bodyPool.maxIndex;
        for (int32_t i = 0; i < maxBody; ++i)
        {
            if (world->bodyPool.alive[i] != 0 && world->awake[i] == 0 &&
                world->types[i] == (uint8_t)m3_dynamicBody)
            {
                m3SetAwakeInternal(world, i, 1);
            }
        }
    }
}

void m3EnableContinuousInternal(m3World* world, int32_t on)
{
    world->continuousEnabled = on != 0 ? 1 : 0;
}

void m3World_SetGravity(m3WorldId worldId, m3Vec3 gravity)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || !m3FiniteV3(gravity))
    {
        return;
    }
    if (world->journalActive != 0)
    {
        m3JournalRecord(world, m3_opSetGravity, &gravity, (int32_t)sizeof(gravity));
    }
    m3SetGravityInternal(world, gravity);
}

m3Vec3 m3World_GetGravity(m3WorldId worldId)
{
    m3World* world = m3WorldFromId(worldId);
    m3Vec3 zero = {0.0f, 0.0f, 0.0f};
    return world != NULL ? world->gravity : zero;
}

void m3World_SetContactTuning(m3WorldId worldId, float hertz, float dampingRatio,
                              float pushMaxSpeed)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || !m3FiniteF(hertz) || hertz <= 0.0f || !m3FiniteF(dampingRatio) ||
        dampingRatio <= 0.0f || !m3FiniteF(pushMaxSpeed) || pushMaxSpeed <= 0.0f)
    {
        return;
    }
    if (world->journalActive != 0)
    {
        struct
        {
            float hertz;
            float dampingRatio;
            float pushSpeed;
        } record;
        memset(&record, 0, sizeof(record));
        record.hertz = hertz;
        record.dampingRatio = dampingRatio;
        record.pushSpeed = pushMaxSpeed;
        m3JournalRecord(world, m3_opSetContactTuning, &record, (int32_t)sizeof(record));
    }
    m3SetContactTuningInternal(world, hertz, dampingRatio, pushMaxSpeed);
}

void m3World_SetRestitutionThreshold(m3WorldId worldId, float value)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || !m3FiniteF(value) || value < 0.0f)
    {
        return;
    }
    if (world->journalActive != 0)
    {
        m3JournalRecord(world, m3_opSetRestitutionThreshold, &value, (int32_t)sizeof(value));
    }
    m3SetRestitutionThresholdInternal(world, value);
}

void m3World_SetMaximumLinearSpeed(m3WorldId worldId, float value)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || !m3FiniteF(value) || value <= 0.0f)
    {
        return;
    }
    if (world->journalActive != 0)
    {
        m3JournalRecord(world, m3_opSetMaximumLinearSpeed, &value, (int32_t)sizeof(value));
    }
    m3SetMaximumLinearSpeedInternal(world, value);
}

void m3World_SetMaximumAngularSpeed(m3WorldId worldId, float value)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || !m3FiniteF(value) || value <= 0.0f)
    {
        return;
    }
    if (world->journalActive != 0)
    {
        m3JournalRecord(world, m3_opSetMaximumAngularSpeed, &value, (int32_t)sizeof(value));
    }
    m3SetMaximumAngularSpeedInternal(world, value);
}

// Live slots without a scan: the pool hands out from the free queue
// or bumps maxIndex, and retirement is the only other exit (14-1).
static int32_t PoolLive(const m3IdPool* pool)
{
    return pool->maxIndex - pool->freeCount - pool->retiredCount;
}

m3Counters m3World_GetCounters(m3WorldId worldId)
{
    m3Counters counters;
    memset(&counters, 0, sizeof(counters));
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL)
    {
        return counters;
    }
    counters.bodyCount = PoolLive(&world->bodyPool);
    counters.shapeCount = PoolLive(&world->shapePool);
    counters.jointCount = PoolLive(&world->jointPool);
    counters.contactCount = world->pairCount;
    counters.characterCount = PoolLive(&world->charPool);
    counters.vehicleCount = PoolLive(&world->vehPool);
    counters.softBodyCount = PoolLive(&world->softPool);
    counters.voxelChunkCount = PoolLive(&world->voxelPool);
    counters.hullCount = PoolLive(&world->hullPool);
    counters.meshCount = PoolLive(&world->meshPool);
    int32_t awake = 0;
    int32_t maxBody = world->bodyPool.maxIndex;
    for (int32_t i = 0; i < maxBody; ++i)
    {
        if (world->bodyPool.alive[i] != 0 && world->types[i] == (uint8_t)m3_dynamicBody &&
            world->awake[i] != 0)
        {
            awake += 1;
        }
    }
    counters.awakeCount = awake;
    counters.islandCount = world->lastIslandCount;
    counters.colorCount = world->lastColorCount;
    counters.treeHeight =
        world->tree.root != M3_TREE_NULL ? world->tree.nodes[world->tree.root].height : 0;
    counters.scratchPeak = world->lastScratchPeak;
    counters.scratchCapacity = world->scratch.capacity;
    counters.snapshotBytes = m3World_SnapshotSize(worldId);
    m3DebugAllocCounts(&counters.allocCalls, &counters.freeCalls);
    return counters;
}

m3Profile m3World_GetProfile(m3WorldId worldId)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL)
    {
        m3Profile zero;
        memset(&zero, 0, sizeof(zero));
        return zero;
    }
    return world->profile;
}

void m3World_EnableSleeping(m3WorldId worldId, bool flag)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL)
    {
        return;
    }
    if (world->journalActive != 0)
    {
        int32_t on = flag ? 1 : 0;
        m3JournalRecord(world, m3_opEnableSleeping, &on, (int32_t)sizeof(on));
    }
    m3EnableSleepingInternal(world, flag ? 1 : 0);
}

bool m3World_IsSleepingEnabled(m3WorldId worldId)
{
    m3World* world = m3WorldFromId(worldId);
    return world != NULL && world->sleepEnabled != 0;
}

void m3World_EnableContinuous(m3WorldId worldId, bool flag)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL)
    {
        return;
    }
    if (world->journalActive != 0)
    {
        int32_t on = flag ? 1 : 0;
        m3JournalRecord(world, m3_opEnableContinuous, &on, (int32_t)sizeof(on));
    }
    m3EnableContinuousInternal(world, flag ? 1 : 0);
}

bool m3World_IsContinuousEnabled(m3WorldId worldId)
{
    m3World* world = m3WorldFromId(worldId);
    return world != NULL && world->continuousEnabled != 0;
}

// --- Events (8-5) -----------------------------------------------------------

void m3SetHitEventThresholdInternal(m3World* world, float value)
{
    world->hitEventThreshold = value;
}

void m3World_SetHitEventThreshold(m3WorldId worldId, float value)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || !m3FiniteF(value) || value < 0.0f)
    {
        return;
    }
    if (world->journalActive != 0)
    {
        m3JournalRecord(world, m3_opSetHitEventThreshold, &value, (int32_t)sizeof(value));
    }
    m3SetHitEventThresholdInternal(world, value);
}

const m3HitEvent* m3World_HitEvents(m3WorldId worldId, int32_t* count)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL)
    {
        *count = 0;
        return NULL;
    }
    *count = world->hitEventCount;
    return world->hitEvents;
}

int32_t m3World_HitEventsDropped(m3WorldId worldId)
{
    m3World* world = m3WorldFromId(worldId);
    return world != NULL ? world->hitEventsDropped : 0;
}

const m3BodyMoveEvent* m3World_BodyMoveEvents(m3WorldId worldId, int32_t* count)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL)
    {
        *count = 0;
        return NULL;
    }
    *count = world->moveEventCount;
    return world->moveEvents;
}

const m3JointBreakEvent* m3World_JointBreakEvents(m3WorldId worldId, int32_t* count)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL)
    {
        *count = 0;
        return NULL;
    }
    *count = world->jointBreakEventCount;
    return world->jointBreakEvents;
}

void m3SetWindInternal(m3World* world, m3Vec3 dir, float speed, float gustHertz, float gustScale)
{
    world->windDir = dir;
    world->windSpeed = speed;
    world->windGustHertz = gustHertz;
    world->windGustScale = gustScale;
    // The phase deliberately SURVIVES a retune: the wave continues.
}

void m3World_SetWind(m3WorldId worldId, m3Vec3 direction, float speed, float gustHertz,
                     float gustScale)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || !m3FiniteV3(direction) || !m3FiniteF(speed) || speed < 0.0f ||
        !m3FiniteF(gustHertz) || gustHertz < 0.0f || !m3FiniteF(gustScale) || gustScale < 0.0f)
    {
        return;
    }
    if (speed > 0.0f)
    {
        m3real len2 = m3Dot3(direction, direction);
        if (len2 < 0.99f || len2 > 1.01f)
        {
            return; // a blowing wind demands a near-unit direction
        }
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m3Vec3 dir;
            float speed;
            float gustHertz;
            float gustScale;
        } record;
        memset(&record, 0, sizeof(record));
        record.dir = direction;
        record.speed = speed;
        record.gustHertz = gustHertz;
        record.gustScale = gustScale;
        m3JournalRecord(world, m3_opSetWind, &record, (int32_t)sizeof(record));
    }
    m3SetWindInternal(world, direction, speed, gustHertz, gustScale);
}

void m3AppendJointBreakEvent(m3World* world, m3JointId joint)
{
    // Capacity is jointCapacity: at most every joint breaks once.
    world->jointBreakEvents[world->jointBreakEventCount].joint = joint;
    world->jointBreakEventCount += 1;
}

void m3World_SetPreSolveCallback(m3WorldId worldId, m3PreSolveFn* fn, void* context)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL)
    {
        return;
    }
    world->preSolveFn = fn;
    world->preSolveContext = context;
}

// --- Journal ---------------------------------------------------------------

void m3RebuildBroadphaseInternal(m3World* world)
{
    // Collect the live proxies in shape-slot order (the canonical
    // list), carrying their CURRENT fat bounds: fatness is state,
    // and preserving it keeps every downstream pair decision
    // exactly where it was.
    int32_t maxShape = world->shapePool.maxIndex;
    int32_t count = 0;
    for (int32_t s = 0; s < maxShape; ++s)
    {
        if (world->shapePool.alive[s] != 0 && world->proxyIds[s] >= 0)
        {
            count += 1;
        }
    }
    if (count == 0)
    {
        return;
    }
    double (*los)[3] = (double (*)[3])m3AllocZeroed(count * 3 * (int32_t)sizeof(double));
    double (*his)[3] = (double (*)[3])m3AllocZeroed(count * 3 * (int32_t)sizeof(double));
    int32_t* uds = (int32_t*)m3AllocZeroed(count * (int32_t)sizeof(int32_t));
    int32_t* outNodes = (int32_t*)m3AllocZeroed(count * (int32_t)sizeof(int32_t));
    if (los == NULL || his == NULL || uds == NULL || outNodes == NULL)
    {
        m3Free(los);
        m3Free(his);
        m3Free(uds);
        m3Free(outNodes);
        return; // no memory: the old tree stays, correct either way
    }
    int32_t n = 0;
    for (int32_t s = 0; s < maxShape; ++s)
    {
        if (world->shapePool.alive[s] == 0 || world->proxyIds[s] < 0)
        {
            continue;
        }
        const m3TreeNode* leaf = &world->tree.nodes[world->proxyIds[s]];
        for (int32_t k = 0; k < 3; ++k)
        {
            los[n][k] = leaf->lo[k];
            his[n][k] = leaf->hi[k];
        }
        uds[n] = s;
        n += 1;
    }
    if (m3TreeRebuild(&world->tree, los, his, uds, n, outNodes))
    {
        for (int32_t i = 0; i < n; ++i)
        {
            world->proxyIds[uds[i]] = outNodes[i];
        }
    }
    m3Free(los);
    m3Free(his);
    m3Free(uds);
    m3Free(outNodes);
}

void m3World_RebuildBroadphase(m3WorldId worldId)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL)
    {
        return;
    }
    if (world->journalActive != 0)
    {
        int32_t zero = 0;
        m3JournalRecord(world, m3_opRebuildBroadphase, &zero, 4);
    }
    m3RebuildBroadphaseInternal(world);
}

#define M3_WATER_COOKIE ((int32_t)(M3_COOKIE ^ ((int32_t)sizeof(m3WaterVolumeDef) << 8) ^ 7))

m3WaterVolumeDef m3DefaultWaterVolumeDef(void)
{
    m3WaterVolumeDef def;
    memset(&def, 0, sizeof(def));
    def.hi = (m3Pos3){1.0, 1.0, 1.0};
    def.density = 1000.0f;
    def.linearDrag = 2.0f;
    def.angularDrag = 1.0f;
    def.internalValue = M3_WATER_COOKIE;
    return def;
}

static bool WakeInBoxFn(int32_t shape, void* context)
{
    m3World* world = (m3World*)context;
    int32_t body = world->shapeBody[shape];
    if (world->types[body] == (uint8_t)m3_dynamicBody)
    {
        m3SetAwakeInternal(world, body, 1);
    }
    return true;
}

static void WakeAroundWater(m3World* world, int32_t slot)
{
    // The tide moves things: sleepers touching the volume wake on
    // create AND destroy (without water under it, a sleeper falls).
    double lo[3] = {world->waterLo[slot].x, world->waterLo[slot].y, world->waterLo[slot].z};
    double hi[3] = {world->waterHi[slot].x, world->waterHi[slot].y, world->waterHi[slot].z};
    m3TreeQuery(&world->tree, lo, hi, WakeInBoxFn, world);
}

int32_t m3CreateWaterVolumeInternal(m3World* world, const m3WaterVolumeDef* def)
{
    // The full wall (18-1), here because replay hands this function
    // raw journal bytes.
    if (!m3FinitePos3(def->lo) || !m3FinitePos3(def->hi) || !(def->hi.x > def->lo.x) ||
        !(def->hi.y > def->lo.y) || !(def->hi.z > def->lo.z) || !m3FiniteF(def->density) ||
        !(def->density > 0.0f) || !m3FiniteF(def->linearDrag) || def->linearDrag < 0.0f ||
        !m3FiniteF(def->angularDrag) || def->angularDrag < 0.0f || !m3FiniteV3(def->flow))
    {
        return -1;
    }
    int32_t slot = m3IdPoolAlloc(&world->waterPool);
    if (slot < 0)
    {
        return -1; // all 8 slots taken: loud at the caller
    }
    world->waterLo[slot] = def->lo;
    world->waterHi[slot] = def->hi;
    world->waterDensity[slot] = def->density;
    world->waterLinDrag[slot] = def->linearDrag;
    world->waterAngDrag[slot] = def->angularDrag;
    world->waterFlow[slot] = def->flow;
    WakeAroundWater(world, slot);
    return slot;
}

void m3DestroyWaterVolumeInternal(m3World* world, int32_t slot)
{
    WakeAroundWater(world, slot);
    world->waterLo[slot] = (m3Pos3){0.0, 0.0, 0.0};
    world->waterHi[slot] = (m3Pos3){0.0, 0.0, 0.0};
    world->waterDensity[slot] = 0.0f;
    world->waterLinDrag[slot] = 0.0f;
    world->waterAngDrag[slot] = 0.0f;
    world->waterFlow[slot] = (m3Vec3){0.0f, 0.0f, 0.0f};
    m3IdPoolFree(&world->waterPool, slot);
}

m3WaterVolumeId m3CreateWaterVolume(m3WorldId worldId, const m3WaterVolumeDef* def)
{
    m3WaterVolumeId null = {0, 0, 0};
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || def == NULL || def->internalValue != M3_WATER_COOKIE)
    {
        return null;
    }
    int32_t slot = m3CreateWaterVolumeInternal(world, def);
    if (slot < 0)
    {
        return null;
    }
    m3WaterVolumeId id = {slot + 1, world->worldIndex0, world->waterPool.generations[slot]};
    if (world->journalActive != 0)
    {
        struct
        {
            m3WaterVolumeDef def;
            m3WaterVolumeId expected;
        } record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.expected = id;
        m3JournalRecord(world, m3_opCreateWaterVolume, &record, (int32_t)sizeof(record));
    }
    return id;
}

static int32_t WaterSlot(const m3World* world, m3WaterVolumeId id)
{
    int32_t index = id.index1 - 1;
    if (world == NULL || id.world0 != world->worldIndex0 ||
        !m3IdPoolValid(&world->waterPool, index, id.generation))
    {
        return -1;
    }
    return index;
}

bool m3WaterVolume_IsValid(m3WaterVolumeId id)
{
    m3World* world = m3WorldFromIndex0(id.world0);
    return world != NULL && WaterSlot(world, id) >= 0;
}

void m3DestroyWaterVolume(m3WaterVolumeId id)
{
    m3World* world = m3WorldFromIndex0(id.world0);
    int32_t slot = world != NULL ? WaterSlot(world, id) : -1;
    if (slot < 0)
    {
        return;
    }
    if (world->journalActive != 0)
    {
        m3JournalRecord(world, m3_opDestroyWaterVolume, &id, (int32_t)sizeof(id));
    }
    m3DestroyWaterVolumeInternal(world, slot);
}

void m3JournalRecord(m3World* world, int32_t op, const void* payload, int32_t bytes)
{
    if (world->journalActive == 0)
    {
        return;
    }
    int32_t need = 8 + bytes;
    if (world->journalCursor + need > world->journalCapacity)
    {
        // Loud overflow: latch, stop recording, End reports -1.
        world->journalOverflow = 1;
        world->journalActive = 0;
        return;
    }
    uint8_t* out = world->journalBuffer + world->journalCursor;
    memcpy(out, &op, 4);
    memcpy(out + 4, &bytes, 4);
    memcpy(out + 8, payload, (size_t)bytes);
    world->journalCursor += need;
}

bool m3World_JournalBegin(m3WorldId worldId, void* buffer, int32_t capacity)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || buffer == NULL || capacity < 8 || world->journalActive != 0)
    {
        return false; // contract, not invariant
    }
    world->journalBuffer = (uint8_t*)buffer;
    world->journalCapacity = capacity;
    world->journalCursor = 0;
    world->journalActive = 1;
    world->journalOverflow = 0;
    return true;
}

int32_t m3World_JournalEnd(m3WorldId worldId)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL)
    {
        return -1; // contract, not invariant
    }
    int32_t bytes = world->journalOverflow != 0 ? -1 : world->journalCursor;
    world->journalBuffer = NULL;
    world->journalCapacity = 0;
    world->journalCursor = 0;
    world->journalActive = 0;
    world->journalOverflow = 0;
    return bytes;
}

const m3ContactEvent* m3World_SensorBeginEvents(m3WorldId worldId, int32_t* count)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || count == NULL)
    {
        if (count != NULL)
        {
            *count = 0;
        }
        return NULL;
    }
    *count = world->sensorBeginEventCount;
    return world->sensorBeginEvents;
}

const m3ContactEvent* m3World_SensorEndEvents(m3WorldId worldId, int32_t* count)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || count == NULL)
    {
        if (count != NULL)
        {
            *count = 0;
        }
        return NULL;
    }
    *count = world->sensorEndEventCount;
    return world->sensorEndEvents;
}

const m3FragmentEvent* m3World_FragmentEvents(m3WorldId worldId, int32_t* count)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL)
    {
        if (count != NULL)
        {
            *count = 0;
        }
        return NULL;
    }
    if (count != NULL)
    {
        *count = world->fragmentEventCount;
    }
    return world->fragmentEvents;
}

const uint16_t* m3World_FragmentRecipe(m3WorldId worldId, int32_t* count)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL)
    {
        if (count != NULL)
        {
            *count = 0;
        }
        return NULL;
    }
    if (count != NULL)
    {
        *count = world->fragmentRecipeCount;
    }
    return world->fragmentRecipe;
}

int32_t m3World_FragmentEventsDropped(m3WorldId worldId)
{
    m3World* world = m3WorldFromId(worldId);
    return world != NULL ? world->fragmentDropped : 0;
}

const m3ContactEvent* m3World_ContactBeginEvents(m3WorldId worldId, int32_t* count)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || count == NULL)
    {
        if (count != NULL)
        {
            *count = 0;
        }
        return NULL;
    }
    *count = world->beginEventCount;
    return world->beginEvents;
}

const m3ContactEvent* m3World_ContactEndEvents(m3WorldId worldId, int32_t* count)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || count == NULL)
    {
        if (count != NULL)
        {
            *count = 0;
        }
        return NULL;
    }
    *count = world->endEventCount;
    return world->endEvents;
}

// The replay worker: applies ops in order and reports the first
// refusal. Partial application is possible HERE; the public wrapper
// below makes the whole call atomic.
static bool JournalReplayApply(m3World* world, const void* data, int32_t size)
{
    const uint8_t* stream = (const uint8_t*)data;
    int32_t cursor = 0;
    while (cursor < size)
    {
        if (cursor + 8 > size)
        {
            return false; // truncated header: reject loudly
        }
        int32_t op;
        int32_t bytes;
        memcpy(&op, stream + cursor, 4);
        memcpy(&bytes, stream + cursor + 4, 4);
        cursor += 8;
        if (bytes < 0 || cursor + bytes > size)
        {
            return false; // truncated payload
        }
        const uint8_t* payload = stream + cursor;
        cursor += bytes;

        switch (op)
        {
        case m3_opCreateBody:
        {
            struct
            {
                m3BodyDef def;
                m3BodyId expected;
            } record;
            if (bytes != (int32_t)sizeof(record))
            {
                return false;
            }
            memcpy(&record, payload, sizeof(record));
            NormalizeBoolByte(&record.def, offsetof(m3BodyDef, isBullet));
            int32_t index = m3CreateBodyInternal(world, &record.def);
            // Id determinism: the replayed world must mint the exact
            // id the original minted, or the replay is invalid.
            if (index < 0 || index + 1 != record.expected.index1 ||
                world->bodyPool.generations[index] != record.expected.generation)
            {
                return false;
            }
            break;
        }
        case m3_opDestroyBody:
        {
            m3BodyId id;
            if (bytes != (int32_t)sizeof(id))
            {
                return false;
            }
            memcpy(&id, payload, sizeof(id));
            // Recorded ids carry the ORIGINAL world's slot; replay
            // retargets them to this world (the Maul2D rule).
            id.world0 = world->worldIndex0;
            int32_t index = m3BodySlot(world, id);
            if (index < 0)
            {
                return false;
            }
            m3DestroyBodyInternal(world, index);
            break;
        }
        case m3_opSetLinearVelocity:
        {
            struct
            {
                m3BodyId id;
                m3Vec3 v;
            } record;
            if (bytes != (int32_t)sizeof(record))
            {
                return false;
            }
            memcpy(&record, payload, sizeof(record));
            record.id.world0 = world->worldIndex0;
            int32_t index = m3BodySlot(world, record.id);
            if (index < 0 || !m3FiniteV3(record.v))
            {
                return false; // hostile bytes fail loudly (16-7)
            }
            m3SetLinearVelocityInternal(world, index, record.v);
            break;
        }
        case m3_opSetAngularVelocity:
        {
            struct
            {
                m3BodyId id;
                m3Vec3 v;
            } record;
            if (bytes != (int32_t)sizeof(record))
            {
                return false;
            }
            memcpy(&record, payload, sizeof(record));
            record.id.world0 = world->worldIndex0;
            int32_t index = m3BodySlot(world, record.id);
            if (index < 0 || !m3FiniteV3(record.v))
            {
                return false; // hostile bytes fail loudly (16-7)
            }
            m3SetAngularVelocityInternal(world, index, record.v);
            break;
        }
        case m3_opStep:
        {
            struct
            {
                float dt;
                int32_t substeps;
            } record;
            if (bytes != (int32_t)sizeof(record))
            {
                return false;
            }
            memcpy(&record, payload, sizeof(record));
            if (!(record.dt > 0.0f) || record.substeps < 1 || record.substeps > M3_MAX_SUBSTEPS)
            {
                // The upper bound is the 13-4 red-team scar: one
                // flipped bit turned substeps 4 into a billion and a
                // two-second replay into hours. Hostile tapes refuse
                // in proportion to their crime.
                return false;
            }
            m3StepInternal(world, record.dt, record.substeps);
            break;
        }
        case m3_opCreateShape:
        {
            m3CreateShapeOp record;
            if (bytes != (int32_t)sizeof(record))
            {
                return false;
            }
            memcpy(&record, payload, sizeof(record));
            record.body.world0 = world->worldIndex0;
            int32_t bodyIndex = m3BodySlot(world, record.body);
            if (bodyIndex < 0)
            {
                return false;
            }
            NormalizeShapeDefBools(&record.def);
            int32_t index = m3CreateShapeInternal(world, bodyIndex, record.type, &record.geom,
                                                  &record.def, NULL, NULL, NULL, NULL);
            if (index < 0 || index + 1 != record.expected.index1 ||
                world->shapePool.generations[index] != record.expected.generation)
            {
                return false; // id determinism holds for shapes too
            }
            break;
        }
        case m3_opCreateMeshShape:
        {
            m3CreateMeshShapeOp record;
            if (bytes < (int32_t)sizeof(record))
            {
                return false;
            }
            memcpy(&record, payload, sizeof(record));
            if (record.vertexCount < 3 || record.vertexCount > M3_MESH_MAX_VERTS ||
                record.triangleCount < 1 || record.triangleCount > M3_MESH_MAX_TRIS)
            {
                return false;
            }
            int32_t vertexBytes = record.vertexCount * (int32_t)sizeof(m3Vec3);
            int32_t indexBytes = 3 * record.triangleCount * (int32_t)sizeof(uint16_t);
            if (bytes != (int32_t)sizeof(record) + vertexBytes + indexBytes)
            {
                return false;
            }
            record.body.world0 = world->worldIndex0;
            int32_t bodyIndex = m3BodySlot(world, record.body);
            if (bodyIndex < 0)
            {
                return false;
            }
            m3MeshData mesh;
            memset(&mesh, 0, sizeof(mesh));
            mesh.vertexCount = record.vertexCount;
            mesh.triangleCount = record.triangleCount;
            if (!m3MeshDataAlloc(&mesh))
            {
                return false;
            }
            memcpy(mesh.vertices, (const uint8_t*)payload + sizeof(record), (size_t)vertexBytes);
            memcpy(mesh.indices, (const uint8_t*)payload + sizeof(record) + vertexBytes,
                   (size_t)indexBytes);
            m3ShapeGeom geom;
            memset(&geom, 0, sizeof(geom));
            NormalizeShapeDefBools(&record.def);
            // On success the slot owns the arrays; an id mismatch
            // leaves them with the slot too, and the atomic-replay
            // restore reclaims them through the alloc gate.
            int32_t index = m3CreateShapeInternal(world, bodyIndex, (uint8_t)m3_meshShape, &geom,
                                                  &record.def, NULL, &mesh, NULL, NULL);
            if (index < 0)
            {
                m3MeshDataFree(&mesh);
                return false;
            }
            if (index + 1 != record.expected.index1 ||
                world->shapePool.generations[index] != record.expected.generation)
            {
                return false; // id determinism holds for mesh shapes too
            }
            break;
        }
        case m3_opCreateJoint:
        {
            m3CreateJointOp record;
            if (bytes != (int32_t)sizeof(record))
            {
                return false;
            }
            memcpy(&record, payload, sizeof(record));
            NormalizeBoolByte(&record.def, offsetof(m3JointDef, enableLimit));
            NormalizeBoolByte(&record.def, offsetof(m3JointDef, enableMotor));
            NormalizeBoolByte(&record.def, offsetof(m3JointDef, enableCone));
            NormalizeBoolByte(&record.def, offsetof(m3JointDef, collideConnected));
            record.def.bodyA.world0 = world->worldIndex0;
            record.def.bodyB.world0 = world->worldIndex0;
            int32_t bodyA = m3BodySlot(world, record.def.bodyA);
            int32_t bodyB = m3BodySlot(world, record.def.bodyB);
            if (bodyA < 0 || bodyB < 0)
            {
                return false;
            }
            int32_t index = m3CreateJointInternal(world, &record.def, bodyA, bodyB);
            if (index < 0 || index + 1 != record.expected.index1 ||
                world->jointPool.generations[index] != record.expected.generation)
            {
                return false; // id determinism holds for joints too
            }
            break;
        }
        case m3_opDestroyJoint:
        {
            m3JointId id;
            if (bytes != (int32_t)sizeof(id))
            {
                return false;
            }
            memcpy(&id, payload, sizeof(id));
            id.world0 = world->worldIndex0;
            int32_t index = m3JointSlot(world, id);
            if (index < 0)
            {
                return false;
            }
            m3DestroyJointInternal(world, index);
            break;
        }
        case m3_opCreateHullShape:
        {
            m3CreateHullShapeOp record;
            if (bytes != (int32_t)sizeof(record))
            {
                return false;
            }
            memcpy(&record, payload, sizeof(record));
            record.body.world0 = world->worldIndex0;
            int32_t bodyIndex = m3BodySlot(world, record.body);
            if (bodyIndex < 0)
            {
                return false;
            }
            m3HullData rebuilt;
            if (!m3ComputeHull(record.points, record.count, &rebuilt))
            {
                return false; // the recipe must rebuild
            }
            m3ShapeGeom geom;
            memset(&geom, 0, sizeof(geom));
            NormalizeShapeDefBools(&record.def);
            int32_t index = m3CreateShapeInternal(world, bodyIndex, (uint8_t)m3_hullShape, &geom,
                                                  &record.def, &rebuilt, NULL, NULL, NULL);
            if (index < 0 || index + 1 != record.expected.index1 ||
                world->shapePool.generations[index] != record.expected.generation)
            {
                return false; // id determinism holds for hull shapes too
            }
            break;
        }
        case m3_opCreateVoxelChunkShape:
        {
            struct
            {
                m3ShapeDef def;
                m3BodyId body;
                m3ShapeId expected;
                m3real cellSize;
            } record;
            int32_t occBytes = (int32_t)(M3_VOXEL_COUNT / 8);
            int32_t payBytes = (int32_t)(M3_VOXEL_COUNT * sizeof(uint16_t));
            int32_t fillBytes = (int32_t)M3_VOXEL_COUNT;
            if (bytes != (int32_t)sizeof(record) + occBytes + payBytes + fillBytes)
            {
                return false;
            }
            memcpy(&record, payload, sizeof(record));
            record.body.world0 = world->worldIndex0;
            int32_t bodyIndex = m3BodySlot(world, record.body);
            if (bodyIndex < 0 || !(record.cellSize > 0.0f))
            {
                return false;
            }
            m3VoxelChunkData* chunk =
                (m3VoxelChunkData*)m3AllocZeroed((int32_t)sizeof(m3VoxelChunkData));
            if (chunk == NULL)
            {
                return false;
            }
            chunk->cellSize = record.cellSize;
            memcpy(chunk->occupancy, (const uint8_t*)payload + sizeof(record), (size_t)occBytes);
            memcpy(chunk->payload, (const uint8_t*)payload + sizeof(record) + occBytes,
                   (size_t)payBytes);
            memcpy(chunk->fill, (const uint8_t*)payload + sizeof(record) + occBytes + payBytes,
                   (size_t)fillBytes);
            int32_t filled = 0;
            for (int32_t v = 0; v < M3_VOXEL_COUNT; ++v)
            {
                filled += (chunk->occupancy[v >> 3] >> (v & 7)) & 1;
            }
            chunk->filledCount = filled;
            m3ShapeGeom geom;
            memset(&geom, 0, sizeof(geom));
            geom.s = record.cellSize;
            NormalizeShapeDefBools(&record.def);
            int32_t index = m3CreateShapeInternal(world, bodyIndex, (uint8_t)m3_voxelShape, &geom,
                                                  &record.def, NULL, NULL, chunk, NULL);
            m3Free(chunk);
            if (index < 0 || index + 1 != record.expected.index1 ||
                world->shapePool.generations[index] != record.expected.generation)
            {
                return false; // id determinism holds for voxels too
            }
            break;
        }
        case m3_opVoxelSet:
        {
            struct
            {
                m3ShapeId id;
                int32_t x, y, z;
                uint16_t payload;
                uint16_t pad;
            } record;
            if (bytes != (int32_t)sizeof(record))
            {
                return false;
            }
            memcpy(&record, payload, sizeof(record));
            record.id.world0 = world->worldIndex0;
            int32_t shape = m3ShapeSlot(world, record.id);
            if (shape < 0 || world->shapeType[shape] != (uint8_t)m3_voxelShape)
            {
                return false;
            }
            m3VoxelSetInternal(world, shape, record.x, record.y, record.z, record.payload);
            break;
        }
        case m3_opVoxelClear:
        {
            struct
            {
                m3ShapeId id;
                int32_t x, y, z;
            } record;
            if (bytes != (int32_t)sizeof(record))
            {
                return false;
            }
            memcpy(&record, payload, sizeof(record));
            record.id.world0 = world->worldIndex0;
            int32_t shape = m3ShapeSlot(world, record.id);
            if (shape < 0 || world->shapeType[shape] != (uint8_t)m3_voxelShape)
            {
                return false;
            }
            m3VoxelClearInternal(world, shape, record.x, record.y, record.z);
            break;
        }
        case m3_opVoxelSetFill:
        {
            struct
            {
                m3ShapeId id;
                int32_t x, y, z;
                uint8_t fill;
                uint8_t pad[3];
            } record;
            if (bytes != (int32_t)sizeof(record))
            {
                return false;
            }
            memcpy(&record, payload, sizeof(record));
            record.id.world0 = world->worldIndex0;
            int32_t shape = m3ShapeSlot(world, record.id);
            if (shape < 0 || world->shapeType[shape] != (uint8_t)m3_voxelShape || record.fill == 0)
            {
                return false;
            }
            m3VoxelSetFillInternal(world, shape, record.x, record.y, record.z, record.fill);
            break;
        }
        case m3_opVoxelClearBox:
        {
            struct
            {
                m3ShapeId id;
                int32_t lo[3];
                int32_t hi[3];
            } record;
            if (bytes != (int32_t)sizeof(record))
            {
                return false;
            }
            memcpy(&record, payload, sizeof(record));
            record.id.world0 = world->worldIndex0;
            int32_t shape = m3ShapeSlot(world, record.id);
            if (shape < 0 || world->shapeType[shape] != (uint8_t)m3_voxelShape)
            {
                return false;
            }
            for (int32_t k = 0; k < 3; ++k)
            {
                if (record.lo[k] < 0 || record.hi[k] >= M3_VOXEL_DIM || record.lo[k] > record.hi[k])
                {
                    return false;
                }
            }
            m3VoxelClearBoxInternal(world, shape, record.lo, record.hi);
            break;
        }
        case m3_opCreateCharacter:
        {
            struct
            {
                m3CharacterDef def;
                m3CharacterId expected;
            } record;
            if (bytes != (int32_t)sizeof(record))
            {
                return false;
            }
            memcpy(&record, payload, sizeof(record));
            int32_t slot = m3CreateCharacterInternal(world, &record.def);
            if (slot < 0 || slot + 1 != record.expected.index1 ||
                world->charPool.generations[slot] != record.expected.generation)
            {
                return false; // id determinism holds for characters too
            }
            break;
        }
        case m3_opDestroyCharacter:
        {
            m3CharacterId id;
            if (bytes != (int32_t)sizeof(id))
            {
                return false;
            }
            memcpy(&id, payload, sizeof(id));
            id.world0 = world->worldIndex0;
            int32_t slot = m3CharacterSlot(world, id);
            if (slot < 0)
            {
                return false;
            }
            m3DestroyCharacterInternal(world, slot);
            break;
        }
        case m3_opCharacterMove:
        {
            struct
            {
                m3CharacterId id;
                m3Vec3 translation;
            } record;
            if (bytes != (int32_t)sizeof(record))
            {
                return false;
            }
            memcpy(&record, payload, sizeof(record));
            record.id.world0 = world->worldIndex0;
            int32_t slot = m3CharacterSlot(world, record.id);
            if (slot < 0 || !m3FiniteV3(record.translation))
            {
                return false; // hostile bytes fail loudly (16-7)
            }
            m3CharacterMoveInternal(world, slot, record.translation);
            break;
        }
        case m3_opCharacterStance:
        {
            struct
            {
                m3CharacterId id;
                m3real halfHeight;
                m3real radius;
            } record;
            if (bytes != (int32_t)sizeof(record))
            {
                return false;
            }
            memcpy(&record, payload, sizeof(record));
            record.id.world0 = world->worldIndex0;
            int32_t slot = m3CharacterSlot(world, record.id);
            if (slot < 0 ||
                !m3CharacterStanceInternal(world, slot, record.halfHeight, record.radius))
            {
                // A journaled stance was APPLIED at record time; a
                // replay that cannot re-apply it (fuzzed bytes, a
                // diverged world) fails loudly.
                return false;
            }
            break;
        }
        case m3_opCreateVehicle:
        {
            struct
            {
                m3VehicleDef def;
                m3VehicleId expected;
            } record;
            if (bytes != (int32_t)sizeof(record))
            {
                return false;
            }
            memcpy(&record, payload, sizeof(record));
            for (int32_t w = 0; w < M3_VEHICLE_MAX_WHEELS; ++w)
            {
                size_t wheelBase = offsetof(m3VehicleDef, wheels) + (size_t)w * sizeof(m3WheelDef);
                NormalizeBoolByte(&record.def, wheelBase + offsetof(m3WheelDef, steerable));
                NormalizeBoolByte(&record.def, wheelBase + offsetof(m3WheelDef, driven));
            }
            record.def.chassis.world0 = world->worldIndex0;
            int32_t slot = m3CreateVehicleInternal(world, &record.def);
            if (slot < 0 || slot + 1 != record.expected.index1 ||
                world->vehPool.generations[slot] != record.expected.generation)
            {
                return false; // id determinism holds for vehicles too
            }
            break;
        }
        case m3_opDestroyVehicle:
        {
            m3VehicleId id;
            if (bytes != (int32_t)sizeof(id))
            {
                return false;
            }
            memcpy(&id, payload, sizeof(id));
            id.world0 = world->worldIndex0;
            int32_t slot = m3VehicleSlot(world, id);
            if (slot < 0)
            {
                return false;
            }
            m3DestroyVehicleInternal(world, slot);
            break;
        }
        case m3_opVehicleTankCommands:
        {
            struct
            {
                m3VehicleId id;
                m3real left;
                m3real right;
                m3real brake;
            } record;
            if (bytes != (int32_t)sizeof(record))
            {
                return false;
            }
            memcpy(&record, payload, sizeof(record));
            record.id.world0 = world->worldIndex0;
            int32_t slot = m3VehicleSlot(world, record.id);
            if (slot < 0 || !m3FiniteF(record.left) || !m3FiniteF(record.right) ||
                !m3FiniteF(record.brake))
            {
                return false; // hostile bytes fail loudly
            }
            m3VehicleTankCommandsInternal(world, slot, record.left, record.right, record.brake);
            break;
        }
        case m3_opVehicleCommands:
        {
            struct
            {
                m3VehicleId id;
                m3real throttle;
                m3real steer;
                m3real brake;
            } record;
            if (bytes != (int32_t)sizeof(record))
            {
                return false;
            }
            memcpy(&record, payload, sizeof(record));
            record.id.world0 = world->worldIndex0;
            int32_t slot = m3VehicleSlot(world, record.id);
            if (slot < 0 || !m3FiniteF(record.throttle) || !m3FiniteF(record.steer) ||
                !m3FiniteF(record.brake))
            {
                return false; // hostile bytes fail loudly (16-7)
            }
            m3VehicleCommandsInternal(world, slot, record.throttle, record.steer, record.brake);
            break;
        }
        case m3_opVehicleDrivetrain:
        {
            struct
            {
                m3VehicleId id;
                m3DrivetrainDef def;
            } record;
            if (bytes != (int32_t)sizeof(record))
            {
                return false;
            }
            memcpy(&record, payload, sizeof(record));
            NormalizeBoolByte(&record.def, offsetof(m3DrivetrainDef, autoShift));
            record.id.world0 = world->worldIndex0;
            int32_t slot = m3VehicleSlot(world, record.id);
            if (slot < 0 || !m3VehicleDrivetrainInternal(world, slot, &record.def))
            {
                return false; // journaled defs are UNTRUSTED bytes
            }
            break;
        }
        case m3_opVehicleGear:
        {
            struct
            {
                m3VehicleId id;
                int32_t gear;
            } record;
            if (bytes != (int32_t)sizeof(record))
            {
                return false;
            }
            memcpy(&record, payload, sizeof(record));
            record.id.world0 = world->worldIndex0;
            int32_t slot = m3VehicleSlot(world, record.id);
            if (slot < 0 || !m3VehicleGearInternal(world, slot, record.gear))
            {
                return false;
            }
            break;
        }
        case m3_opCreateSoftBody:
        {
            struct
            {
                m3SoftBodyDef def;
                m3SoftBodyId expected;
            } record;
            if (bytes != (int32_t)sizeof(record))
            {
                return false;
            }
            memcpy(&record, payload, sizeof(record));
            int32_t slot = m3CreateSoftBodyInternal(world, &record.def);
            if (slot < 0 || slot + 1 != record.expected.index1 ||
                world->softPool.generations[slot] != record.expected.generation)
            {
                return false; // id determinism holds for soft bodies too
            }
            break;
        }
        case m3_opDestroySoftBody:
        {
            m3SoftBodyId id;
            if (bytes != (int32_t)sizeof(id))
            {
                return false;
            }
            memcpy(&id, payload, sizeof(id));
            id.world0 = world->worldIndex0;
            int32_t slot = m3SoftBodySlot(world, id);
            if (slot < 0)
            {
                return false;
            }
            m3DestroySoftBodyInternal(world, slot);
            break;
        }
        case m3_opSoftBodyPin:
        {
            struct
            {
                m3SoftBodyId id;
                int32_t particle;
            } record;
            if (bytes != (int32_t)sizeof(record))
            {
                return false;
            }
            memcpy(&record, payload, sizeof(record));
            record.id.world0 = world->worldIndex0;
            int32_t slot = m3SoftBodySlot(world, record.id);
            if (slot < 0 || record.particle < 0 ||
                record.particle >= world->softParticleCount[slot])
            {
                return false;
            }
            m3SoftBodyPinInternal(world, slot, record.particle);
            break;
        }
        case m3_opSoftBodyAnchor:
        {
            struct
            {
                m3SoftBodyId id;
                int32_t particle;
                m3BodyId body;
            } record;
            if (bytes != (int32_t)sizeof(record))
            {
                return false;
            }
            memcpy(&record, payload, sizeof(record));
            record.id.world0 = world->worldIndex0;
            record.body.world0 = world->worldIndex0;
            int32_t slot = m3SoftBodySlot(world, record.id);
            int32_t body = m3BodySlot(world, record.body);
            if (slot < 0 || body < 0 || record.particle < 0 ||
                record.particle >= world->softParticleCount[slot] ||
                world->softAnchorCount[slot] >= M3_SOFTBODY_MAX_ANCHORS)
            {
                return false;
            }
            m3SoftBodyAnchorInternal(world, slot, record.particle, body);
            break;
        }
        case m3_opSoftBodyAnchorSoft:
        {
            struct
            {
                m3SoftBodyId idA;
                int32_t particleA;
                m3SoftBodyId idB;
                int32_t particleB;
            } record;
            if (bytes != (int32_t)sizeof(record))
            {
                return false;
            }
            memcpy(&record, payload, sizeof(record));
            record.idA.world0 = world->worldIndex0;
            record.idB.world0 = world->worldIndex0;
            int32_t slotA = m3SoftBodySlot(world, record.idA);
            int32_t slotB = m3SoftBodySlot(world, record.idB);
            if (slotA < 0 || slotB < 0 || slotA == slotB || record.particleA < 0 ||
                record.particleB < 0 || record.particleA >= world->softParticleCount[slotA] ||
                record.particleB >= world->softParticleCount[slotB] ||
                world->softSoftCount[slotA < slotB ? slotA : slotB] >= M3_SOFTBODY_MAX_ANCHORS)
            {
                // A flipped particle index would become an out of
                // bounds solver read: the full public wall (16-7).
                return false;
            }
            m3SoftBodyAnchorSoftInternal(world, slotA, record.particleA, slotB, record.particleB);
            break;
        }
        case m3_opApplyForce:
        case m3_opApplyTorque:
        case m3_opApplyLinearImpulse:
        case m3_opApplyAngularImpulse:
        {
            struct
            {
                m3BodyId id;
                m3Vec3 v;
            } record;
            if (bytes != (int32_t)sizeof(record))
            {
                return false;
            }
            memcpy(&record, payload, sizeof(record));
            record.id.world0 = world->worldIndex0;
            int32_t index = m3BodySlot(world, record.id);
            if (index < 0 || !m3FiniteV3(record.v))
            {
                return false; // hostile bytes fail loudly (16-7)
            }
            if (op == m3_opApplyForce)
            {
                m3ApplyForceInternal(world, index, record.v);
            }
            else if (op == m3_opApplyTorque)
            {
                m3ApplyTorqueInternal(world, index, record.v);
            }
            else if (op == m3_opApplyLinearImpulse)
            {
                m3ApplyLinearImpulseInternal(world, index, record.v);
            }
            else
            {
                m3ApplyAngularImpulseInternal(world, index, record.v);
            }
            break;
        }
        case m3_opApplyForceAtPoint:
        case m3_opApplyImpulseAtPoint:
        {
            struct
            {
                m3BodyId id;
                m3Vec3 v;
                m3Pos3 p;
            } record;
            if (bytes != (int32_t)sizeof(record))
            {
                return false;
            }
            memcpy(&record, payload, sizeof(record));
            record.id.world0 = world->worldIndex0;
            int32_t index = m3BodySlot(world, record.id);
            if (index < 0 || !m3FiniteV3(record.v) || !m3FinitePos3(record.p))
            {
                return false; // hostile bytes fail loudly (16-7)
            }
            if (op == m3_opApplyForceAtPoint)
            {
                m3ApplyForceAtPointInternal(world, index, record.v, record.p);
            }
            else
            {
                m3ApplyImpulseAtPointInternal(world, index, record.v, record.p);
            }
            break;
        }
        case m3_opSetTransform:
        case m3_opSetTargetTransform:
        {
            struct
            {
                m3BodyId id;
                m3Transform pose;
            } record;
            if (bytes != (int32_t)sizeof(record))
            {
                return false;
            }
            memcpy(&record, payload, sizeof(record));
            record.id.world0 = world->worldIndex0;
            int32_t index = m3BodySlot(world, record.id);
            float qq = record.pose.q.x * record.pose.q.x + record.pose.q.y * record.pose.q.y +
                       record.pose.q.z * record.pose.q.z + record.pose.q.w * record.pose.q.w;
            if (index < 0 || !m3FinitePos3(record.pose.p) || !m3FiniteQuat(record.pose.q) ||
                !(qq > 0.98f) || !(qq < 1.02f))
            {
                return false; // hostile bytes fail loudly (16-7)
            }
            if (op == m3_opSetTransform)
            {
                m3SetTransformInternal(world, index, record.pose);
            }
            else
            {
                m3SetTargetTransformInternal(world, index, record.pose);
            }
            break;
        }
        case m3_opSetType:
        case m3_opSetEnabled:
        case m3_opSetAwake:
        {
            struct
            {
                m3BodyId id;
                int32_t value;
            } record;
            if (bytes != (int32_t)sizeof(record))
            {
                return false;
            }
            memcpy(&record, payload, sizeof(record));
            record.id.world0 = world->worldIndex0;
            int32_t index = m3BodySlot(world, record.id);
            if (index < 0)
            {
                return false;
            }
            if (op == m3_opSetType)
            {
                m3SetTypeInternal(world, index, (uint8_t)record.value);
            }
            else if (op == m3_opSetEnabled)
            {
                m3SetEnabledInternal(world, index, record.value);
            }
            else
            {
                m3SetAwakeInternal(world, index, record.value);
            }
            break;
        }
        case m3_opSetMotionLocks:
        {
            struct
            {
                m3BodyId id;
                uint32_t locks;
            } record;
            if (bytes != (int32_t)sizeof(record))
            {
                return false;
            }
            memcpy(&record, payload, sizeof(record));
            record.id.world0 = world->worldIndex0;
            int32_t index = m3BodySlot(world, record.id);
            if (index < 0)
            {
                return false;
            }
            m3SetMotionLocksInternal(world, index, (uint8_t)record.locks);
            break;
        }
        case m3_opSetSleepControls:
        {
            struct
            {
                m3BodyId id;
                float threshold;
                int32_t canSleep;
            } record;
            if (bytes != (int32_t)sizeof(record))
            {
                return false;
            }
            memcpy(&record, payload, sizeof(record));
            record.id.world0 = world->worldIndex0;
            int32_t index = m3BodySlot(world, record.id);
            if (index < 0)
            {
                return false;
            }
            m3SetSleepControlsInternal(world, index, record.threshold, record.canSleep);
            break;
        }
        case m3_opDestroyShape:
        {
            m3ShapeId id;
            if (bytes != (int32_t)sizeof(id))
            {
                return false;
            }
            memcpy(&id, payload, sizeof(id));
            id.world0 = world->worldIndex0;
            int32_t index = m3ShapeSlot(world, id);
            if (index < 0)
            {
                return false;
            }
            int32_t bodyIndex = world->shapeBody[index];
            m3DestroyShapeInternal(world, index);
            m3RecomputeMass(world, bodyIndex);
            if (world->types[bodyIndex] == (uint8_t)m3_dynamicBody)
            {
                m3SetAwakeInternal(world, bodyIndex, 1);
            }
            break;
        }
        case m3_opSetGravity:
        {
            m3Vec3 gravity;
            if (bytes != (int32_t)sizeof(gravity))
            {
                return false;
            }
            memcpy(&gravity, payload, sizeof(gravity));
            if (!m3FiniteV3(gravity))
            {
                return false; // hostile bytes fail loudly (16-7)
            }
            m3SetGravityInternal(world, gravity);
            break;
        }
        case m3_opSetShapeFriction:
        case m3_opSetShapeRestitution:
        case m3_opSetShapeRolling:
        {
            struct
            {
                m3ShapeId id;
                float value;
            } record;
            if (bytes != (int32_t)sizeof(record))
            {
                return false;
            }
            memcpy(&record, payload, sizeof(record));
            record.id.world0 = world->worldIndex0;
            int32_t slot = m3ShapeSlot(world, record.id);
            if (slot < 0 || !m3FiniteF(record.value) || record.value < 0.0f)
            {
                return false; // hostile bytes fail loudly (16-7)
            }
            if (op == m3_opSetShapeFriction)
            {
                m3SetShapeFrictionInternal(world, slot, record.value);
            }
            else if (op == m3_opSetShapeRestitution)
            {
                m3SetShapeRestitutionInternal(world, slot, record.value);
            }
            else
            {
                m3SetShapeRollingInternal(world, slot, record.value);
            }
            break;
        }
        case m3_opSetShapeDensity:
        {
            struct
            {
                m3ShapeId id;
                float value;
                int32_t updateMass;
            } record;
            if (bytes != (int32_t)sizeof(record))
            {
                return false;
            }
            memcpy(&record, payload, sizeof(record));
            record.id.world0 = world->worldIndex0;
            int32_t slot = m3ShapeSlot(world, record.id);
            if (slot < 0 || !m3FiniteF(record.value) || record.value <= 0.0f)
            {
                return false; // hostile bytes fail loudly (16-7)
            }
            m3SetShapeDensityInternal(world, slot, record.value, record.updateMass);
            break;
        }
        case m3_opSetContactTuning:
        {
            struct
            {
                float hertz;
                float dampingRatio;
                float pushSpeed;
            } record;
            if (bytes != (int32_t)sizeof(record))
            {
                return false;
            }
            memcpy(&record, payload, sizeof(record));
            if (!m3FiniteF(record.hertz) || record.hertz <= 0.0f ||
                !m3FiniteF(record.dampingRatio) || record.dampingRatio <= 0.0f ||
                !m3FiniteF(record.pushSpeed) || record.pushSpeed <= 0.0f)
            {
                return false; // hostile bytes fail loudly (16-7)
            }
            m3SetContactTuningInternal(world, record.hertz, record.dampingRatio, record.pushSpeed);
            break;
        }
        case m3_opSetRestitutionThreshold:
        case m3_opSetMaximumLinearSpeed:
        {
            float value;
            if (bytes != (int32_t)sizeof(value))
            {
                return false;
            }
            memcpy(&value, payload, sizeof(value));
            if (!m3FiniteF(value) || (op == m3_opSetRestitutionThreshold && value < 0.0f) ||
                (op == m3_opSetMaximumLinearSpeed && value <= 0.0f))
            {
                return false; // hostile bytes fail loudly (16-7)
            }
            if (op == m3_opSetRestitutionThreshold)
            {
                m3SetRestitutionThresholdInternal(world, value);
            }
            else
            {
                m3SetMaximumLinearSpeedInternal(world, value);
            }
            break;
        }
        case m3_opSetMaximumAngularSpeed:
        {
            // A new op gets the strict wall: hostile caps (NaN,
            // nonpositive) fail the replay loudly instead of riding
            // into the solver. Op 46 keeps its 8-4 byte contract.
            float value;
            if (bytes != (int32_t)sizeof(value))
            {
                return false;
            }
            memcpy(&value, payload, sizeof(value));
            if (!m3FiniteF(value) || value <= 0.0f)
            {
                return false;
            }
            m3SetMaximumAngularSpeedInternal(world, value);
            break;
        }
        case m3_opSetBodyName:
        {
            struct
            {
                m3BodyId id;
                char name[M3_BODY_NAME_CAPACITY];
            } record;
            if (bytes != (int32_t)sizeof(record))
            {
                return false;
            }
            memcpy(&record, payload, sizeof(record));
            record.id.world0 = world->worldIndex0;
            int32_t index = m3BodySlot(world, record.id);
            if (index < 0)
            {
                return false;
            }
            record.name[M3_BODY_NAME_CAPACITY - 1] = 0;
            m3SetBodyNameInternal(world, index, record.name);
            break;
        }
        case m3_opCreateSoftBodyTet:
        {
            m3CreateSoftBodyTetOp head;
            if (bytes < (int32_t)sizeof(head))
            {
                return false;
            }
            memcpy(&head, payload, sizeof(head));
            if (head.pointCount < 4 || head.pointCount > M3_SOFTBODY_MAX_PARTICLES ||
                head.tetCount < 1 || head.tetCount > M3_SOFTBODY_MAX_TETS ||
                bytes != (int32_t)sizeof(head) + head.pointCount * (int32_t)sizeof(m3Vec3) +
                             4 * head.tetCount * (int32_t)sizeof(uint16_t))
            {
                return false;
            }
            const m3Vec3* pts = (const m3Vec3*)((const uint8_t*)payload + sizeof(head));
            const uint16_t* tets = (const uint16_t*)((const uint8_t*)payload + sizeof(head) +
                                                     (size_t)head.pointCount * sizeof(m3Vec3));
            int32_t slot = m3CreateSoftBodyTetInternal(world, &head.def, pts, head.pointCount, tets,
                                                       head.tetCount);
            if (slot < 0 || slot + 1 != head.expected.index1 ||
                world->softPool.generations[slot] != head.expected.generation)
            {
                return false; // id determinism holds for jelly too
            }
            break;
        }
        case m3_opCreateHeightFieldGrid:
        {
            m3CreateHeightFieldGridOp head;
            if (bytes < (int32_t)sizeof(head))
            {
                return false;
            }
            memcpy(&head, payload, sizeof(head));
            NormalizeShapeDefBools(&head.def);
            head.body.world0 = world->worldIndex0;
            int32_t bodyIndex = m3BodySlot(world, head.body);
            if (bodyIndex < 0 || world->types[bodyIndex] != (uint8_t)m3_staticBody || head.nx < 2 ||
                head.nx > M3_HEIGHTFIELD_MAX_DIM || head.nz < 2 ||
                head.nz > M3_HEIGHTFIELD_MAX_DIM || !m3FiniteF(head.cellSize) ||
                !(head.cellSize > 0.0f) ||
                bytes != (int32_t)sizeof(head) + head.nx * head.nz * (int32_t)sizeof(float))
            {
                return false; // hostile grid bytes fail loudly
            }
            const float* samples = (const float*)((const uint8_t*)payload + sizeof(head));
            float mn = samples[0];
            float mx = samples[0];
            for (int32_t i = 0; i < head.nx * head.nz; ++i)
            {
                if (!m3FiniteF(samples[i]))
                {
                    return false;
                }
                mn = samples[i] < mn ? samples[i] : mn;
                mx = samples[i] > mx ? samples[i] : mx;
            }
            m3HeightFieldData hf;
            memset(&hf, 0, sizeof(hf));
            hf.nx = head.nx;
            hf.nz = head.nz;
            hf.cellSize = head.cellSize;
            hf.minHeight = mn;
            hf.maxHeight = mx;
            if (!m3HeightFieldDataAlloc(&hf))
            {
                return false;
            }
            memcpy(hf.heights, samples, (size_t)(head.nx * head.nz) * sizeof(float));
            m3ShapeGeom geom;
            memset(&geom, 0, sizeof(geom));
            int32_t index = m3CreateShapeInternal(world, bodyIndex, (uint8_t)m3_heightFieldShape,
                                                  &geom, &head.def, NULL, NULL, NULL, &hf);
            if (index < 0)
            {
                m3HeightFieldDataFree(&hf);
                return false;
            }
            if (index + 1 != head.expected.index1 ||
                world->shapePool.generations[index] != head.expected.generation)
            {
                return false; // id determinism holds for terrain too
            }
            break;
        }
        case m3_opCreateWaterVolume:
        {
            struct
            {
                m3WaterVolumeDef def;
                m3WaterVolumeId expected;
            } record;
            if (bytes != (int32_t)sizeof(record))
            {
                return false;
            }
            memcpy(&record, payload, sizeof(record));
            int32_t slot = m3CreateWaterVolumeInternal(world, &record.def);
            if (slot < 0 || slot + 1 != record.expected.index1 ||
                world->waterPool.generations[slot] != record.expected.generation)
            {
                return false; // id determinism holds for water too
            }
            break;
        }
        case m3_opDestroyWaterVolume:
        {
            m3WaterVolumeId id;
            if (bytes != (int32_t)sizeof(id))
            {
                return false;
            }
            memcpy(&id, payload, sizeof(id));
            id.world0 = world->worldIndex0;
            int32_t index = id.index1 - 1;
            if (!m3IdPoolValid(&world->waterPool, index, id.generation))
            {
                return false;
            }
            m3DestroyWaterVolumeInternal(world, index);
            break;
        }
        case m3_opRebuildBroadphase:
        {
            int32_t zero;
            if (bytes != (int32_t)sizeof(zero))
            {
                return false;
            }
            m3RebuildBroadphaseInternal(world);
            break;
        }
        case m3_opSetMeshMaterials:
        {
            m3SetMeshMaterialsOp head;
            if (bytes < (int32_t)sizeof(head))
            {
                return false;
            }
            memcpy(&head, payload, sizeof(head));
            head.id.world0 = world->worldIndex0;
            int32_t slot = m3ShapeSlot(world, head.id);
            if (slot < 0 || world->shapeType[slot] != (uint8_t)m3_meshShape)
            {
                return false;
            }
            int32_t meshIndex = world->shapeMeshIndex[slot];
            if (meshIndex < 0 || head.triangleCount != world->meshData[meshIndex].triangleCount ||
                bytes != (int32_t)sizeof(head) + head.triangleCount)
            {
                return false; // the byte array must match THIS mesh
            }
            if (!m3SetMeshMaterialsInternal(world, meshIndex, head.materials, head.materialCount,
                                            (const uint8_t*)payload + sizeof(head)))
            {
                return false; // hostile paint fails loudly
            }
            break;
        }
        case m3_opJointSetMotorPose:
        {
            struct
            {
                m3JointId id;
                m3Vec3 offset;
                m3Quat rotation;
            } record;
            if (bytes != (int32_t)sizeof(record))
            {
                return false;
            }
            memcpy(&record, payload, sizeof(record));
            record.id.world0 = world->worldIndex0;
            int32_t slot = m3JointSlot(world, record.id);
            float q2 =
                record.rotation.x * record.rotation.x + record.rotation.y * record.rotation.y +
                record.rotation.z * record.rotation.z + record.rotation.w * record.rotation.w;
            if (slot < 0 || world->jointType[slot] != (uint8_t)m3_motorJoint ||
                !m3FiniteV3(record.offset) || !m3FiniteQuat(record.rotation) || q2 < 0.81f ||
                q2 > 1.21f)
            {
                return false; // hostile servo bytes fail loudly
            }
            m3JointSetMotorPoseInternal(world, slot, record.offset, record.rotation);
            break;
        }
        case m3_opJointSetSteer:
        {
            struct
            {
                m3JointId id;
                int32_t enable;
                float target;
                float hertz;
                float zeta;
                float effort;
            } record;
            if (bytes != (int32_t)sizeof(record))
            {
                return false;
            }
            memcpy(&record, payload, sizeof(record));
            record.id.world0 = world->worldIndex0;
            int32_t slot = m3JointSlot(world, record.id);
            if (slot < 0 || world->jointType[slot] != (uint8_t)m3_wheelJoint ||
                !m3FiniteF(record.target) || m3AbsF(record.target) > 1.0f ||
                !m3FiniteF(record.hertz) || !m3FiniteF(record.zeta) || !m3FiniteF(record.effort) ||
                record.zeta < 0.0f || record.effort < 0.0f ||
                (record.enable != 0 && !(record.hertz > 0.0f)) ||
                (record.enable != 0 && record.enable != 1))
            {
                return false; // hostile steer bytes fail loudly
            }
            m3JointSetSteerInternal(world, slot, record.enable, record.target, record.hertz,
                                    record.zeta, record.effort);
            break;
        }
        case m3_opSetShapeGeom:
        {
            struct
            {
                m3ShapeId id;
                uint32_t type;
                m3ShapeGeom geom;
            } record;
            if (bytes != (int32_t)sizeof(record))
            {
                return false;
            }
            memcpy(&record, payload, sizeof(record));
            if (record.type > 255u)
            {
                return false;
            }
            record.id.world0 = world->worldIndex0;
            int32_t slot = record.id.index1 - 1;
            if (slot < 0 || slot >= world->shapePool.maxIndex ||
                world->shapePool.alive[slot] == 0 ||
                world->shapePool.generations[slot] != record.id.generation)
            {
                return false;
            }
            if (!m3SetShapeGeomInternal(world, slot, (uint8_t)record.type, &record.geom))
            {
                return false; // hostile swaps fail the replay loudly
            }
            break;
        }
        case m3_opSetAllowFastRotation:
        {
            struct
            {
                m3BodyId id;
                uint32_t allow;
            } record;
            if (bytes != (int32_t)sizeof(record))
            {
                return false;
            }
            memcpy(&record, payload, sizeof(record));
            if (record.allow > 1u)
            {
                return false;
            }
            record.id.world0 = world->worldIndex0;
            int32_t index = m3BodySlot(world, record.id);
            if (index < 0)
            {
                return false;
            }
            m3SetAllowFastRotationInternal(world, index, (int32_t)record.allow);
            break;
        }
        case m3_opWorldExplode:
        {
            m3ExplosionDef def;
            if (bytes != (int32_t)sizeof(def))
            {
                return false;
            }
            memcpy(&def, payload, sizeof(def));
            if (!m3WorldExplodeInternal(world, &def))
            {
                return false; // hostile blast fields fail loudly
            }
            break;
        }
        case m3_opEnableSleeping:
        case m3_opEnableContinuous:
        {
            int32_t on;
            if (bytes != (int32_t)sizeof(on))
            {
                return false;
            }
            memcpy(&on, payload, sizeof(on));
            if (op == m3_opEnableSleeping)
            {
                m3EnableSleepingInternal(world, on);
            }
            else
            {
                m3EnableContinuousInternal(world, on);
            }
            break;
        }
        case m3_opSetWind:
        {
            struct
            {
                m3Vec3 dir;
                float speed;
                float gustHertz;
                float gustScale;
            } record;
            if (bytes != (int32_t)sizeof(record))
            {
                return false;
            }
            memcpy(&record, payload, sizeof(record));
            if (!m3FiniteV3(record.dir) || !m3FiniteF(record.speed) || record.speed < 0.0f ||
                !m3FiniteF(record.gustHertz) || record.gustHertz < 0.0f ||
                !m3FiniteF(record.gustScale) || record.gustScale < 0.0f)
            {
                return false; // hostile bytes fail loudly (16-7)
            }
            m3SetWindInternal(world, record.dir, record.speed, record.gustHertz, record.gustScale);
            break;
        }
        case m3_opSetSurfaceVelocity:
        {
            struct
            {
                m3ShapeId id;
                m3Vec3 v;
            } record;
            if (bytes != (int32_t)sizeof(record))
            {
                return false;
            }
            memcpy(&record, payload, sizeof(record));
            record.id.world0 = world->worldIndex0;
            int32_t slot = m3ShapeSlot(world, record.id);
            if (slot < 0 || !m3FiniteV3(record.v))
            {
                return false; // hostile bytes fail loudly (16-7)
            }
            m3SetSurfaceVelocityInternal(world, slot, record.v);
            break;
        }
        case m3_opSetHitEventThreshold:
        {
            float value;
            if (bytes != (int32_t)sizeof(value))
            {
                return false;
            }
            memcpy(&value, payload, sizeof(value));
            if (!m3FiniteF(value) || value < 0.0f)
            {
                return false; // hostile bytes fail loudly (16-7)
            }
            m3SetHitEventThresholdInternal(world, value);
            break;
        }
        case m3_opEnableShapeHitEvents:
        case m3_opEnableShapePreSolve:
        {
            struct
            {
                m3ShapeId id;
                int32_t on;
            } record;
            if (bytes != (int32_t)sizeof(record))
            {
                return false;
            }
            memcpy(&record, payload, sizeof(record));
            record.id.world0 = world->worldIndex0;
            int32_t slot = m3ShapeSlot(world, record.id);
            if (slot < 0)
            {
                return false;
            }
            if (op == m3_opEnableShapeHitEvents)
            {
                m3EnableShapeHitEventsInternal(world, slot, record.on);
            }
            else
            {
                m3EnableShapePreSolveInternal(world, slot, record.on);
            }
            break;
        }
        case m3_opJointSetLimits:
        case m3_opJointSetMotor:
        {
            struct
            {
                m3JointId id;
                int32_t enable;
                float a;
                float b;
            } record;
            if (bytes != (int32_t)sizeof(record))
            {
                return false;
            }
            memcpy(&record, payload, sizeof(record));
            record.id.world0 = world->worldIndex0;
            int32_t slot = m3JointSlot(world, record.id);
            if (slot < 0 || !m3FiniteF(record.a) || !m3FiniteF(record.b))
            {
                return false; // hostile bytes fail loudly (16-7)
            }
            if (op == m3_opJointSetLimits)
            {
                // Mirror the public contract: the motor joint's
                // budgets are independent nonnegatives, every other
                // type wants an ordered range (16-5).
                if (world->jointType[slot] == (uint8_t)m3_motorJoint
                        ? (record.a < 0.0f || record.b < 0.0f)
                        : record.a > record.b)
                {
                    return false;
                }
                m3JointSetLimitsInternal(world, slot, record.enable, record.a, record.b);
            }
            else
            {
                m3JointSetMotorInternal(world, slot, record.enable, record.a, record.b);
            }
            break;
        }
        case m3_opJointSetCollide:
        {
            struct
            {
                m3JointId id;
                int32_t on;
            } record;
            if (bytes != (int32_t)sizeof(record))
            {
                return false;
            }
            memcpy(&record, payload, sizeof(record));
            record.id.world0 = world->worldIndex0;
            int32_t slot = m3JointSlot(world, record.id);
            if (slot < 0)
            {
                return false;
            }
            m3JointSetCollideInternal(world, slot, record.on);
            break;
        }
        case m3_opJointSetBreak:
        {
            struct
            {
                m3JointId id;
                float maxForce;
                float maxTorque;
            } record;
            if (bytes != (int32_t)sizeof(record))
            {
                return false;
            }
            memcpy(&record, payload, sizeof(record));
            record.id.world0 = world->worldIndex0;
            int32_t slot = m3JointSlot(world, record.id);
            if (slot < 0 || !m3FiniteF(record.maxForce) || !m3FiniteF(record.maxTorque) ||
                record.maxForce < 0.0f || record.maxTorque < 0.0f)
            {
                return false; // hostile bytes fail loudly (16-7)
            }
            m3JointSetBreakInternal(world, slot, record.maxForce, record.maxTorque);
            break;
        }
        case m3_opJointSetSpring:
        {
            struct
            {
                m3JointId id;
                int32_t enable;
                float hertz;
                float zeta;
            } record;
            if (bytes != (int32_t)sizeof(record))
            {
                return false;
            }
            memcpy(&record, payload, sizeof(record));
            record.id.world0 = world->worldIndex0;
            int32_t slot = m3JointSlot(world, record.id);
            if (slot < 0 || !m3FiniteF(record.hertz) || record.hertz <= 0.0f ||
                !m3FiniteF(record.zeta) || record.zeta < 0.0f)
            {
                return false; // hostile bytes fail loudly (16-7)
            }
            m3JointSetSpringInternal(world, slot, record.enable, record.hertz, record.zeta);
            break;
        }
        case m3_opJointSetTarget:
        {
            struct
            {
                m3JointId id;
                float scalar;
                m3Quat q;
            } record;
            if (bytes != (int32_t)sizeof(record))
            {
                return false;
            }
            memcpy(&record, payload, sizeof(record));
            record.id.world0 = world->worldIndex0;
            int32_t slot = m3JointSlot(world, record.id);
            if (slot < 0 || !m3FiniteF(record.scalar) || !m3FiniteQuat(record.q))
            {
                return false; // hostile bytes fail loudly (16-7)
            }
            m3JointSetTargetInternal(world, slot, record.scalar, record.q);
            break;
        }
        default:
            return false; // unknown op: reject loudly, never skip
        }
    }
    return cursor == size;
}

bool m3World_JournalReplay(m3WorldId worldId, const void* data, int32_t size)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || data == NULL || size < 0)
    {
        return false; // contract, not invariant
    }
    // Atomic replay (2d-2): the world either takes the whole session
    // or none of it. A pre-replay snapshot backs out any partial
    // application on refusal, so a corrupted or truncated journal
    // can never leave a half-built world behind.
    int32_t snapBytes = m3World_SnapshotSize(worldId);
    uint8_t* snap = (uint8_t*)m3AllocZeroed(snapBytes);
    if (snap == NULL)
    {
        return false; // no memory for the guarantee means no replay
    }
    if (m3World_Snapshot(worldId, snap, snapBytes) != snapBytes)
    {
        m3Free(snap);
        return false;
    }
    bool ok = JournalReplayApply(world, data, size);
    if (!ok)
    {
        bool restored = m3World_Restore(worldId, snap, snapBytes);
        M3_ASSERT(restored); // our own snapshot must restore: invariant
        (void)restored;
    }
    m3Free(snap);
    return ok;
}
