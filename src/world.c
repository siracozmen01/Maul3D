// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// World lifecycle and the journal. Structure follows Maul2D's proven
// world.c (itself adapted from Box2D v3, MIT, Erin Catto): a static
// world table with generations, def-cookie validation, SoA arrays
// allocated per field, and a journal whose replay goes through the
// same internal functions the public API uses.

#include "world_internal.h"

#include <string.h>

static m3World* s_worlds[M3_MAX_WORLDS];
static uint16_t s_worldGenerations[M3_MAX_WORLDS];

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
        (def->enqueueTask == NULL) != (def->finishTask == NULL) || !m3FiniteV3(def->gravity))
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
    M3_ALLOC(world->bodyShapeHead, cap, int32_t);
    for (int32_t i = 0; i < cap; ++i)
    {
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
    M3_ALLOC(world->jointMotor, def->jointCapacity, m3Vec3);
    M3_ALLOC(world->jointLimits, def->jointCapacity, m3Vec3);
    M3_ALLOC(world->jointGenericModes, def->jointCapacity, uint16_t);
    M3_ALLOC(world->jointGenLinLower, def->jointCapacity, m3Vec3);
    M3_ALLOC(world->jointGenLinUpper, def->jointCapacity, m3Vec3);
    M3_ALLOC(world->jointGenAngLower, def->jointCapacity, m3Vec3);
    M3_ALLOC(world->jointGenAngUpper, def->jointCapacity, m3Vec3);
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
    M3_ALLOC(world->vehWheelCompression, def->vehicleCapacity * M3_VEHICLE_MAX_WHEELS, m3real);
    M3_ALLOC(world->vehWheelContact, def->vehicleCapacity * M3_VEHICLE_MAX_WHEELS, uint8_t);
    M3_ALLOC(world->vehTireGrip, def->vehicleCapacity, m3real);
    M3_ALLOC(world->vehThrottle, def->vehicleCapacity, m3real);
    M3_ALLOC(world->vehSteer, def->vehicleCapacity, m3real);
    M3_ALLOC(world->vehBrake, def->vehicleCapacity, m3real);
    M3_ALLOC(world->vehWheelSpin, def->vehicleCapacity * M3_VEHICLE_MAX_WHEELS, m3real);
    world->softPool = m3IdPoolCreate(def->softBodyCapacity);
    M3_ALLOC(world->softParticleCount, def->softBodyCapacity, int32_t);
    M3_ALLOC(world->softEdgeCount, def->softBodyCapacity, int32_t);
    M3_ALLOC(world->softCompliance, def->softBodyCapacity, m3real);
    M3_ALLOC(world->softRadius, def->softBodyCapacity, m3real);
    M3_ALLOC(world->softGravityScale, def->softBodyCapacity, m3real);
    M3_ALLOC(world->softUserData, def->softBodyCapacity, uint64_t);
    M3_ALLOC(world->softPos, def->softBodyCapacity * M3_SOFTBODY_MAX_PARTICLES, m3Pos3);
    M3_ALLOC(world->softPrev, def->softBodyCapacity * M3_SOFTBODY_MAX_PARTICLES, m3Pos3);
    M3_ALLOC(world->softInvMass, def->softBodyCapacity * M3_SOFTBODY_MAX_PARTICLES, m3real);
    M3_ALLOC(world->softEdgeA, def->softBodyCapacity * M3_SOFTBODY_MAX_EDGES, uint16_t);
    M3_ALLOC(world->softEdgeB, def->softBodyCapacity * M3_SOFTBODY_MAX_EDGES, uint16_t);
    M3_ALLOC(world->softEdgeRest, def->softBodyCapacity * M3_SOFTBODY_MAX_EDGES, m3real);
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
    m3Free(world->bodyShapeHead);
    m3IdPoolDestroy(&world->shapePool);
    m3Free(world->shapeBody);
    m3Free(world->shapeType);
    m3Free(world->shapeGeom);
    m3Free(world->shapeDensity);
    m3Free(world->shapeFriction);
    m3Free(world->shapeRestitution);
    m3Free(world->shapeRollingResistance);
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
    m3Free(world->jointMotor);
    m3Free(world->jointLimits);
    m3Free(world->jointGenericModes);
    m3Free(world->jointGenLinLower);
    m3Free(world->jointGenLinUpper);
    m3Free(world->jointGenAngLower);
    m3Free(world->jointGenAngUpper);
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
    m3Free(world->vehWheelCompression);
    m3Free(world->vehWheelContact);
    m3Free(world->vehTireGrip);
    m3Free(world->vehThrottle);
    m3Free(world->vehSteer);
    m3Free(world->vehBrake);
    m3Free(world->vehWheelSpin);
    m3IdPoolDestroy(&world->softPool);
    m3Free(world->softParticleCount);
    m3Free(world->softEdgeCount);
    m3Free(world->softCompliance);
    m3Free(world->softRadius);
    m3Free(world->softGravityScale);
    m3Free(world->softUserData);
    m3Free(world->softPos);
    m3Free(world->softPrev);
    m3Free(world->softInvMass);
    m3Free(world->softEdgeA);
    m3Free(world->softEdgeB);
    m3Free(world->softEdgeRest);
    m3Free(world->charGrounded);
    m3Free(world->charGroundNormal);
    m3Free(world->jointNextA);
    m3Free(world->jointNextB);
    m3Free(world->bodyJointHead);
    m3IdPoolDestroy(&world->meshPool);
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

// --- Journal ---------------------------------------------------------------

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
            if (index < 0)
            {
                return false;
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
            if (index < 0)
            {
                return false;
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
            if (!(record.dt > 0.0f) || record.substeps < 1)
            {
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
            int32_t index = m3CreateShapeInternal(world, bodyIndex, record.type, &record.geom,
                                                  &record.def, NULL, NULL, NULL);
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
            m3MeshData* mesh = (m3MeshData*)m3AllocZeroed((int32_t)sizeof(m3MeshData));
            if (mesh == NULL)
            {
                return false;
            }
            mesh->vertexCount = record.vertexCount;
            mesh->triangleCount = record.triangleCount;
            memcpy(mesh->vertices, (const uint8_t*)payload + sizeof(record), (size_t)vertexBytes);
            memcpy(mesh->indices, (const uint8_t*)payload + sizeof(record) + vertexBytes,
                   (size_t)indexBytes);
            m3ShapeGeom geom;
            memset(&geom, 0, sizeof(geom));
            int32_t index = m3CreateShapeInternal(world, bodyIndex, (uint8_t)m3_meshShape, &geom,
                                                  &record.def, NULL, mesh, NULL);
            m3Free(mesh);
            if (index < 0 || index + 1 != record.expected.index1 ||
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
            int32_t index = m3CreateShapeInternal(world, bodyIndex, (uint8_t)m3_hullShape, &geom,
                                                  &record.def, &rebuilt, NULL, NULL);
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
            int32_t index = m3CreateShapeInternal(world, bodyIndex, (uint8_t)m3_voxelShape, &geom,
                                                  &record.def, NULL, NULL, chunk);
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
            if (slot < 0)
            {
                return false;
            }
            m3CharacterMoveInternal(world, slot, record.translation);
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
            if (slot < 0)
            {
                return false;
            }
            m3VehicleCommandsInternal(world, slot, record.throttle, record.steer, record.brake);
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
