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
    def.workerCount = 1;
    def.internalValue = M3_WORLD_COOKIE;
    return def;
}

m3WorldId m3CreateWorld(const m3WorldDef* def)
{
    m3WorldId nullId = {0, 0};
    if (def == NULL || def->internalValue != M3_WORLD_COOKIE || def->bodyCapacity <= 0 ||
        def->shapeCapacity <= 0 || def->meshCapacity <= 0 || def->workerCount <= 0 ||
        (def->enqueueTask == NULL) != (def->finishTask == NULL))
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
        M3_ASSERT(false);
        return nullId; // table exhausted: loud, never silent
    }

    m3World* world = (m3World*)m3AllocZeroed((int32_t)sizeof(m3World));
    int32_t cap = def->bodyCapacity;
    world->gravity = def->gravity;
    world->bodyCapacity = cap;
    world->shapeCapacity = def->shapeCapacity;
    world->meshCapacity = def->meshCapacity;
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
    world->meshPool = m3IdPoolCreate(def->meshCapacity);
    M3_ALLOC(world->meshData, def->meshCapacity, m3MeshData);
    M3_ALLOC(world->meshRefCounts, def->meshCapacity, int32_t);
    M3_ALLOC(world->shapeMeshIndex, shapeCap, int32_t);
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
        M3_ASSERT(false);
        return;
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
    m3Free(world->shapeUserData);
    m3Free(world->shapeNext);
    m3Free(world->shapeHullIndex);
    m3IdPoolDestroy(&world->hullPool);
    m3Free(world->hullData);
    m3Free(world->hullRefCounts);
    m3IdPoolDestroy(&world->meshPool);
    m3Free(world->meshData);
    m3Free(world->meshRefCounts);
    m3Free(world->shapeMeshIndex);
    m3Free(world->beginEvents);
    m3Free(world->endEvents);
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
        M3_ASSERT(false);
        return false;
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
        M3_ASSERT(false);
        return -1;
    }
    int32_t bytes = world->journalOverflow != 0 ? -1 : world->journalCursor;
    world->journalBuffer = NULL;
    world->journalCapacity = 0;
    world->journalCursor = 0;
    world->journalActive = 0;
    world->journalOverflow = 0;
    return bytes;
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

bool m3World_JournalReplay(m3WorldId worldId, const void* data, int32_t size)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || data == NULL || size < 0)
    {
        M3_ASSERT(false);
        return false;
    }
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
                                                  &record.def, NULL, NULL);
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
                                                  &record.def, NULL, mesh);
            m3Free(mesh);
            if (index < 0 || index + 1 != record.expected.index1 ||
                world->shapePool.generations[index] != record.expected.generation)
            {
                return false; // id determinism holds for mesh shapes too
            }
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
                                                  &record.def, &rebuilt, NULL);
            if (index < 0 || index + 1 != record.expected.index1 ||
                world->shapePool.generations[index] != record.expected.generation)
            {
                return false; // id determinism holds for hull shapes too
            }
            break;
        }
        default:
            return false; // unknown op: reject loudly, never skip
        }
    }
    return cursor == size;
}
