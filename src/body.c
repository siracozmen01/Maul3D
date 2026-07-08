// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Bodies: creation, destruction, and the journaled setters. The public
// functions validate and journal; the internal functions are the only
// paths that mutate state, and replay drives them directly. World
// resolution from a body id goes by slot (m3WorldFromIndex0): the body
// id carries its own generation, the world slot does not need one.

#include "world_internal.h"

#include <string.h>

int32_t m3BodySlot(const m3World* world, m3BodyId bodyId)
{
    int32_t index = bodyId.index1 - 1;
    if (world == NULL || bodyId.world0 != world->worldIndex0 ||
        !m3IdPoolValid(&world->bodyPool, index, bodyId.generation))
    {
        return -1;
    }
    return index;
}

// Resolve a body id to its world and slot; NULL world or -1 index for
// anything stale or foreign.
static m3World* ResolveBody(m3BodyId bodyId, int32_t* indexOut)
{
    m3World* world = m3WorldFromIndex0(bodyId.world0);
    int32_t index = world != NULL ? m3BodySlot(world, bodyId) : -1;
    *indexOut = index;
    return index >= 0 ? world : NULL;
}

m3BodyDef m3DefaultBodyDef(void)
{
    m3BodyDef def;
    memset(&def, 0, sizeof(def));
    def.type = m3_staticBody;
    def.rotation = m3MakeIdentityQuat();
    def.gravityScale = 1.0f;
    def.internalValue = M3_BODY_COOKIE;
    return def;
}

int32_t m3CreateBodyInternal(m3World* world, const m3BodyDef* def)
{
    int32_t index = m3IdPoolAlloc(&world->bodyPool);
    if (index < 0)
    {
        return -1; // pool exhausted: the caller fails loudly
    }
    world->transforms[index].p = def->position;
    world->transforms[index].q = m3NormalizeQuat(def->rotation);
    world->linearVelocities[index] = def->linearVelocity;
    world->angularVelocities[index] = def->angularVelocity;
    // A shapeless dynamic body has unit mass and zero inertia (the
    // reference convention); shape mass replaces this in task 7.
    world->invMass[index] = def->type == m3_dynamicBody ? 1.0f : 0.0f;
    world->invInertiaLocal[index] = m3MakeZeroMat3();
    world->inertiaLocal[index] = m3MakeZeroMat3();
    world->bulletFlags[index] = 0;
    world->awake[index] = 1;
    world->sleepTimes[index] = 0.0f;
    world->minExtents[index] = 1.0e30f;
    world->maxExtents[index] = 0.0f;
    world->localCenters[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
    world->gravityScales[index] = def->gravityScale;
    world->linearDamping[index] = def->linearDamping;
    world->angularDamping[index] = def->angularDamping;
    world->types[index] = (uint8_t)def->type;
    world->bulletFlags[index] = def->isBullet ? 1 : 0;
    world->userData[index] = def->userData;
    world->bodyShapeHead[index] = -1;
    return index;
}

void m3DestroyBodyInternal(m3World* world, int32_t index)
{
    // Destroy attached joints first (the cascade, like shapes).
    while (world->bodyJointHead[index] != -1)
    {
        m3DestroyJointInternal(world, world->bodyJointHead[index]);
    }
    // Cascade: a body takes its shapes with it (each destroy unlinks
    // the list head, so this drains deterministically).
    while (world->bodyShapeHead[index] != -1)
    {
        m3DestroyShapeInternal(world, world->bodyShapeHead[index]);
    }
    // Zero the slot so recycled state can never leak into a new body
    // or into the snapshot bytes.
    world->transforms[index] = (m3Transform){{0.0, 0.0, 0.0}, {0.0f, 0.0f, 0.0f, 1.0f}};
    world->linearVelocities[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
    world->angularVelocities[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
    world->invMass[index] = 0.0f;
    world->invInertiaLocal[index] = m3MakeZeroMat3();
    world->inertiaLocal[index] = m3MakeZeroMat3();
    world->bulletFlags[index] = 0;
    world->awake[index] = 1;
    world->sleepTimes[index] = 0.0f;
    world->minExtents[index] = 1.0e30f;
    world->maxExtents[index] = 0.0f;
    world->localCenters[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
    world->gravityScales[index] = 0.0f;
    world->linearDamping[index] = 0.0f;
    world->angularDamping[index] = 0.0f;
    world->types[index] = 0;
    world->userData[index] = 0;
    m3IdPoolFree(&world->bodyPool, index);
}

void m3SetLinearVelocityInternal(m3World* world, int32_t index, m3Vec3 velocity)
{
    world->awake[index] = 1; // a commanded velocity always wakes
    world->sleepTimes[index] = 0.0f;
    world->linearVelocities[index] = velocity;
}

void m3SetAngularVelocityInternal(m3World* world, int32_t index, m3Vec3 velocity)
{
    world->awake[index] = 1;
    world->sleepTimes[index] = 0.0f;
    world->angularVelocities[index] = velocity;
}

m3BodyId m3CreateBody(m3WorldId worldId, const m3BodyDef* def)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || def == NULL || def->internalValue != M3_BODY_COOKIE ||
        (def->type != m3_staticBody && def->type != m3_kinematicBody &&
         def->type != m3_dynamicBody))
    {
        // Contract, not invariant: a bad def or stale world returns
        // the null id (see m3CreateWorld).
        return m3_nullBodyId;
    }
    int32_t index = m3CreateBodyInternal(world, def);
    if (index < 0)
    {
        return m3_nullBodyId;
    }
    m3BodyId id = {index + 1, world->worldIndex0, world->bodyPool.generations[index]};
    if (world->journalActive != 0)
    {
        struct
        {
            m3BodyDef def;
            m3BodyId expected;
        } record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.expected = id;
        m3JournalRecord(world, m3_opCreateBody, &record, (int32_t)sizeof(record));
    }
    return id;
}

void m3DestroyBody(m3BodyId bodyId)
{
    int32_t index;
    m3World* world = ResolveBody(bodyId, &index);
    if (world == NULL)
    {
        M3_ASSERT(false);
        return;
    }
    if (world->journalActive != 0)
    {
        m3JournalRecord(world, m3_opDestroyBody, &bodyId, (int32_t)sizeof(bodyId));
    }
    m3DestroyBodyInternal(world, index);
}

bool m3Body_IsValid(m3BodyId bodyId)
{
    int32_t index;
    return ResolveBody(bodyId, &index) != NULL;
}

m3Pos3 m3Body_GetPosition(m3BodyId bodyId)
{
    int32_t index;
    m3World* world = ResolveBody(bodyId, &index);
    return world != NULL ? world->transforms[index].p : (m3Pos3){0.0, 0.0, 0.0};
}

m3Quat m3Body_GetRotation(m3BodyId bodyId)
{
    int32_t index;
    m3World* world = ResolveBody(bodyId, &index);
    return world != NULL ? world->transforms[index].q : m3MakeIdentityQuat();
}

m3Vec3 m3Body_GetLinearVelocity(m3BodyId bodyId)
{
    int32_t index;
    m3World* world = ResolveBody(bodyId, &index);
    return world != NULL ? world->linearVelocities[index] : (m3Vec3){0.0f, 0.0f, 0.0f};
}

m3Vec3 m3Body_GetAngularVelocity(m3BodyId bodyId)
{
    int32_t index;
    m3World* world = ResolveBody(bodyId, &index);
    return world != NULL ? world->angularVelocities[index] : (m3Vec3){0.0f, 0.0f, 0.0f};
}

uint64_t m3Body_GetUserData(m3BodyId bodyId)
{
    int32_t index;
    m3World* world = ResolveBody(bodyId, &index);
    return world != NULL ? world->userData[index] : 0;
}

m3BodyType m3Body_GetType(m3BodyId bodyId)
{
    int32_t index;
    m3World* world = ResolveBody(bodyId, &index);
    return world != NULL ? (m3BodyType)world->types[index] : m3_staticBody;
}

void m3Body_SetLinearVelocity(m3BodyId bodyId, m3Vec3 velocity)
{
    int32_t index;
    m3World* world = ResolveBody(bodyId, &index);
    if (world == NULL)
    {
        M3_ASSERT(false);
        return;
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m3BodyId id;
            m3Vec3 v;
        } record;
        memset(&record, 0, sizeof(record));
        record.id = bodyId;
        record.v = velocity;
        m3JournalRecord(world, m3_opSetLinearVelocity, &record, (int32_t)sizeof(record));
    }
    m3SetLinearVelocityInternal(world, index, velocity);
}

void m3Body_SetAngularVelocity(m3BodyId bodyId, m3Vec3 velocity)
{
    int32_t index;
    m3World* world = ResolveBody(bodyId, &index);
    if (world == NULL)
    {
        M3_ASSERT(false);
        return;
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m3BodyId id;
            m3Vec3 v;
        } record;
        memset(&record, 0, sizeof(record));
        record.id = bodyId;
        record.v = velocity;
        m3JournalRecord(world, m3_opSetAngularVelocity, &record, (int32_t)sizeof(record));
    }
    m3SetAngularVelocityInternal(world, index, velocity);
}
