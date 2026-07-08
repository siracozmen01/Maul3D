// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Joint lifecycle (2c-2): the same law as bodies and shapes. Public
// functions validate and journal, internal functions mutate, replay
// drives the internals and verifies minted ids. The warm-start
// impulse lives in the persistent arena and rides the snapshot.

#include "world_internal.h"

#include <string.h>

#define M3_JOINT_COOKIE ((int32_t)(M3_COOKIE ^ ((int32_t)sizeof(m3JointDef) << 8) ^ 3))

m3JointDef m3DefaultJointDef(void)
{
    m3JointDef def;
    memset(&def, 0, sizeof(def));
    def.type = m3_sphericalJoint;
    def.internalValue = M3_JOINT_COOKIE;
    return def;
}

int32_t m3JointSlot(const m3World* world, m3JointId jointId)
{
    int32_t index = jointId.index1 - 1;
    if (world == NULL || jointId.world0 != world->worldIndex0 ||
        !m3IdPoolValid(&world->jointPool, index, jointId.generation))
    {
        return -1;
    }
    return index;
}

int32_t m3CreateJointInternal(m3World* world, const m3JointDef* def, int32_t bodyA, int32_t bodyB)
{
    int32_t index = m3IdPoolAlloc(&world->jointPool);
    if (index < 0)
    {
        return -1; // exhausted: loud at the caller
    }
    world->jointType[index] = (uint8_t)def->type;
    world->jointBodyA[index] = bodyA;
    world->jointBodyB[index] = bodyB;
    world->jointLocalA[index] = def->localAnchorA;
    world->jointLocalB[index] = def->localAnchorB;
    world->jointCollide[index] = def->collideConnected ? 1 : 0;
    world->jointImpulse[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
    // Push onto both bodies' joint lists (creation order recoverable:
    // replay recreates in the same order).
    world->jointNextA[index] = world->bodyJointHead[bodyA];
    world->bodyJointHead[bodyA] = index;
    world->jointNextB[index] = world->bodyJointHead[bodyB];
    world->bodyJointHead[bodyB] = index;
    // A new joint wakes both sides: articulation is a disturbance.
    if (world->types[bodyA] == (uint8_t)m3_dynamicBody)
    {
        world->awake[bodyA] = 1;
        world->sleepTimes[bodyA] = 0.0f;
    }
    if (world->types[bodyB] == (uint8_t)m3_dynamicBody)
    {
        world->awake[bodyB] = 1;
        world->sleepTimes[bodyB] = 0.0f;
    }
    return index;
}

// Unlink from one body's list (the list is threaded through nextA
// for joints where the body plays A and nextB where it plays B).
static void UnlinkJoint(m3World* world, int32_t body, int32_t index)
{
    int32_t* cursor = &world->bodyJointHead[body];
    while (*cursor != -1)
    {
        int32_t j = *cursor;
        if (j == index)
        {
            *cursor = world->jointBodyA[j] == body ? world->jointNextA[j] : world->jointNextB[j];
            return;
        }
        cursor = world->jointBodyA[j] == body ? &world->jointNextA[j] : &world->jointNextB[j];
    }
}

void m3DestroyJointInternal(m3World* world, int32_t index)
{
    int32_t bodyA = world->jointBodyA[index];
    int32_t bodyB = world->jointBodyB[index];
    UnlinkJoint(world, bodyA, index);
    UnlinkJoint(world, bodyB, index);
    if (world->types[bodyA] == (uint8_t)m3_dynamicBody && world->bodyPool.alive[bodyA] != 0)
    {
        world->awake[bodyA] = 1;
        world->sleepTimes[bodyA] = 0.0f;
    }
    if (world->types[bodyB] == (uint8_t)m3_dynamicBody && world->bodyPool.alive[bodyB] != 0)
    {
        world->awake[bodyB] = 1;
        world->sleepTimes[bodyB] = 0.0f;
    }
    world->jointType[index] = 0;
    world->jointBodyA[index] = -1;
    world->jointBodyB[index] = -1;
    world->jointLocalA[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
    world->jointLocalB[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
    world->jointCollide[index] = 0;
    world->jointImpulse[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
    world->jointNextA[index] = -1;
    world->jointNextB[index] = -1;
    m3IdPoolFree(&world->jointPool, index);
}

m3JointId m3CreateJoint(const m3JointDef* def)
{
    if (def == NULL || def->internalValue != M3_JOINT_COOKIE ||
        def->type != (int32_t)m3_sphericalJoint)
    {
        return m3_nullJointId;
    }
    m3World* world = m3WorldFromIndex0(def->bodyA.world0);
    if (world == NULL || def->bodyB.world0 != def->bodyA.world0)
    {
        return m3_nullJointId; // both bodies must share a world
    }
    int32_t bodyA = m3BodySlot(world, def->bodyA);
    int32_t bodyB = m3BodySlot(world, def->bodyB);
    if (bodyA < 0 || bodyB < 0 || bodyA == bodyB)
    {
        return m3_nullJointId;
    }
    if (world->types[bodyA] != (uint8_t)m3_dynamicBody &&
        world->types[bodyB] != (uint8_t)m3_dynamicBody)
    {
        return m3_nullJointId; // a joint between immovables is inert
    }
    int32_t index = m3CreateJointInternal(world, def, bodyA, bodyB);
    if (index < 0)
    {
        return m3_nullJointId;
    }
    m3JointId id = {index + 1, world->worldIndex0, world->jointPool.generations[index]};
    if (world->journalActive != 0)
    {
        m3CreateJointOp record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.expected = id;
        m3JournalRecord(world, m3_opCreateJoint, &record, (int32_t)sizeof(record));
    }
    return id;
}

void m3DestroyJoint(m3JointId jointId)
{
    m3World* world = m3WorldFromIndex0(jointId.world0);
    int32_t index = world != NULL ? m3JointSlot(world, jointId) : -1;
    if (index < 0)
    {
        return; // stale id: a quiet no-op is the destroy contract
    }
    if (world->journalActive != 0)
    {
        m3JournalRecord(world, m3_opDestroyJoint, &jointId, (int32_t)sizeof(jointId));
    }
    m3DestroyJointInternal(world, index);
}

bool m3Joint_IsValid(m3JointId jointId)
{
    m3World* world = m3WorldFromIndex0(jointId.world0);
    return world != NULL && m3JointSlot(world, jointId) >= 0;
}
