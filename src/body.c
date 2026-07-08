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
    world->bodyForce[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
    world->bodyTorque[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
    // Characters standing on this body lose their ground reference
    // NOW (4-6): the generation guard would catch a recycled slot,
    // but a cleared reference never even asks.
    for (int32_t c = 0; c < world->charPool.maxIndex; ++c)
    {
        if (world->charPool.alive[c] != 0 && world->charGroundBody[c] == index)
        {
            world->charGroundBody[c] = -1;
            world->charGroundGen[c] = 0;
        }
    }
    // A chassis takes its vehicle with it (5-1): the cascade rule,
    // same as shapes and joints, in ascending slot order.
    for (int32_t v = 0; v < world->vehPool.maxIndex; ++v)
    {
        if (world->vehPool.alive[v] != 0 && world->vehChassis[v] == index)
        {
            m3DestroyVehicleInternal(world, v);
        }
    }
    m3IdPoolFree(&world->bodyPool, index);
}

// Forces and impulses (8-2). One internal per op so replay and the
// public wrappers share exactly one application path.
static int ForceTargetValid(m3World* world, int32_t index)
{
    return world->types[index] == (uint8_t)m3_dynamicBody && world->invMass[index] > 0.0f;
}

static void ForceWake(m3World* world, int32_t index, m3Vec3 a, m3Vec3 b)
{
    if (a.x != 0.0f || a.y != 0.0f || a.z != 0.0f || b.x != 0.0f || b.y != 0.0f || b.z != 0.0f)
    {
        world->awake[index] = 1;
        world->sleepTimes[index] = 0.0f;
    }
}

void m3ApplyForceInternal(m3World* world, int32_t index, m3Vec3 force)
{
    world->bodyForce[index] = m3Add3(world->bodyForce[index], force);
    ForceWake(world, index, force, (m3Vec3){0.0f, 0.0f, 0.0f});
}

void m3ApplyTorqueInternal(m3World* world, int32_t index, m3Vec3 torque)
{
    world->bodyTorque[index] = m3Add3(world->bodyTorque[index], torque);
    ForceWake(world, index, torque, (m3Vec3){0.0f, 0.0f, 0.0f});
}

void m3ApplyLinearImpulseInternal(m3World* world, int32_t index, m3Vec3 impulse)
{
    world->linearVelocities[index] =
        m3Add3(world->linearVelocities[index], m3MulSV3(world->invMass[index], impulse));
    ForceWake(world, index, impulse, (m3Vec3){0.0f, 0.0f, 0.0f});
}

void m3ApplyAngularImpulseInternal(m3World* world, int32_t index, m3Vec3 impulse)
{
    world->angularVelocities[index] =
        m3Add3(world->angularVelocities[index], m3MulMV3(m3WorldInvInertia(world, index), impulse));
    ForceWake(world, index, impulse, (m3Vec3){0.0f, 0.0f, 0.0f});
}

// The arm is measured from the CENTER OF MASS: an application at
// the COM adds no spin, wherever the body origin sits.
static m3Vec3 ForceArm(const m3World* world, int32_t index, m3Pos3 point)
{
    m3Vec3 rlc = m3RotateVec3(world->transforms[index].q, world->localCenters[index]);
    return (m3Vec3){(m3real)(point.x - world->transforms[index].p.x) - rlc.x,
                    (m3real)(point.y - world->transforms[index].p.y) - rlc.y,
                    (m3real)(point.z - world->transforms[index].p.z) - rlc.z};
}

void m3ApplyForceAtPointInternal(m3World* world, int32_t index, m3Vec3 force, m3Pos3 point)
{
    world->bodyForce[index] = m3Add3(world->bodyForce[index], force);
    world->bodyTorque[index] =
        m3Add3(world->bodyTorque[index], m3Cross3(ForceArm(world, index, point), force));
    ForceWake(world, index, force, (m3Vec3){0.0f, 0.0f, 0.0f});
}

void m3ApplyImpulseAtPointInternal(m3World* world, int32_t index, m3Vec3 impulse, m3Pos3 point)
{
    world->linearVelocities[index] =
        m3Add3(world->linearVelocities[index], m3MulSV3(world->invMass[index], impulse));
    world->angularVelocities[index] =
        m3Add3(world->angularVelocities[index],
               m3MulMV3(m3WorldInvInertia(world, index),
                        m3Cross3(ForceArm(world, index, point), impulse)));
    ForceWake(world, index, impulse, (m3Vec3){0.0f, 0.0f, 0.0f});
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
    // Hostile-input wall (2d-1): nothing non-finite reaches state,
    // and a rotation that is not close to unit is a corrupted def,
    // not a request (loud refusal beats silent renormalization).
    m3real qq = def->rotation.x * def->rotation.x + def->rotation.y * def->rotation.y +
                def->rotation.z * def->rotation.z + def->rotation.w * def->rotation.w;
    if (!m3FinitePos3(def->position) || !m3FiniteQuat(def->rotation) ||
        !m3FiniteV3(def->linearVelocity) || !m3FiniteV3(def->angularVelocity) ||
        !m3FiniteF(def->gravityScale) || !m3FiniteF(def->linearDamping) ||
        !m3FiniteF(def->angularDamping) || def->linearDamping < 0.0f ||
        def->angularDamping < 0.0f || !(qq > 0.98f) || !(qq < 1.02f))
    {
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
        return; // stale or foreign id: contract, not invariant
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

// The shared 8-2 wrapper skeleton: resolve, refuse hostiles and
// non-dynamic targets quietly, journal, apply through the one
// internal path replay uses.
void m3Body_ApplyForce(m3BodyId bodyId, m3Vec3 force)
{
    int32_t index;
    m3World* world = ResolveBody(bodyId, &index);
    if (world == NULL || !m3FiniteV3(force) || !ForceTargetValid(world, index))
    {
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
        record.v = force;
        m3JournalRecord(world, m3_opApplyForce, &record, (int32_t)sizeof(record));
    }
    m3ApplyForceInternal(world, index, force);
}

void m3Body_ApplyTorque(m3BodyId bodyId, m3Vec3 torque)
{
    int32_t index;
    m3World* world = ResolveBody(bodyId, &index);
    if (world == NULL || !m3FiniteV3(torque) || !ForceTargetValid(world, index))
    {
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
        record.v = torque;
        m3JournalRecord(world, m3_opApplyTorque, &record, (int32_t)sizeof(record));
    }
    m3ApplyTorqueInternal(world, index, torque);
}

void m3Body_ApplyLinearImpulse(m3BodyId bodyId, m3Vec3 impulse)
{
    int32_t index;
    m3World* world = ResolveBody(bodyId, &index);
    if (world == NULL || !m3FiniteV3(impulse) || !ForceTargetValid(world, index))
    {
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
        record.v = impulse;
        m3JournalRecord(world, m3_opApplyLinearImpulse, &record, (int32_t)sizeof(record));
    }
    m3ApplyLinearImpulseInternal(world, index, impulse);
}

void m3Body_ApplyAngularImpulse(m3BodyId bodyId, m3Vec3 impulse)
{
    int32_t index;
    m3World* world = ResolveBody(bodyId, &index);
    if (world == NULL || !m3FiniteV3(impulse) || !ForceTargetValid(world, index))
    {
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
        record.v = impulse;
        m3JournalRecord(world, m3_opApplyAngularImpulse, &record, (int32_t)sizeof(record));
    }
    m3ApplyAngularImpulseInternal(world, index, impulse);
}

void m3Body_ApplyForceAtPoint(m3BodyId bodyId, m3Vec3 force, m3Pos3 point)
{
    int32_t index;
    m3World* world = ResolveBody(bodyId, &index);
    if (world == NULL || !m3FiniteV3(force) || !m3FinitePos3(point) ||
        !ForceTargetValid(world, index))
    {
        return;
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m3BodyId id;
            m3Vec3 v;
            m3Pos3 p;
        } record;
        memset(&record, 0, sizeof(record));
        record.id = bodyId;
        record.v = force;
        record.p = point;
        m3JournalRecord(world, m3_opApplyForceAtPoint, &record, (int32_t)sizeof(record));
    }
    m3ApplyForceAtPointInternal(world, index, force, point);
}

void m3Body_ApplyLinearImpulseAtPoint(m3BodyId bodyId, m3Vec3 impulse, m3Pos3 point)
{
    int32_t index;
    m3World* world = ResolveBody(bodyId, &index);
    if (world == NULL || !m3FiniteV3(impulse) || !m3FinitePos3(point) ||
        !ForceTargetValid(world, index))
    {
        return;
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m3BodyId id;
            m3Vec3 v;
            m3Pos3 p;
        } record;
        memset(&record, 0, sizeof(record));
        record.id = bodyId;
        record.v = impulse;
        record.p = point;
        m3JournalRecord(world, m3_opApplyImpulseAtPoint, &record, (int32_t)sizeof(record));
    }
    m3ApplyImpulseAtPointInternal(world, index, impulse, point);
}

void m3Body_SetLinearVelocity(m3BodyId bodyId, m3Vec3 velocity)
{
    int32_t index;
    m3World* world = ResolveBody(bodyId, &index);
    if (world == NULL)
    {
        return; // stale or foreign id: contract, not invariant
    }
    if (!m3FiniteV3(velocity))
    {
        return; // hostile command: a documented no-op, never poison
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
        return; // stale or foreign id: contract, not invariant
    }
    if (!m3FiniteV3(velocity))
    {
        return; // hostile command: a documented no-op, never poison
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
