// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Shapes: spheres and static half-spaces, their mass, and the body
// shape lists. Same law as bodies: public functions validate and
// journal, internal functions mutate, replay drives the internals.

#include "world_internal.h"

#include <string.h>

int32_t m3ShapeSlot(const m3World* world, m3ShapeId shapeId)
{
    int32_t index = shapeId.index1 - 1;
    if (world == NULL || shapeId.world0 != world->worldIndex0 ||
        !m3IdPoolValid(&world->shapePool, index, shapeId.generation))
    {
        return -1;
    }
    return index;
}

m3ShapeDef m3DefaultShapeDef(void)
{
    m3ShapeDef def;
    memset(&def, 0, sizeof(def));
    def.density = 1.0f;
    def.friction = 0.6f;
    def.restitution = 0.0f;
    def.internalValue = M3_SHAPE_COOKIE;
    return def;
}

void m3RecomputeMass(m3World* world, int32_t bodyIndex)
{
    if (world->types[bodyIndex] != (uint8_t)m3_dynamicBody)
    {
        world->invMass[bodyIndex] = 0.0f;
        world->invInertia[bodyIndex] = 0.0f;
        return;
    }
    // Sum sphere masses and inertias. 2a spheres are pinned to the
    // body origin, so the scalar (isotropic) inertia is exact and no
    // parallel-axis term exists; the full tensor arrives in 2b.
    float mass = 0.0f;
    float inertia = 0.0f;
    for (int32_t s = world->bodyShapeHead[bodyIndex]; s != -1; s = world->shapeNext[s])
    {
        if (world->shapeType[s] != (uint8_t)m3_sphereShape)
        {
            continue;
        }
        float r = world->shapeGeom[s].s;
        float m = world->shapeDensity[s] * (4.0f / 3.0f) * M3_PI * r * r * r;
        mass += m;
        inertia += 0.4f * m * r * r;
    }
    if (mass > 0.0f)
    {
        world->invMass[bodyIndex] = 1.0f / mass;
        world->invInertia[bodyIndex] = inertia > 0.0f ? 1.0f / inertia : 0.0f;
    }
    else
    {
        // Shapeless dynamic body: unit mass, zero inertia (the
        // reference convention).
        world->invMass[bodyIndex] = 1.0f;
        world->invInertia[bodyIndex] = 0.0f;
    }
}

int32_t m3CreateShapeInternal(m3World* world, int32_t bodyIndex, uint8_t type,
                              const m3ShapeGeom* geom, const m3ShapeDef* def)
{
    int32_t index = m3IdPoolAlloc(&world->shapePool);
    if (index < 0)
    {
        return -1;
    }
    world->shapeBody[index] = bodyIndex;
    world->shapeType[index] = type;
    world->shapeGeom[index] = *geom;
    world->shapeDensity[index] = def->density;
    world->shapeFriction[index] = def->friction;
    world->shapeRestitution[index] = def->restitution;
    world->shapeUserData[index] = def->userData;
    // Push onto the body's list head (canonical: creation order is
    // recoverable because replay recreates in the same order).
    world->shapeNext[index] = world->bodyShapeHead[bodyIndex];
    world->bodyShapeHead[bodyIndex] = index;
    m3RecomputeMass(world, bodyIndex);
    return index;
}

void m3DestroyShapeInternal(m3World* world, int32_t index)
{
    int32_t bodyIndex = world->shapeBody[index];
    // Unlink from the body's list.
    int32_t* cursor = &world->bodyShapeHead[bodyIndex];
    while (*cursor != -1)
    {
        if (*cursor == index)
        {
            *cursor = world->shapeNext[index];
            break;
        }
        cursor = &world->shapeNext[*cursor];
    }
    world->shapeBody[index] = -1;
    world->shapeType[index] = 0;
    world->shapeGeom[index] = (m3ShapeGeom){{0.0f, 0.0f, 0.0f}, 0.0f};
    world->shapeDensity[index] = 0.0f;
    world->shapeFriction[index] = 0.0f;
    world->shapeRestitution[index] = 0.0f;
    world->shapeUserData[index] = 0;
    world->shapeNext[index] = -1;
    m3IdPoolFree(&world->shapePool, index);
    m3RecomputeMass(world, bodyIndex);
}

static m3ShapeId CreateShapeCommon(m3BodyId bodyId, const m3ShapeDef* def, uint8_t type,
                                   const m3ShapeGeom* geom)
{
    m3World* world = m3WorldFromIndex0(bodyId.world0);
    int32_t bodyIndex = world != NULL ? m3BodySlot(world, bodyId) : -1;
    if (bodyIndex < 0 || def == NULL || def->internalValue != M3_SHAPE_COOKIE)
    {
        // Contract, not invariant: bad input returns the null id.
        return m3_nullShapeId;
    }
    int32_t index = m3CreateShapeInternal(world, bodyIndex, type, geom, def);
    if (index < 0)
    {
        return m3_nullShapeId;
    }
    m3ShapeId id = {index + 1, world->worldIndex0, world->shapePool.generations[index]};
    if (world->journalActive != 0)
    {
        m3CreateShapeOp record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.geom = *geom;
        record.body = bodyId;
        record.expected = id;
        record.type = type;
        m3JournalRecord(world, m3_opCreateShape, &record, (int32_t)sizeof(record));
    }
    return id;
}

m3ShapeId m3CreateSphereShape(m3BodyId bodyId, const m3ShapeDef* def, const m3Sphere* sphere)
{
    if (sphere == NULL || !(sphere->radius > 0.0f))
    {
        return m3_nullShapeId;
    }
    m3World* world = m3WorldFromIndex0(bodyId.world0);
    int32_t bodyIndex = world != NULL ? m3BodySlot(world, bodyId) : -1;
    if (bodyIndex < 0)
    {
        return m3_nullShapeId;
    }
    if (world->types[bodyIndex] == (uint8_t)m3_dynamicBody &&
        (sphere->center.x != 0.0f || sphere->center.y != 0.0f || sphere->center.z != 0.0f))
    {
        // The 2a rule: an off-origin center on a dynamic body would
        // need the full inertia tensor (2b). Refuse loudly rather
        // than simulate wrong.
        return m3_nullShapeId;
    }
    m3ShapeGeom geom = {sphere->center, sphere->radius};
    return CreateShapeCommon(bodyId, def, (uint8_t)m3_sphereShape, &geom);
}

m3ShapeId m3CreatePlaneShape(m3BodyId bodyId, const m3ShapeDef* def, const m3Plane* plane)
{
    if (plane == NULL)
    {
        return m3_nullShapeId;
    }
    m3World* world = m3WorldFromIndex0(bodyId.world0);
    int32_t bodyIndex = world != NULL ? m3BodySlot(world, bodyId) : -1;
    if (bodyIndex < 0 || world->types[bodyIndex] != (uint8_t)m3_staticBody)
    {
        // A plane on a dynamic body is refused loudly: an infinite
        // shape has no mass.
        return m3_nullShapeId;
    }
    m3ShapeGeom geom = {m3Normalize3(plane->normal), plane->offset};
    return CreateShapeCommon(bodyId, def, (uint8_t)m3_planeShape, &geom);
}

bool m3Shape_IsValid(m3ShapeId shapeId)
{
    m3World* world = m3WorldFromIndex0(shapeId.world0);
    return world != NULL && m3ShapeSlot(world, shapeId) >= 0;
}
