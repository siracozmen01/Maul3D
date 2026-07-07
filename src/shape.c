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

// Invert a symmetric positive-definite 3x3 via the adjugate. A
// singular or non-positive matrix returns zero (an unrotatable body),
// never NaN.
static m3Mat3 InvertSymmetric(m3Mat3 m)
{
    m3real a = m.cx.x;
    m3real b = m.cy.x; // = m.cx.y by symmetry
    m3real c = m.cz.x;
    m3real d = m.cy.y;
    m3real e = m.cz.y;
    m3real f = m.cz.z;
    m3real co00 = d * f - e * e;
    m3real co01 = c * e - b * f;
    m3real co02 = b * e - c * d;
    m3real det = a * co00 + b * co01 + c * co02;
    if (!(det > 0.0f))
    {
        return m3MakeZeroMat3();
    }
    m3real inv = 1.0f / det;
    m3Mat3 r;
    r.cx = (m3Vec3){co00 * inv, co01 * inv, co02 * inv};
    r.cy = (m3Vec3){co01 * inv, (a * f - c * c) * inv, (b * c - a * e) * inv};
    r.cz = (m3Vec3){co02 * inv, (b * c - a * e) * inv, (a * d - b * b) * inv};
    return r;
}

void m3RecomputeMass(m3World* world, int32_t bodyIndex)
{
    if (world->types[bodyIndex] != (uint8_t)m3_dynamicBody)
    {
        world->invMass[bodyIndex] = 0.0f;
        world->invInertiaLocal[bodyIndex] = m3MakeZeroMat3();
        world->localCenters[bodyIndex] = (m3Vec3){0.0f, 0.0f, 0.0f};
        return;
    }
    // Two passes, the Maul2D lesson: first total mass and the mass
    // weighted center, THEN inertia about that center via the parallel
    // axis theorem. Every term is non-negative and small; no
    // big-minus-big cancellation can occur.
    float mass = 0.0f;
    m3Vec3 center = {0.0f, 0.0f, 0.0f};
    for (int32_t s = world->bodyShapeHead[bodyIndex]; s != -1; s = world->shapeNext[s])
    {
        if (world->shapeType[s] != (uint8_t)m3_sphereShape)
        {
            continue;
        }
        float r = world->shapeGeom[s].s;
        float m = world->shapeDensity[s] * (4.0f / 3.0f) * M3_PI * r * r * r;
        mass += m;
        center = m3Add3(center, m3MulSV3(m, world->shapeGeom[s].v));
    }
    if (!(mass > 0.0f))
    {
        // Shapeless dynamic body: unit mass, zero inertia (the
        // reference convention).
        world->invMass[bodyIndex] = 1.0f;
        world->invInertiaLocal[bodyIndex] = m3MakeZeroMat3();
        world->localCenters[bodyIndex] = (m3Vec3){0.0f, 0.0f, 0.0f};
        return;
    }
    center = m3MulSV3(1.0f / mass, center);

    m3Mat3 inertia = m3MakeZeroMat3();
    for (int32_t s = world->bodyShapeHead[bodyIndex]; s != -1; s = world->shapeNext[s])
    {
        if (world->shapeType[s] != (uint8_t)m3_sphereShape)
        {
            continue;
        }
        float r = world->shapeGeom[s].s;
        float m = world->shapeDensity[s] * (4.0f / 3.0f) * M3_PI * r * r * r;
        float ic = 0.4f * m * r * r; // solid sphere about its centroid
        m3Vec3 d = m3Sub3(world->shapeGeom[s].v, center);
        float d2 = m3Dot3(d, d);
        // I += ic * Identity + m * (|d|^2 Identity - d outer d).
        inertia.cx.x += ic + m * (d2 - d.x * d.x);
        inertia.cy.y += ic + m * (d2 - d.y * d.y);
        inertia.cz.z += ic + m * (d2 - d.z * d.z);
        inertia.cy.x += -m * d.x * d.y;
        inertia.cx.y += -m * d.x * d.y;
        inertia.cz.x += -m * d.x * d.z;
        inertia.cx.z += -m * d.x * d.z;
        inertia.cz.y += -m * d.y * d.z;
        inertia.cy.z += -m * d.y * d.z;
    }
    world->invMass[bodyIndex] = 1.0f / mass;
    world->invInertiaLocal[bodyIndex] = InvertSymmetric(inertia);
    world->localCenters[bodyIndex] = center;
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
    // The 2a off-origin refusal is gone: the full inertia tensor and
    // center-of-mass bookkeeping (2b-1) make offset spheres exact.
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
