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

// Density-scaled mass, centroid, and centroid inertia of one shape.
// Returns 0 for shapes that carry no mass (planes).
static int ShapeMassProps(const m3World* world, int32_t s, float* massOut, m3Vec3* comOut,
                          m3Mat3* inertiaOut)
{
    uint8_t type = world->shapeType[s];
    if (type == (uint8_t)m3_sphereShape)
    {
        float r = world->shapeGeom[s].s;
        float m = world->shapeDensity[s] * (4.0f / 3.0f) * M3_PI * r * r * r;
        float ic = 0.4f * m * r * r;
        *massOut = m;
        *comOut = world->shapeGeom[s].v;
        *inertiaOut = m3MakeZeroMat3();
        inertiaOut->cx.x = ic;
        inertiaOut->cy.y = ic;
        inertiaOut->cz.z = ic;
        return 1;
    }
    if (type == (uint8_t)m3_capsuleShape)
    {
        // Closed form: a cylinder of length L plus two hemispheres.
        // About the COM (the segment midpoint), with u the unit axis:
        //   I = Iperp * Identity + (Iaxial - Iperp) * (u outer u)
        // because t1(x)t1 + t2(x)t2 = Identity - u(x)u for any
        // orthonormal basis {t1, u, t2}. No basis matrix needed and
        // the result is exactly symmetric.
        m3Vec3 p1 = world->shapeGeom[s].v;
        m3Vec3 p2 = world->shapeGeom[s].v2;
        float r = world->shapeGeom[s].s;
        m3Vec3 axis = m3Sub3(p2, p1);
        float length = sqrtf(m3Dot3(axis, axis));
        m3Vec3 u = m3MulSV3(1.0f / length, axis); // length > 0 by contract
        float density = world->shapeDensity[s];
        float mCyl = density * M3_PI * r * r * length;
        float mSph = density * (4.0f / 3.0f) * M3_PI * r * r * r;
        float axial = 0.5f * mCyl * r * r + 0.4f * mSph * r * r;
        float perp = mCyl * (length * length / 12.0f + 0.25f * r * r) +
                     mSph * (0.4f * r * r + 0.25f * length * length + 0.375f * length * r);
        *massOut = mCyl + mSph;
        *comOut = m3MulSV3(0.5f, m3Add3(p1, p2));
        m3Mat3 ic2 = m3MakeZeroMat3();
        float d = axial - perp;
        ic2.cx = (m3Vec3){perp + d * u.x * u.x, d * u.x * u.y, d * u.x * u.z};
        ic2.cy = (m3Vec3){d * u.x * u.y, perp + d * u.y * u.y, d * u.y * u.z};
        ic2.cz = (m3Vec3){d * u.x * u.z, d * u.y * u.z, perp + d * u.z * u.z};
        *inertiaOut = ic2;
        return 1;
    }
    if (type == (uint8_t)m3_hullShape)
    {
        const m3HullData* hull = &world->hullData[world->shapeHullIndex[s]];
        float density = world->shapeDensity[s];
        *massOut = density * hull->unitMass;
        *comOut = hull->unitCom;
        m3Mat3 ic = hull->unitInertiaCom;
        ic.cx = m3MulSV3(density, ic.cx);
        ic.cy = m3MulSV3(density, ic.cy);
        ic.cz = m3MulSV3(density, ic.cz);
        *inertiaOut = ic;
        return 1;
    }
    return 0;
}

void m3RecomputeMass(m3World* world, int32_t bodyIndex)
{
    if (world->types[bodyIndex] != (uint8_t)m3_dynamicBody)
    {
        world->invMass[bodyIndex] = 0.0f;
        world->invInertiaLocal[bodyIndex] = m3MakeZeroMat3();
        world->inertiaLocal[bodyIndex] = m3MakeZeroMat3();
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
        float m;
        m3Vec3 c;
        m3Mat3 ic;
        if (!ShapeMassProps(world, s, &m, &c, &ic))
        {
            continue;
        }
        mass += m;
        center = m3Add3(center, m3MulSV3(m, c));
    }
    if (!(mass > 0.0f))
    {
        // Shapeless dynamic body: unit mass, zero inertia (the
        // reference convention).
        world->invMass[bodyIndex] = 1.0f;
        world->invInertiaLocal[bodyIndex] = m3MakeZeroMat3();
        world->inertiaLocal[bodyIndex] = m3MakeZeroMat3();
        world->localCenters[bodyIndex] = (m3Vec3){0.0f, 0.0f, 0.0f};
        return;
    }
    center = m3MulSV3(1.0f / mass, center);

    m3Mat3 inertia = m3MakeZeroMat3();
    for (int32_t s = world->bodyShapeHead[bodyIndex]; s != -1; s = world->shapeNext[s])
    {
        float m;
        m3Vec3 c;
        m3Mat3 ic;
        if (!ShapeMassProps(world, s, &m, &c, &ic))
        {
            continue;
        }
        m3Vec3 d = m3Sub3(c, center);
        float d2 = m3Dot3(d, d);
        // I += Ic + m * (|d|^2 Identity - d outer d), the full 3D
        // parallel axis theorem, term by term non-negative diagonals.
        inertia.cx.x += ic.cx.x + m * (d2 - d.x * d.x);
        inertia.cy.y += ic.cy.y + m * (d2 - d.y * d.y);
        inertia.cz.z += ic.cz.z + m * (d2 - d.z * d.z);
        inertia.cy.x += ic.cy.x - m * d.x * d.y;
        inertia.cx.y += ic.cx.y - m * d.x * d.y;
        inertia.cz.x += ic.cz.x - m * d.x * d.z;
        inertia.cx.z += ic.cx.z - m * d.x * d.z;
        inertia.cz.y += ic.cz.y - m * d.y * d.z;
        inertia.cy.z += ic.cy.z - m * d.y * d.z;
    }
    world->invMass[bodyIndex] = 1.0f / mass;
    world->inertiaLocal[bodyIndex] = inertia; // the gyroscopic solve reads it
    world->invInertiaLocal[bodyIndex] = InvertSymmetric(inertia);
    world->localCenters[bodyIndex] = center;
}

int32_t m3CreateShapeInternal(m3World* world, int32_t bodyIndex, uint8_t type,
                              const m3ShapeGeom* geom, const m3ShapeDef* def,
                              const m3HullData* prebuilt)
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
    world->shapeHullIndex[index] = -1;
    if (type == (uint8_t)m3_hullShape)
    {
        // Boxes rebuild from geom.v (the journaled half extents);
        // general hulls arrive prebuilt from QuickHull (their recipe
        // rides the dedicated journal op instead).
        m3HullData data;
        if (prebuilt == NULL)
        {
            m3BuildBoxHull(&data, geom->v);
        }
        world->shapeHullIndex[index] = m3InternHull(world, prebuilt != NULL ? prebuilt : &data);
        if (world->shapeHullIndex[index] < 0)
        {
            world->bodyShapeHead[bodyIndex] = world->shapeNext[index];
            world->shapeNext[index] = -1;
            world->shapeBody[index] = -1;
            m3IdPoolFree(&world->shapePool, index);
            return -1;
        }
    }
    // Spheres and hulls enter the broadphase tree; infinite planes
    // stay out and take the dedicated pair pass.
    if (type != (uint8_t)m3_planeShape)
    {
        double lo[3];
        double hi[3];
        m3ShapeFatAabb(world, index, lo, hi);
        world->proxyIds[index] = m3TreeInsert(&world->tree, lo, hi, index);
        if (world->proxyIds[index] == M3_TREE_NULL)
        {
            // Tree pool exhausted: undo loudly, never a half-created
            // shape.
            m3ReleaseHull(world, world->shapeHullIndex[index]);
            world->shapeHullIndex[index] = -1;
            world->bodyShapeHead[bodyIndex] = world->shapeNext[index];
            world->shapeNext[index] = -1;
            world->shapeBody[index] = -1;
            m3IdPoolFree(&world->shapePool, index);
            return -1;
        }
    }
    else
    {
        world->proxyIds[index] = M3_TREE_NULL;
    }
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
    world->shapeGeom[index] = (m3ShapeGeom){{0.0f, 0.0f, 0.0f}, 0.0f, {0.0f, 0.0f, 0.0f}, 0.0f};
    world->shapeDensity[index] = 0.0f;
    world->shapeFriction[index] = 0.0f;
    world->shapeRestitution[index] = 0.0f;
    world->shapeUserData[index] = 0;
    world->shapeNext[index] = -1;
    if (world->proxyIds[index] != M3_TREE_NULL)
    {
        m3TreeRemove(&world->tree, world->proxyIds[index]);
        world->proxyIds[index] = M3_TREE_NULL;
    }
    m3ReleaseHull(world, world->shapeHullIndex[index]);
    world->shapeHullIndex[index] = -1;
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
    int32_t index = m3CreateShapeInternal(world, bodyIndex, type, geom, def, NULL);
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
    m3ShapeGeom geom = {sphere->center, sphere->radius, {0.0f, 0.0f, 0.0f}, 0.0f};
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
    m3ShapeGeom geom = {m3Normalize3(plane->normal), plane->offset, {0.0f, 0.0f, 0.0f}, 0.0f};
    return CreateShapeCommon(bodyId, def, (uint8_t)m3_planeShape, &geom);
}

m3ShapeId m3CreateCapsuleShape(m3BodyId bodyId, const m3ShapeDef* def, const m3Capsule* capsule)
{
    if (capsule == NULL || !(capsule->radius > 0.0f))
    {
        return m3_nullShapeId;
    }
    m3Vec3 axis = m3Sub3(capsule->point2, capsule->point1);
    if (!(m3Dot3(axis, axis) > 0.0f))
    {
        // A zero-length capsule is a sphere; asking for one is a
        // contract violation, refused loudly (use m3CreateSphereShape).
        return m3_nullShapeId;
    }
    m3ShapeGeom geom;
    geom.v = capsule->point1;
    geom.s = capsule->radius;
    geom.v2 = capsule->point2;
    geom.s2 = 0.0f;
    return CreateShapeCommon(bodyId, def, (uint8_t)m3_capsuleShape, &geom);
}

m3ShapeId m3CreateHullShape(m3BodyId bodyId, const m3ShapeDef* def, const m3Vec3* points,
                            int32_t count)
{
    m3World* world = m3WorldFromIndex0(bodyId.world0);
    int32_t bodyIndex = world != NULL ? m3BodySlot(world, bodyId) : -1;
    if (bodyIndex < 0 || def == NULL || def->internalValue != M3_SHAPE_COOKIE)
    {
        return m3_nullShapeId;
    }
    m3HullData data;
    if (!m3ComputeHull(points, count, &data))
    {
        // Degenerate cloud or over the caps: contract, null id.
        return m3_nullShapeId;
    }
    m3ShapeGeom geom;
    memset(&geom, 0, sizeof(geom));
    int32_t index =
        m3CreateShapeInternal(world, bodyIndex, (uint8_t)m3_hullShape, &geom, def, &data);
    if (index < 0)
    {
        return m3_nullShapeId;
    }
    m3ShapeId id = {index + 1, world->worldIndex0, world->shapePool.generations[index]};
    if (world->journalActive != 0)
    {
        m3CreateHullShapeOp record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.body = bodyId;
        record.expected = id;
        record.count = count;
        memcpy(record.points, points, (size_t)count * sizeof(m3Vec3));
        m3JournalRecord(world, m3_opCreateHullShape, &record, (int32_t)sizeof(record));
    }
    return id;
}

m3ShapeId m3CreateBoxShape(m3BodyId bodyId, const m3ShapeDef* def, m3Vec3 halfExtents)
{
    if (!(halfExtents.x > 0.0f) || !(halfExtents.y > 0.0f) || !(halfExtents.z > 0.0f))
    {
        return m3_nullShapeId; // contract: bad extents return null
    }
    m3ShapeGeom geom = {halfExtents, 0.0f, {0.0f, 0.0f, 0.0f}, 0.0f};
    return CreateShapeCommon(bodyId, def, (uint8_t)m3_hullShape, &geom);
}

bool m3Shape_IsValid(m3ShapeId shapeId)
{
    m3World* world = m3WorldFromIndex0(shapeId.world0);
    return world != NULL && m3ShapeSlot(world, shapeId) >= 0;
}
