// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Narrowphase v1: sphere-sphere and plane-sphere manifolds, the
// speculative margin, the deterministic tangent basis, and the
// warm-start carry. The collide kernels are pure functions of their
// inputs (the reference law); m3UpdateContacts walks the pairs in
// canonical order and matches impulses forward by feature id.

#include "world_internal.h"

#include <string.h>

// Contacts exist slightly before touch so the solver can catch
// approaches speculatively (the reference model).
#define M3_SPECULATIVE_DISTANCE M3_AABB_MARGIN

void m3MakeTangentBasis(m3Vec3 normal, m3Vec3* t1, m3Vec3* t2)
{
    m3real ax = m3AbsF(normal.x);
    m3real ay = m3AbsF(normal.y);
    m3real az = m3AbsF(normal.z);
    m3Vec3 axis;
    if (ax <= ay && ax <= az)
    {
        axis = (m3Vec3){1.0f, 0.0f, 0.0f};
    }
    else if (ay <= az)
    {
        axis = (m3Vec3){0.0f, 1.0f, 0.0f};
    }
    else
    {
        axis = (m3Vec3){0.0f, 0.0f, 1.0f};
    }
    *t1 = m3Normalize3(m3Cross3(normal, axis));
    *t2 = m3Cross3(normal, *t1);
}

m3Manifold m3CollideSpheres(m3Vec3 d, m3real radiusA, m3real radiusB)
{
    m3Manifold manifold;
    memset(&manifold, 0, sizeof(manifold));

    m3real distance = m3Length3(d);
    m3real separation = distance - radiusA - radiusB;
    if (separation > M3_SPECULATIVE_DISTANCE)
    {
        return manifold;
    }
    // Concentric centers take the fixed +y fallback (one rule, never
    // NaN, never caller-dependent).
    m3Vec3 normal = m3Normalize3(d);
    manifold.normal = normal;
    manifold.pointCount = 1;
    manifold.points[0].anchorA = m3MulSV3(radiusA, normal);
    manifold.points[0].anchorB = m3MulSV3(-radiusB, normal);
    manifold.points[0].separation = separation;
    manifold.points[0].id = 0;
    return manifold;
}

m3Manifold m3CollidePlaneSphere(m3Vec3 planeNormal, m3real dist, m3real radius)
{
    m3Manifold manifold;
    memset(&manifold, 0, sizeof(manifold));

    m3real separation = dist - radius;
    if (separation > M3_SPECULATIVE_DISTANCE)
    {
        return manifold;
    }
    manifold.normal = planeNormal; // A (plane) to B (sphere)
    manifold.pointCount = 1;
    // The sphere's deepest point toward the plane; anchorA is filled
    // by the contact update, which knows body A's center.
    manifold.points[0].anchorB = m3MulSV3(-radius, planeNormal);
    manifold.points[0].separation = separation;
    manifold.points[0].id = 0;
    return manifold;
}

// World center of a shape's sphere (double positions, float offsets).
static void SphereWorldCenter(const m3World* world, int32_t shape, double* cx, double* cy,
                              double* cz)
{
    int32_t body = world->shapeBody[shape];
    const m3Transform* xf = &world->transforms[body];
    m3Vec3 r = m3RotateVec3(xf->q, world->shapeGeom[shape].v);
    *cx = xf->p.x + (double)r.x;
    *cy = xf->p.y + (double)r.y;
    *cz = xf->p.z + (double)r.z;
}

// Offset from a body's world center of mass to a world point (float
// is exact enough near contact). Anchors are COM-relative because
// impulses and rotation act about the COM (2b-1).
static m3Vec3 FromCom(const m3World* world, int32_t body, double px, double py, double pz)
{
    const m3Transform* xf = &world->transforms[body];
    m3Vec3 rlc = m3RotateVec3(xf->q, world->localCenters[body]);
    return (m3Vec3){(m3real)(px - xf->p.x - (double)rlc.x), (m3real)(py - xf->p.y - (double)rlc.y),
                    (m3real)(pz - xf->p.z - (double)rlc.z)};
}

m3Result m3UpdateContacts(m3World* world, const uint64_t* oldKeys, const m3Manifold* oldManifolds,
                          int32_t oldCount)
{
    // Rebuild manifolds in pair order, carrying impulses forward by
    // feature id from the caller's stash. The old keys are sorted
    // (canonical order), so the lookup is a binary search.
    for (int32_t i = 0; i < world->pairCount; ++i)
    {
        uint64_t key = world->pairKeys[i];
        int32_t shapeA = (int32_t)(key >> 32);
        int32_t shapeB = (int32_t)(key & 0xFFFFFFFFu);
        uint8_t typeA = world->shapeType[shapeA];
        uint8_t typeB = world->shapeType[shapeB];

        m3Manifold fresh;
        if (typeA == (uint8_t)m3_planeShape || typeB == (uint8_t)m3_planeShape)
        {
            // Canonical orientation: the plane plays A. If the sphere
            // has the lower index the manifold flips on the way out.
            int32_t planeShape = typeA == (uint8_t)m3_planeShape ? shapeA : shapeB;
            int32_t sphereShape = planeShape == shapeA ? shapeB : shapeA;
            m3Vec3 n = world->shapeGeom[planeShape].v;
            m3real offset = world->shapeGeom[planeShape].s;
            double cx;
            double cy;
            double cz;
            SphereWorldCenter(world, sphereShape, &cx, &cy, &cz);
            double distD = (double)n.x * cx + (double)n.y * cy + (double)n.z * cz - (double)offset;
            fresh = m3CollidePlaneSphere(n, (m3real)distD, world->shapeGeom[sphereShape].s);
            if (fresh.pointCount > 0)
            {
                // anchorA: from the plane BODY's origin to the contact
                // point (the sphere's deepest point projected).
                int32_t planeBody = world->shapeBody[planeShape];
                int32_t ballBody = world->shapeBody[sphereShape];
                double px = cx - (double)(fresh.normal.x * (m3real)distD);
                double py = cy - (double)(fresh.normal.y * (m3real)distD);
                double pz = cz - (double)(fresh.normal.z * (m3real)distD);
                fresh.points[0].anchorA = FromCom(world, planeBody, px, py, pz);
                fresh.points[0].anchorB =
                    m3Add3(FromCom(world, ballBody, cx, cy, cz), fresh.points[0].anchorB);
                if (planeShape != shapeA)
                {
                    // Flip to keep the manifold in key order (A = the
                    // lower shape index, always).
                    m3Vec3 tmp = fresh.points[0].anchorA;
                    fresh.points[0].anchorA = fresh.points[0].anchorB;
                    fresh.points[0].anchorB = tmp;
                    fresh.normal = m3Neg3(fresh.normal);
                }
            }
        }
        else
        {
            double ax;
            double ay;
            double az;
            double bx;
            double by;
            double bz;
            SphereWorldCenter(world, shapeA, &ax, &ay, &az);
            SphereWorldCenter(world, shapeB, &bx, &by, &bz);
            m3Vec3 d = {(m3real)(bx - ax), (m3real)(by - ay), (m3real)(bz - az)};
            fresh = m3CollideSpheres(d, world->shapeGeom[shapeA].s, world->shapeGeom[shapeB].s);
            if (fresh.pointCount > 0)
            {
                // Kernel anchors are from the sphere CENTERS; re-base
                // them to each body's COM.
                fresh.points[0].anchorA = m3Add3(
                    FromCom(world, world->shapeBody[shapeA], ax, ay, az), fresh.points[0].anchorA);
                fresh.points[0].anchorB = m3Add3(
                    FromCom(world, world->shapeBody[shapeB], bx, by, bz), fresh.points[0].anchorB);
            }
        }

        // Warm-start carry: find the old manifold for this key and
        // match points by feature id.
        if (fresh.pointCount > 0 && oldCount > 0)
        {
            int32_t lo = 0;
            int32_t hi = oldCount - 1;
            while (lo <= hi)
            {
                int32_t mid = (lo + hi) / 2;
                if (oldKeys[mid] == key)
                {
                    const m3Manifold* previous = &oldManifolds[mid];
                    for (int32_t k = 0; k < fresh.pointCount; ++k)
                    {
                        for (int32_t o = 0; o < previous->pointCount; ++o)
                        {
                            if (previous->points[o].id == fresh.points[k].id)
                            {
                                fresh.points[k].normalImpulse = previous->points[o].normalImpulse;
                                fresh.points[k].tangentImpulse1 =
                                    previous->points[o].tangentImpulse1;
                                fresh.points[k].tangentImpulse2 =
                                    previous->points[o].tangentImpulse2;
                                fresh.points[k].flags |= 1; // persisted
                                break;
                            }
                        }
                    }
                    break;
                }
                if (oldKeys[mid] < key)
                {
                    lo = mid + 1;
                }
                else
                {
                    hi = mid - 1;
                }
            }
        }
        world->manifolds[i] = fresh;
    }
    return m3_success;
}
