// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Broadphase v1: a canonical brute-force scan behind the swappable
// module contract (fill pairKeys in ascending key order). Sphere
// counts in 2a are small, so O(n^2) is honest and simple; the dynamic
// tree adapted from the reference replaces this in 2b without anyone
// downstream noticing. AABBs are computed in double (positions are
// double) so distance from the origin never degrades the test.

#include "world_internal.h"

typedef struct m3Aabb3d
{
    double lo[3];
    double hi[3];
} m3Aabb3d;

static m3Aabb3d SphereAabb(const m3World* world, int32_t shape)
{
    int32_t body = world->shapeBody[shape];
    const m3Transform* xf = &world->transforms[body];
    m3Vec3 local = world->shapeGeom[shape].v;
    m3Vec3 r = m3RotateVec3(xf->q, local);
    double cx = xf->p.x + (double)r.x;
    double cy = xf->p.y + (double)r.y;
    double cz = xf->p.z + (double)r.z;
    double fat = (double)(world->shapeGeom[shape].s + M3_AABB_MARGIN);
    m3Aabb3d box = {{cx - fat, cy - fat, cz - fat}, {cx + fat, cy + fat, cz + fat}};
    return box;
}

static int Overlap(const m3Aabb3d* a, const m3Aabb3d* b)
{
    return a->lo[0] <= b->hi[0] && b->lo[0] <= a->hi[0] && a->lo[1] <= b->hi[1] &&
           b->lo[1] <= a->hi[1] && a->lo[2] <= b->hi[2] && b->lo[2] <= a->hi[2];
}

m3Result m3UpdatePairs(m3World* world)
{
    world->pairCount = 0;
    int32_t maxShape = world->shapePool.maxIndex;
    for (int32_t i = 0; i < maxShape; ++i)
    {
        if (world->shapePool.alive[i] == 0)
        {
            continue;
        }
        for (int32_t j = i + 1; j < maxShape; ++j)
        {
            if (world->shapePool.alive[j] == 0)
            {
                continue;
            }
            int32_t bodyI = world->shapeBody[i];
            int32_t bodyJ = world->shapeBody[j];
            if (bodyI == bodyJ)
            {
                continue; // a body never collides with itself
            }
            if (world->types[bodyI] == (uint8_t)m3_staticBody &&
                world->types[bodyJ] == (uint8_t)m3_staticBody)
            {
                continue; // two statics never make a contact
            }

            uint8_t typeI = world->shapeType[i];
            uint8_t typeJ = world->shapeType[j];
            int candidate;
            if (typeI == (uint8_t)m3_planeShape || typeJ == (uint8_t)m3_planeShape)
            {
                // A half-space is infinite: every sphere is a
                // candidate. Plane-plane never pairs (both static).
                candidate = typeI != typeJ;
            }
            else
            {
                m3Aabb3d a = SphereAabb(world, i);
                m3Aabb3d b = SphereAabb(world, j);
                candidate = Overlap(&a, &b);
            }
            if (!candidate)
            {
                continue;
            }
            if (world->pairCount == world->pairCapacity)
            {
                // Loud capacity failure: the step reports it and the
                // world grows the pool between steps. Never silent.
                return m3_errorCapacity;
            }
            // i < j and the outer loop ascends, so the key stream is
            // canonical ascending by construction: no sort, and no
            // dependence on any sort implementation.
            world->pairKeys[world->pairCount] = ((uint64_t)i << 32) | (uint64_t)j;
            world->pairCount += 1;
        }
    }
    return m3_success;
}
