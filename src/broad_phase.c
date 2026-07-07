// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Broadphase v2: the fat-AABB dynamic tree behind the same module
// contract as the 2a scan (pairs out in canonical ascending key
// order). Planes are infinite, so they stay out of the tree and take
// a dedicated pass; spheres refresh their proxies in shape order,
// query the tree, and every emitted key list is sorted once at the
// end (unique keys, so any correct sort yields the identical result).
// The 2a brute-force scan stays below as the referee: on any scene
// both paths must produce the same list, and a test holds that gate.

#include "world_internal.h"

#include <stdlib.h>

typedef struct m3Aabb3d
{
    double lo[3];
    double hi[3];
} m3Aabb3d;

static m3Aabb3d SphereAabb(const m3World* world, int32_t shape)
{
    int32_t body = world->shapeBody[shape];
    const m3Transform* xf = &world->transforms[body];

    if (world->shapeType[shape] == (uint8_t)m3_hullShape)
    {
        // Hull bounds: rotate every vertex, min and max in double.
        const m3HullData* hull = &world->hullData[world->shapeHullIndex[shape]];
        m3Aabb3d box = {{1.0e30, 1.0e30, 1.0e30}, {-1.0e30, -1.0e30, -1.0e30}};
        for (int32_t v = 0; v < hull->vertexCount; ++v)
        {
            m3Vec3 r = m3RotateVec3(xf->q, hull->vertices[v]);
            double p[3] = {xf->p.x + (double)r.x, xf->p.y + (double)r.y, xf->p.z + (double)r.z};
            for (int32_t k = 0; k < 3; ++k)
            {
                box.lo[k] = p[k] < box.lo[k] ? p[k] : box.lo[k];
                box.hi[k] = p[k] > box.hi[k] ? p[k] : box.hi[k];
            }
        }
        for (int32_t k = 0; k < 3; ++k)
        {
            box.lo[k] -= (double)M3_AABB_MARGIN;
            box.hi[k] += (double)M3_AABB_MARGIN;
        }
        return box;
    }

    if (world->shapeType[shape] == (uint8_t)m3_meshShape)
    {
        // Mesh bounds: min and max over all vertices in double.
        const m3MeshData* mesh = &world->meshData[world->shapeMeshIndex[shape]];
        m3Aabb3d box = {{1.0e30, 1.0e30, 1.0e30}, {-1.0e30, -1.0e30, -1.0e30}};
        for (int32_t v = 0; v < mesh->vertexCount; ++v)
        {
            m3Vec3 r = m3RotateVec3(xf->q, mesh->vertices[v]);
            double p[3] = {xf->p.x + (double)r.x, xf->p.y + (double)r.y, xf->p.z + (double)r.z};
            for (int32_t k = 0; k < 3; ++k)
            {
                box.lo[k] = p[k] < box.lo[k] ? p[k] : box.lo[k];
                box.hi[k] = p[k] > box.hi[k] ? p[k] : box.hi[k];
            }
        }
        for (int32_t k = 0; k < 3; ++k)
        {
            box.lo[k] -= (double)M3_AABB_MARGIN;
            box.hi[k] += (double)M3_AABB_MARGIN;
        }
        return box;
    }

    if (world->shapeType[shape] == (uint8_t)m3_capsuleShape)
    {
        // Capsule bounds: the two cap-center spheres, min and max in
        // double, same fattening as everything else.
        m3Vec3 r1 = m3RotateVec3(xf->q, world->shapeGeom[shape].v);
        m3Vec3 r2 = m3RotateVec3(xf->q, world->shapeGeom[shape].v2);
        double c1[3] = {xf->p.x + (double)r1.x, xf->p.y + (double)r1.y, xf->p.z + (double)r1.z};
        double c2[3] = {xf->p.x + (double)r2.x, xf->p.y + (double)r2.y, xf->p.z + (double)r2.z};
        double fat = (double)(world->shapeGeom[shape].s + M3_AABB_MARGIN);
        m3Aabb3d box;
        for (int32_t k = 0; k < 3; ++k)
        {
            box.lo[k] = (c1[k] < c2[k] ? c1[k] : c2[k]) - fat;
            box.hi[k] = (c1[k] > c2[k] ? c1[k] : c2[k]) + fat;
        }
        return box;
    }

    m3Vec3 local = world->shapeGeom[shape].v;
    m3Vec3 r = m3RotateVec3(xf->q, local);
    double cx = xf->p.x + (double)r.x;
    double cy = xf->p.y + (double)r.y;
    double cz = xf->p.z + (double)r.z;
    double fat = (double)(world->shapeGeom[shape].s + M3_AABB_MARGIN);
    m3Aabb3d box = {{cx - fat, cy - fat, cz - fat}, {cx + fat, cy + fat, cz + fat}};
    return box;
}

void m3ShapeFatAabb(const m3World* world, int32_t shape, double lo[3], double hi[3])
{
    m3Aabb3d box = SphereAabb(world, shape);
    lo[0] = box.lo[0];
    lo[1] = box.lo[1];
    lo[2] = box.lo[2];
    hi[0] = box.hi[0];
    hi[1] = box.hi[1];
    hi[2] = box.hi[2];
}

static int Overlap(const m3Aabb3d* a, const m3Aabb3d* b)
{
    return a->lo[0] <= b->hi[0] && b->lo[0] <= a->hi[0] && a->lo[1] <= b->hi[1] &&
           b->lo[1] <= a->hi[1] && a->lo[2] <= b->hi[2] && b->lo[2] <= a->hi[2];
}

// Shared pair filter: no self pairs, no static-static pairs.
static int PairAllowed(const m3World* world, int32_t i, int32_t j)
{
    int32_t bodyI = world->shapeBody[i];
    int32_t bodyJ = world->shapeBody[j];
    if (bodyI == bodyJ)
    {
        return 0;
    }
    if (world->types[bodyI] == (uint8_t)m3_staticBody &&
        world->types[bodyJ] == (uint8_t)m3_staticBody)
    {
        return 0;
    }
    return 1;
}

static int EmitPair(m3World* world, int32_t i, int32_t j)
{
    if (world->pairCount == world->pairCapacity)
    {
        return 0; // loud at the caller
    }
    uint64_t key =
        i < j ? (((uint64_t)i << 32) | (uint64_t)j) : (((uint64_t)j << 32) | (uint64_t)i);
    world->pairKeys[world->pairCount] = key;
    world->pairCount += 1;
    return 1;
}

static int CompareKeys(const void* a, const void* b)
{
    uint64_t ka = *(const uint64_t*)a;
    uint64_t kb = *(const uint64_t*)b;
    return ka < kb ? -1 : (ka > kb ? 1 : 0);
}

typedef struct m3QueryCtx
{
    m3World* world;
    m3Aabb3d selfBounds;
    int32_t self;
    int32_t overflow;
} m3QueryCtx;

static bool QueryHit(int32_t other, void* context)
{
    m3QueryCtx* ctx = (m3QueryCtx*)context;
    // Emit each pair once: only when the other leaf has the larger
    // index (both directions get queried, one emits).
    if (other <= ctx->self || !PairAllowed(ctx->world, ctx->self, other))
    {
        return true;
    }
    // The stored leaf bounds can be stale-but-containing (a leaf only
    // moves when its fresh bounds escape), so the tree can return a
    // SUPERSET of the true fat overlaps. Re-test with fresh bounds so
    // the pair list equals the brute-force referee STRUCTURALLY, not
    // by luck.
    m3Aabb3d fresh = SphereAabb(ctx->world, other);
    if (!Overlap(&ctx->selfBounds, &fresh))
    {
        return true;
    }
    if (!EmitPair(ctx->world, ctx->self, other))
    {
        ctx->overflow = 1;
        return false;
    }
    return true;
}

m3Result m3UpdatePairs(m3World* world)
{
    world->pairCount = 0;
    int32_t maxShape = world->shapePool.maxIndex;

    // Refresh proxies in shape order: a leaf moves only when its tight
    // bounds escape the fat bounds, so the tree shape (and therefore
    // everything downstream) is a pure function of the op history.
    for (int32_t i = 0; i < maxShape; ++i)
    {
        if (world->shapePool.alive[i] == 0 || world->proxyIds[i] == M3_TREE_NULL)
        {
            continue;
        }
        m3Aabb3d tight = SphereAabb(world, i);
        if (!m3TreeContains(&world->tree, world->proxyIds[i], tight.lo, tight.hi))
        {
            m3TreeRemove(&world->tree, world->proxyIds[i]);
            world->proxyIds[i] = m3TreeInsert(&world->tree, tight.lo, tight.hi, i);
            if (world->proxyIds[i] == M3_TREE_NULL)
            {
                return m3_errorCapacity;
            }
        }
    }

    // Plane pass: infinite shapes pair with every allowed sphere.
    for (int32_t p = 0; p < maxShape; ++p)
    {
        if (world->shapePool.alive[p] == 0 || world->shapeType[p] != (uint8_t)m3_planeShape)
        {
            continue;
        }
        for (int32_t s = 0; s < maxShape; ++s)
        {
            if (world->shapePool.alive[s] == 0 || world->shapeType[s] == (uint8_t)m3_planeShape ||
                !PairAllowed(world, p, s))
            {
                continue;
            }
            if (!EmitPair(world, p, s))
            {
                return m3_errorCapacity;
            }
        }
    }

    // Tree pass: each sphere queries the tree; the j > i rule emits
    // every overlap exactly once.
    for (int32_t i = 0; i < maxShape; ++i)
    {
        if (world->shapePool.alive[i] == 0 || world->proxyIds[i] == M3_TREE_NULL)
        {
            continue;
        }
        m3Aabb3d fat = SphereAabb(world, i);
        m3QueryCtx ctx;
        ctx.world = world;
        ctx.selfBounds = fat;
        ctx.self = i;
        ctx.overflow = 0;
        m3TreeQuery(&world->tree, fat.lo, fat.hi, QueryHit, &ctx);
        if (ctx.overflow != 0)
        {
            return m3_errorCapacity;
        }
    }

    // One sort restores the canonical ascending order. Keys are
    // unique, so the result is independent of the sort implementation.
    qsort(world->pairKeys, (size_t)world->pairCount, sizeof(uint64_t), CompareKeys);
    return m3_success;
}

// The 2a scan, verbatim in behavior: the referee the tree must match.
m3Result m3UpdatePairsBruteForce(m3World* world)
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
            if (world->shapePool.alive[j] == 0 || !PairAllowed(world, i, j))
            {
                continue;
            }
            uint8_t typeI = world->shapeType[i];
            uint8_t typeJ = world->shapeType[j];
            int candidate;
            if (typeI == (uint8_t)m3_planeShape || typeJ == (uint8_t)m3_planeShape)
            {
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
            if (!EmitPair(world, i, j))
            {
                return m3_errorCapacity;
            }
        }
    }
    return m3_success;
}
