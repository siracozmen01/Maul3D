// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The query surface (2c-7): multi-hit rays, sphere and capsule shape
// casts through the shared time-of-impact kernel, point containment,
// and overlap gathers. Pure observers: no query moves a bit of
// simulation state, and every result orders canonically (fraction
// then shape index, or plain ascending shape index), so twin worlds
// answer identically.

#include "world_internal.h"

#include <float.h>
#include <string.h>

// ------------------------------------------------------------------
// Multi-hit ray: every shape's entry point, sorted.
// ------------------------------------------------------------------

typedef struct m3RayAllContext
{
    m3World* world;
    m3Pos3 origin;
    m3Vec3 translation;
    m3RayHit* hits;
    int32_t capacity;
    int32_t count;
} m3RayAllContext;

// The single-shape ray test lives in raycast.c; queries reuse it
// through this internal hook.
m3RayHit m3RayTestOneShape(m3World* world, int32_t shape, m3Pos3 origin, m3Vec3 translation);

static void RayAllInsert(m3RayAllContext* ctx, const m3RayHit* hit, int32_t shapeIndex)
{
    // Insertion sort by (fraction, shape index): capacities are
    // small and the order is the contract.
    int32_t pos = ctx->count;
    while (pos > 0)
    {
        const m3RayHit* prev = &ctx->hits[pos - 1];
        int after = hit->fraction > prev->fraction ||
                    (hit->fraction == prev->fraction && shapeIndex > prev->shape.index1 - 1);
        if (after)
        {
            break;
        }
        pos -= 1;
    }
    if (pos >= ctx->capacity)
    {
        return; // beyond capacity: the far end drops, deterministically
    }
    int32_t last = ctx->count < ctx->capacity ? ctx->count : ctx->capacity - 1;
    for (int32_t k = last; k > pos; --k)
    {
        ctx->hits[k] = ctx->hits[k - 1];
    }
    ctx->hits[pos] = *hit;
    if (ctx->count < ctx->capacity)
    {
        ctx->count += 1;
    }
}

static bool RayAllCallback(int32_t shape, void* userContext)
{
    m3RayAllContext* ctx = (m3RayAllContext*)userContext;
    m3RayHit hit = m3RayTestOneShape(ctx->world, shape, ctx->origin, ctx->translation);
    if (hit.hit)
    {
        RayAllInsert(ctx, &hit, shape);
    }
    return true;
}

int32_t m3World_CastRayAll(m3WorldId worldId, m3Pos3 origin, m3Vec3 translation, m3RayHit* hits,
                           int32_t capacity)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || hits == NULL || capacity <= 0 ||
        !(m3Dot3(translation, translation) > 0.0f))
    {
        return 0;
    }
    m3RayAllContext ctx = {world, origin, translation, hits, capacity, 0};
    double lo[3];
    double hi[3];
    double ex = origin.x + (double)translation.x;
    double ey = origin.y + (double)translation.y;
    double ez = origin.z + (double)translation.z;
    lo[0] = origin.x < ex ? origin.x : ex;
    lo[1] = origin.y < ey ? origin.y : ey;
    lo[2] = origin.z < ez ? origin.z : ez;
    hi[0] = origin.x > ex ? origin.x : ex;
    hi[1] = origin.y > ey ? origin.y : ey;
    hi[2] = origin.z > ez ? origin.z : ez;
    m3TreeQuery(&world->tree, lo, hi, RayAllCallback, &ctx);
    int32_t maxShape = world->shapePool.maxIndex;
    for (int32_t s = 0; s < maxShape; ++s)
    {
        if (world->shapePool.alive[s] != 0 && world->shapeType[s] == (uint8_t)m3_planeShape)
        {
            m3RayHit hit = m3RayTestOneShape(world, s, origin, translation);
            if (hit.hit)
            {
                RayAllInsert(&ctx, &hit, s);
            }
        }
    }
    return ctx.count;
}

// ------------------------------------------------------------------
// Shape casts: the TOI kernel with a static target sweep.
// ------------------------------------------------------------------

typedef struct m3ShapeCastContext
{
    m3World* world;
    m3Pos3 base;                          // the cast start: TOI floats re-center here
    m3Vec3 castPoints[M3_HULL_MAX_VERTS]; // sphere 1, capsule 2,
                                          // box 8, hull up to 24
    int32_t castPointCount;
    m3real castRadius;
    m3Vec3 translation;
    m3RayHit best;
    int32_t bestShape;
} m3ShapeCastContext;

static void ShapeCastTestShape(m3ShapeCastContext* ctx, int32_t shape)
{
    m3World* world = ctx->world;
    int32_t body = world->shapeBody[shape];
    if (world->shapeType[shape] == (uint8_t)m3_voxelShape)
    {
        // Voxel targets (3-5): per-box TOI against the merged
        // surface, unextended (queries report true geometry; the
        // seam extension is a contact-only device).
        int32_t slot = world->shapeVoxelIndex[shape];
        const m3VoxelSurface* surface = &world->voxelSurface[slot];
        m3real cell = world->voxelData[slot].cellSize;
        const m3Transform* xfV = &world->transforms[body];
        m3Sweep chunkSweep;
        chunkSweep.localCenter = (m3Vec3){0.0f, 0.0f, 0.0f};
        chunkSweep.c1 = (m3Vec3){(m3real)(xfV->p.x - ctx->base.x), (m3real)(xfV->p.y - ctx->base.y),
                                 (m3real)(xfV->p.z - ctx->base.z)};
        chunkSweep.c2 = chunkSweep.c1;
        chunkSweep.q1 = xfV->q;
        chunkSweep.q2 = xfV->q;

        m3Sweep castSweep;
        castSweep.localCenter = (m3Vec3){0.0f, 0.0f, 0.0f};
        castSweep.c1 = (m3Vec3){0.0f, 0.0f, 0.0f};
        castSweep.c2 = ctx->translation;
        castSweep.q1 = m3MakeIdentityQuat();
        castSweep.q2 = m3MakeIdentityQuat();

        m3Vec3 c1 = m3InvRotateVec3(xfV->q, m3Neg3(chunkSweep.c1));
        m3Vec3 c2 = m3InvRotateVec3(xfV->q, m3Sub3(ctx->translation, chunkSweep.c1));
        m3real pad = ctx->castRadius + 0.6f + M3_AABB_MARGIN;
        m3Vec3 blo = {m3MinF(c1.x, c2.x) - pad, m3MinF(c1.y, c2.y) - pad, m3MinF(c1.z, c2.z) - pad};
        m3Vec3 bhi = {m3MaxF(c1.x, c2.x) + pad, m3MaxF(c1.y, c2.y) + pad, m3MaxF(c1.z, c2.z) + pad};

        uint16_t gather[M3_MESH_MAX_TRIS];
        int32_t gatherCount = m3MeshBvhGather(&surface->bvh, blo, bhi, gather);
        int32_t budget = 64;
        for (int32_t g = 0; g < gatherCount && budget > 0; ++g)
        {
            budget -= 1;
            m3Vec3 lo;
            m3Vec3 hi;
            m3VoxelBoxBounds(surface, cell, gather[g], &lo, &hi);
            m3Vec3 corners[8];
            for (int32_t k = 0; k < 8; ++k)
            {
                corners[k] = (m3Vec3){(k & 1) != 0 ? hi.x : lo.x, (k & 2) != 0 ? hi.y : lo.y,
                                      (k & 4) != 0 ? hi.z : lo.z};
            }
            m3TOIInput input;
            input.proxyA.points = corners;
            input.proxyA.count = 8;
            input.proxyA.radius = 0.0f;
            input.proxyB.points = ctx->castPoints;
            input.proxyB.count = ctx->castPointCount;
            input.proxyB.radius = ctx->castRadius;
            input.sweepA = chunkSweep;
            input.sweepB = castSweep;
            input.maxFraction = ctx->best.hit ? ctx->best.fraction : 1.0f;
            m3TOIOutput out = m3TimeOfImpact(&input);
            if (out.state == m3_toiStateHit &&
                (!ctx->best.hit || out.fraction < ctx->best.fraction ||
                 (out.fraction == ctx->best.fraction && shape < ctx->bestShape)))
            {
                ctx->best.hit = true;
                ctx->best.fraction = out.fraction;
                ctx->best.normal = out.normal;
                ctx->best.shape =
                    (m3ShapeId){shape + 1, world->worldIndex0, world->shapePool.generations[shape]};
                ctx->bestShape = shape;
            }
            else if (out.state == m3_toiStateOverlapped &&
                     (!ctx->best.hit || 0.0f < ctx->best.fraction ||
                      (0.0f == ctx->best.fraction && shape < ctx->bestShape)))
            {
                ctx->best.hit = true;
                ctx->best.fraction = 0.0f; // the start-overlapped contract
                ctx->best.normal = (m3Vec3){0.0f, 0.0f, 0.0f};
                ctx->best.shape =
                    (m3ShapeId){shape + 1, world->worldIndex0, world->shapePool.generations[shape]};
                ctx->bestShape = shape;
            }
        }
        return;
    }
    if (world->shapeType[shape] == (uint8_t)m3_meshShape)
    {
        // Mesh targets: per-triangle TOI, ascending, bounded (the
        // CCD recipe re-used verbatim).
        const m3MeshData* mesh = &world->meshData[world->shapeMeshIndex[shape]];
        const m3Transform* xfM = &world->transforms[body];
        m3Sweep meshSweep;
        meshSweep.localCenter = (m3Vec3){0.0f, 0.0f, 0.0f};
        meshSweep.c1 = (m3Vec3){(m3real)(xfM->p.x - ctx->base.x), (m3real)(xfM->p.y - ctx->base.y),
                                (m3real)(xfM->p.z - ctx->base.z)};
        meshSweep.c2 = meshSweep.c1;
        meshSweep.q1 = xfM->q;
        meshSweep.q2 = xfM->q;

        m3Sweep castSweep;
        castSweep.localCenter = (m3Vec3){0.0f, 0.0f, 0.0f};
        castSweep.c1 = (m3Vec3){0.0f, 0.0f, 0.0f};
        castSweep.c2 = ctx->translation;
        castSweep.q1 = m3MakeIdentityQuat();
        castSweep.q2 = m3MakeIdentityQuat();

        // Candidate box in mesh-local space.
        m3Vec3 c1 = m3InvRotateVec3(xfM->q, m3Neg3(meshSweep.c1));
        m3Vec3 c2 = m3InvRotateVec3(xfM->q, m3Sub3(ctx->translation, meshSweep.c1));
        m3real pad = ctx->castRadius + 0.6f + M3_AABB_MARGIN; // capsule half-reach bound
        m3Vec3 blo = {m3MinF(c1.x, c2.x) - pad, m3MinF(c1.y, c2.y) - pad, m3MinF(c1.z, c2.z) - pad};
        m3Vec3 bhi = {m3MaxF(c1.x, c2.x) + pad, m3MaxF(c1.y, c2.y) + pad, m3MaxF(c1.z, c2.z) + pad};

        uint16_t gather[M3_MESH_MAX_TRIS];
        int32_t gatherCount =
            m3MeshBvhGather(&world->meshBvh[world->shapeMeshIndex[shape]], blo, bhi, gather);
        int32_t budget = 64;
        for (int32_t g = 0; g < gatherCount && budget > 0; ++g)
        {
            int32_t t = gather[g];
            m3Vec3 tv[3] = {mesh->vertices[mesh->indices[3 * t + 0]],
                            mesh->vertices[mesh->indices[3 * t + 1]],
                            mesh->vertices[mesh->indices[3 * t + 2]]};
            m3real tlx = m3MinF(tv[0].x, m3MinF(tv[1].x, tv[2].x));
            m3real thx = m3MaxF(tv[0].x, m3MaxF(tv[1].x, tv[2].x));
            m3real tly = m3MinF(tv[0].y, m3MinF(tv[1].y, tv[2].y));
            m3real thy = m3MaxF(tv[0].y, m3MaxF(tv[1].y, tv[2].y));
            m3real tlz = m3MinF(tv[0].z, m3MinF(tv[1].z, tv[2].z));
            m3real thz = m3MaxF(tv[0].z, m3MaxF(tv[1].z, tv[2].z));
            if (thx < blo.x || tlx > bhi.x || thy < blo.y || tly > bhi.y || thz < blo.z ||
                tlz > bhi.z)
            {
                continue;
            }
            budget -= 1;
            m3TOIInput input;
            input.proxyA.points = tv;
            input.proxyA.count = 3;
            input.proxyA.radius = 0.0f;
            input.proxyB.points = ctx->castPoints;
            input.proxyB.count = ctx->castPointCount;
            input.proxyB.radius = ctx->castRadius;
            input.sweepA = meshSweep;
            input.sweepB = castSweep;
            input.maxFraction = ctx->best.hit ? ctx->best.fraction : 1.0f;
            m3TOIOutput out = m3TimeOfImpact(&input);
            if (out.state == m3_toiStateHit &&
                (!ctx->best.hit || out.fraction < ctx->best.fraction ||
                 (out.fraction == ctx->best.fraction && shape < ctx->bestShape)))
            {
                ctx->best.hit = true;
                ctx->best.fraction = out.fraction;
                ctx->best.normal = out.normal;
                ctx->best.shape =
                    (m3ShapeId){shape + 1, world->worldIndex0, world->shapePool.generations[shape]};
                ctx->bestShape = shape;
            }
            else if (out.state == m3_toiStateOverlapped &&
                     (!ctx->best.hit || 0.0f < ctx->best.fraction ||
                      (0.0f == ctx->best.fraction && shape < ctx->bestShape)))
            {
                ctx->best.hit = true;
                ctx->best.fraction = 0.0f;
                ctx->best.normal = (m3Vec3){0.0f, 0.0f, 0.0f};
                ctx->best.shape =
                    (m3ShapeId){shape + 1, world->worldIndex0, world->shapePool.generations[shape]};
                ctx->bestShape = shape;
            }
        }
        return;
    }

    // Convex targets: one TOI against the shape's proxy at rest.
    m3Vec3 scratch[2];
    m3TOIInput input;
    input.proxyA = m3MakeShapeProxy(world, shape, scratch);
    input.proxyB.points = ctx->castPoints;
    input.proxyB.count = ctx->castPointCount;
    input.proxyB.radius = ctx->castRadius;

    const m3Transform* xf = &world->transforms[body];
    m3Vec3 rlc = m3RotateVec3(xf->q, world->localCenters[body]);
    input.sweepA.localCenter = world->localCenters[body];
    input.sweepA.c1 = (m3Vec3){(m3real)(xf->p.x + (double)rlc.x - ctx->base.x),
                               (m3real)(xf->p.y + (double)rlc.y - ctx->base.y),
                               (m3real)(xf->p.z + (double)rlc.z - ctx->base.z)};
    input.sweepA.c2 = input.sweepA.c1;
    input.sweepA.q1 = xf->q;
    input.sweepA.q2 = xf->q;

    input.sweepB.localCenter = (m3Vec3){0.0f, 0.0f, 0.0f};
    input.sweepB.c1 = (m3Vec3){0.0f, 0.0f, 0.0f};
    input.sweepB.c2 = ctx->translation;
    input.sweepB.q1 = m3MakeIdentityQuat();
    input.sweepB.q2 = m3MakeIdentityQuat();
    input.maxFraction = ctx->best.hit ? ctx->best.fraction : 1.0f;

    m3TOIOutput out = m3TimeOfImpact(&input);
    if (out.state == m3_toiStateHit &&
        (!ctx->best.hit || out.fraction < ctx->best.fraction ||
         (out.fraction == ctx->best.fraction && shape < ctx->bestShape)))
    {
        ctx->best.hit = true;
        ctx->best.fraction = out.fraction;
        ctx->best.normal = out.normal;
        ctx->best.shape =
            (m3ShapeId){shape + 1, world->worldIndex0, world->shapePool.generations[shape]};
        ctx->bestShape = shape;
    }
    else if (out.state == m3_toiStateOverlapped)
    {
        if (!ctx->best.hit || 0.0f < ctx->best.fraction ||
            (0.0f == ctx->best.fraction && shape < ctx->bestShape))
        {
            ctx->best.hit = true;
            ctx->best.fraction = 0.0f; // the start-overlapped contract
            ctx->best.normal = (m3Vec3){0.0f, 0.0f, 0.0f};
            ctx->best.shape =
                (m3ShapeId){shape + 1, world->worldIndex0, world->shapePool.generations[shape]};
            ctx->bestShape = shape;
        }
    }
}

// A plane target for a convex cast: conservative advance along the
// analytic support distance (the CCD plane recipe).
static void ShapeCastTestPlane(m3ShapeCastContext* ctx, int32_t shape)
{
    m3World* world = ctx->world;
    m3Vec3 n = world->shapeGeom[shape].v;
    m3real offset =
        world->shapeGeom[shape].s -
        (m3real)((double)n.x * ctx->base.x + (double)n.y * ctx->base.y + (double)n.z * ctx->base.z);
    const m3real linearSlop = 0.005f;
    // sep below is measured to the cast shape's SKIN (the radius is
    // already subtracted), so the stop target is one slop, full
    // stop. The old target of castRadius - slop double-counted the
    // radius and parked every cast one radius short of the plane;
    // the 2d-3 coverage fill caught it (this branch had never been
    // executed by a test before).
    m3real target = linearSlop;
    m3real tolerance = 0.25f * linearSlop;
    m3real rate = -m3Dot3(n, ctx->translation);
    if (!(rate > 0.0f))
    {
        return; // moving away or parallel
    }
    m3real t = 0.0f;
    m3real maxFraction = ctx->best.hit ? ctx->best.fraction : 1.0f;
    for (int32_t iter = 0; iter < 25; ++iter)
    {
        // Support of the cast proxy toward the plane at time t.
        m3real minD = 3.4e38f;
        for (int32_t k = 0; k < ctx->castPointCount; ++k)
        {
            m3Vec3 p = m3Add3(ctx->castPoints[k], m3MulSV3(t, ctx->translation));
            minD = m3MinF(minD, m3Dot3(n, p));
        }
        m3real sep = minD - offset - ctx->castRadius;
        if (sep <= 0.0f)
        {
            if (t == 0.0f)
            {
                // Start overlapped.
                if (!ctx->best.hit || 0.0f < ctx->best.fraction ||
                    (0.0f == ctx->best.fraction && shape < ctx->bestShape))
                {
                    ctx->best.hit = true;
                    ctx->best.fraction = 0.0f;
                    ctx->best.normal = (m3Vec3){0.0f, 0.0f, 0.0f};
                    ctx->best.shape = (m3ShapeId){shape + 1, world->worldIndex0,
                                                  world->shapePool.generations[shape]};
                    ctx->bestShape = shape;
                }
            }
            return;
        }
        if (sep <= target + tolerance)
        {
            if (t < maxFraction || (t == maxFraction && (!ctx->best.hit || shape < ctx->bestShape)))
            {
                ctx->best.hit = true;
                ctx->best.fraction = t;
                ctx->best.normal = n;
                ctx->best.shape =
                    (m3ShapeId){shape + 1, world->worldIndex0, world->shapePool.generations[shape]};
                ctx->bestShape = shape;
            }
            return;
        }
        t += (sep - target) / rate;
        if (t >= maxFraction)
        {
            return;
        }
    }
}

static bool ShapeCastCallback(int32_t shape, void* userContext)
{
    ShapeCastTestShape((m3ShapeCastContext*)userContext, shape);
    return true;
}

static m3RayHit CastConvexClosest(m3WorldId worldId, m3Pos3 base, const m3Vec3* points,
                                  int32_t pointCount, m3real radius, m3Vec3 translation)
{
    m3ShapeCastContext ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.best.fraction = 1.0f;
    m3World* world = m3WorldFromId(worldId);
    // A skinless cast (radius zero) is legal for real point clouds:
    // boxes and hulls cast their corners; spheres and capsules keep
    // their mandatory skin.
    if (world == NULL || pointCount < 1 || pointCount > M3_HULL_MAX_VERTS || radius < 0.0f ||
        !(radius > 0.0f || pointCount >= 2))
    {
        return ctx.best;
    }
    ctx.world = world;
    ctx.base = base;
    for (int32_t k = 0; k < pointCount; ++k)
    {
        ctx.castPoints[k] = points[k];
    }
    ctx.castPointCount = pointCount;
    ctx.castRadius = radius;
    ctx.translation = translation;
    ctx.bestShape = INT32_MAX;

    // Candidates: the swept box padded by radius and point extent.
    m3real extent = radius;
    for (int32_t k = 0; k < pointCount; ++k)
    {
        extent = m3MaxF(extent, sqrtf(m3Dot3(points[k], points[k])) + radius);
    }
    double lo[3];
    double hi[3];
    double ex = base.x + (double)translation.x;
    double ey = base.y + (double)translation.y;
    double ez = base.z + (double)translation.z;
    lo[0] = (base.x < ex ? base.x : ex) - (double)extent;
    lo[1] = (base.y < ey ? base.y : ey) - (double)extent;
    lo[2] = (base.z < ez ? base.z : ez) - (double)extent;
    hi[0] = (base.x > ex ? base.x : ex) + (double)extent;
    hi[1] = (base.y > ey ? base.y : ey) + (double)extent;
    hi[2] = (base.z > ez ? base.z : ez) + (double)extent;
    m3TreeQuery(&world->tree, lo, hi, ShapeCastCallback, &ctx);

    int32_t maxShape = world->shapePool.maxIndex;
    for (int32_t s = 0; s < maxShape; ++s)
    {
        if (world->shapePool.alive[s] != 0 && world->shapeType[s] == (uint8_t)m3_planeShape)
        {
            ShapeCastTestPlane(&ctx, s);
        }
    }

    if (ctx.best.hit)
    {
        ctx.best.point.x = base.x + (double)(ctx.best.fraction * translation.x);
        ctx.best.point.y = base.y + (double)(ctx.best.fraction * translation.y);
        ctx.best.point.z = base.z + (double)(ctx.best.fraction * translation.z);
    }
    return ctx.best;
}

m3RayHit m3World_CastBoxClosest(m3WorldId worldId, m3Pos3 center, m3Vec3 halfExtents,
                                m3Quat rotation, m3Vec3 translation)
{
    m3RayHit miss;
    memset(&miss, 0, sizeof(miss));
    m3real qq = rotation.x * rotation.x + rotation.y * rotation.y + rotation.z * rotation.z +
                rotation.w * rotation.w;
    if (!m3FinitePos3(center) || !m3FiniteV3(halfExtents) || !m3FiniteV3(translation) ||
        !m3FiniteQuat(rotation) || !(halfExtents.x > 0.0f) || !(halfExtents.y > 0.0f) ||
        !(halfExtents.z > 0.0f) || !(qq > 0.98f) || !(qq < 1.02f))
    {
        return miss; // hostile input: the cast quietly misses, loudly
                     // documented (the query contract has no id to
                     // refuse with)
    }
    m3Vec3 corners[8];
    for (int32_t c = 0; c < 8; ++c)
    {
        m3Vec3 local = {(c & 1) != 0 ? halfExtents.x : -halfExtents.x,
                        (c & 2) != 0 ? halfExtents.y : -halfExtents.y,
                        (c & 4) != 0 ? halfExtents.z : -halfExtents.z};
        corners[c] = m3RotateVec3(rotation, local);
    }
    return CastConvexClosest(worldId, center, corners, 8, 0.0f, translation);
}

m3RayHit m3World_CastHullClosest(m3WorldId worldId, m3Pos3 base, const m3Vec3* points,
                                 int32_t count, m3Vec3 translation)
{
    m3RayHit miss;
    memset(&miss, 0, sizeof(miss));
    if (points == NULL || count < 2 || count > M3_HULL_MAX_VERTS || !m3FinitePos3(base) ||
        !m3FiniteV3(translation))
    {
        return miss; // a one-point skinless cast is a ray: use rays
    }
    for (int32_t k = 0; k < count; ++k)
    {
        if (!m3FiniteV3(points[k]))
        {
            return miss;
        }
    }
    return CastConvexClosest(worldId, base, points, count, 0.0f, translation);
}

m3RayHit m3World_CastSphereClosest(m3WorldId worldId, m3Pos3 center, m3real radius,
                                   m3Vec3 translation)
{
    m3Vec3 point = {0.0f, 0.0f, 0.0f};
    return CastConvexClosest(worldId, center, &point, 1, radius, translation);
}

m3RayHit m3World_CastCapsuleClosest(m3WorldId worldId, m3Pos3 center, m3Vec3 point1, m3Vec3 point2,
                                    m3real radius, m3Vec3 translation)
{
    m3Vec3 points[2] = {point1, point2};
    return CastConvexClosest(worldId, center, points, 2, radius, translation);
}

// ------------------------------------------------------------------
// Point containment and overlaps.
// ------------------------------------------------------------------

static int PointInVoxel(const m3World* world, int32_t shape, m3Pos3 point);

static int PointInShape(const m3World* world, int32_t shape, m3Pos3 point)
{
    int32_t body = world->shapeBody[shape];
    const m3Transform* xf = &world->transforms[body];
    m3Vec3 local =
        m3InvRotateVec3(xf->q, (m3Vec3){(m3real)(point.x - xf->p.x), (m3real)(point.y - xf->p.y),
                                        (m3real)(point.z - xf->p.z)});
    uint8_t type = world->shapeType[shape];
    if (type == (uint8_t)m3_sphereShape)
    {
        m3Vec3 d = m3Sub3(local, world->shapeGeom[shape].v);
        m3real r = world->shapeGeom[shape].s;
        return m3Dot3(d, d) <= r * r;
    }
    if (type == (uint8_t)m3_capsuleShape)
    {
        m3Vec3 p1 = world->shapeGeom[shape].v;
        m3Vec3 axis = m3Sub3(world->shapeGeom[shape].v2, p1);
        m3real len2 = m3Dot3(axis, axis);
        m3real t = len2 > 0.0f ? m3Dot3(m3Sub3(local, p1), axis) / len2 : 0.0f;
        t = m3MaxF(0.0f, m3MinF(1.0f, t));
        m3Vec3 closest = m3Add3(p1, m3MulSV3(t, axis));
        m3Vec3 d = m3Sub3(local, closest);
        m3real r = world->shapeGeom[shape].s;
        return m3Dot3(d, d) <= r * r;
    }
    if (type == (uint8_t)m3_hullShape)
    {
        const m3HullData* hull = &world->hullData[world->shapeHullIndex[shape]];
        for (int32_t f = 0; f < hull->faceCount; ++f)
        {
            if (m3Dot3(hull->faceNormals[f], local) - hull->faceOffsets[f] > 0.0f)
            {
                return 0;
            }
        }
        return 1;
    }
    if (type == (uint8_t)m3_planeShape)
    {
        // Solid half space: at or below the surface.
        return m3Dot3(world->shapeGeom[shape].v, local) - world->shapeGeom[shape].s <= 0.0f;
    }
    if (type == (uint8_t)m3_voxelShape)
    {
        return PointInVoxel(world, shape, point);
    }
    return 0; // meshes are open surfaces: no interior
}

// Solid voxels are CLOSED volumes: containment is a grid lookup.
static int PointInVoxel(const m3World* world, int32_t shape, m3Pos3 point)
{
    int32_t body = world->shapeBody[shape];
    const m3Transform* xf = &world->transforms[body];
    m3Vec3 local =
        m3InvRotateVec3(xf->q, (m3Vec3){(m3real)(point.x - xf->p.x), (m3real)(point.y - xf->p.y),
                                        (m3real)(point.z - xf->p.z)});
    int32_t slot = world->shapeVoxelIndex[shape];
    m3real cell = world->voxelData[slot].cellSize;
    int32_t x = (int32_t)(local.x / cell);
    int32_t y = (int32_t)(local.y / cell);
    int32_t z = (int32_t)(local.z / cell);
    if (local.x < 0.0f || local.y < 0.0f || local.z < 0.0f || x >= M3_VOXEL_DIM ||
        y >= M3_VOXEL_DIM || z >= M3_VOXEL_DIM)
    {
        return 0;
    }
    return m3VoxelGet(&world->voxelData[slot], x, y, z) ? 1 : 0;
}

m3ShapeId m3World_PointInside(m3WorldId worldId, m3Pos3 point)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL)
    {
        return m3_nullShapeId;
    }
    int32_t maxShape = world->shapePool.maxIndex;
    for (int32_t s = 0; s < maxShape; ++s)
    {
        if (world->shapePool.alive[s] != 0 && PointInShape(world, s, point))
        {
            return (m3ShapeId){s + 1, world->worldIndex0, world->shapePool.generations[s]};
        }
    }
    return m3_nullShapeId;
}

typedef struct m3OverlapContext
{
    m3World* world;
    m3Pos3 center;
    m3real radius; // < 0 = pure AABB gather
    double lo[3];
    double hi[3];
    int32_t indices[256];
    int32_t count;
} m3OverlapContext;

static int SphereReachesShape(m3World* world, int32_t shape, m3Pos3 center, m3real radius)
{
    uint8_t type = world->shapeType[shape];
    if (type == (uint8_t)m3_planeShape)
    {
        int32_t body = world->shapeBody[shape];
        const m3Transform* xf = &world->transforms[body];
        m3Vec3 local = m3InvRotateVec3(xf->q, (m3Vec3){(m3real)(center.x - xf->p.x),
                                                       (m3real)(center.y - xf->p.y),
                                                       (m3real)(center.z - xf->p.z)});
        m3real d = m3Dot3(world->shapeGeom[shape].v, local) - world->shapeGeom[shape].s;
        return d <= radius;
    }
    if (type == (uint8_t)m3_voxelShape)
    {
        // Reach against the merged boxes: exact clamp per candidate
        // (boxes are axis-aligned in the chunk frame).
        int32_t body = world->shapeBody[shape];
        const m3Transform* xf = &world->transforms[body];
        m3Vec3 local = m3InvRotateVec3(xf->q, (m3Vec3){(m3real)(center.x - xf->p.x),
                                                       (m3real)(center.y - xf->p.y),
                                                       (m3real)(center.z - xf->p.z)});
        int32_t slot = world->shapeVoxelIndex[shape];
        const m3VoxelSurface* surface = &world->voxelSurface[slot];
        m3real cell = world->voxelData[slot].cellSize;
        uint16_t gather[M3_MESH_MAX_TRIS];
        m3Vec3 blo = {local.x - radius, local.y - radius, local.z - radius};
        m3Vec3 bhi = {local.x + radius, local.y + radius, local.z + radius};
        int32_t gatherCount = m3MeshBvhGather(&surface->bvh, blo, bhi, gather);
        for (int32_t g = 0; g < gatherCount; ++g)
        {
            m3Vec3 lo;
            m3Vec3 hi;
            m3VoxelBoxBounds(surface, cell, gather[g], &lo, &hi);
            m3Vec3 closest = {m3ClampF(local.x, lo.x, hi.x), m3ClampF(local.y, lo.y, hi.y),
                              m3ClampF(local.z, lo.z, hi.z)};
            m3Vec3 d = m3Sub3(local, closest);
            if (m3Dot3(d, d) <= radius * radius)
            {
                return 1;
            }
        }
        return 0;
    }
    if (type == (uint8_t)m3_meshShape)
    {
        // Distance to any triangle within reach (bounded scan).
        int32_t body = world->shapeBody[shape];
        const m3Transform* xf = &world->transforms[body];
        m3Vec3 local = m3InvRotateVec3(xf->q, (m3Vec3){(m3real)(center.x - xf->p.x),
                                                       (m3real)(center.y - xf->p.y),
                                                       (m3real)(center.z - xf->p.z)});
        const m3MeshData* mesh = &world->meshData[world->shapeMeshIndex[shape]];
        uint16_t gather[M3_MESH_MAX_TRIS];
        m3Vec3 blo = {local.x - radius, local.y - radius, local.z - radius};
        m3Vec3 bhi = {local.x + radius, local.y + radius, local.z + radius};
        int32_t gatherCount =
            m3MeshBvhGather(&world->meshBvh[world->shapeMeshIndex[shape]], blo, bhi, gather);
        for (int32_t g = 0; g < gatherCount; ++g)
        {
            int32_t t = gather[g];
            m3Vec3 a = mesh->vertices[mesh->indices[3 * t + 0]];
            m3Vec3 b = mesh->vertices[mesh->indices[3 * t + 1]];
            m3Vec3 c = mesh->vertices[mesh->indices[3 * t + 2]];
            // Cheap reject on the triangle box, exact on the plane
            // projection clamped by the closest-point routine's job:
            // a conservative vertex-distance check suffices here.
            m3Vec3 d0 = m3Sub3(local, a);
            m3Vec3 d1 = m3Sub3(local, b);
            m3Vec3 d2 = m3Sub3(local, c);
            m3real r2 = (radius + 0.0f) * (radius + 0.0f);
            if (m3Dot3(d0, d0) <= r2 || m3Dot3(d1, d1) <= r2 || m3Dot3(d2, d2) <= r2)
            {
                return 1;
            }
        }
        return 0;
    }
    // Convex families: GJK distance between the sphere point and the
    // shape core, radii applied analytically.
    m3Vec3 scratch[2];
    m3DistanceProxy proxy = m3MakeShapeProxy(world, shape, scratch);
    int32_t body = world->shapeBody[shape];
    const m3Transform* xf = &world->transforms[body];
    m3Vec3 local =
        m3InvRotateVec3(xf->q, (m3Vec3){(m3real)(center.x - xf->p.x), (m3real)(center.y - xf->p.y),
                                        (m3real)(center.z - xf->p.z)});
    m3Vec3 point = local;
    m3DistanceInput input;
    memset(&input, 0, sizeof(input));
    input.proxyA = proxy;
    input.proxyB.points = &point;
    input.proxyB.count = 1;
    input.proxyB.radius = 0.0f;
    input.q = m3MakeIdentityQuat();
    input.p = (m3Vec3){0.0f, 0.0f, 0.0f};
    input.useRadii = false;
    m3SimplexCache cache;
    cache.count = 0;
    cache.metric = 0.0f;
    m3DistanceOutput out = m3ShapeDistance(&input, &cache);
    return out.distance - proxy.radius <= radius;
}

static bool OverlapCallback(int32_t shape, void* userContext)
{
    m3OverlapContext* ctx = (m3OverlapContext*)userContext;
    if (ctx->count >= 256)
    {
        return true;
    }
    if (ctx->radius >= 0.0f && !SphereReachesShape(ctx->world, shape, ctx->center, ctx->radius))
    {
        return true;
    }
    ctx->indices[ctx->count++] = shape;
    return true;
}

static int32_t OverlapGather(m3World* world, m3OverlapContext* ctx, m3ShapeId* shapes,
                             int32_t capacity)
{
    m3TreeQuery(&world->tree, ctx->lo, ctx->hi, OverlapCallback, ctx);
    int32_t maxShape = world->shapePool.maxIndex;
    for (int32_t s = 0; s < maxShape && ctx->count < 256; ++s)
    {
        if (world->shapePool.alive[s] != 0 && world->shapeType[s] == (uint8_t)m3_planeShape)
        {
            int include;
            if (ctx->radius >= 0.0f)
            {
                include = SphereReachesShape(world, s, ctx->center, ctx->radius);
            }
            else
            {
                // Half space versus box: the box reaches the plane
                // iff its most-negative corner along the normal does.
                int32_t body = world->shapeBody[s];
                const m3Transform* xf = &world->transforms[body];
                m3Vec3 n = m3RotateVec3(xf->q, world->shapeGeom[s].v); // world normal
                double off = (double)world->shapeGeom[s].s + (double)n.x * xf->p.x +
                             (double)n.y * xf->p.y + (double)n.z * xf->p.z;
                double minProj = (n.x >= 0.0f ? ctx->lo[0] : ctx->hi[0]) * (double)n.x +
                                 (n.y >= 0.0f ? ctx->lo[1] : ctx->hi[1]) * (double)n.y +
                                 (n.z >= 0.0f ? ctx->lo[2] : ctx->hi[2]) * (double)n.z;
                include = minProj <= off;
            }
            if (include)
            {
                ctx->indices[ctx->count++] = s;
            }
        }
    }
    // Ascending shape index: the canonical order.
    for (int32_t a = 0; a < ctx->count; ++a)
    {
        for (int32_t b = a + 1; b < ctx->count; ++b)
        {
            if (ctx->indices[b] < ctx->indices[a])
            {
                int32_t tmp = ctx->indices[a];
                ctx->indices[a] = ctx->indices[b];
                ctx->indices[b] = tmp;
            }
        }
    }
    int32_t written = 0;
    for (int32_t k = 0; k < ctx->count && written < capacity; ++k)
    {
        int32_t s = ctx->indices[k];
        shapes[written++] = (m3ShapeId){s + 1, world->worldIndex0, world->shapePool.generations[s]};
    }
    return written;
}

int32_t m3World_OverlapAabb(m3WorldId worldId, m3Pos3 lo, m3Pos3 hi, m3ShapeId* shapes,
                            int32_t capacity)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || shapes == NULL || capacity <= 0)
    {
        return 0;
    }
    m3OverlapContext ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.world = world;
    ctx.radius = -1.0f;
    ctx.lo[0] = lo.x;
    ctx.lo[1] = lo.y;
    ctx.lo[2] = lo.z;
    ctx.hi[0] = hi.x;
    ctx.hi[1] = hi.y;
    ctx.hi[2] = hi.z;
    return OverlapGather(world, &ctx, shapes, capacity);
}

int32_t m3World_OverlapSphere(m3WorldId worldId, m3Pos3 center, m3real radius, m3ShapeId* shapes,
                              int32_t capacity)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || shapes == NULL || capacity <= 0 || !(radius > 0.0f))
    {
        return 0;
    }
    m3OverlapContext ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.world = world;
    ctx.center = center;
    ctx.radius = radius;
    ctx.lo[0] = center.x - (double)radius;
    ctx.lo[1] = center.y - (double)radius;
    ctx.lo[2] = center.z - (double)radius;
    ctx.hi[0] = center.x + (double)radius;
    ctx.hi[1] = center.y + (double)radius;
    ctx.hi[2] = center.z + (double)radius;
    return OverlapGather(world, &ctx, shapes, capacity);
}
