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
    m3QueryFilter filter; // 8-1
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
    if (ctx->world->bodyEnabled[ctx->world->shapeBody[shape]] == 0)
    {
        return true; // disabled bodies vanish from queries (8-3)
    }
    if (!m3FilterPass(ctx->filter.categoryBits, ctx->filter.maskBits,
                      ctx->world->shapeCategory[shape], ctx->world->shapeMask[shape]))
    {
        return true; // filtered out (8-1)
    }
    m3RayHit hit = m3RayTestOneShape(ctx->world, shape, ctx->origin, ctx->translation);
    if (hit.hit)
    {
        RayAllInsert(ctx, &hit, shape);
    }
    return true;
}

int32_t m3World_CastRayAllEx(m3WorldId worldId, m3Pos3 origin, m3Vec3 translation, m3RayHit* hits,
                             int32_t capacity, m3QueryFilter filter);

int32_t m3World_CastRayAll(m3WorldId worldId, m3Pos3 origin, m3Vec3 translation, m3RayHit* hits,
                           int32_t capacity)
{
    return m3World_CastRayAllEx(worldId, origin, translation, hits, capacity,
                                m3DefaultQueryFilter());
}

int32_t m3World_CastRayAllEx(m3WorldId worldId, m3Pos3 origin, m3Vec3 translation, m3RayHit* hits,
                             int32_t capacity, m3QueryFilter filter)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || hits == NULL || capacity <= 0 ||
        !(m3Dot3(translation, translation) > 0.0f) ||
        !(translation.x >= -M3_CAST_LIMIT && translation.x <= M3_CAST_LIMIT) ||
        !(translation.y >= -M3_CAST_LIMIT && translation.y <= M3_CAST_LIMIT) ||
        !(translation.z >= -M3_CAST_LIMIT && translation.z <= M3_CAST_LIMIT))
    {
        return 0;
    }
    m3RayAllContext ctx = {world, origin, translation, hits, capacity, 0, m3DefaultQueryFilter()};
    ctx.filter = filter;
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
        if (world->shapePool.alive[s] != 0 && world->shapeType[s] == (uint8_t)m3_planeShape &&
            m3FilterPass(filter.categoryBits, filter.maskBits, world->shapeCategory[s],
                         world->shapeMask[s]))
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
    int32_t ignoreBody;                   // -1 none: the character excludes itself
    m3Pos3 base;                          // the cast start: TOI floats re-center here
    m3Vec3 castPoints[M3_HULL_MAX_VERTS]; // sphere 1, capsule 2,
                                          // box 8, hull up to 24
    int32_t castPointCount;
    m3real castRadius;
    m3Vec3 translation;
    m3RayHit best;
    int32_t bestShape;
    m3QueryFilter filter; // 8-1
} m3ShapeCastContext;

static void ShapeCastTestShape(m3ShapeCastContext* ctx, int32_t shape)
{
    m3World* world = ctx->world;
    int32_t body = world->shapeBody[shape];
    if (body == ctx->ignoreBody)
    {
        return; // the caster's own body never blocks its cast
    }
    if (world->bodyEnabled[body] == 0)
    {
        return; // disabled bodies vanish from queries (8-3)
    }
    if (!m3FilterPass(ctx->filter.categoryBits, ctx->filter.maskBits,
                      ctx->world->shapeCategory[shape], ctx->world->shapeMask[shape]))
    {
        return; // filtered out (8-1)
    }
    if (world->shapeType[shape] == (uint8_t)m3_voxelShape)
    {
        // Voxel targets (3-5): per-box TOI against the merged
        // surface, unextended (queries report true geometry; the
        // seam extension is a contact-only device).
        int32_t slot = world->shapeVoxelIndex[shape];
        const m3VoxelSurface* surface = &world->voxelSurface[slot];
        m3real cell = world->voxelData[slot].cellSize;
        m3Transform xfVv = m3ShapeWorldTransform(world, shape);
        const m3Transform* xfV = &xfVv;
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
        m3Transform xfMv = m3ShapeWorldTransform(world, shape);
        const m3Transform* xfM = &xfMv;
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
    if (world->shapeBody[shape] == ctx->ignoreBody)
    {
        return;
    }
    if (world->bodyEnabled[world->shapeBody[shape]] == 0)
    {
        return; // disabled bodies vanish from queries (8-3)
    }
    if (!m3FilterPass(ctx->filter.categoryBits, ctx->filter.maskBits, world->shapeCategory[shape],
                      world->shapeMask[shape]))
    {
        return; // filtered out (8-1)
    }
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

m3RayHit m3CastConvexFiltered(m3World* worldPtr, m3Pos3 base, const m3Vec3* points,
                              int32_t pointCount, m3real radius, m3Vec3 translation,
                              int32_t ignoreBody, m3QueryFilter filter)
{
    m3ShapeCastContext ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.ignoreBody = ignoreBody;
    ctx.filter = filter;
    ctx.best.fraction = 1.0f;
    m3World* world = worldPtr;
    // A skinless cast (radius zero) is legal for real point clouds:
    // boxes and hulls cast their corners; spheres and capsules keep
    // their mandatory skin.
    if (world == NULL || pointCount < 1 || pointCount > M3_HULL_MAX_VERTS || radius < 0.0f ||
        !(radius > 0.0f || pointCount >= 2) ||
        !(translation.x >= -M3_CAST_LIMIT && translation.x <= M3_CAST_LIMIT) ||
        !(translation.y >= -M3_CAST_LIMIT && translation.y <= M3_CAST_LIMIT) ||
        !(translation.z >= -M3_CAST_LIMIT && translation.z <= M3_CAST_LIMIT))
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

m3RayHit m3World_CastBoxClosestEx(m3WorldId worldId, m3Pos3 center, m3Vec3 halfExtents,
                                  m3Quat rotation, m3Vec3 translation, m3QueryFilter filter)
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
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL)
    {
        return miss;
    }
    return m3CastConvexFiltered(world, center, corners, 8, 0.0f, translation, -1, filter);
}

m3RayHit m3World_CastBoxClosest(m3WorldId worldId, m3Pos3 center, m3Vec3 halfExtents,
                                m3Quat rotation, m3Vec3 translation)
{
    return m3World_CastBoxClosestEx(worldId, center, halfExtents, rotation, translation,
                                    m3DefaultQueryFilter());
}

m3RayHit m3World_CastHullClosestEx(m3WorldId worldId, m3Pos3 base, const m3Vec3* points,
                                   int32_t count, m3Vec3 translation, m3QueryFilter filter)
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
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL)
    {
        return miss;
    }
    return m3CastConvexFiltered(world, base, points, count, 0.0f, translation, -1, filter);
}

m3RayHit m3World_CastHullClosest(m3WorldId worldId, m3Pos3 base, const m3Vec3* points,
                                 int32_t count, m3Vec3 translation)
{
    return m3World_CastHullClosestEx(worldId, base, points, count, translation,
                                     m3DefaultQueryFilter());
}

m3RayHit m3CastConvexClosestEx(m3World* worldPtr, m3Pos3 base, const m3Vec3* points,
                               int32_t pointCount, m3real radius, m3Vec3 translation,
                               int32_t ignoreBody)
{
    return m3CastConvexFiltered(worldPtr, base, points, pointCount, radius, translation, ignoreBody,
                                m3DefaultQueryFilter());
}

m3RayHit m3World_CastSphereClosestEx(m3WorldId worldId, m3Pos3 center, m3real radius,
                                     m3Vec3 translation, m3QueryFilter filter)
{
    m3Vec3 point = {0.0f, 0.0f, 0.0f};
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL)
    {
        m3RayHit miss;
        memset(&miss, 0, sizeof(miss));
        return miss;
    }
    return m3CastConvexFiltered(world, center, &point, 1, radius, translation, -1, filter);
}

m3RayHit m3World_CastSphereClosest(m3WorldId worldId, m3Pos3 center, m3real radius,
                                   m3Vec3 translation)
{
    return m3World_CastSphereClosestEx(worldId, center, radius, translation,
                                       m3DefaultQueryFilter());
}

m3RayHit m3World_CastCapsuleClosestEx(m3WorldId worldId, m3Pos3 center, m3Vec3 point1,
                                      m3Vec3 point2, m3real radius, m3Vec3 translation,
                                      m3QueryFilter filter)
{
    m3Vec3 points[2] = {point1, point2};
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL)
    {
        m3RayHit miss;
        memset(&miss, 0, sizeof(miss));
        return miss;
    }
    return m3CastConvexFiltered(world, center, points, 2, radius, translation, -1, filter);
}

m3RayHit m3World_CastCapsuleClosest(m3WorldId worldId, m3Pos3 center, m3Vec3 point1, m3Vec3 point2,
                                    m3real radius, m3Vec3 translation)
{
    return m3World_CastCapsuleClosestEx(worldId, center, point1, point2, radius, translation,
                                        m3DefaultQueryFilter());
}

// ------------------------------------------------------------------
// Point containment and overlaps.
// ------------------------------------------------------------------

static int PointInVoxel(const m3World* world, int32_t shape, m3Pos3 point);

static int PointInShape(const m3World* world, int32_t shape, m3Pos3 point)
{
    m3Transform xfS7 = m3ShapeWorldTransform(world, shape);
    const m3Transform* xf = &xfS7;
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
    m3Transform xfS = m3ShapeWorldTransform(world, shape);
    const m3Transform* xf = &xfS;
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
    m3QueryFilter filter; // 8-1
} m3OverlapContext;

static int SphereReachesShape(m3World* world, int32_t shape, m3Pos3 center, m3real radius)
{
    uint8_t type = world->shapeType[shape];
    if (type == (uint8_t)m3_planeShape)
    {
        m3Transform xfS3 = m3ShapeWorldTransform(world, shape);
        const m3Transform* xf = &xfS3;
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
        m3Transform xfS4 = m3ShapeWorldTransform(world, shape);
        const m3Transform* xf = &xfS4;
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
    if (type == (uint8_t)m3_heightFieldShape)
    {
        // Distance to any terrain triangle within reach (19-3): the
        // mesh recipe over the cell gather.
        m3Transform xfS6 = m3ShapeWorldTransform(world, shape);
        const m3Transform* xfH = &xfS6;
        m3Vec3 localH = m3InvRotateVec3(xfH->q, (m3Vec3){(m3real)(center.x - xfH->p.x),
                                                         (m3real)(center.y - xfH->p.y),
                                                         (m3real)(center.z - xfH->p.z)});
        const m3HeightFieldData* hf = &world->hfData[world->shapeHfIndex[shape]];
        m3Vec3 hfTris[512][3];
        int32_t hfCount = m3HeightFieldGather(
            hf, (m3Vec3){localH.x - radius, localH.y - radius, localH.z - radius},
            (m3Vec3){localH.x + radius, localH.y + radius, localH.z + radius}, hfTris, 512);
        for (int32_t t = 0; t < hfCount; ++t)
        {
            // The mesh recipe verbatim: the conservative
            // vertex-distance check suffices here.
            m3Vec3 d0 = m3Sub3(localH, hfTris[t][0]);
            m3Vec3 d1 = m3Sub3(localH, hfTris[t][1]);
            m3Vec3 d2 = m3Sub3(localH, hfTris[t][2]);
            m3real r2 = radius * radius;
            if (m3Dot3(d0, d0) <= r2 || m3Dot3(d1, d1) <= r2 || m3Dot3(d2, d2) <= r2)
            {
                return 1;
            }
        }
        return 0;
    }
    if (type == (uint8_t)m3_meshShape)
    {
        // Distance to any triangle within reach (bounded scan).
        m3Transform xfS5 = m3ShapeWorldTransform(world, shape);
        const m3Transform* xf = &xfS5;
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
    m3Transform xfS8 = m3ShapeWorldTransform(world, shape);
    const m3Transform* xf = &xfS8;
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
    {
        m3OverlapContext* fctx = (m3OverlapContext*)userContext;
        if (fctx->world->bodyEnabled[fctx->world->shapeBody[shape]] == 0)
        {
            return true; // disabled bodies vanish from queries (8-3)
        }
        if (!m3FilterPass(fctx->filter.categoryBits, fctx->filter.maskBits,
                          fctx->world->shapeCategory[shape], fctx->world->shapeMask[shape]))
        {
            return true; // filtered out (8-1)
        }
    }
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
                m3Transform xfS6 = m3ShapeWorldTransform(world, s);
                const m3Transform* xf = &xfS6;
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

int32_t m3World_OverlapAabbEx(m3WorldId worldId, m3Pos3 lo, m3Pos3 hi, m3ShapeId* shapes,
                              int32_t capacity, m3QueryFilter filter)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || shapes == NULL || capacity <= 0)
    {
        return 0;
    }
    m3OverlapContext ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.filter = filter; // AFTER the memset (the 5-1 constructor
                         // lesson caught this very line in review:
                         // a wiped filter of zeros filters ALL)
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

int32_t m3World_OverlapAabb(m3WorldId worldId, m3Pos3 lo, m3Pos3 hi, m3ShapeId* shapes,
                            int32_t capacity)
{
    return m3World_OverlapAabbEx(worldId, lo, hi, shapes, capacity, m3DefaultQueryFilter());
}

int32_t m3World_OverlapSphereEx(m3WorldId worldId, m3Pos3 center, m3real radius, m3ShapeId* shapes,
                                int32_t capacity, m3QueryFilter filter)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || shapes == NULL || capacity <= 0 || !(radius > 0.0f))
    {
        return 0;
    }
    m3OverlapContext ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.filter = filter; // AFTER the memset (the 5-1 constructor
                         // lesson caught this very line in review:
                         // a wiped filter of zeros filters ALL)
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

int32_t m3World_OverlapSphere(m3WorldId worldId, m3Pos3 center, m3real radius, m3ShapeId* shapes,
                              int32_t capacity)
{
    return m3World_OverlapSphereEx(worldId, center, radius, shapes, capacity,
                                   m3DefaultQueryFilter());
}

// --- Explosions (13-2) ------------------------------------------------------

#define M3_EXPLOSION_COOKIE ((int32_t)(M3_COOKIE ^ ((int32_t)sizeof(m3ExplosionDef) << 8) ^ 13))

// Projected area of a convex shape onto a plane facing `direction`
// (a unit vector in the shape's local frame): the reference scales
// blast impulse by the area the shape shows to the front.
static m3real ShapeProjectedArea(const m3World* world, int32_t shape, m3Vec3 direction)
{
    uint8_t type = world->shapeType[shape];
    const m3ShapeGeom* geom = &world->shapeGeom[shape];
    if (type == (uint8_t)m3_sphereShape)
    {
        return M3_PI * geom->s * geom->s;
    }
    if (type == (uint8_t)m3_capsuleShape)
    {
        m3Vec3 axis = m3Sub3(geom->v2, geom->v);
        m3real projected = m3Length3(m3Cross3(axis, direction));
        return M3_PI * geom->s * geom->s + 2.0f * geom->s * projected;
    }
    if (type == (uint8_t)m3_hullShape)
    {
        // Fan every face from its first vertex and keep the facing
        // triangles; half the summed cross products is the area.
        const m3HullData* hull = &world->hullData[world->shapeHullIndex[shape]];
        m3real area = 0.0f;
        for (int32_t f = 0; f < hull->faceCount; ++f)
        {
            int32_t start = (int32_t)hull->faceVertStart[f];
            int32_t count = (int32_t)hull->faceVertCounts[f];
            m3Vec3 p1 = hull->vertices[hull->faceIndices[start]];
            for (int32_t k = 2; k < count; ++k)
            {
                m3Vec3 p2 = hull->vertices[hull->faceIndices[start + k - 1]];
                m3Vec3 p3 = hull->vertices[hull->faceIndices[start + k]];
                m3real a = m3Dot3(m3Cross3(m3Sub3(p2, p1), m3Sub3(p3, p1)), direction);
                area += a > 0.0f ? a : 0.0f;
            }
        }
        return 0.5f * area;
    }
    return 0.0f;
}

// The shape's own stable interior point, local frame: the fallback
// direction anchor when the blast center sits inside the shape.
static m3Vec3 ShapeLocalCentroid(const m3World* world, int32_t shape)
{
    uint8_t type = world->shapeType[shape];
    const m3ShapeGeom* geom = &world->shapeGeom[shape];
    if (type == (uint8_t)m3_capsuleShape)
    {
        return m3MulSV3(0.5f, m3Add3(geom->v, geom->v2));
    }
    if (type == (uint8_t)m3_hullShape)
    {
        return world->hullData[world->shapeHullIndex[shape]].center;
    }
    return geom->v; // sphere center
}

typedef struct m3ExplodeContext
{
    m3World* world;
    const m3ExplosionDef* def;
} m3ExplodeContext;

static bool ExplodeCallback(int32_t shape, void* userContext)
{
    m3ExplodeContext* ctx = (m3ExplodeContext*)userContext;
    m3World* world = ctx->world;
    const m3ExplosionDef* def = ctx->def;
    uint8_t type = world->shapeType[shape];
    if (type == (uint8_t)m3_voxelShape)
    {
        // The carve couples the blast to destruction (13-3): one
        // call bites the chunk, the fracture sweep frees islands,
        // and the fragment events carry the rest to the host.
        if (def->voxelCarve > 0.0f)
        {
            m3Transform vxf = m3ShapeWorldTransform(world, shape);
            m3Vec3 vlocal = m3InvRotateVec3(vxf.q, (m3Vec3){(m3real)(def->position.x - vxf.p.x),
                                                            (m3real)(def->position.y - vxf.p.y),
                                                            (m3real)(def->position.z - vxf.p.z)});
            m3VoxelCarveSphereInternal(world, shape, vlocal, def->voxelCarve);
        }
        return true;
    }
    if (type != (uint8_t)m3_sphereShape && type != (uint8_t)m3_capsuleShape &&
        type != (uint8_t)m3_hullShape)
    {
        return true; // meshes and planes are static scenery
    }
    int32_t body = world->shapeBody[shape];
    if (world->types[body] != (uint8_t)m3_dynamicBody || world->bodyEnabled[body] == 0 ||
        world->invMass[body] <= 0.0f)
    {
        return true;
    }
    if (!m3FilterPass(def->filter.categoryBits, def->filter.maskBits, world->shapeCategory[shape],
                      world->shapeMask[shape]))
    {
        return true;
    }
    // Work in the shape's local frame so distance and direction stay
    // precise far from the origin (the reference recentering).
    m3Transform xf = m3ShapeWorldTransform(world, shape);
    m3Vec3 local = m3InvRotateVec3(xf.q, (m3Vec3){(m3real)(def->position.x - xf.p.x),
                                                  (m3real)(def->position.y - xf.p.y),
                                                  (m3real)(def->position.z - xf.p.z)});
    m3Vec3 scratch[2];
    m3DistanceProxy proxy = m3MakeShapeProxy(world, shape, scratch);
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
    m3real surface = out.distance - proxy.radius;
    if (surface > def->radius + def->falloff)
    {
        return true;
    }
    // The blast wakes everything it reaches, sleepers included.
    m3SetAwakeInternal(world, body, 1);
    m3Vec3 direction;
    m3Vec3 contact;
    if (out.distance > 1e-6f && surface > 1e-6f)
    {
        direction = m3Normalize3(m3Sub3(out.pointA, point));
        contact = m3Sub3(out.pointA, m3MulSV3(proxy.radius, direction));
    }
    else
    {
        // The center sits inside the shape: push through the
        // centroid (the reference fallback), fixed axis when even
        // that is degenerate.
        m3Vec3 centroid = ShapeLocalCentroid(world, shape);
        m3Vec3 d = m3Sub3(centroid, point);
        direction = m3Dot3(d, d) > 1e-10f ? m3Normalize3(d) : (m3Vec3){1.0f, 0.0f, 0.0f};
        contact = centroid;
    }
    m3real scale = 1.0f;
    if (surface > def->radius && def->falloff > 0.0f)
    {
        scale = (def->radius + def->falloff - surface) / def->falloff;
        scale = scale < 0.0f ? 0.0f : (scale > 1.0f ? 1.0f : scale);
    }
    m3real magnitude = def->impulsePerArea * ShapeProjectedArea(world, shape, direction) * scale;
    if (magnitude != 0.0f)
    {
        m3Vec3 impulse = m3MulSV3(magnitude, m3RotateVec3(xf.q, direction));
        m3Vec3 arm = m3RotateVec3(xf.q, contact);
        m3Pos3 at = {xf.p.x + (double)arm.x, xf.p.y + (double)arm.y, xf.p.z + (double)arm.z};
        m3ApplyImpulseAtPointInternal(world, body, impulse, at);
    }
    return true;
}

bool m3WorldExplodeInternal(m3World* world, const m3ExplosionDef* def)
{
    if (!m3FinitePos3(def->position) || !m3FiniteF(def->radius) || def->radius < 0.0f ||
        !m3FiniteF(def->falloff) || def->falloff < 0.0f || !m3FiniteF(def->impulsePerArea) ||
        !m3FiniteF(def->voxelCarve) || def->voxelCarve < 0.0f || !m3FiniteF(def->softPush))
    {
        return false;
    }
    double reach = (double)def->radius + (double)def->falloff;
    double extent = reach > (double)def->voxelCarve ? reach : (double)def->voxelCarve;
    double lo[3] = {def->position.x - extent, def->position.y - extent, def->position.z - extent};
    double hi[3] = {def->position.x + extent, def->position.y + extent, def->position.z + extent};
    m3ExplodeContext ctx = {world, def};
    m3TreeQuery(&world->tree, lo, hi, ExplodeCallback, &ctx);
    // Soft particles: a canonical linear pass over the pool. Verlet
    // has no velocity to poke, so the push lands as a pending kick
    // the next step integrates exactly once (13-3).
    if (def->softPush != 0.0f && def->impulsePerArea != 0.0f)
    {
        int32_t maxSoft = world->softPool.maxIndex;
        for (int32_t slot = 0; slot < maxSoft; ++slot)
        {
            if (world->softPool.alive[slot] == 0)
            {
                continue;
            }
            int32_t count = world->softParticleCount[slot];
            int32_t base = slot * M3_SOFTBODY_MAX_PARTICLES;
            m3real pr = world->softRadius[slot];
            m3real area = M3_PI * pr * pr;
            for (int32_t i = 0; i < count; ++i)
            {
                int32_t k = base + i;
                if (world->softInvMass[k] == 0.0f)
                {
                    continue;
                }
                m3Vec3 d = {(m3real)(world->softPos[k].x - def->position.x),
                            (m3real)(world->softPos[k].y - def->position.y),
                            (m3real)(world->softPos[k].z - def->position.z)};
                m3real dist = m3Length3(d);
                m3real surface = dist - pr;
                if (surface > def->radius + def->falloff)
                {
                    continue;
                }
                m3Vec3 direction =
                    dist > 1e-6f ? m3MulSV3(1.0f / dist, d) : (m3Vec3){1.0f, 0.0f, 0.0f};
                m3real scale = 1.0f;
                if (surface > def->radius && def->falloff > 0.0f)
                {
                    scale = (def->radius + def->falloff - surface) / def->falloff;
                    scale = scale < 0.0f ? 0.0f : (scale > 1.0f ? 1.0f : scale);
                }
                m3real magnitude = def->impulsePerArea * def->softPush * area * scale;
                world->softKick[k] = m3Add3(world->softKick[k],
                                            m3MulSV3(magnitude * world->softInvMass[k], direction));
            }
        }
    }
    return true;
}

m3ExplosionDef m3DefaultExplosionDef(void)
{
    m3ExplosionDef def;
    memset(&def, 0, sizeof(def));
    def.filter = m3DefaultQueryFilter();
    def.radius = 10.0f;
    def.falloff = 5.0f;
    def.impulsePerArea = 0.0f;
    def.voxelCarve = 0.0f;
    def.softPush = 1.0f;
    def.internalValue = M3_EXPLOSION_COOKIE;
    return def;
}

void m3World_Explode(m3WorldId worldId, const m3ExplosionDef* def)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || def == NULL || def->internalValue != M3_EXPLOSION_COOKIE)
    {
        return;
    }
    if (!m3WorldExplodeInternal(world, def))
    {
        return; // hostile fields apply nothing and journal nothing
    }
    if (world->journalActive != 0)
    {
        m3JournalRecord(world, m3_opWorldExplode, def, (int32_t)sizeof(*def));
    }
}

// --- Contact readback (14-3) ------------------------------------------------

static void FillContactData(const m3World* world, int32_t pair, m3ContactData* out)
{
    const m3Manifold* manifold = &world->manifolds[pair];
    uint64_t key = world->pairKeys[pair];
    int32_t shapeA = (int32_t)(key >> 32);
    int32_t shapeB = (int32_t)(key & 0xFFFFFFFFu);
    out->shapeA = (m3ShapeId){shapeA + 1, world->worldIndex0, world->shapePool.generations[shapeA]};
    out->shapeB = (m3ShapeId){shapeB + 1, world->worldIndex0, world->shapePool.generations[shapeB]};
    out->normal = manifold->normal;
    int32_t count = manifold->pointCount;
    out->pointCount = count;
    int32_t bodyA = world->shapeBody[shapeA];
    m3Vec3 rcA = m3RotateVec3(world->transforms[bodyA].q, world->localCenters[bodyA]);
    for (int32_t k = 0; k < count; ++k)
    {
        const m3ManifoldPoint* point = &world->manifolds[pair].points[k];
        // Anchors are measured from body A's center in world axes:
        // COM plus anchor is the world contact point at read time.
        out->points[k] =
            (m3Pos3){world->transforms[bodyA].p.x + (double)(rcA.x + point->anchorA.x),
                     world->transforms[bodyA].p.y + (double)(rcA.y + point->anchorA.y),
                     world->transforms[bodyA].p.z + (double)(rcA.z + point->anchorA.z)};
        out->separations[k] = point->separation;
        out->normalImpulses[k] = point->normalImpulse;
    }
    for (int32_t k = count; k < 4; ++k)
    {
        out->points[k] = (m3Pos3){0.0, 0.0, 0.0};
        out->separations[k] = 0.0f;
        out->normalImpulses[k] = 0.0f;
    }
}

int32_t m3Shape_GetContactData(m3ShapeId shapeId, m3ContactData* out, int32_t capacity)
{
    m3World* world = m3WorldFromIndex0(shapeId.world0);
    if (world == NULL || out == NULL || capacity <= 0)
    {
        return 0;
    }
    int32_t shape = shapeId.index1 - 1;
    if (shape < 0 || shape >= world->shapePool.maxIndex || world->shapePool.alive[shape] == 0 ||
        world->shapePool.generations[shape] != shapeId.generation)
    {
        return 0;
    }
    int32_t written = 0;
    for (int32_t i = 0; i < world->pairCount && written < capacity; ++i)
    {
        uint64_t key = world->pairKeys[i];
        int32_t shapeA = (int32_t)(key >> 32);
        int32_t shapeB = (int32_t)(key & 0xFFFFFFFFu);
        if ((shapeA == shape || shapeB == shape) && world->manifolds[i].pointCount > 0)
        {
            FillContactData(world, i, &out[written]);
            written += 1;
        }
    }
    return written;
}

int32_t m3Body_GetContactData(m3BodyId bodyId, m3ContactData* out, int32_t capacity)
{
    m3World* world = m3WorldFromIndex0(bodyId.world0);
    if (world == NULL || out == NULL || capacity <= 0)
    {
        return 0;
    }
    int32_t body = m3BodySlot(world, bodyId);
    if (body < 0)
    {
        return 0;
    }
    int32_t written = 0;
    for (int32_t i = 0; i < world->pairCount && written < capacity; ++i)
    {
        uint64_t key = world->pairKeys[i];
        int32_t shapeA = (int32_t)(key >> 32);
        int32_t shapeB = (int32_t)(key & 0xFFFFFFFFu);
        if ((world->shapeBody[shapeA] == body || world->shapeBody[shapeB] == body) &&
            world->manifolds[i].pointCount > 0)
        {
            FillContactData(world, i, &out[written]);
            written += 1;
        }
    }
    return written;
}

// --- Proxy overlaps (15-3) --------------------------------------------------

typedef struct m3ProxyOverlapContext
{
    m3World* world;
    m3Pos3 base;
    const m3Vec3* points; // base-relative query cloud, <= 64
    int32_t pointCount;
    m3real radius;
    double lo[3];
    double hi[3];
    int32_t indices[256];
    int32_t count;
    m3QueryFilter filter;
} m3ProxyOverlapContext;

// GJK between the localized query cloud and an arbitrary point set
// (a shape core, a voxel merged box, a mesh triangle), radii applied
// analytically like the whole distance family.
static int ProxyCloudReach(const m3Vec3* cloud, int32_t cloudCount, m3real cloudRadius,
                           const m3Vec3* target, int32_t targetCount, m3real targetRadius)
{
    m3DistanceInput input;
    memset(&input, 0, sizeof(input));
    input.proxyA.points = target;
    input.proxyA.count = targetCount;
    input.proxyA.radius = 0.0f;
    input.proxyB.points = cloud;
    input.proxyB.count = cloudCount;
    input.proxyB.radius = 0.0f;
    input.q = m3MakeIdentityQuat();
    input.p = (m3Vec3){0.0f, 0.0f, 0.0f};
    input.useRadii = false;
    m3SimplexCache cache;
    cache.count = 0;
    cache.metric = 0.0f;
    m3DistanceOutput out = m3ShapeDistance(&input, &cache);
    return out.distance <= cloudRadius + targetRadius;
}

static int ProxyReachesShape(const m3ProxyOverlapContext* ctx, int32_t shape)
{
    m3World* world = ctx->world;
    if (ctx->pointCount < 1)
    {
        // The public walls refuse empty clouds already; this guard
        // exists so every compiler can PROVE local[0] is written
        // below (mingw's maybe-uninitialized fired on the release
        // asset build, the one gcc the CI matrix does not run).
        return 0;
    }
    uint8_t type = world->shapeType[shape];
    m3Transform xf = m3ShapeWorldTransform(world, shape);
    m3Vec3 local[64];
    for (int32_t k = 0; k < ctx->pointCount; ++k)
    {
        m3Vec3 w = {(m3real)(ctx->base.x + (double)ctx->points[k].x - xf.p.x),
                    (m3real)(ctx->base.y + (double)ctx->points[k].y - xf.p.y),
                    (m3real)(ctx->base.z + (double)ctx->points[k].z - xf.p.z)};
        local[k] = m3InvRotateVec3(xf.q, w);
    }
    if (type == (uint8_t)m3_planeShape)
    {
        m3real best = 0.0f;
        for (int32_t k = 0; k < ctx->pointCount; ++k)
        {
            m3real d = m3Dot3(world->shapeGeom[shape].v, local[k]) - world->shapeGeom[shape].s;
            if (k == 0 || d < best)
            {
                best = d;
            }
        }
        return best <= ctx->radius;
    }
    if (type == (uint8_t)m3_voxelShape)
    {
        int32_t slot = world->shapeVoxelIndex[shape];
        const m3VoxelSurface* surface = &world->voxelSurface[slot];
        m3real cell = world->voxelData[slot].cellSize;
        m3Vec3 blo = local[0];
        m3Vec3 bhi = local[0];
        for (int32_t k = 1; k < ctx->pointCount; ++k)
        {
            blo.x = local[k].x < blo.x ? local[k].x : blo.x;
            blo.y = local[k].y < blo.y ? local[k].y : blo.y;
            blo.z = local[k].z < blo.z ? local[k].z : blo.z;
            bhi.x = local[k].x > bhi.x ? local[k].x : bhi.x;
            bhi.y = local[k].y > bhi.y ? local[k].y : bhi.y;
            bhi.z = local[k].z > bhi.z ? local[k].z : bhi.z;
        }
        blo = (m3Vec3){blo.x - ctx->radius, blo.y - ctx->radius, blo.z - ctx->radius};
        bhi = (m3Vec3){bhi.x + ctx->radius, bhi.y + ctx->radius, bhi.z + ctx->radius};
        uint16_t gather[M3_MESH_MAX_TRIS];
        int32_t gatherCount = m3MeshBvhGather(&surface->bvh, blo, bhi, gather);
        for (int32_t g = 0; g < gatherCount; ++g)
        {
            m3Vec3 lo;
            m3Vec3 hi;
            m3VoxelBoxBounds(surface, cell, gather[g], &lo, &hi);
            m3Vec3 corners[8];
            for (int32_t c = 0; c < 8; ++c)
            {
                corners[c] = (m3Vec3){(c & 1) != 0 ? hi.x : lo.x, (c & 2) != 0 ? hi.y : lo.y,
                                      (c & 4) != 0 ? hi.z : lo.z};
            }
            if (ProxyCloudReach(local, ctx->pointCount, ctx->radius, corners, 8, 0.0f))
            {
                return 1;
            }
        }
        return 0;
    }
    if (type == (uint8_t)m3_heightFieldShape)
    {
        // The overlap family sees terrain (19-3): cloud box, cell
        // gather, the same reach test per triangle.
        const m3HeightFieldData* hf = &world->hfData[world->shapeHfIndex[shape]];
        m3Vec3 hlo = local[0];
        m3Vec3 hhi = local[0];
        for (int32_t k = 1; k < ctx->pointCount; ++k)
        {
            hlo.x = local[k].x < hlo.x ? local[k].x : hlo.x;
            hlo.y = local[k].y < hlo.y ? local[k].y : hlo.y;
            hlo.z = local[k].z < hlo.z ? local[k].z : hlo.z;
            hhi.x = local[k].x > hhi.x ? local[k].x : hhi.x;
            hhi.y = local[k].y > hhi.y ? local[k].y : hhi.y;
            hhi.z = local[k].z > hhi.z ? local[k].z : hhi.z;
        }
        hlo = (m3Vec3){hlo.x - ctx->radius, hlo.y - ctx->radius, hlo.z - ctx->radius};
        hhi = (m3Vec3){hhi.x + ctx->radius, hhi.y + ctx->radius, hhi.z + ctx->radius};
        m3Vec3 hfTris[512][3];
        int32_t hfCount = m3HeightFieldGather(hf, hlo, hhi, hfTris, 512);
        for (int32_t t = 0; t < hfCount; ++t)
        {
            if (ProxyCloudReach(local, ctx->pointCount, ctx->radius, hfTris[t], 3, 0.0f))
            {
                return 1;
            }
        }
        return 0;
    }
    if (type == (uint8_t)m3_meshShape)
    {
        const m3MeshData* mesh = &world->meshData[world->shapeMeshIndex[shape]];
        m3Vec3 blo = local[0];
        m3Vec3 bhi = local[0];
        for (int32_t k = 1; k < ctx->pointCount; ++k)
        {
            blo.x = local[k].x < blo.x ? local[k].x : blo.x;
            blo.y = local[k].y < blo.y ? local[k].y : blo.y;
            blo.z = local[k].z < blo.z ? local[k].z : blo.z;
            bhi.x = local[k].x > bhi.x ? local[k].x : bhi.x;
            bhi.y = local[k].y > bhi.y ? local[k].y : bhi.y;
            bhi.z = local[k].z > bhi.z ? local[k].z : bhi.z;
        }
        blo = (m3Vec3){blo.x - ctx->radius, blo.y - ctx->radius, blo.z - ctx->radius};
        bhi = (m3Vec3){bhi.x + ctx->radius, bhi.y + ctx->radius, bhi.z + ctx->radius};
        uint16_t gather[M3_MESH_MAX_TRIS];
        int32_t gatherCount =
            m3MeshBvhGather(&world->meshBvh[world->shapeMeshIndex[shape]], blo, bhi, gather);
        for (int32_t g = 0; g < gatherCount; ++g)
        {
            int32_t t = gather[g];
            m3Vec3 tri[3] = {mesh->vertices[mesh->indices[3 * t + 0]],
                             mesh->vertices[mesh->indices[3 * t + 1]],
                             mesh->vertices[mesh->indices[3 * t + 2]]};
            if (ProxyCloudReach(local, ctx->pointCount, ctx->radius, tri, 3, 0.0f))
            {
                return 1;
            }
        }
        return 0;
    }
    m3Vec3 scratch[2];
    m3DistanceProxy proxy = m3MakeShapeProxy(world, shape, scratch);
    return ProxyCloudReach(local, ctx->pointCount, ctx->radius, proxy.points, proxy.count,
                           proxy.radius);
}

static bool ProxyOverlapCallback(int32_t shape, void* userContext)
{
    m3ProxyOverlapContext* ctx = (m3ProxyOverlapContext*)userContext;
    if (ctx->world->bodyEnabled[ctx->world->shapeBody[shape]] == 0)
    {
        return true;
    }
    if (!m3FilterPass(ctx->filter.categoryBits, ctx->filter.maskBits,
                      ctx->world->shapeCategory[shape], ctx->world->shapeMask[shape]))
    {
        return true;
    }
    if (ctx->count >= 256)
    {
        return true;
    }
    if (!ProxyReachesShape(ctx, shape))
    {
        return true;
    }
    ctx->indices[ctx->count++] = shape;
    return true;
}

static int32_t ProxyOverlapGather(m3World* world, m3ProxyOverlapContext* ctx, m3ShapeId* shapes,
                                  int32_t capacity)
{
    m3TreeQuery(&world->tree, ctx->lo, ctx->hi, ProxyOverlapCallback, ctx);
    int32_t maxShape = world->shapePool.maxIndex;
    for (int32_t s = 0; s < maxShape && ctx->count < 256; ++s)
    {
        if (world->shapePool.alive[s] != 0 && world->shapeType[s] == (uint8_t)m3_planeShape &&
            world->bodyEnabled[world->shapeBody[s]] != 0 &&
            m3FilterPass(ctx->filter.categoryBits, ctx->filter.maskBits, world->shapeCategory[s],
                         world->shapeMask[s]) &&
            ProxyReachesShape(ctx, s))
        {
            ctx->indices[ctx->count++] = s;
        }
    }
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

int32_t m3World_OverlapHullPointsEx(m3WorldId worldId, m3Pos3 base, const m3Vec3* points,
                                    int32_t count, m3real radius, m3ShapeId* shapes,
                                    int32_t capacity, m3QueryFilter filter)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || points == NULL || count < 1 || count > 64 || shapes == NULL ||
        capacity <= 0 || !m3FinitePos3(base) || !m3FiniteF(radius) || radius < 0.0f)
    {
        return 0;
    }
    for (int32_t k = 0; k < count; ++k)
    {
        if (!m3FiniteV3(points[k]))
        {
            return 0;
        }
    }
    m3ProxyOverlapContext ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.world = world;
    ctx.base = base;
    ctx.points = points;
    ctx.pointCount = count;
    ctx.radius = radius;
    ctx.filter = filter;
    ctx.lo[0] = base.x + (double)points[0].x;
    ctx.lo[1] = base.y + (double)points[0].y;
    ctx.lo[2] = base.z + (double)points[0].z;
    ctx.hi[0] = ctx.lo[0];
    ctx.hi[1] = ctx.lo[1];
    ctx.hi[2] = ctx.lo[2];
    for (int32_t k = 1; k < count; ++k)
    {
        double x = base.x + (double)points[k].x;
        double y = base.y + (double)points[k].y;
        double z = base.z + (double)points[k].z;
        ctx.lo[0] = x < ctx.lo[0] ? x : ctx.lo[0];
        ctx.lo[1] = y < ctx.lo[1] ? y : ctx.lo[1];
        ctx.lo[2] = z < ctx.lo[2] ? z : ctx.lo[2];
        ctx.hi[0] = x > ctx.hi[0] ? x : ctx.hi[0];
        ctx.hi[1] = y > ctx.hi[1] ? y : ctx.hi[1];
        ctx.hi[2] = z > ctx.hi[2] ? z : ctx.hi[2];
    }
    for (int32_t a = 0; a < 3; ++a)
    {
        ctx.lo[a] -= (double)radius;
        ctx.hi[a] += (double)radius;
    }
    return ProxyOverlapGather(world, &ctx, shapes, capacity);
}

int32_t m3World_OverlapHullPoints(m3WorldId worldId, m3Pos3 base, const m3Vec3* points,
                                  int32_t count, m3real radius, m3ShapeId* shapes, int32_t capacity)
{
    return m3World_OverlapHullPointsEx(worldId, base, points, count, radius, shapes, capacity,
                                       m3DefaultQueryFilter());
}

int32_t m3World_OverlapCapsuleEx(m3WorldId worldId, m3Pos3 p1, m3Pos3 p2, m3real radius,
                                 m3ShapeId* shapes, int32_t capacity, m3QueryFilter filter)
{
    if (!m3FinitePos3(p1) || !m3FinitePos3(p2))
    {
        return 0;
    }
    m3Vec3 pts[2] = {{0.0f, 0.0f, 0.0f},
                     {(m3real)(p2.x - p1.x), (m3real)(p2.y - p1.y), (m3real)(p2.z - p1.z)}};
    return m3World_OverlapHullPointsEx(worldId, p1, pts, 2, radius, shapes, capacity, filter);
}

int32_t m3World_OverlapCapsule(m3WorldId worldId, m3Pos3 p1, m3Pos3 p2, m3real radius,
                               m3ShapeId* shapes, int32_t capacity)
{
    return m3World_OverlapCapsuleEx(worldId, p1, p2, radius, shapes, capacity,
                                    m3DefaultQueryFilter());
}

int32_t m3World_OverlapBoxEx(m3WorldId worldId, m3Pos3 center, m3Vec3 halfExtents, m3Quat rotation,
                             m3ShapeId* shapes, int32_t capacity, m3QueryFilter filter)
{
    if (!m3FiniteV3(halfExtents) || !(halfExtents.x > 0.0f) || !(halfExtents.y > 0.0f) ||
        !(halfExtents.z > 0.0f) || !m3FiniteQuat(rotation))
    {
        return 0;
    }
    m3Vec3 corners[8];
    for (int32_t c = 0; c < 8; ++c)
    {
        m3Vec3 e = {(c & 1) != 0 ? halfExtents.x : -halfExtents.x,
                    (c & 2) != 0 ? halfExtents.y : -halfExtents.y,
                    (c & 4) != 0 ? halfExtents.z : -halfExtents.z};
        corners[c] = m3RotateVec3(rotation, e);
    }
    return m3World_OverlapHullPointsEx(worldId, center, corners, 8, 0.0f, shapes, capacity, filter);
}

int32_t m3World_OverlapBox(m3WorldId worldId, m3Pos3 center, m3Vec3 halfExtents, m3Quat rotation,
                           m3ShapeId* shapes, int32_t capacity)
{
    return m3World_OverlapBoxEx(worldId, center, halfExtents, rotation, shapes, capacity,
                                m3DefaultQueryFilter());
}
