// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Closest-hit ray casting (2b-13). The world cast localizes the
// double origin per shape (the hybrid precision pattern), runs an
// analytic kernel per shape family in the body frame, and keeps the
// smallest fraction with ties to the lower shape index. Candidates
// come from the tree via the ray's bounding box plus the dedicated
// plane and nothing-else passes; a descending ray traversal is a
// performance refinement that joins the BVH work when profiles ask.
// Front faces only, everywhere: winding is a contract.

#include "world_internal.h"

#include <float.h>
#include <string.h>

typedef struct m3RayLocalHit
{
    m3real fraction; // along the full translation
    m3Vec3 normal;   // body frame
    int hit;
} m3RayLocalHit;

// Sphere: |o + t*d - c|^2 = r^2, smallest root in [0, 1].
static m3RayLocalHit RaySphere(m3Vec3 o, m3Vec3 d, m3Vec3 center, m3real radius)
{
    m3RayLocalHit out = {0.0f, {0.0f, 0.0f, 0.0f}, 0};
    m3Vec3 m = m3Sub3(o, center);
    m3real a = m3Dot3(d, d);
    m3real b = m3Dot3(m, d);
    m3real c = m3Dot3(m, m) - radius * radius;
    if (c <= 0.0f)
    {
        return out; // inside OR on the surface: no front-face hit
    }
    m3real disc = b * b - a * c;
    if (!(a > 0.0f) || disc < 0.0f)
    {
        return out;
    }
    m3real t = (-b - sqrtf(disc)) / a;
    if (t < 0.0f || t > 1.0f)
    {
        return out;
    }
    m3Vec3 p = m3Add3(o, m3MulSV3(t, d));
    out.fraction = t;
    out.normal = m3Normalize3(m3Sub3(p, center));
    out.hit = 1;
    return out;
}

// Capsule: the infinite cylinder about the segment plus both cap
// spheres; smallest valid root wins.
static m3RayLocalHit RayCapsule(m3Vec3 o, m3Vec3 d, m3Vec3 p1, m3Vec3 p2, m3real radius)
{
    m3RayLocalHit best = {0.0f, {0.0f, 0.0f, 0.0f}, 0};
    m3Vec3 axis = m3Sub3(p2, p1);
    m3real axisLen2 = m3Dot3(axis, axis);

    // Cylinder part: components perpendicular to the axis.
    if (axisLen2 > 0.0f)
    {
        m3Vec3 m = m3Sub3(o, p1);
        m3real md = m3Dot3(m, axis);
        m3real dd = m3Dot3(d, axis);
        m3Vec3 mPerp = m3Sub3(m, m3MulSV3(md / axisLen2, axis));
        m3Vec3 dPerp = m3Sub3(d, m3MulSV3(dd / axisLen2, axis));
        m3real a = m3Dot3(dPerp, dPerp);
        m3real b = m3Dot3(mPerp, dPerp);
        m3real c = m3Dot3(mPerp, mPerp) - radius * radius;
        if (c > 0.0f && a > 0.0f)
        {
            m3real disc = b * b - a * c;
            if (disc >= 0.0f)
            {
                m3real t = (-b - sqrtf(disc)) / a;
                if (t >= 0.0f && t <= 1.0f)
                {
                    // Accept only between the cap planes.
                    m3real s = md + t * dd;
                    if (s >= 0.0f && s <= axisLen2)
                    {
                        m3Vec3 p = m3Add3(o, m3MulSV3(t, d));
                        m3Vec3 onAxis = m3Add3(p1, m3MulSV3(s / axisLen2, axis));
                        best.fraction = t;
                        best.normal = m3Normalize3(m3Sub3(p, onAxis));
                        best.hit = 1;
                    }
                }
            }
        }
    }

    // Cap spheres.
    m3RayLocalHit cap1 = RaySphere(o, d, p1, radius);
    if (cap1.hit && (!best.hit || cap1.fraction < best.fraction))
    {
        best = cap1;
    }
    m3RayLocalHit cap2 = RaySphere(o, d, p2, radius);
    if (cap2.hit && (!best.hit || cap2.fraction < best.fraction))
    {
        best = cap2;
    }
    return best;
}

// Convex hull: clip the ray against every face plane (enter on
// front-facing planes, exit on back-facing); classic slab logic.
static m3RayLocalHit RayHull(m3Vec3 o, m3Vec3 d, const m3HullData* hull)
{
    m3RayLocalHit out = {0.0f, {0.0f, 0.0f, 0.0f}, 0};
    m3real tEnter = 0.0f;
    m3real tExit = 1.0f;
    m3Vec3 enterNormal = {0.0f, 0.0f, 0.0f};
    int haveEnter = 0;
    for (int32_t f = 0; f < hull->faceCount; ++f)
    {
        m3Vec3 n = hull->faceNormals[f];
        m3real dist = m3Dot3(n, o) - hull->faceOffsets[f];
        m3real denom = m3Dot3(n, d);
        if (denom == 0.0f)
        {
            if (dist > 0.0f)
            {
                return out; // parallel and outside this face: miss
            }
            continue;
        }
        m3real t = -dist / denom;
        if (denom < 0.0f)
        {
            // Entering through this plane.
            if (t > tEnter)
            {
                tEnter = t;
                enterNormal = n;
                haveEnter = 1;
            }
        }
        else
        {
            // Exiting through this plane.
            tExit = m3MinF(tExit, t);
        }
        if (tEnter > tExit)
        {
            return out;
        }
    }
    if (!haveEnter)
    {
        return out; // started inside: no front-face hit
    }
    out.fraction = tEnter;
    out.normal = enterNormal;
    out.hit = 1;
    return out;
}

// Mesh: bounded per-triangle scan, front faces only.
// Ray versus the merged-box surface: slab clip per candidate box,
// front faces only, a ray starting inside reports a miss (the ray
// contract; ask point-inside for containment). Ascending candidates
// keep tie winners at the lowest box index.
static m3RayLocalHit RayVoxel(m3Vec3 o, m3Vec3 d, const m3VoxelSurface* surface, m3real cellSize)
{
    m3RayLocalHit best = {0.0f, {0.0f, 0.0f, 0.0f}, 0};
    m3Vec3 end = m3Add3(o, d);
    m3Vec3 blo = {m3MinF(o.x, end.x), m3MinF(o.y, end.y), m3MinF(o.z, end.z)};
    m3Vec3 bhi = {m3MaxF(o.x, end.x), m3MaxF(o.y, end.y), m3MaxF(o.z, end.z)};
    uint16_t gather[M3_MESH_MAX_TRIS];
    int32_t gatherCount = m3MeshBvhGather(&surface->bvh, blo, bhi, gather);
    for (int32_t g = 0; g < gatherCount; ++g)
    {
        m3Vec3 lo;
        m3Vec3 hi;
        m3VoxelBoxBounds(surface, cellSize, gather[g], &lo, &hi);
        m3real tEnter = 0.0f;
        m3real tExit = 1.0f;
        int32_t enterAxis = -1;
        m3real enterSign = 0.0f;
        bool miss = false;
        for (int32_t k = 0; k < 3 && !miss; ++k)
        {
            m3real ok = k == 0 ? o.x : (k == 1 ? o.y : o.z);
            m3real dk = k == 0 ? d.x : (k == 1 ? d.y : d.z);
            m3real lok = k == 0 ? lo.x : (k == 1 ? lo.y : lo.z);
            m3real hik = k == 0 ? hi.x : (k == 1 ? hi.y : hi.z);
            if (dk == 0.0f)
            {
                if (ok < lok || ok > hik)
                {
                    miss = true;
                }
                continue;
            }
            m3real t1 = (lok - ok) / dk;
            m3real t2 = (hik - ok) / dk;
            m3real sign = -1.0f;
            if (t1 > t2)
            {
                m3real tmp = t1;
                t1 = t2;
                t2 = tmp;
                sign = 1.0f;
            }
            if (t1 > tEnter)
            {
                tEnter = t1;
                enterAxis = k;
                enterSign = sign;
            }
            tExit = m3MinF(tExit, t2);
        }
        if (miss || tEnter > tExit || enterAxis < 0)
        {
            continue; // no entry face inside [0, 1]: a ray born
                      // inside the box has no front face to hit
        }
        if (best.hit && tEnter >= best.fraction)
        {
            continue;
        }
        m3Vec3 n = {0.0f, 0.0f, 0.0f};
        if (enterAxis == 0)
        {
            n.x = enterSign;
        }
        else if (enterAxis == 1)
        {
            n.y = enterSign;
        }
        else
        {
            n.z = enterSign;
        }
        best.hit = 1;
        best.fraction = tEnter;
        best.normal = n;
    }
    return best;
}

static m3RayLocalHit RayMesh(m3Vec3 o, m3Vec3 d, const m3MeshData* mesh, const m3MeshBvh* bvh)
{
    m3RayLocalHit best = {0.0f, {0.0f, 0.0f, 0.0f}, 0};
    // Segment box gather (2c-10): a hit point lies on the segment and
    // in the triangle, so it lies in both boxes; the pruned set is a
    // safe superset and ascending order keeps tie winners identical.
    m3Vec3 end = m3Add3(o, d);
    m3Vec3 blo = {m3MinF(o.x, end.x), m3MinF(o.y, end.y), m3MinF(o.z, end.z)};
    m3Vec3 bhi = {m3MaxF(o.x, end.x), m3MaxF(o.y, end.y), m3MaxF(o.z, end.z)};
    uint16_t gather[M3_MESH_MAX_TRIS];
    int32_t gatherCount = m3MeshBvhGather(bvh, blo, bhi, gather);
    for (int32_t g = 0; g < gatherCount; ++g)
    {
        int32_t t = gather[g];
        m3Vec3 a = mesh->vertices[mesh->indices[3 * t + 0]];
        m3Vec3 b = mesh->vertices[mesh->indices[3 * t + 1]];
        m3Vec3 c = mesh->vertices[mesh->indices[3 * t + 2]];
        m3Vec3 e1 = m3Sub3(b, a);
        m3Vec3 e2 = m3Sub3(c, a);
        m3Vec3 n = m3Cross3(e1, e2);
        m3real denom = m3Dot3(n, d);
        if (denom >= 0.0f)
        {
            continue; // back face or parallel
        }
        m3real dist = m3Dot3(n, m3Sub3(o, a));
        m3real tHit = dist / -denom; // denom < 0: tHit >= 0 when o is in front
        if (tHit < 0.0f || tHit > 1.0f)
        {
            continue;
        }
        if (best.hit && tHit >= best.fraction)
        {
            continue;
        }
        // Inside test via edge cross products.
        m3Vec3 p = m3Add3(o, m3MulSV3(tHit, d));
        m3Vec3 ap = m3Sub3(p, a);
        m3Vec3 bp = m3Sub3(p, b);
        m3Vec3 cp = m3Sub3(p, c);
        if (m3Dot3(m3Cross3(e1, ap), n) < 0.0f || m3Dot3(m3Cross3(m3Sub3(c, b), bp), n) < 0.0f ||
            m3Dot3(m3Cross3(m3Sub3(a, c), cp), n) < 0.0f)
        {
            continue;
        }
        best.fraction = tHit;
        best.normal = m3Normalize3(n);
        best.hit = 1;
    }
    return best;
}

static m3RayLocalHit RayHeightField(m3Vec3 o, m3Vec3 d, const m3HeightFieldData* hf)
{
    // Cell-span scan (19-3): the segment's XZ box picks the cells;
    // each contributes its two parity triangles to the same
    // front-face test the mesh path runs. A long diagonal ray
    // scans its whole span box: honest, deterministic, and the DDA
    // walk stays on the ledger for a profiling day.
    m3RayLocalHit best = {0.0f, {0.0f, 0.0f, 0.0f}, 0};
    m3Vec3 end = m3Add3(o, d);
    m3real inv = 1.0f / hf->cellSize;
    int32_t cx0 = m3CellFromF(floorf(m3MinF(o.x, end.x) * inv), 2.0e9f);
    int32_t cx1 = m3CellFromF(floorf(m3MaxF(o.x, end.x) * inv), -2.0e9f);
    int32_t cz0 = m3CellFromF(floorf(m3MinF(o.z, end.z) * inv), 2.0e9f);
    int32_t cz1 = m3CellFromF(floorf(m3MaxF(o.z, end.z) * inv), -2.0e9f);
    cx0 = cx0 < 0 ? 0 : cx0;
    cz0 = cz0 < 0 ? 0 : cz0;
    cx1 = cx1 > hf->nx - 2 ? hf->nx - 2 : cx1;
    cz1 = cz1 > hf->nz - 2 ? hf->nz - 2 : cz1;
    for (int32_t cz = cz0; cz <= cz1; ++cz)
    {
        for (int32_t cx = cx0; cx <= cx1; ++cx)
        {
            m3Vec3 cell[2][3];
            m3HeightFieldCellTris(hf, cx, cz, cell);
            for (int32_t t = 0; t < 2; ++t)
            {
                m3Vec3 a = cell[t][0];
                m3Vec3 b = cell[t][1];
                m3Vec3 c = cell[t][2];
                m3Vec3 e1 = m3Sub3(b, a);
                m3Vec3 e2 = m3Sub3(c, a);
                m3Vec3 n = m3Cross3(e1, e2);
                m3real denom = m3Dot3(n, d);
                if (denom >= 0.0f)
                {
                    continue;
                }
                m3real dist = m3Dot3(n, m3Sub3(o, a));
                m3real tHit = dist / -denom;
                if (tHit < 0.0f || tHit > 1.0f)
                {
                    continue;
                }
                if (best.hit && tHit >= best.fraction)
                {
                    continue;
                }
                m3Vec3 pnt = m3Add3(o, m3MulSV3(tHit, d));
                m3Vec3 ap = m3Sub3(pnt, a);
                m3Vec3 bp = m3Sub3(pnt, b);
                m3Vec3 cp = m3Sub3(pnt, c);
                if (m3Dot3(m3Cross3(e1, ap), n) < 0.0f ||
                    m3Dot3(m3Cross3(m3Sub3(c, b), bp), n) < 0.0f ||
                    m3Dot3(m3Cross3(m3Sub3(a, c), cp), n) < 0.0f)
                {
                    continue;
                }
                best.fraction = tHit;
                best.normal = m3Normalize3(n);
                best.hit = 1;
            }
        }
    }
    return best;
}

typedef struct m3RayCastContext
{
    m3World* world;
    m3Pos3 origin;
    m3Vec3 translation;
    m3RayHit best;
    int32_t bestShape;
    int32_t ignoreBody;   // -1 none: the suspension casts' self filter
    m3QueryFilter filter; // 8-1: the query behaves like a shape
} m3RayCastContext;

static void RayTestShape(m3RayCastContext* ctx, int32_t shape)
{
    m3World* world = ctx->world;
    int32_t body = world->shapeBody[shape];
    if (body == ctx->ignoreBody)
    {
        return; // the caller's own body never blocks its ray (4-4's
                // cast hook, extended to rays for the vehicle arc)
    }
    if (world->bodyEnabled[body] == 0)
    {
        return; // disabled bodies are invisible to rays (8-3)
    }
    if (!m3FilterPass(ctx->filter.categoryBits, ctx->filter.maskBits, world->shapeCategory[shape],
                      world->shapeMask[shape]))
    {
        return; // filtered out (8-1)
    }
    m3Transform xfS = m3ShapeWorldTransform(world, shape);
    const m3Transform* xf = &xfS;

    // Localize the double origin into the SHAPE frame (10-1).
    m3Vec3 rel = {(m3real)(ctx->origin.x - xf->p.x), (m3real)(ctx->origin.y - xf->p.y),
                  (m3real)(ctx->origin.z - xf->p.z)};
    m3Vec3 o = m3InvRotateVec3(xf->q, rel);
    m3Vec3 d = m3InvRotateVec3(xf->q, ctx->translation);

    m3RayLocalHit local = {0.0f, {0.0f, 0.0f, 0.0f}, 0};
    uint8_t type = world->shapeType[shape];
    if (type == (uint8_t)m3_sphereShape)
    {
        local = RaySphere(o, d, world->shapeGeom[shape].v, world->shapeGeom[shape].s);
    }
    else if (type == (uint8_t)m3_capsuleShape)
    {
        local = RayCapsule(o, d, world->shapeGeom[shape].v, world->shapeGeom[shape].v2,
                           world->shapeGeom[shape].s);
    }
    else if (type == (uint8_t)m3_hullShape)
    {
        local = RayHull(o, d, &world->hullData[world->shapeHullIndex[shape]]);
    }
    else if (type == (uint8_t)m3_meshShape)
    {
        local = RayMesh(o, d, &world->meshData[world->shapeMeshIndex[shape]],
                        &world->meshBvh[world->shapeMeshIndex[shape]]);
    }
    else if (type == (uint8_t)m3_heightFieldShape)
    {
        local = RayHeightField(o, d, &world->hfData[world->shapeHfIndex[shape]]);
    }
    else if (type == (uint8_t)m3_voxelShape)
    {
        local = RayVoxel(o, d, &world->voxelSurface[world->shapeVoxelIndex[shape]],
                         world->voxelData[world->shapeVoxelIndex[shape]].cellSize);
    }
    else if (type == (uint8_t)m3_planeShape)
    {
        m3Vec3 n = world->shapeGeom[shape].v;
        m3real dist = m3Dot3(n, o) - world->shapeGeom[shape].s;
        m3real denom = m3Dot3(n, d);
        if (dist >= 0.0f && denom < 0.0f)
        {
            m3real t = -dist / denom;
            if (t >= 0.0f && t <= 1.0f)
            {
                local.fraction = t;
                local.normal = n;
                local.hit = 1;
            }
        }
    }

    if (!local.hit)
    {
        return;
    }
    if (ctx->best.hit && (local.fraction > ctx->best.fraction ||
                          (local.fraction == ctx->best.fraction && shape >= ctx->bestShape)))
    {
        return; // farther, or the canonical lower-index tie loss
    }
    ctx->best.hit = true;
    ctx->best.fraction = local.fraction;
    ctx->best.normal = m3RotateVec3(xf->q, local.normal);
    ctx->best.point.x = ctx->origin.x + (double)(local.fraction * ctx->translation.x);
    ctx->best.point.y = ctx->origin.y + (double)(local.fraction * ctx->translation.y);
    ctx->best.point.z = ctx->origin.z + (double)(local.fraction * ctx->translation.z);
    ctx->best.shape =
        (m3ShapeId){shape + 1, world->worldIndex0, world->shapePool.generations[shape]};
    ctx->bestShape = shape;
}

// The single-shape test as an internal hook (the multi-hit query in
// query.c walks candidates itself and reuses this).
m3RayHit m3RayTestOneShape(m3World* world, int32_t shape, m3Pos3 origin, m3Vec3 translation)
{
    m3RayCastContext ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.ignoreBody = -1;                 // zero after memset would silently filter
                                         // body slot ZERO (the 5-1 lesson: a new
                                         // context field visits every constructor)
    ctx.filter = m3DefaultQueryFilter(); // the same lesson, 8-1
    ctx.best.fraction = 1.0f;
    ctx.world = world;
    ctx.origin = origin;
    ctx.translation = translation;
    ctx.bestShape = INT32_MAX;
    RayTestShape(&ctx, shape);
    return ctx.best;
}

static bool RayQueryCallback(int32_t shape, void* userContext)
{
    RayTestShape((m3RayCastContext*)userContext, shape);
    return true;
}

m3RayHit m3RayClosestFiltered(m3World* world, m3Pos3 origin, m3Vec3 translation, int32_t ignoreBody,
                              m3QueryFilter filter)
{
    m3RayCastContext ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.ignoreBody = ignoreBody;
    ctx.filter = filter;
    ctx.best.fraction = 1.0f;
    if (world == NULL || !(m3Dot3(translation, translation) > 0.0f) ||
        !(translation.x >= -M3_CAST_LIMIT && translation.x <= M3_CAST_LIMIT) ||
        !(translation.y >= -M3_CAST_LIMIT && translation.y <= M3_CAST_LIMIT) ||
        !(translation.z >= -M3_CAST_LIMIT && translation.z <= M3_CAST_LIMIT))
    {
        return ctx.best; // null world, zero ray, or a translation
                         // past the float budget: a clean miss
    }
    ctx.world = world;
    ctx.origin = origin;
    ctx.translation = translation;
    ctx.bestShape = INT32_MAX;

    // Candidates: the tree under the ray's bounding box (a coarse
    // superset; the kernels decide), then the plane pass.
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
    m3TreeQuery(&world->tree, lo, hi, RayQueryCallback, &ctx);

    int32_t maxShape = world->shapePool.maxIndex;
    for (int32_t s = 0; s < maxShape; ++s)
    {
        if (world->shapePool.alive[s] != 0 && world->shapeType[s] == (uint8_t)m3_planeShape)
        {
            RayTestShape(&ctx, s);
        }
    }
    return ctx.best;
}

m3RayHit m3RayClosestInternalEx(m3World* world, m3Pos3 origin, m3Vec3 translation,
                                int32_t ignoreBody)
{
    return m3RayClosestFiltered(world, origin, translation, ignoreBody, m3DefaultQueryFilter());
}

m3RayHit m3RayClosestInternal(m3World* world, m3Pos3 origin, m3Vec3 translation)
{
    return m3RayClosestFiltered(world, origin, translation, -1, m3DefaultQueryFilter());
}

m3RayHit m3World_CastRayClosestEx(m3WorldId worldId, m3Pos3 origin, m3Vec3 translation,
                                  m3QueryFilter filter)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL)
    {
        m3RayHit miss;
        memset(&miss, 0, sizeof(miss));
        return miss;
    }
    return m3RayClosestFiltered(world, origin, translation, -1, filter);
}

m3RayHit m3World_CastRayClosest(m3WorldId worldId, m3Pos3 origin, m3Vec3 translation)
{
    return m3World_CastRayClosestEx(worldId, origin, translation, m3DefaultQueryFilter());
}
