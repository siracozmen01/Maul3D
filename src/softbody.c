// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// XPBD particle lattices (7-1). The soft pass runs inside the step:
// per substep, integrate every live particle under gravity, satisfy
// the distance constraints once in fixed edge order (the small-steps
// XPBD schedule: one Gauss-Seidel sweep per substep beats many
// sweeps per big step), collide against the world's infinite planes
// (the wider world arrives in 7-2), then derive velocities from the
// position delta. Every loop runs in ascending slot, particle, and
// edge order: deterministic by construction.

#include "maul3d/softbody.h"

#include "world_internal.h"

#include <math.h>
#include <string.h>

#define M3_SOFTBODY_COOKIE ((int32_t)(M3_COOKIE ^ ((int32_t)sizeof(m3SoftBodyDef) << 8) ^ 9))

m3SoftBodyDef m3DefaultSoftBodyDef(void)
{
    m3SoftBodyDef def;
    memset(&def, 0, sizeof(def));
    def.countX = 4;
    def.countY = 4;
    def.countZ = 4;
    def.spacing = 0.25f;
    def.particleMass = 0.1f;
    def.compliance = 0.0f;
    def.radius = 0.05f;
    def.gravityScale = 1.0f;
    def.internalValue = M3_SOFTBODY_COOKIE;
    return def;
}

int32_t m3SoftBodySlot(const m3World* world, m3SoftBodyId softId)
{
    int32_t index = softId.index1 - 1;
    if (world == NULL || softId.world0 != world->worldIndex0 ||
        !m3IdPoolValid(&world->softPool, index, softId.generation))
    {
        return -1;
    }
    return index;
}

static void AddEdge(m3World* world, int32_t slot, int32_t a, int32_t b, m3real rest)
{
    int32_t e = world->softEdgeCount[slot];
    if (e >= M3_SOFTBODY_MAX_EDGES)
    {
        return; // the factory sizes below the cap by construction
    }
    int32_t k = slot * M3_SOFTBODY_MAX_EDGES + e;
    world->softEdgeA[k] = (uint16_t)a;
    world->softEdgeB[k] = (uint16_t)b;
    world->softEdgeRest[k] = rest;
    world->softEdgeCount[slot] = e + 1;
}

int32_t m3CreateSoftBodyInternal(m3World* world, const m3SoftBodyDef* def)
{
    int32_t slot = m3IdPoolAlloc(&world->softPool);
    if (slot < 0)
    {
        return -1;
    }
    int32_t nx = def->countX;
    int32_t ny = def->countY;
    int32_t nz = def->countZ;
    int32_t count = nx * ny * nz;
    world->softParticleCount[slot] = count;
    world->softEdgeCount[slot] = 0;
    world->softAnchorCount[slot] = 0;
    world->softCompliance[slot] = def->compliance;
    world->softRadius[slot] = def->radius;
    world->softGravityScale[slot] = def->gravityScale;
    world->softUserData[slot] = def->userData;

    m3real invMass = 1.0f / def->particleMass;
    for (int32_t z = 0; z < nz; ++z)
    {
        for (int32_t y = 0; y < ny; ++y)
        {
            for (int32_t x = 0; x < nx; ++x)
            {
                int32_t i = x + nx * (y + ny * z);
                int32_t k = slot * M3_SOFTBODY_MAX_PARTICLES + i;
                m3Pos3 p = {def->position.x + (double)((m3real)x * def->spacing),
                            def->position.y + (double)((m3real)y * def->spacing),
                            def->position.z + (double)((m3real)z * def->spacing)};
                world->softPos[k] = p;
                world->softPrev[k] = p;
                world->softInvMass[k] = invMass;
            }
        }
    }

    // Structural edges along the axes, then face diagonals, all in
    // one fixed nested order (determinism lives in this ordering).
    m3real s = def->spacing;
    m3real d = s * 1.41421356237f;
    for (int32_t z = 0; z < nz; ++z)
    {
        for (int32_t y = 0; y < ny; ++y)
        {
            for (int32_t x = 0; x < nx; ++x)
            {
                int32_t i = x + nx * (y + ny * z);
                if (x + 1 < nx)
                {
                    AddEdge(world, slot, i, i + 1, s);
                }
                if (y + 1 < ny)
                {
                    AddEdge(world, slot, i, i + nx, s);
                }
                if (z + 1 < nz)
                {
                    AddEdge(world, slot, i, i + nx * ny, s);
                }
                if (x + 1 < nx && y + 1 < ny)
                {
                    AddEdge(world, slot, i, i + 1 + nx, d);
                    AddEdge(world, slot, i + 1, i + nx, d);
                }
                if (x + 1 < nx && z + 1 < nz)
                {
                    AddEdge(world, slot, i, i + 1 + nx * ny, d);
                    AddEdge(world, slot, i + 1, i + nx * ny, d);
                }
                if (y + 1 < ny && z + 1 < nz)
                {
                    AddEdge(world, slot, i, i + nx + nx * ny, d);
                    AddEdge(world, slot, i + nx, i + nx * ny, d);
                }
            }
        }
    }
    return slot;
}

void m3DestroySoftBodyInternal(m3World* world, int32_t slot)
{
    int32_t count = world->softParticleCount[slot];
    for (int32_t i = 0; i < count; ++i)
    {
        int32_t k = slot * M3_SOFTBODY_MAX_PARTICLES + i;
        world->softPos[k] = (m3Pos3){0.0, 0.0, 0.0};
        world->softPrev[k] = (m3Pos3){0.0, 0.0, 0.0};
        world->softInvMass[k] = 0.0f;
    }
    int32_t edges = world->softEdgeCount[slot];
    for (int32_t e = 0; e < edges; ++e)
    {
        int32_t k = slot * M3_SOFTBODY_MAX_EDGES + e;
        world->softEdgeA[k] = 0;
        world->softEdgeB[k] = 0;
        world->softEdgeRest[k] = 0.0f;
    }
    for (int32_t a = 0; a < world->softAnchorCount[slot]; ++a)
    {
        int32_t k = slot * M3_SOFTBODY_MAX_ANCHORS + a;
        world->softAnchorParticle[k] = 0;
        world->softAnchorBody[k] = 0;
        world->softAnchorGen[k] = 0;
        world->softAnchorLocal[k] = (m3Vec3){0.0f, 0.0f, 0.0f};
    }
    world->softAnchorCount[slot] = 0;
    world->softParticleCount[slot] = 0;
    world->softEdgeCount[slot] = 0;
    world->softCompliance[slot] = 0.0f;
    world->softRadius[slot] = 0.0f;
    world->softGravityScale[slot] = 0.0f;
    world->softUserData[slot] = 0;
    m3IdPoolFree(&world->softPool, slot);
}

// Closest point on a triangle to a point (barycentric clamp, the
// textbook regions walk).
static m3Vec3 ClosestOnTriangle(m3Vec3 p, m3Vec3 a, m3Vec3 b, m3Vec3 c)
{
    m3Vec3 ab = m3Sub3(b, a);
    m3Vec3 ac = m3Sub3(c, a);
    m3Vec3 ap = m3Sub3(p, a);
    m3real d1 = m3Dot3(ab, ap);
    m3real d2 = m3Dot3(ac, ap);
    if (d1 <= 0.0f && d2 <= 0.0f)
    {
        return a;
    }
    m3Vec3 bp = m3Sub3(p, b);
    m3real d3 = m3Dot3(ab, bp);
    m3real d4 = m3Dot3(ac, bp);
    if (d3 >= 0.0f && d4 <= d3)
    {
        return b;
    }
    m3real vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
    {
        m3real v = d1 / (d1 - d3);
        return m3Add3(a, m3MulSV3(v, ab));
    }
    m3Vec3 cp = m3Sub3(p, c);
    m3real d5 = m3Dot3(ab, cp);
    m3real d6 = m3Dot3(ac, cp);
    if (d6 >= 0.0f && d5 <= d6)
    {
        return c;
    }
    m3real vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
    {
        m3real w = d2 / (d2 - d6);
        return m3Add3(a, m3MulSV3(w, ac));
    }
    m3real va = d3 * d6 - d5 * d4;
    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f)
    {
        m3real w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return m3Add3(b, m3MulSV3(w, m3Sub3(c, b)));
    }
    m3real denom = 1.0f / (va + vb + vc);
    m3real v = vb * denom;
    m3real w = vc * denom;
    return m3Add3(a, m3Add3(m3MulSV3(v, ab), m3MulSV3(w, ac)));
}

// Apply a projection to particle k: push along n (unit, outward) by
// depth, then the PBD friction rule against the shape's friction:
// tangential motion this substep dies when it is smaller than
// mu times the correction, and shrinks by that budget otherwise.
static void SoftProject(m3World* world, int32_t k, m3Vec3 n, m3real depth, m3real mu, int32_t body,
                        m3real invH)
{
    world->softPos[k].x += (double)(depth * n.x);
    world->softPos[k].y += (double)(depth * n.y);
    world->softPos[k].z += (double)(depth * n.z);
    // Two-way (7-3): a particle pushed out of a dynamic body
    // pushes back, the wheel-reaction pattern: the projection is a
    // velocity change of depth over h on the particle's mass,
    // mirrored onto the body at the contact and waking it. Jelly
    // has weight now.
    if (body >= 0 && world->types[body] == (uint8_t)m3_dynamicBody && world->invMass[body] > 0.0f &&
        world->softInvMass[k] > 0.0f)
    {
        m3real mp = 1.0f / world->softInvMass[k];
        m3Vec3 J = m3MulSV3(-depth * invH * mp, n);
        m3Vec3 rlc = m3RotateVec3(world->transforms[body].q, world->localCenters[body]);
        m3Vec3 arm = {(m3real)(world->softPos[k].x - world->transforms[body].p.x) - rlc.x,
                      (m3real)(world->softPos[k].y - world->transforms[body].p.y) - rlc.y,
                      (m3real)(world->softPos[k].z - world->transforms[body].p.z) - rlc.z};
        world->linearVelocities[body] =
            m3Add3(world->linearVelocities[body], m3MulSV3(world->invMass[body], J));
        world->angularVelocities[body] =
            m3Add3(world->angularVelocities[body],
                   m3MulMV3(m3WorldInvInertia(world, body), m3Cross3(arm, J)));
        world->awake[body] = 1;
        world->sleepTimes[body] = 0.0f;
    }
    if (mu <= 0.0f)
    {
        return;
    }
    m3Vec3 move = {(m3real)(world->softPos[k].x - world->softPrev[k].x),
                   (m3real)(world->softPos[k].y - world->softPrev[k].y),
                   (m3real)(world->softPos[k].z - world->softPrev[k].z)};
    m3Vec3 tang = m3Sub3(move, m3MulSV3(m3Dot3(move, n), n));
    m3real tl = sqrtf(m3Dot3(tang, tang));
    if (tl < 1.0e-9f)
    {
        return;
    }
    m3real budget = mu * depth;
    m3real scale = tl <= budget ? 1.0f : budget / tl;
    world->softPos[k].x -= (double)(tang.x * scale);
    world->softPos[k].y -= (double)(tang.y * scale);
    world->softPos[k].z -= (double)(tang.z * scale);
}

// One particle against one shape, in the shape body's local frame.
static void SoftCollideParticle(m3World* world, int32_t slot, int32_t k, int32_t shape,
                                uint8_t stype, int32_t body, m3real radius, m3real invH)
{
    (void)slot;
    m3Transform xfS = m3ShapeWorldTransform(world, shape);
    const m3Transform* xf = &xfS;
    m3real mu = world->shapeFriction[shape];

    if (stype == (uint8_t)m3_planeShape)
    {
        m3Vec3 n = m3RotateVec3(xf->q, world->shapeGeom[shape].v);
        m3real offset =
            world->shapeGeom[shape].s +
            (m3real)((double)n.x * xf->p.x + (double)n.y * xf->p.y + (double)n.z * xf->p.z);
        m3real dist =
            (m3real)((double)n.x * world->softPos[k].x + (double)n.y * world->softPos[k].y +
                     (double)n.z * world->softPos[k].z) -
            offset - radius;
        if (dist < 0.0f)
        {
            SoftProject(world, k, n, -dist, mu, body, invH);
        }
        return;
    }

    // Localize the particle into the body frame.
    m3Vec3 rel = {(m3real)(world->softPos[k].x - xf->p.x), (m3real)(world->softPos[k].y - xf->p.y),
                  (m3real)(world->softPos[k].z - xf->p.z)};
    m3Vec3 lp = m3InvRotateVec3(xf->q, rel);

    if (stype == (uint8_t)m3_sphereShape)
    {
        m3Vec3 d = m3Sub3(lp, world->shapeGeom[shape].v);
        m3real len = sqrtf(m3Dot3(d, d));
        m3real gap = len - world->shapeGeom[shape].s - radius;
        if (gap < 0.0f && len > 1.0e-9f)
        {
            m3Vec3 n = m3RotateVec3(xf->q, m3MulSV3(1.0f / len, d));
            SoftProject(world, k, n, -gap, mu, body, invH);
        }
        return;
    }
    if (stype == (uint8_t)m3_capsuleShape)
    {
        m3Vec3 a = world->shapeGeom[shape].v;
        m3Vec3 ab = m3Sub3(world->shapeGeom[shape].v2, a);
        m3real t = m3Dot3(m3Sub3(lp, a), ab) / m3MaxF(m3Dot3(ab, ab), 1.0e-12f);
        t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);
        m3Vec3 c = m3Add3(a, m3MulSV3(t, ab));
        m3Vec3 d = m3Sub3(lp, c);
        m3real len = sqrtf(m3Dot3(d, d));
        m3real gap = len - world->shapeGeom[shape].s - radius;
        if (gap < 0.0f && len > 1.0e-9f)
        {
            m3Vec3 n = m3RotateVec3(xf->q, m3MulSV3(1.0f / len, d));
            SoftProject(world, k, n, -gap, mu, body, invH);
        }
        return;
    }
    if (stype == (uint8_t)m3_hullShape)
    {
        // Boxes intern as hulls, so this branch carries them too.
        // Face planes serve both sides: outside within radius of
        // the closest face projects out; inside pushes along the
        // least penetrated face. Edge and vertex regions round
        // slightly toward the face answer: documented.
        const m3HullData* hull = &world->hullData[world->shapeHullIndex[shape]];
        m3real best = -3.4e38f;
        int32_t bestFace = -1;
        for (int32_t f = 0; f < hull->faceCount; ++f)
        {
            m3real sd = m3Dot3(hull->faceNormals[f], lp) - hull->faceOffsets[f];
            if (sd > best)
            {
                best = sd;
                bestFace = f;
            }
        }
        if (bestFace >= 0)
        {
            m3real gap = best - world->shapeGeom[shape].s - radius;
            if (gap < 0.0f)
            {
                m3Vec3 n = m3RotateVec3(xf->q, hull->faceNormals[bestFace]);
                SoftProject(world, k, n, -gap, mu, body, invH);
            }
        }
        return;
    }
    if (stype == (uint8_t)m3_voxelShape)
    {
        int32_t vslot = world->shapeVoxelIndex[shape];
        const m3VoxelChunkData* chunk = &world->voxelData[vslot];
        m3real cell = chunk->cellSize;
        int32_t cx = (int32_t)floorf(lp.x / cell);
        int32_t cy = (int32_t)floorf(lp.y / cell);
        int32_t cz = (int32_t)floorf(lp.z / cell);
        if (cx >= 0 && cx < 16 && cy >= 0 && cy < 16 && cz >= 0 && cz < 16 &&
            m3VoxelGet(chunk, cx, cy, cz))
        {
            // Deep inside filled voxels: the escape kernel owns it.
            m3Vec3 ln;
            m3real plane;
            if (m3VoxelEscape(world, vslot, lp, &ln, &plane))
            {
                m3real depth = plane - m3Dot3(ln, lp) + radius;
                if (depth > 0.0f)
                {
                    SoftProject(world, k, m3RotateVec3(xf->q, ln), depth, mu, body, invH);
                }
            }
            return;
        }
        // Near the surface: clamp against the merged boxes the BVH
        // hands back around the particle.
        const m3VoxelSurface* surface = &world->voxelSurface[vslot];
        m3Vec3 qlo = {lp.x - radius, lp.y - radius, lp.z - radius};
        m3Vec3 qhi = {lp.x + radius, lp.y + radius, lp.z + radius};
        uint16_t boxes[32];
        int32_t nb = m3MeshBvhGather(&surface->bvh, qlo, qhi, boxes);
        nb = nb > 32 ? 32 : nb;
        for (int32_t b = 0; b < nb; ++b)
        {
            m3Vec3 blo;
            m3Vec3 bhi;
            m3VoxelBoxBounds(surface, cell, (int32_t)boxes[b], &blo, &bhi);
            m3Vec3 c = {lp.x < blo.x ? blo.x : (lp.x > bhi.x ? bhi.x : lp.x),
                        lp.y < blo.y ? blo.y : (lp.y > bhi.y ? bhi.y : lp.y),
                        lp.z < blo.z ? blo.z : (lp.z > bhi.z ? bhi.z : lp.z)};
            m3Vec3 d = m3Sub3(lp, c);
            m3real len2 = m3Dot3(d, d);
            if (len2 > 1.0e-12f && len2 < radius * radius)
            {
                m3real len = sqrtf(len2);
                m3Vec3 n = m3RotateVec3(xf->q, m3MulSV3(1.0f / len, d));
                SoftProject(world, k, n, radius - len, mu, body, invH);
                // Re-localize after the push for the next box.
                m3Vec3 rel2 = {(m3real)(world->softPos[k].x - xf->p.x),
                               (m3real)(world->softPos[k].y - xf->p.y),
                               (m3real)(world->softPos[k].z - xf->p.z)};
                lp = m3InvRotateVec3(xf->q, rel2);
            }
        }
        return;
    }
    if (stype == (uint8_t)m3_meshShape) // heightfields intern as meshes
    {
        const m3MeshData* mesh = &world->meshData[world->shapeMeshIndex[shape]];
        const m3MeshBvh* bvh = &world->meshBvh[world->shapeMeshIndex[shape]];
        m3Vec3 qlo = {lp.x - radius, lp.y - radius, lp.z - radius};
        m3Vec3 qhi = {lp.x + radius, lp.y + radius, lp.z + radius};
        uint16_t tris[32];
        int32_t nt = m3MeshBvhGather(bvh, qlo, qhi, tris);
        nt = nt > 32 ? 32 : nt;
        for (int32_t t = 0; t < nt; ++t)
        {
            int32_t tri = (int32_t)tris[t];
            m3Vec3 a = mesh->vertices[mesh->indices[3 * tri + 0]];
            m3Vec3 bb2 = mesh->vertices[mesh->indices[3 * tri + 1]];
            m3Vec3 cc = mesh->vertices[mesh->indices[3 * tri + 2]];
            m3Vec3 cp = ClosestOnTriangle(lp, a, bb2, cc);
            m3Vec3 d = m3Sub3(lp, cp);
            m3real len2 = m3Dot3(d, d);
            if (len2 > 1.0e-12f && len2 < radius * radius)
            {
                m3real len = sqrtf(len2);
                m3Vec3 n = m3RotateVec3(xf->q, m3MulSV3(1.0f / len, d));
                SoftProject(world, k, n, radius - len, mu, body, invH);
                m3Vec3 rel2 = {(m3real)(world->softPos[k].x - xf->p.x),
                               (m3real)(world->softPos[k].y - xf->p.y),
                               (m3real)(world->softPos[k].z - xf->p.z)};
                lp = m3InvRotateVec3(xf->q, rel2);
            }
        }
        return;
    }
}

// The soft pass: XPBD small steps over the full shape set (7-2).
void m3SoftBodyPass(m3World* world, float dt, int32_t substeps)
{
    if (world->softPool.maxIndex == 0)
    {
        return;
    }
    m3real h = dt / (m3real)substeps;
    m3real invH = h > 0.0f ? 1.0f / h : 0.0f;
    int32_t maxShape = world->shapePool.maxIndex;

    for (int32_t sub = 0; sub < substeps; ++sub)
    {
        for (int32_t slot = 0; slot < world->softPool.maxIndex; ++slot)
        {
            if (world->softPool.alive[slot] == 0)
            {
                continue;
            }
            int32_t count = world->softParticleCount[slot];
            int32_t base = slot * M3_SOFTBODY_MAX_PARTICLES;

            // Anchored particles are driven, not integrated: mark
            // them for this substep (32 max, a cheap bitmask).
            uint8_t anchored[M3_SOFTBODY_MAX_PARTICLES / 8];
            memset(anchored, 0, sizeof(anchored));
            int32_t anchorCount = world->softAnchorCount[slot];
            int32_t abase = slot * M3_SOFTBODY_MAX_ANCHORS;
            for (int32_t a = 0; a < anchorCount; ++a)
            {
                // A dead or recycled body releases its anchor: the
                // particle must return to the integrator, or it
                // hangs frozen in the air where its beam died.
                int32_t abody = world->softAnchorBody[abase + a];
                if (abody < 0 || world->bodyPool.alive[abody] == 0 ||
                    world->bodyPool.generations[abody] != world->softAnchorGen[abase + a])
                {
                    continue;
                }
                int32_t particle = world->softAnchorParticle[abase + a];
                anchored[particle >> 3] |= (uint8_t)(1u << (particle & 7));
            }

            // Integrate: velocity from the previous position pair,
            // gravity, then the predicted position.
            m3Vec3 g = m3MulSV3(world->softGravityScale[slot], world->gravity);
            for (int32_t i = 0; i < count; ++i)
            {
                int32_t k = base + i;
                if (world->softInvMass[k] == 0.0f ||
                    (anchored[i >> 3] & (uint8_t)(1u << (i & 7))) != 0)
                {
                    world->softPrev[k] = world->softPos[k];
                    continue;
                }
                m3Vec3 v = {(m3real)(world->softPos[k].x - world->softPrev[k].x) * invH,
                            (m3real)(world->softPos[k].y - world->softPrev[k].y) * invH,
                            (m3real)(world->softPos[k].z - world->softPrev[k].z) * invH};
                v = m3Add3(v, m3MulSV3(h, g));
                world->softPrev[k] = world->softPos[k];
                world->softPos[k].x += (double)(v.x * h);
                world->softPos[k].y += (double)(v.y * h);
                world->softPos[k].z += (double)(v.z * h);
            }

            // One Gauss-Seidel sweep over the edges, fixed order.
            m3real alpha = world->softCompliance[slot] * invH * invH;
            int32_t edges = world->softEdgeCount[slot];
            int32_t ebase = slot * M3_SOFTBODY_MAX_EDGES;
            for (int32_t e = 0; e < edges; ++e)
            {
                int32_t ka = base + (int32_t)world->softEdgeA[ebase + e];
                int32_t kb = base + (int32_t)world->softEdgeB[ebase + e];
                m3real wa = world->softInvMass[ka];
                m3real wb = world->softInvMass[kb];
                m3real wSum = wa + wb;
                if (wSum == 0.0f)
                {
                    continue;
                }
                m3Vec3 diff = {(m3real)(world->softPos[ka].x - world->softPos[kb].x),
                               (m3real)(world->softPos[ka].y - world->softPos[kb].y),
                               (m3real)(world->softPos[ka].z - world->softPos[kb].z)};
                m3real len = sqrtf(m3Dot3(diff, diff));
                if (len < 1.0e-9f)
                {
                    continue; // coincident: no gradient, no correction
                }
                m3real c = len - world->softEdgeRest[ebase + e];
                m3real scale = -c / ((wSum + alpha) * len);
                world->softPos[ka].x += (double)(wa * scale * diff.x);
                world->softPos[ka].y += (double)(wa * scale * diff.y);
                world->softPos[ka].z += (double)(wa * scale * diff.z);
                world->softPos[kb].x -= (double)(wb * scale * diff.x);
                world->softPos[kb].y -= (double)(wb * scale * diff.y);
                world->softPos[kb].z -= (double)(wb * scale * diff.z);
            }

            // Anchors (7-3): snap each anchored particle to its
            // body-frame target; the pull the lattice exerted on it
            // this substep (where the edges dragged it versus where
            // the body says it must be) lands on the body as an
            // impulse at the anchor. A dead or recycled body
            // releases its anchors silently.
            for (int32_t a = 0; a < anchorCount; ++a)
            {
                int32_t ak = abase + a;
                int32_t body = world->softAnchorBody[ak];
                if (body < 0 || world->bodyPool.alive[body] == 0 ||
                    world->bodyPool.generations[body] != world->softAnchorGen[ak])
                {
                    continue; // released
                }
                int32_t particle = world->softAnchorParticle[ak];
                int32_t k = base + particle;
                const m3Transform* bxf = &world->transforms[body];
                m3Vec3 wl = m3RotateVec3(bxf->q, world->softAnchorLocal[ak]);
                m3Pos3 target = {bxf->p.x + (double)wl.x, bxf->p.y + (double)wl.y,
                                 bxf->p.z + (double)wl.z};
                m3Vec3 wish = {(m3real)(world->softPos[k].x - target.x),
                               (m3real)(world->softPos[k].y - target.y),
                               (m3real)(world->softPos[k].z - target.z)};
                world->softPos[k] = target;
                world->softPrev[k] = target;
                if (world->types[body] == (uint8_t)m3_dynamicBody && world->invMass[body] > 0.0f &&
                    world->softInvMass[k] > 0.0f)
                {
                    m3real mp = 1.0f / world->softInvMass[k];
                    m3Vec3 J = m3MulSV3(mp * invH, wish);
                    m3Vec3 rlc = m3RotateVec3(bxf->q, world->localCenters[body]);
                    m3Vec3 arm = m3Sub3(wl, rlc);
                    world->linearVelocities[body] =
                        m3Add3(world->linearVelocities[body], m3MulSV3(world->invMass[body], J));
                    world->angularVelocities[body] =
                        m3Add3(world->angularVelocities[body],
                               m3MulMV3(m3WorldInvInertia(world, body), m3Cross3(arm, J)));
                    world->awake[body] = 1;
                    world->sleepTimes[body] = 0.0f;
                }
            }

            // The world's surfaces (7-2): every particle projects
            // out of every shape family through the shared local
            // kernels, ascending shape order, friction from the
            // touched shape (the PBD tangential rule). Planes stay
            // world-frame; everything else works in body space.
            m3real radius = world->softRadius[slot];
            for (int32_t sShape = 0; sShape < maxShape; ++sShape)
            {
                if (world->shapePool.alive[sShape] == 0 || world->shapeSensor[sShape] != 0)
                {
                    continue;
                }
                uint8_t stype = world->shapeType[sShape];
                int32_t body = world->shapeBody[sShape];
                if (world->bodyEnabled[body] == 0)
                {
                    continue; // disabled bodies are ghosts to lattices too (8-3)
                }
                for (int32_t i = 0; i < count; ++i)
                {
                    int32_t k = base + i;
                    if (world->softInvMass[k] == 0.0f)
                    {
                        continue;
                    }
                    SoftCollideParticle(world, slot, k, sShape, stype, body, radius, invH);
                }
            }
        }
    }
}

m3SoftBodyId m3CreateSoftBody(m3WorldId worldId, const m3SoftBodyDef* def)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || def == NULL || def->internalValue != M3_SOFTBODY_COOKIE ||
        def->countX < 1 || def->countY < 1 || def->countZ < 1 ||
        (int64_t)def->countX * def->countY * def->countZ > M3_SOFTBODY_MAX_PARTICLES ||
        !m3FinitePos3(def->position) || !m3FiniteF(def->spacing) || !(def->spacing > 0.0f) ||
        !m3FiniteF(def->particleMass) || !(def->particleMass > 0.0f) ||
        !m3FiniteF(def->compliance) || def->compliance < 0.0f || !m3FiniteF(def->radius) ||
        !(def->radius > 0.0f) || !m3FiniteF(def->gravityScale))
    {
        return m3_nullSoftBodyId;
    }
    int32_t slot = m3CreateSoftBodyInternal(world, def);
    if (slot < 0)
    {
        return m3_nullSoftBodyId;
    }
    m3SoftBodyId id = {slot + 1, world->worldIndex0, world->softPool.generations[slot]};
    if (world->journalActive != 0)
    {
        struct
        {
            m3SoftBodyDef def;
            m3SoftBodyId expected;
        } record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.expected = id;
        m3JournalRecord(world, m3_opCreateSoftBody, &record, (int32_t)sizeof(record));
    }
    return id;
}

void m3DestroySoftBody(m3SoftBodyId softId)
{
    m3World* world = m3WorldFromIndex0(softId.world0);
    int32_t slot = world != NULL ? m3SoftBodySlot(world, softId) : -1;
    if (slot < 0)
    {
        return; // stale: the quiet destroy contract
    }
    if (world->journalActive != 0)
    {
        m3JournalRecord(world, m3_opDestroySoftBody, &softId, (int32_t)sizeof(softId));
    }
    m3DestroySoftBodyInternal(world, slot);
}

bool m3SoftBody_IsValid(m3SoftBodyId softId)
{
    m3World* world = m3WorldFromIndex0(softId.world0);
    return world != NULL && m3SoftBodySlot(world, softId) >= 0;
}

void m3SoftBody_PinParticle(m3SoftBodyId softId, int32_t particle)
{
    m3World* world = m3WorldFromIndex0(softId.world0);
    int32_t slot = world != NULL ? m3SoftBodySlot(world, softId) : -1;
    if (slot < 0 || particle < 0 || particle >= world->softParticleCount[slot])
    {
        return; // stale or out of range: a documented no-op
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m3SoftBodyId id;
            int32_t particle;
        } record;
        memset(&record, 0, sizeof(record));
        record.id = softId;
        record.particle = particle;
        m3JournalRecord(world, m3_opSoftBodyPin, &record, (int32_t)sizeof(record));
    }
    m3SoftBodyPinInternal(world, slot, particle);
}

void m3SoftBodyPinInternal(m3World* world, int32_t slot, int32_t particle)
{
    int32_t k = slot * M3_SOFTBODY_MAX_PARTICLES + particle;
    world->softInvMass[k] = 0.0f;
    world->softPrev[k] = world->softPos[k];
}

void m3SoftBodyAnchorInternal(m3World* world, int32_t slot, int32_t particle, int32_t body)
{
    int32_t a = world->softAnchorCount[slot];
    int32_t ak = slot * M3_SOFTBODY_MAX_ANCHORS + a;
    int32_t k = slot * M3_SOFTBODY_MAX_PARTICLES + particle;
    const m3Transform* bxf = &world->transforms[body];
    m3Vec3 rel = {(m3real)(world->softPos[k].x - bxf->p.x),
                  (m3real)(world->softPos[k].y - bxf->p.y),
                  (m3real)(world->softPos[k].z - bxf->p.z)};
    world->softAnchorParticle[ak] = particle;
    world->softAnchorBody[ak] = body;
    world->softAnchorGen[ak] = world->bodyPool.generations[body];
    world->softAnchorLocal[ak] = m3InvRotateVec3(bxf->q, rel);
    world->softAnchorCount[slot] = a + 1;
}

void m3SoftBody_AnchorParticle(m3SoftBodyId softId, int32_t particle, m3BodyId bodyId)
{
    m3World* world = m3WorldFromIndex0(softId.world0);
    int32_t slot = world != NULL ? m3SoftBodySlot(world, softId) : -1;
    int32_t body = world != NULL ? m3BodySlot(world, bodyId) : -1;
    if (slot < 0 || body < 0 || particle < 0 || particle >= world->softParticleCount[slot] ||
        world->softAnchorCount[slot] >= M3_SOFTBODY_MAX_ANCHORS)
    {
        return; // stale, out of range, or a full table: quiet no-op
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m3SoftBodyId id;
            int32_t particle;
            m3BodyId body;
        } record;
        memset(&record, 0, sizeof(record));
        record.id = softId;
        record.particle = particle;
        record.body = bodyId;
        m3JournalRecord(world, m3_opSoftBodyAnchor, &record, (int32_t)sizeof(record));
    }
    m3SoftBodyAnchorInternal(world, slot, particle, body);
}

int32_t m3SoftBody_GetParticleCount(m3SoftBodyId softId)
{
    m3World* world = m3WorldFromIndex0(softId.world0);
    int32_t slot = world != NULL ? m3SoftBodySlot(world, softId) : -1;
    return slot >= 0 ? world->softParticleCount[slot] : 0;
}

m3Pos3 m3SoftBody_GetParticlePosition(m3SoftBodyId softId, int32_t particle)
{
    m3World* world = m3WorldFromIndex0(softId.world0);
    int32_t slot = world != NULL ? m3SoftBodySlot(world, softId) : -1;
    if (slot < 0 || particle < 0 || particle >= world->softParticleCount[slot])
    {
        return (m3Pos3){0.0, 0.0, 0.0};
    }
    return world->softPos[slot * M3_SOFTBODY_MAX_PARTICLES + particle];
}
