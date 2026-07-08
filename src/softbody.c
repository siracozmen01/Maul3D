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
    world->softParticleCount[slot] = 0;
    world->softEdgeCount[slot] = 0;
    world->softCompliance[slot] = 0.0f;
    world->softRadius[slot] = 0.0f;
    world->softGravityScale[slot] = 0.0f;
    world->softUserData[slot] = 0;
    m3IdPoolFree(&world->softPool, slot);
}

// The soft pass: XPBD small steps, planes only in 7-1.
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

            // Integrate: velocity from the previous position pair,
            // gravity, then the predicted position.
            m3Vec3 g = m3MulSV3(world->softGravityScale[slot], world->gravity);
            for (int32_t i = 0; i < count; ++i)
            {
                int32_t k = base + i;
                if (world->softInvMass[k] == 0.0f)
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

            // Planes: project particles out along the plane normal
            // (7-2 brings the rest of the shape set and friction).
            m3real radius = world->softRadius[slot];
            for (int32_t sShape = 0; sShape < maxShape; ++sShape)
            {
                if (world->shapePool.alive[sShape] == 0 ||
                    world->shapeType[sShape] != (uint8_t)m3_planeShape)
                {
                    continue;
                }
                int32_t body = world->shapeBody[sShape];
                m3Vec3 n = m3RotateVec3(world->transforms[body].q, world->shapeGeom[sShape].v);
                m3real offset = world->shapeGeom[sShape].s +
                                (m3real)((double)n.x * world->transforms[body].p.x +
                                         (double)n.y * world->transforms[body].p.y +
                                         (double)n.z * world->transforms[body].p.z);
                for (int32_t i = 0; i < count; ++i)
                {
                    int32_t k = base + i;
                    if (world->softInvMass[k] == 0.0f)
                    {
                        continue;
                    }
                    m3real dist = (m3real)((double)n.x * world->softPos[k].x +
                                           (double)n.y * world->softPos[k].y +
                                           (double)n.z * world->softPos[k].z) -
                                  offset - radius;
                    if (dist < 0.0f)
                    {
                        world->softPos[k].x -= (double)(dist * n.x);
                        world->softPos[k].y -= (double)(dist * n.y);
                        world->softPos[k].z -= (double)(dist * n.z);
                    }
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
