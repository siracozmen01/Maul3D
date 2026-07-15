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
    def.bendCompliance = 0.0f;
    def.pressure = 0.0f;
    def.maxDeviation = 0.0f;
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
    // The validation wall (14-4 cure): replay hands this function
    // raw mutated bytes, and the 13-4 storm proved a flipped bit in
    // a soft def could mint NaN positions or an overflowing
    // particle count. Every field check the public door ran now
    // lives here, where BOTH doors pass through; the cookie stays
    // a public-door concern like every def.
    if (def->countX < 1 || def->countY < 1 || def->countZ < 1 ||
        (int64_t)def->countX * def->countY * def->countZ > M3_SOFTBODY_MAX_PARTICLES ||
        !m3FinitePos3(def->position) || !m3FiniteF(def->spacing) || !(def->spacing > 0.0f) ||
        !m3FiniteF(def->particleMass) || !(def->particleMass > 0.0f) ||
        !m3FiniteF(def->compliance) || def->compliance < 0.0f || !m3FiniteF(def->radius) ||
        !(def->radius > 0.0f) || !m3FiniteF(def->gravityScale) || !m3FiniteF(def->bendCompliance) ||
        def->bendCompliance < 0.0f || !m3FiniteF(def->pressure) || def->pressure < 0.0f ||
        (def->pressure > 0.0f && (def->countX < 2 || def->countY < 2 || def->countZ < 2)) ||
        !m3FiniteF(def->maxDeviation) || def->maxDeviation < 0.0f)
    {
        return -1;
    }
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
    world->softSoftCount[slot] = 0;
    world->softCompliance[slot] = def->compliance;
    world->softRadius[slot] = def->radius;
    world->softGravityScale[slot] = def->gravityScale;
    world->softUserData[slot] = def->userData;
    world->softBendCompliance[slot] = def->bendCompliance;
    world->softDimX[slot] = (uint16_t)def->countX;
    world->softDimY[slot] = (uint16_t)def->countY;
    world->softDimZ[slot] = (uint16_t)def->countZ;
    world->softPressure[slot] = def->pressure;
    // The create lattice is a perfect grid: the rest volume is the
    // closed box, no surface walk needed at create.
    world->softRestVolume[slot] = (m3real)(def->countX - 1) * (m3real)(def->countY - 1) *
                                  (m3real)(def->countZ - 1) * def->spacing * def->spacing *
                                  def->spacing;
    world->softTetCount[slot] = 0;
    world->softMaxDeviation[slot] = def->maxDeviation;

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
                world->softBindPos[k] = p;
                world->softInvMass[k] = invMass;
                world->softKick[k] = (m3Vec3){0.0f, 0.0f, 0.0f};
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
    // Bend tethers (20-1): second-neighbor edges along each axis,
    // AFTER every structural edge so one boundary index splits the
    // two compliances. Capacity is checked up front: a lattice
    // whose tethers cannot fit refuses loudly instead of shipping
    // half a spine.
    world->softBendStart[slot] = world->softEdgeCount[slot];
    if (def->bendCompliance > 0.0f)
    {
        int32_t need = (nx > 2 ? (nx - 2) * ny * nz : 0) + (ny > 2 ? nx * (ny - 2) * nz : 0) +
                       (nz > 2 ? nx * ny * (nz - 2) : 0);
        if (world->softEdgeCount[slot] + need > M3_SOFTBODY_MAX_EDGES)
        {
            m3DestroySoftBodyInternal(world, slot);
            return -1;
        }
        m3real s2 = 2.0f * s;
        for (int32_t z = 0; z < nz; ++z)
        {
            for (int32_t y = 0; y < ny; ++y)
            {
                for (int32_t x = 0; x < nx; ++x)
                {
                    int32_t i = x + nx * (y + ny * z);
                    if (x + 2 < nx)
                    {
                        AddEdge(world, slot, i, i + 2, s2);
                    }
                    if (y + 2 < ny)
                    {
                        AddEdge(world, slot, i, i + 2 * nx, s2);
                    }
                    if (z + 2 < nz)
                    {
                        AddEdge(world, slot, i, i + 2 * nx * ny, s2);
                    }
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
        world->softKick[k] = (m3Vec3){0.0f, 0.0f, 0.0f};
    }
    int32_t edges = world->softEdgeCount[slot];
    for (int32_t e = 0; e < edges; ++e)
    {
        int32_t k = slot * M3_SOFTBODY_MAX_EDGES + e;
        world->softEdgeA[k] = 0;
        world->softEdgeB[k] = 0;
        world->softEdgeRest[k] = 0.0f;
    }
    world->softSoftCount[slot] = 0;
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
    world->softBendStart[slot] = 0;
    world->softBendCompliance[slot] = 0.0f;
    world->softDimX[slot] = 0;
    world->softDimY[slot] = 0;
    world->softDimZ[slot] = 0;
    world->softRestVolume[slot] = 0.0f;
    world->softPressure[slot] = 0.0f;
    int32_t tets = world->softTetCount[slot];
    for (int32_t t = 0; t < tets; ++t)
    {
        int32_t k = slot * M3_SOFTBODY_MAX_TETS + t;
        world->softTetA[k] = 0;
        world->softTetB[k] = 0;
        world->softTetC[k] = 0;
        world->softTetD[k] = 0;
        world->softTetRestV6[k] = 0.0f;
    }
    world->softTetCount[slot] = 0;
    int32_t bcount = M3_SOFTBODY_MAX_PARTICLES;
    for (int32_t i = 0; i < bcount; ++i)
    {
        world->softBindPos[slot * M3_SOFTBODY_MAX_PARTICLES + i] = (m3Pos3){0.0, 0.0, 0.0};
    }
    world->softMaxDeviation[slot] = 0.0f;
    m3IdPoolFree(&world->softPool, slot);
}

// Tet soft bodies (20-3): explicit points and tets, edges deduped
// from tet edges in first-touch order, one rigid volume row per
// tet. The full wall lives here: replay hands this raw bytes.
static int32_t TetEdgeSeen(const uint16_t* ea, const uint16_t* eb, int32_t count, uint16_t lo,
                           uint16_t hi)
{
    for (int32_t e = 0; e < count; ++e)
    {
        if (ea[e] == lo && eb[e] == hi)
        {
            return 1;
        }
    }
    return 0;
}

int32_t m3CreateSoftBodyTetInternal(m3World* world, const m3SoftBodyDef* def, const m3Vec3* points,
                                    int32_t pointCount, const uint16_t* tets, int32_t tetCount)
{
    // Lattice knobs must sit at their defaults: a tet body has no
    // grid to bend or pressurize (documented, refused loudly).
    if (pointCount < 4 || pointCount > M3_SOFTBODY_MAX_PARTICLES || tetCount < 1 ||
        tetCount > M3_SOFTBODY_MAX_TETS || !m3FinitePos3(def->position) ||
        !m3FiniteF(def->particleMass) || !(def->particleMass > 0.0f) ||
        !m3FiniteF(def->compliance) || def->compliance < 0.0f || !m3FiniteF(def->radius) ||
        !(def->radius > 0.0f) || !m3FiniteF(def->gravityScale) || def->bendCompliance != 0.0f ||
        def->pressure != 0.0f || !m3FiniteF(def->maxDeviation) || def->maxDeviation < 0.0f)
    {
        return -1;
    }
    for (int32_t i = 0; i < pointCount; ++i)
    {
        if (!m3FiniteV3(points[i]))
        {
            return -1;
        }
    }
    // Edge budget: 6 per tet worst case, checked against the cap
    // after dedup would be too late; pre-check the worst case and
    // let dedup only shrink it.
    for (int32_t t = 0; t < tetCount; ++t)
    {
        uint16_t a = tets[4 * t + 0];
        uint16_t b = tets[4 * t + 1];
        uint16_t c = tets[4 * t + 2];
        uint16_t d = tets[4 * t + 3];
        if (a >= pointCount || b >= pointCount || c >= pointCount || d >= pointCount || a == b ||
            a == c || a == d || b == c || b == d || c == d)
        {
            return -1;
        }
        m3Vec3 e1 = m3Sub3(points[b], points[a]);
        m3Vec3 e2 = m3Sub3(points[c], points[a]);
        m3Vec3 e3 = m3Sub3(points[d], points[a]);
        if (!(m3Dot3(e1, m3Cross3(e2, e3)) > 1.0e-9f))
        {
            return -1; // flat or inverted tets refuse: the rest
                       // volume IS the constraint target
        }
    }
    int32_t slot = m3IdPoolAlloc(&world->softPool);
    if (slot < 0)
    {
        return -1;
    }
    world->softParticleCount[slot] = pointCount;
    world->softEdgeCount[slot] = 0;
    world->softAnchorCount[slot] = 0;
    world->softSoftCount[slot] = 0;
    world->softCompliance[slot] = def->compliance;
    world->softRadius[slot] = def->radius;
    world->softGravityScale[slot] = def->gravityScale;
    world->softUserData[slot] = def->userData;
    world->softBendStart[slot] = 0;
    world->softBendCompliance[slot] = 0.0f;
    world->softDimX[slot] = 0;
    world->softDimY[slot] = 0;
    world->softDimZ[slot] = 0;
    world->softRestVolume[slot] = 0.0f;
    world->softPressure[slot] = 0.0f;
    world->softMaxDeviation[slot] = def->maxDeviation;
    m3real invMass = 1.0f / def->particleMass;
    for (int32_t i = 0; i < pointCount; ++i)
    {
        int32_t k = slot * M3_SOFTBODY_MAX_PARTICLES + i;
        m3Pos3 p = {def->position.x + (double)points[i].x, def->position.y + (double)points[i].y,
                    def->position.z + (double)points[i].z};
        world->softPos[k] = p;
        world->softPrev[k] = p;
        world->softBindPos[k] = p;
        world->softInvMass[k] = invMass;
        world->softKick[k] = (m3Vec3){0.0f, 0.0f, 0.0f};
    }
    // Edges: the six edges of every tet, first-touch dedup in tet
    // order (deterministic; the scratch lists live on the slot's
    // own edge arrays as they fill).
    int32_t ebase = slot * M3_SOFTBODY_MAX_EDGES;
    static const int32_t pairs[6][2] = {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}};
    for (int32_t t = 0; t < tetCount; ++t)
    {
        for (int32_t e = 0; e < 6; ++e)
        {
            uint16_t va = tets[4 * t + pairs[e][0]];
            uint16_t vb = tets[4 * t + pairs[e][1]];
            uint16_t lo = va < vb ? va : vb;
            uint16_t hi = va < vb ? vb : va;
            if (TetEdgeSeen(&world->softEdgeA[ebase], &world->softEdgeB[ebase],
                            world->softEdgeCount[slot], lo, hi))
            {
                continue;
            }
            if (world->softEdgeCount[slot] >= M3_SOFTBODY_MAX_EDGES)
            {
                m3DestroySoftBodyInternal(world, slot);
                return -1; // edge budget blown: refuse loudly
            }
            m3Vec3 dvec = m3Sub3(points[hi], points[lo]);
            AddEdge(world, slot, lo, hi, sqrtf(m3Dot3(dvec, dvec)));
        }
    }
    world->softBendStart[slot] = world->softEdgeCount[slot];
    // Tets: rest 6-volumes from the create pose.
    world->softTetCount[slot] = tetCount;
    for (int32_t t = 0; t < tetCount; ++t)
    {
        int32_t k = slot * M3_SOFTBODY_MAX_TETS + t;
        uint16_t a = tets[4 * t + 0];
        uint16_t b = tets[4 * t + 1];
        uint16_t c = tets[4 * t + 2];
        uint16_t d = tets[4 * t + 3];
        world->softTetA[k] = a;
        world->softTetB[k] = b;
        world->softTetC[k] = c;
        world->softTetD[k] = d;
        m3Vec3 e1 = m3Sub3(points[b], points[a]);
        m3Vec3 e2 = m3Sub3(points[c], points[a]);
        m3Vec3 e3 = m3Sub3(points[d], points[a]);
        world->softTetRestV6[k] = m3Dot3(e1, m3Cross3(e2, e3));
    }
    return slot;
}

m3SoftBodyId m3CreateSoftBodyTet(m3WorldId worldId, const m3SoftBodyDef* def, const m3Vec3* points,
                                 int32_t pointCount, const uint16_t* tets, int32_t tetCount)
{
    m3SoftBodyId null = {0, 0, 0};
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || def == NULL || def->internalValue != M3_SOFTBODY_COOKIE ||
        points == NULL || tets == NULL)
    {
        return null;
    }
    int32_t slot = m3CreateSoftBodyTetInternal(world, def, points, pointCount, tets, tetCount);
    if (slot < 0)
    {
        return null;
    }
    m3SoftBodyId id = {slot + 1, world->worldIndex0, world->softPool.generations[slot]};
    if (world->journalActive != 0)
    {
        int32_t bytes = (int32_t)sizeof(m3CreateSoftBodyTetOp) +
                        pointCount * (int32_t)sizeof(m3Vec3) +
                        4 * tetCount * (int32_t)sizeof(uint16_t);
        uint8_t* payload = (uint8_t*)m3AllocZeroed(bytes);
        if (payload != NULL)
        {
            m3CreateSoftBodyTetOp head;
            memset(&head, 0, sizeof(head));
            head.def = *def;
            head.pointCount = pointCount;
            head.tetCount = tetCount;
            head.expected = id;
            memcpy(payload, &head, sizeof(head));
            memcpy(payload + sizeof(head), points, (size_t)pointCount * sizeof(m3Vec3));
            memcpy(payload + sizeof(head) + (size_t)pointCount * sizeof(m3Vec3), tets,
                   (size_t)(4 * tetCount) * sizeof(uint16_t));
            m3JournalRecord(world, m3_opCreateSoftBodyTet, payload, bytes);
            m3Free(payload);
        }
    }
    return id;
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
    if (stype == (uint8_t)m3_heightFieldShape)
    {
        // Native terrain (19-3): the mesh recipe over the cell
        // gather, particle-sized window.
        const m3HeightFieldData* hf = &world->hfData[world->shapeHfIndex[shape]];
        m3Vec3 hfTris[32][3];
        int32_t nt =
            m3HeightFieldGather(hf, (m3Vec3){lp.x - radius, lp.y - radius, lp.z - radius},
                                (m3Vec3){lp.x + radius, lp.y + radius, lp.z + radius}, hfTris, 32);
        for (int32_t t = 0; t < nt; ++t)
        {
            m3Vec3 cp = ClosestOnTriangle(lp, hfTris[t][0], hfTris[t][1], hfTris[t][2]);
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
    if (stype == (uint8_t)m3_meshShape) // mesh-backed terrain interns here
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

// The closed-lattice volume and its gradients (20-2): six faces
// walked in fixed order, outward winding, signed tet sum against a
// local origin (the first particle: double-safe far from zero).
// dV/da for triangle (a, b, c) is cross(b, c) / 6; the division by
// six is folded once at the call site.
static m3real SoftSurfaceVolume6(m3World* world, int32_t slot, m3Vec3* grads, m3Pos3 origin)
{
    int32_t nx = world->softDimX[slot];
    int32_t ny = world->softDimY[slot];
    int32_t nz = world->softDimZ[slot];
    int32_t base = slot * M3_SOFTBODY_MAX_PARTICLES;
    int32_t count = world->softParticleCount[slot];
    for (int32_t i = 0; i < count; ++i)
    {
        grads[i] = (m3Vec3){0.0f, 0.0f, 0.0f};
    }
    m3real volume6 = 0.0f;
    for (int32_t face = 0; face < 6; ++face)
    {
        int32_t nu = face < 2 ? ny : nx;
        int32_t nv = face < 4 ? nz : ny;
        for (int32_t v = 0; v + 1 < nv; ++v)
        {
            for (int32_t u = 0; u + 1 < nu; ++u)
            {
                int32_t i00;
                int32_t i10;
                int32_t i01;
                int32_t i11;
                if (face < 2)
                {
                    int32_t x = face == 0 ? 0 : nx - 1;
                    i00 = x + nx * (u + ny * v);
                    i10 = x + nx * ((u + 1) + ny * v);
                    i01 = x + nx * (u + ny * (v + 1));
                    i11 = x + nx * ((u + 1) + ny * (v + 1));
                }
                else if (face < 4)
                {
                    int32_t y = face == 2 ? 0 : ny - 1;
                    i00 = u + nx * (y + ny * v);
                    i10 = (u + 1) + nx * (y + ny * v);
                    i01 = u + nx * (y + ny * (v + 1));
                    i11 = (u + 1) + nx * (y + ny * (v + 1));
                }
                else
                {
                    int32_t z = face == 4 ? 0 : nz - 1;
                    i00 = u + nx * (v + ny * z);
                    i10 = (u + 1) + nx * (v + ny * z);
                    i01 = u + nx * ((v + 1) + ny * z);
                    i11 = (u + 1) + nx * ((v + 1) + ny * z);
                }
                // Outward winding: faces 1 (+x), 2 (-y), 5 (+z) take
                // one diagonal orientation, their mirrors flip.
                int flip = face == 0 || face == 3 || face == 4;
                int32_t t0b = flip ? i01 : i10;
                int32_t t0c = i11;
                int32_t t1b = i11;
                int32_t t1c = flip ? i10 : i01;
                int32_t tris[2][3] = {{i00, t0b, t0c}, {i00, t1b, t1c}};
                for (int32_t t = 0; t < 2; ++t)
                {
                    m3Vec3 pa = {(m3real)(world->softPos[base + tris[t][0]].x - origin.x),
                                 (m3real)(world->softPos[base + tris[t][0]].y - origin.y),
                                 (m3real)(world->softPos[base + tris[t][0]].z - origin.z)};
                    m3Vec3 pb = {(m3real)(world->softPos[base + tris[t][1]].x - origin.x),
                                 (m3real)(world->softPos[base + tris[t][1]].y - origin.y),
                                 (m3real)(world->softPos[base + tris[t][1]].z - origin.z)};
                    m3Vec3 pc = {(m3real)(world->softPos[base + tris[t][2]].x - origin.x),
                                 (m3real)(world->softPos[base + tris[t][2]].y - origin.y),
                                 (m3real)(world->softPos[base + tris[t][2]].z - origin.z)};
                    volume6 += m3Dot3(pa, m3Cross3(pb, pc));
                    grads[tris[t][0]] = m3Add3(grads[tris[t][0]], m3Cross3(pb, pc));
                    grads[tris[t][1]] = m3Add3(grads[tris[t][1]], m3Cross3(pc, pa));
                    grads[tris[t][2]] = m3Add3(grads[tris[t][2]], m3Cross3(pa, pb));
                }
            }
        }
    }
    return volume6;
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
            // Wind (11-3): proportional drag toward the wind
            // velocity, gusted by the accumulated phase. Soft-only
            // by design (rigid bodies already have the force API).
            m3Vec3 windVel = {0.0f, 0.0f, 0.0f};
            m3real windDrag = 0.0f;
            if (world->windSpeed > 0.0f)
            {
                m3real gust = 1.0f + world->windGustScale * sinf(world->windPhase);
                windVel = m3MulSV3(world->windSpeed * gust, world->windDir);
                windDrag = 0.5f; // documented fixed coefficient (v1)
            }
            for (int32_t i = 0; i < count; ++i)
            {
                int32_t k = base + i;
                if (world->softInvMass[k] == 0.0f ||
                    (anchored[i >> 3] & (uint8_t)(1u << (i & 7))) != 0)
                {
                    world->softPrev[k] = world->softPos[k];
                    // A blast kick on a pinned particle evaporates:
                    // it must not linger in the hash forever (13-3).
                    world->softKick[k] = (m3Vec3){0.0f, 0.0f, 0.0f};
                    continue;
                }
                m3Vec3 v = {(m3real)(world->softPos[k].x - world->softPrev[k].x) * invH,
                            (m3real)(world->softPos[k].y - world->softPrev[k].y) * invH,
                            (m3real)(world->softPos[k].z - world->softPrev[k].z) * invH};
                v = m3Add3(v, m3MulSV3(h, g));
                m3Vec3 kick = world->softKick[k];
                if (kick.x != 0.0f || kick.y != 0.0f || kick.z != 0.0f)
                {
                    // The pending explosion kick lands exactly once,
                    // on the first substep that integrates it (13-3).
                    v = m3Add3(v, kick);
                    world->softKick[k] = (m3Vec3){0.0f, 0.0f, 0.0f};
                }
                // The 8-4 hard speed cap covers particles too (14-4
                // cure): the 13-4 storm rode a mutated blast into
                // float overflow, positions went NaN, and a NaN cell
                // index was undefined behavior. A capped velocity
                // can never outrun the double range.
                m3real pv2 = m3Dot3(v, v);
                m3real pcap = world->maximumLinearSpeed;
                if (pv2 > pcap * pcap)
                {
                    v = m3MulSV3(pcap / sqrtf(pv2), v);
                }
                if (windDrag > 0.0f)
                {
                    v = m3Add3(v, m3MulSV3(h * windDrag, m3Sub3(windVel, v)));
                }
                // Water (18-2): a submerged particle trades gravity
                // for buoyancy under the rho-1000 particle
                // convention (density 1000 suspends, denser fluids
                // lift, thinner ones let it sink) and drags toward
                // the flow like wind. First live volume wins,
                // deterministically, and dry worlds never reach the
                // loop (the pool is empty).
                for (int32_t wv = 0; wv < world->waterPool.maxIndex; ++wv)
                {
                    if (world->waterPool.alive[wv] == 0 ||
                        world->softPos[k].x < world->waterLo[wv].x ||
                        world->softPos[k].x > world->waterHi[wv].x ||
                        world->softPos[k].y < world->waterLo[wv].y ||
                        world->softPos[k].y > world->waterHi[wv].y ||
                        world->softPos[k].z < world->waterLo[wv].z ||
                        world->softPos[k].z > world->waterHi[wv].z)
                    {
                        continue;
                    }
                    v = m3Add3(v, m3MulSV3(-h * world->waterDensity[wv] * (1.0f / 1000.0f), g));
                    v = m3Add3(
                        v, m3MulSV3(h * world->waterLinDrag[wv], m3Sub3(world->waterFlow[wv], v)));
                    break;
                }
                world->softPrev[k] = world->softPos[k];
                world->softPos[k].x += (double)(v.x * h);
                world->softPos[k].y += (double)(v.y * h);
                world->softPos[k].z += (double)(v.z * h);
            }

            // One Gauss-Seidel sweep over the edges, fixed order.
            m3real alpha = world->softCompliance[slot] * invH * invH;
            m3real bendAlpha = world->softBendCompliance[slot] * invH * invH;
            int32_t bendStart = world->softBendStart[slot];
            int32_t edges = world->softEdgeCount[slot];
            int32_t ebase = slot * M3_SOFTBODY_MAX_EDGES;
            for (int32_t e = 0; e < edges; ++e)
            {
                m3real alphaE = e >= bendStart ? bendAlpha : alpha;
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
                m3real scale = -c / ((wSum + alphaE) * len);
                world->softPos[ka].x += (double)(wa * scale * diff.x);
                world->softPos[ka].y += (double)(wa * scale * diff.y);
                world->softPos[ka].z += (double)(wa * scale * diff.z);
                world->softPos[kb].x -= (double)(wb * scale * diff.x);
                world->softPos[kb].y -= (double)(wb * scale * diff.y);
                world->softPos[kb].z -= (double)(wb * scale * diff.z);
            }

            // The bind tether (20-4): a hard clamp to the create
            // pose radius, BEFORE the volume rows so a crushed tet
            // still restores its volume (the tether is a bound,
            // the volumes are promises; documented order).
            if (world->softMaxDeviation[slot] > 0.0f)
            {
                m3real maxDev = world->softMaxDeviation[slot];
                for (int32_t i = 0; i < count; ++i)
                {
                    int32_t k2 = base + i;
                    if (world->softInvMass[k2] == 0.0f)
                    {
                        continue;
                    }
                    m3Vec3 off = {(m3real)(world->softPos[k2].x - world->softBindPos[k2].x),
                                  (m3real)(world->softPos[k2].y - world->softBindPos[k2].y),
                                  (m3real)(world->softPos[k2].z - world->softBindPos[k2].z)};
                    m3real len2 = m3Dot3(off, off);
                    if (len2 > maxDev * maxDev)
                    {
                        m3real scale = maxDev / sqrtf(len2);
                        world->softPos[k2].x = world->softBindPos[k2].x + (double)(off.x * scale);
                        world->softPos[k2].y = world->softBindPos[k2].y + (double)(off.y * scale);
                        world->softPos[k2].z = world->softBindPos[k2].z + (double)(off.z * scale);
                    }
                }
            }
            // Tet volume rows (20-3): one rigid row per tet in
            // fixed order (6V against the rest, the pressure
            // gradients localized to four particles).
            int32_t tetCount2 = world->softTetCount[slot];
            for (int32_t t = 0; t < tetCount2; ++t)
            {
                int32_t tk = slot * M3_SOFTBODY_MAX_TETS + t;
                int32_t ia = base + (int32_t)world->softTetA[tk];
                int32_t ib = base + (int32_t)world->softTetB[tk];
                int32_t ic = base + (int32_t)world->softTetC[tk];
                int32_t id2 = base + (int32_t)world->softTetD[tk];
                m3Pos3 o = world->softPos[ia];
                m3Vec3 pb2 = {(m3real)(world->softPos[ib].x - o.x),
                              (m3real)(world->softPos[ib].y - o.y),
                              (m3real)(world->softPos[ib].z - o.z)};
                m3Vec3 pc2 = {(m3real)(world->softPos[ic].x - o.x),
                              (m3real)(world->softPos[ic].y - o.y),
                              (m3real)(world->softPos[ic].z - o.z)};
                m3Vec3 pd2 = {(m3real)(world->softPos[id2].x - o.x),
                              (m3real)(world->softPos[id2].y - o.y),
                              (m3real)(world->softPos[id2].z - o.z)};
                m3real v6 = m3Dot3(pb2, m3Cross3(pc2, pd2));
                m3real c6 = v6 - world->softTetRestV6[tk];
                m3Vec3 gb = m3Cross3(pc2, pd2);
                m3Vec3 gc = m3Cross3(pd2, pb2);
                m3Vec3 gd = m3Cross3(pb2, pc2);
                m3Vec3 ga = m3Neg3(m3Add3(gb, m3Add3(gc, gd)));
                m3real wa = world->softInvMass[ia];
                m3real wb = world->softInvMass[ib];
                m3real wc = world->softInvMass[ic];
                m3real wd = world->softInvMass[id2];
                int32_t la = ia - base;
                int32_t lb = ib - base;
                int32_t lc = ic - base;
                int32_t ld = id2 - base;
                if ((anchored[la >> 3] & (uint8_t)(1u << (la & 7))) != 0)
                {
                    wa = 0.0f;
                }
                if ((anchored[lb >> 3] & (uint8_t)(1u << (lb & 7))) != 0)
                {
                    wb = 0.0f;
                }
                if ((anchored[lc >> 3] & (uint8_t)(1u << (lc & 7))) != 0)
                {
                    wc = 0.0f;
                }
                if ((anchored[ld >> 3] & (uint8_t)(1u << (ld & 7))) != 0)
                {
                    wd = 0.0f;
                }
                m3real denom = wa * m3Dot3(ga, ga) + wb * m3Dot3(gb, gb) + wc * m3Dot3(gc, gc) +
                               wd * m3Dot3(gd, gd);
                if (denom > 1.0e-9f)
                {
                    m3real lambda = -c6 / denom;
                    m3Vec3 dp;
                    dp = m3MulSV3(lambda * wa, ga);
                    world->softPos[ia].x += (double)dp.x;
                    world->softPos[ia].y += (double)dp.y;
                    world->softPos[ia].z += (double)dp.z;
                    dp = m3MulSV3(lambda * wb, gb);
                    world->softPos[ib].x += (double)dp.x;
                    world->softPos[ib].y += (double)dp.y;
                    world->softPos[ib].z += (double)dp.z;
                    dp = m3MulSV3(lambda * wc, gc);
                    world->softPos[ic].x += (double)dp.x;
                    world->softPos[ic].y += (double)dp.y;
                    world->softPos[ic].z += (double)dp.z;
                    dp = m3MulSV3(lambda * wd, gd);
                    world->softPos[id2].x += (double)dp.x;
                    world->softPos[id2].y += (double)dp.y;
                    world->softPos[id2].z += (double)dp.z;
                }
            }
            // The pressure row (20-2): one global volume constraint
            // per substep, PBD-projected in fixed particle order.
            // Gradients are of 6V, so the projection solves
            // C6 = 6 (V - target) against them directly: the sixes
            // cancel and no epsilon-sensitive division sneaks in.
            if (world->softPressure[slot] > 0.0f)
            {
                m3Vec3 grads[M3_SOFTBODY_MAX_PARTICLES];
                m3Pos3 origin = world->softPos[base];
                m3real vol6 = SoftSurfaceVolume6(world, slot, grads, origin);
                m3real c6 = vol6 - 6.0f * world->softRestVolume[slot] * world->softPressure[slot];
                m3real denom = 0.0f;
                for (int32_t i = 0; i < count; ++i)
                {
                    int pinned = world->softInvMass[base + i] == 0.0f ||
                                 (anchored[i >> 3] & (uint8_t)(1u << (i & 7))) != 0;
                    if (!pinned)
                    {
                        denom += world->softInvMass[base + i] * m3Dot3(grads[i], grads[i]);
                    }
                }
                if (denom > 1.0e-9f)
                {
                    m3real lambda = -c6 / denom;
                    for (int32_t i = 0; i < count; ++i)
                    {
                        int pinned = world->softInvMass[base + i] == 0.0f ||
                                     (anchored[i >> 3] & (uint8_t)(1u << (i & 7))) != 0;
                        if (!pinned)
                        {
                            m3Vec3 dp = m3MulSV3(lambda * world->softInvMass[base + i], grads[i]);
                            world->softPos[base + i].x += (double)dp.x;
                            world->softPos[base + i].y += (double)dp.y;
                            world->softPos[base + i].z += (double)dp.z;
                        }
                    }
                }
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

        // Soft-vs-soft (11-1): particle pairs between DIFFERENT
        // lattices, the gap the competitors' own docs admit.
        // Canonical order: lower slot first, ascending particle
        // indices, one projection per substep. Self-collision stays
        // out on purpose: a box lattice's structure rods already
        // hold it apart at these scales (documented since 7-1).
        // No new snapshot state: contacts are transient projections.
        for (int32_t sa = 0; sa < world->softPool.maxIndex; ++sa)
        {
            if (world->softPool.alive[sa] == 0)
            {
                continue;
            }
            int32_t countA = world->softParticleCount[sa];
            int32_t baseA = sa * M3_SOFTBODY_MAX_PARTICLES;
            m3real ra = world->softRadius[sa];
            // Lattice bounds, current predicted positions.
            double loA[3] = {1.0e30, 1.0e30, 1.0e30};
            double hiA[3] = {-1.0e30, -1.0e30, -1.0e30};
            for (int32_t i = 0; i < countA; ++i)
            {
                const m3Pos3* p = &world->softPos[baseA + i];
                loA[0] = p->x < loA[0] ? p->x : loA[0];
                loA[1] = p->y < loA[1] ? p->y : loA[1];
                loA[2] = p->z < loA[2] ? p->z : loA[2];
                hiA[0] = p->x > hiA[0] ? p->x : hiA[0];
                hiA[1] = p->y > hiA[1] ? p->y : hiA[1];
                hiA[2] = p->z > hiA[2] ? p->z : hiA[2];
            }
            for (int32_t sb = sa + 1; sb < world->softPool.maxIndex; ++sb)
            {
                if (world->softPool.alive[sb] == 0)
                {
                    continue;
                }
                int32_t countB = world->softParticleCount[sb];
                int32_t baseB = sb * M3_SOFTBODY_MAX_PARTICLES;
                m3real rb = world->softRadius[sb];
                double reach = (double)(ra + rb);
                double loB[3] = {1.0e30, 1.0e30, 1.0e30};
                double hiB[3] = {-1.0e30, -1.0e30, -1.0e30};
                for (int32_t j = 0; j < countB; ++j)
                {
                    const m3Pos3* p = &world->softPos[baseB + j];
                    loB[0] = p->x < loB[0] ? p->x : loB[0];
                    loB[1] = p->y < loB[1] ? p->y : loB[1];
                    loB[2] = p->z < loB[2] ? p->z : loB[2];
                    hiB[0] = p->x > hiB[0] ? p->x : hiB[0];
                    hiB[1] = p->y > hiB[1] ? p->y : hiB[1];
                    hiB[2] = p->z > hiB[2] ? p->z : hiB[2];
                }
                if (loA[0] > hiB[0] + reach || loB[0] > hiA[0] + reach || loA[1] > hiB[1] + reach ||
                    loB[1] > hiA[1] + reach || loA[2] > hiB[2] + reach || loB[2] > hiA[2] + reach)
                {
                    continue; // lattices out of reach: no pair work
                }
                m3real target = ra + rb;
                m3real target2 = target * target;
                for (int32_t i = 0; i < countA; ++i)
                {
                    int32_t ka = baseA + i;
                    m3real wa = world->softInvMass[ka];
                    for (int32_t j = 0; j < countB; ++j)
                    {
                        int32_t kb = baseB + j;
                        m3Vec3 d = {(m3real)(world->softPos[ka].x - world->softPos[kb].x),
                                    (m3real)(world->softPos[ka].y - world->softPos[kb].y),
                                    (m3real)(world->softPos[ka].z - world->softPos[kb].z)};
                        m3real dist2 = m3Dot3(d, d);
                        if (dist2 >= target2 || dist2 <= 1.0e-12f)
                        {
                            continue; // apart, or dead-centered (skip:
                                      // no deterministic normal exists)
                        }
                        m3real wb = world->softInvMass[kb];
                        m3real wSum = wa + wb;
                        if (wSum <= 0.0f)
                        {
                            continue;
                        }
                        m3real dist = sqrtf(dist2);
                        m3Vec3 n = m3MulSV3(1.0f / dist, d);
                        m3real pen = target - dist;
                        m3Vec3 pushA = m3MulSV3(pen * wa / wSum, n);
                        m3Vec3 pushB = m3MulSV3(-pen * wb / wSum, n);
                        world->softPos[ka].x += (double)pushA.x;
                        world->softPos[ka].y += (double)pushA.y;
                        world->softPos[ka].z += (double)pushA.z;
                        world->softPos[kb].x += (double)pushB.x;
                        world->softPos[kb].y += (double)pushB.y;
                        world->softPos[kb].z += (double)pushB.z;
                        // PBD friction, the SoftProject rule with a
                        // fixed mix (no per-lattice friction state in
                        // v1, documented): tangential motion this
                        // substep shrinks by mu times the correction.
                        const m3real mu = 0.5f;
                        m3Vec3 velA = {(m3real)(world->softPos[ka].x - world->softPrev[ka].x),
                                       (m3real)(world->softPos[ka].y - world->softPrev[ka].y),
                                       (m3real)(world->softPos[ka].z - world->softPrev[ka].z)};
                        m3Vec3 velB = {(m3real)(world->softPos[kb].x - world->softPrev[kb].x),
                                       (m3real)(world->softPos[kb].y - world->softPrev[kb].y),
                                       (m3real)(world->softPos[kb].z - world->softPrev[kb].z)};
                        m3Vec3 rel = m3Sub3(velA, velB);
                        m3Vec3 tangential = m3Sub3(rel, m3MulSV3(m3Dot3(rel, n), n));
                        m3real tLen = m3Length3(tangential);
                        if (tLen > 1.0e-9f)
                        {
                            m3real budget = mu * pen;
                            m3real cut = tLen < budget ? tLen : budget;
                            m3Vec3 corr = m3MulSV3(cut / tLen, tangential);
                            m3Vec3 corrA = m3MulSV3(-wa / wSum, corr);
                            m3Vec3 corrB = m3MulSV3(wb / wSum, corr);
                            world->softPos[ka].x += (double)corrA.x;
                            world->softPos[ka].y += (double)corrA.y;
                            world->softPos[ka].z += (double)corrA.z;
                            world->softPos[kb].x += (double)corrB.x;
                            world->softPos[kb].y += (double)corrB.y;
                            world->softPos[kb].z += (double)corrB.z;
                        }
                    }
                }
            }
        }

        // Soft-to-soft anchors (11-2): position equality between two
        // lattices' particles, split by inverse mass, canonical
        // owner order (the lower slot holds the pin). EITHER side
        // dying releases the pin silently: the 7-3 liveness lesson
        // applied in both directions.
        for (int32_t sa = 0; sa < world->softPool.maxIndex; ++sa)
        {
            if (world->softPool.alive[sa] == 0)
            {
                continue;
            }
            int32_t pins = world->softSoftCount[sa];
            for (int32_t a = 0; a < pins; ++a)
            {
                int32_t ak = sa * M3_SOFTBODY_MAX_ANCHORS + a;
                int32_t sb = world->softSoftSlotB[ak];
                if (sb < 0 || world->softPool.alive[sb] == 0 ||
                    world->softPool.generations[sb] != world->softSoftGenB[ak])
                {
                    continue; // the far lattice died: silent release
                }
                int32_t ka = sa * M3_SOFTBODY_MAX_PARTICLES + world->softSoftParticleA[ak];
                int32_t kb = sb * M3_SOFTBODY_MAX_PARTICLES + world->softSoftParticleB[ak];
                m3real wa = world->softInvMass[ka];
                m3real wb = world->softInvMass[kb];
                m3real wSum = wa + wb;
                if (wSum <= 0.0f)
                {
                    continue; // both pinned rigid: nothing to split
                }
                m3Vec3 d = {(m3real)(world->softPos[kb].x - world->softPos[ka].x),
                            (m3real)(world->softPos[kb].y - world->softPos[ka].y),
                            (m3real)(world->softPos[kb].z - world->softPos[ka].z)};
                m3Vec3 moveA = m3MulSV3(wa / wSum, d);
                m3Vec3 moveB = m3MulSV3(-wb / wSum, d);
                world->softPos[ka].x += (double)moveA.x;
                world->softPos[ka].y += (double)moveA.y;
                world->softPos[ka].z += (double)moveA.z;
                world->softPos[kb].x += (double)moveB.x;
                world->softPos[kb].y += (double)moveB.y;
                world->softPos[kb].z += (double)moveB.z;
            }
        }
    }
}

m3SoftBodyId m3CreateSoftBody(m3WorldId worldId, const m3SoftBodyDef* def)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || def == NULL || def->internalValue != M3_SOFTBODY_COOKIE)
    {
        return m3_nullSoftBodyId; // field checks live in the internal
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

void m3SoftBodyAnchorSoftInternal(m3World* world, int32_t slotA, int32_t particleA, int32_t slotB,
                                  int32_t particleB)
{
    // The LOWER slot owns the pin: one canonical home per pair.
    if (slotB < slotA)
    {
        int32_t ts = slotA;
        slotA = slotB;
        slotB = ts;
        int32_t tp = particleA;
        particleA = particleB;
        particleB = tp;
    }
    int32_t a = world->softSoftCount[slotA];
    int32_t ak = slotA * M3_SOFTBODY_MAX_ANCHORS + a;
    world->softSoftParticleA[ak] = particleA;
    world->softSoftSlotB[ak] = slotB;
    world->softSoftGenB[ak] = world->softPool.generations[slotB];
    world->softSoftParticleB[ak] = particleB;
    world->softSoftCount[slotA] = a + 1;
}

void m3SoftBody_AnchorToSoft(m3SoftBodyId softIdA, int32_t particleA, m3SoftBodyId softIdB,
                             int32_t particleB)
{
    m3World* world = m3WorldFromIndex0(softIdA.world0);
    int32_t slotA = world != NULL ? m3SoftBodySlot(world, softIdA) : -1;
    int32_t slotB = world != NULL ? m3SoftBodySlot(world, softIdB) : -1;
    if (slotA < 0 || slotB < 0 || slotA == slotB || particleA < 0 || particleB < 0 ||
        particleA >= world->softParticleCount[slotA] ||
        particleB >= world->softParticleCount[slotB] ||
        world->softSoftCount[slotA < slotB ? slotA : slotB] >= M3_SOFTBODY_MAX_ANCHORS)
    {
        return; // stale, self-pin, out of range, or full: quiet no-op
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m3SoftBodyId idA;
            int32_t particleA;
            m3SoftBodyId idB;
            int32_t particleB;
        } record;
        memset(&record, 0, sizeof(record));
        record.idA = softIdA;
        record.particleA = particleA;
        record.idB = softIdB;
        record.particleB = particleB;
        m3JournalRecord(world, m3_opSoftBodyAnchorSoft, &record, (int32_t)sizeof(record));
    }
    m3SoftBodyAnchorSoftInternal(world, slotA, particleA, slotB, particleB);
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
