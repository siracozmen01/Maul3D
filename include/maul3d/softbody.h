// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Deterministic soft bodies (7-1): XPBD particle lattices. A soft
// body is a box lattice of particles bound by distance constraints
// (structural edges plus face diagonals) solved Gauss-Seidel in
// FIXED index order per substep: deterministic by construction,
// under the same four gates as everything else. Rope (n x 1 x 1),
// cloth (n x m x 1), and jelly (n x m x k) all fall out of the one
// factory. Engine-owned like characters and vehicles: pooled ids,
// journaled ops, snapshotted and hashed state, bit-exact rollback.

#ifndef MAUL3D_SOFTBODY_H
#define MAUL3D_SOFTBODY_H

#include "world.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define M3_SOFTBODY_MAX_PARTICLES 512
#define M3_SOFTBODY_MAX_EDGES     5120
#define M3_SOFTBODY_MAX_ANCHORS   32

    typedef struct m3SoftBodyId
    {
        int32_t index1;
        uint16_t world0;
        uint16_t generation;
    } m3SoftBodyId;

    typedef struct m3SoftBodyDef
    {
        m3Pos3 position;     // lattice minimum corner
        int32_t countX;      // particles per axis, each >= 1; the
        int32_t countY;      // product must fit
        int32_t countZ;      // M3_SOFTBODY_MAX_PARTICLES
        m3real spacing;      // rest distance between lattice neighbors
        m3real particleMass; // per particle, kilograms
        m3real compliance;   // XPBD compliance; zero = rigid rods
        m3real radius;       // particle collision radius
        m3real gravityScale;
        uint64_t userData;
        int32_t internalValue;
    } m3SoftBodyDef;

    M3_API m3SoftBodyDef m3DefaultSoftBodyDef(void);

    /// Creates the lattice with structural edges and face diagonals
    /// in fixed index order. Refuses (null id) hostile counts,
    /// non-finite or non-positive geometry, or a full pool.
    /// Journaled with id verification.
    M3_API m3SoftBodyId m3CreateSoftBody(m3WorldId worldId, const m3SoftBodyDef* def);
    M3_API void m3DestroySoftBody(m3SoftBodyId softId);
    M3_API bool m3SoftBody_IsValid(m3SoftBodyId softId);

    /// Pins one particle in place (inverse mass zero): how a rope
    /// hangs and a flag flies. Journaled; out-of-range or stale
    /// pins are quiet no-ops. Particle index is
    /// x + countX * (y + countY * z).
    M3_API void m3SoftBody_PinParticle(m3SoftBodyId softId, int32_t particle);

    /// Anchors one particle to a body at the particle's CURRENT
    /// position, expressed in the body's frame: the particle rides
    /// the body from then on, and the lattice's pull on it lands on
    /// the body as an impulse at the anchor (two-way, 7-3). Cloth
    /// hangs from beams and jelly rides trucks through this.
    /// Anchoring to a static body is a moving pin; the anchor
    /// RELEASES silently if its body dies. Journaled; stale ids,
    /// out-of-range particles, and a full anchor table (32 per
    /// soft body) are quiet no-ops.
    M3_API void m3SoftBody_AnchorParticle(m3SoftBodyId softId, int32_t particle, m3BodyId bodyId);

    M3_API int32_t m3SoftBody_GetParticleCount(m3SoftBodyId softId);
    M3_API m3Pos3 m3SoftBody_GetParticlePosition(m3SoftBodyId softId, int32_t particle);

    static const m3SoftBodyId m3_nullSoftBodyId = {0, 0, 0};

#ifdef __cplusplus
}
#endif

#endif // MAUL3D_SOFTBODY_H
