// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The Soft Step solver in 3D, ported from Maul2D's proven solver.c
// (itself adapted from Box2D v3, MIT, Erin Catto): prepare once, then
// per substep [integrate velocities, warm start, solve with bias,
// integrate positions, relax without bias], then a restitution pass
// and the impulse store. Separations are re-evaluated inside substeps
// from accumulated float deltas (never fresh world-space math), the
// Jacobian uses FIXED prepare-time anchors (reference discipline), and
// friction clamps to the Coulomb disc by scaling, never a per-axis
// box. 2a bodies are spheres: inertia is a scalar and gyroscopic
// torque is exactly zero, so the implicit gyroscopic solve arrives
// with hulls in 2b.

#include "world_internal.h"

#include <string.h>

#define M3_CONTACT_HERTZ          30.0f
#define M3_CONTACT_DAMPING_RATIO  10.0f
#define M3_CONTACT_PUSH_MAX_SPEED 3.0f
#define M3_RESTITUTION_THRESHOLD  1.0f

typedef struct m3Softness
{
    m3real biasRate;
    m3real massScale;
    m3real impulseScale;
} m3Softness;

// Reference formula (b2MakeSoft, copied verbatim from Maul2D):
// bias = w/(2z+hw), massScale = hw(2z+hw)/(1+hw(2z+hw)),
// impulseScale = 1/(1+hw(2z+hw)).
static m3Softness MakeSoft(m3real hertz, m3real zeta, m3real h)
{
    if (hertz == 0.0f)
    {
        return (m3Softness){0.0f, 0.0f, 0.0f};
    }
    m3real omega = 2.0f * M3_PI * hertz;
    m3real a1 = 2.0f * zeta + h * omega;
    m3real a2 = h * omega * a1;
    m3real a3 = 1.0f / (1.0f + a2);
    return (m3Softness){omega / a1, a2 * a3, a3};
}

typedef struct m3ConstraintPoint
{
    m3Vec3 rA; // prepare-time anchors from each body's COM
    m3Vec3 rB;
    m3real baseSeparation; // separation minus dot(rB - rA, n) at prepare
    m3real normalMass;
    m3real tangentMass1;
    m3real tangentMass2;
    m3real relativeVelocity; // vn at prepare, for the restitution pass
    m3real normalImpulse;
    m3real tangentImpulse1;
    m3real tangentImpulse2;
} m3ConstraintPoint;

typedef struct m3ContactConstraint
{
    int32_t bodyA;
    int32_t bodyB;
    int32_t manifoldIndex;
    int32_t pointCount;
    m3Vec3 normal;
    m3Vec3 t1;
    m3Vec3 t2;
    m3real friction;
    m3real restitution;
    m3real invMassA;
    m3real invMassB;
    m3Mat3 invIA; // world-space inverse inertia, frozen at prepare
    m3Mat3 invIB;
    m3Softness softness;
    m3ConstraintPoint points[M3_MANIFOLD_MAX_POINTS];
} m3ContactConstraint;

static m3Vec3 VelocityAt(const m3World* world, int32_t body, m3Vec3 arm)
{
    m3Vec3 v = world->linearVelocities[body];
    m3Vec3 w = world->angularVelocities[body];
    return m3Add3(v, m3Cross3(w, arm));
}

static void ApplyImpulse(m3World* world, const m3ContactConstraint* c, m3Vec3 impulse, m3Vec3 rsA,
                         m3Vec3 rsB)
{
    if (world->types[c->bodyA] == (uint8_t)m3_dynamicBody)
    {
        world->linearVelocities[c->bodyA] =
            m3Sub3(world->linearVelocities[c->bodyA], m3MulSV3(c->invMassA, impulse));
        world->angularVelocities[c->bodyA] =
            m3Sub3(world->angularVelocities[c->bodyA], m3MulMV3(c->invIA, m3Cross3(rsA, impulse)));
    }
    if (world->types[c->bodyB] == (uint8_t)m3_dynamicBody)
    {
        world->linearVelocities[c->bodyB] =
            m3Add3(world->linearVelocities[c->bodyB], m3MulSV3(c->invMassB, impulse));
        world->angularVelocities[c->bodyB] =
            m3Add3(world->angularVelocities[c->bodyB], m3MulMV3(c->invIB, m3Cross3(rsB, impulse)));
    }
}

// Effective mass along one direction for one contact point's arms.
static m3real EffectiveMass(const m3ContactConstraint* c, m3Vec3 rA, m3Vec3 rB, m3Vec3 dir)
{
    m3Vec3 arm1 = m3Cross3(rA, dir);
    m3Vec3 arm2 = m3Cross3(rB, dir);
    m3real k = c->invMassA + c->invMassB + m3Dot3(arm1, m3MulMV3(c->invIA, arm1)) +
               m3Dot3(arm2, m3MulMV3(c->invIB, arm2));
    return k > 0.0f ? 1.0f / k : 0.0f;
}
// I_w^-1 = R I_l^-1 R^T, built by applying the operator to the world
// basis vectors. Frozen at prepare like the anchors (reference
// discipline); the per-substep refresh arrives with the gyroscopic
// slice (2b-6).
static m3Mat3 WorldInvInertia(const m3World* world, int32_t body)
{
    if (world->types[body] != (uint8_t)m3_dynamicBody)
    {
        return m3MakeZeroMat3();
    }
    m3Quat q = world->transforms[body].q;
    m3Mat3 il = world->invInertiaLocal[body];
    m3Mat3 r;
    r.cx = m3RotateVec3(q, m3MulMV3(il, m3InvRotateVec3(q, (m3Vec3){1.0f, 0.0f, 0.0f})));
    r.cy = m3RotateVec3(q, m3MulMV3(il, m3InvRotateVec3(q, (m3Vec3){0.0f, 1.0f, 0.0f})));
    r.cz = m3RotateVec3(q, m3MulMV3(il, m3InvRotateVec3(q, (m3Vec3){0.0f, 0.0f, 1.0f})));
    return r;
}

static int32_t PrepareContacts(m3World* world, m3ContactConstraint* constraints, m3real h)
{
    m3Softness soft = MakeSoft(M3_CONTACT_HERTZ, M3_CONTACT_DAMPING_RATIO, h);
    m3Softness staticSoft = MakeSoft(2.0f * M3_CONTACT_HERTZ, M3_CONTACT_DAMPING_RATIO, h);

    int32_t count = 0;
    for (int32_t i = 0; i < world->pairCount; ++i)
    {
        const m3Manifold* manifold = &world->manifolds[i];
        if (manifold->pointCount == 0)
        {
            continue;
        }
        uint64_t key = world->pairKeys[i];
        int32_t shapeA = (int32_t)(key >> 32);
        int32_t shapeB = (int32_t)(key & 0xFFFFFFFFu);
        int32_t bodyA = world->shapeBody[shapeA];
        int32_t bodyB = world->shapeBody[shapeB];
        int awakeDynA = world->types[bodyA] == (uint8_t)m3_dynamicBody && world->awake[bodyA] != 0;
        int awakeDynB = world->types[bodyB] == (uint8_t)m3_dynamicBody && world->awake[bodyB] != 0;
        if (!awakeDynA && !awakeDynB)
        {
            continue; // both sides frozen or immovable: impulses stay put
        }

        m3ContactConstraint* c = constraints + count;
        count += 1;
        memset(c, 0, sizeof(*c));
        c->bodyA = bodyA;
        c->bodyB = bodyB;
        c->manifoldIndex = i;
        c->pointCount = manifold->pointCount;
        c->normal = manifold->normal;
        m3MakeTangentBasis(c->normal, &c->t1, &c->t2);
        c->invMassA = world->types[bodyA] == (uint8_t)m3_dynamicBody ? world->invMass[bodyA] : 0.0f;
        c->invMassB = world->types[bodyB] == (uint8_t)m3_dynamicBody ? world->invMass[bodyB] : 0.0f;
        c->invIA = WorldInvInertia(world, bodyA);
        c->invIB = WorldInvInertia(world, bodyB);
        // Reference mixing: friction geometric, restitution maximum.
        c->friction = sqrtf(world->shapeFriction[shapeA] * world->shapeFriction[shapeB]);
        c->restitution = m3MaxF(world->shapeRestitution[shapeA], world->shapeRestitution[shapeB]);
        c->softness = (c->invMassA == 0.0f || c->invMassB == 0.0f) ? staticSoft : soft;

        for (int32_t k = 0; k < c->pointCount; ++k)
        {
            m3ConstraintPoint* cp = &c->points[k];
            cp->rA = manifold->points[k].anchorA;
            cp->rB = manifold->points[k].anchorB;
            cp->baseSeparation = manifold->points[k].separation -
                                 (m3Dot3(cp->rB, c->normal) - m3Dot3(cp->rA, c->normal));
            cp->normalMass = EffectiveMass(c, cp->rA, cp->rB, c->normal);
            cp->tangentMass1 = EffectiveMass(c, cp->rA, cp->rB, c->t1);
            cp->tangentMass2 = EffectiveMass(c, cp->rA, cp->rB, c->t2);
            cp->relativeVelocity =
                m3Dot3(m3Sub3(VelocityAt(world, bodyB, cp->rB), VelocityAt(world, bodyA, cp->rA)),
                       c->normal);
            cp->normalImpulse = manifold->points[k].normalImpulse;
            cp->tangentImpulse1 = manifold->points[k].tangentImpulse1;
            cp->tangentImpulse2 = manifold->points[k].tangentImpulse2;
        }
    }
    return count;
}
static void WarmStart(m3World* world, m3ContactConstraint* constraints, int32_t count)
{
    for (int32_t i = 0; i < count; ++i)
    {
        m3ContactConstraint* c = constraints + i;
        for (int32_t k = 0; k < c->pointCount; ++k)
        {
            m3ConstraintPoint* cp = &c->points[k];
            m3Vec3 impulse = m3Add3(
                m3MulSV3(cp->normalImpulse, c->normal),
                m3Add3(m3MulSV3(cp->tangentImpulse1, c->t1), m3MulSV3(cp->tangentImpulse2, c->t2)));
            ApplyImpulse(world, c, impulse, cp->rA, cp->rB);
        }
    }
}
static void SolveContacts(m3World* world, m3ContactConstraint* constraints, int32_t count,
                          const m3Vec3* deltaPos, const m3Quat* deltaRot, m3real invH, bool useBias)
{
    for (int32_t i = 0; i < count; ++i)
    {
        m3ContactConstraint* c = constraints + i;

        // Normal rows first, then friction rows, per the reference
        // schedule. The Jacobian keeps the fixed prepare-time anchors;
        // rotated anchors only measure the separation drift.
        for (int32_t k = 0; k < c->pointCount; ++k)
        {
            m3ConstraintPoint* cp = &c->points[k];
            m3Vec3 rsA = m3RotateVec3(deltaRot[c->bodyA], cp->rA);
            m3Vec3 rsB = m3RotateVec3(deltaRot[c->bodyB], cp->rB);
            m3Vec3 ds = m3Add3(m3Sub3(deltaPos[c->bodyB], deltaPos[c->bodyA]), m3Sub3(rsB, rsA));
            m3real s = cp->baseSeparation + m3Dot3(ds, c->normal);

            m3real bias = 0.0f;
            m3real massScale = 1.0f;
            m3real impulseScale = 0.0f;
            if (s > 0.0f)
            {
                bias = s * invH; // speculative: prevent crossing
            }
            else if (useBias)
            {
                bias = m3MaxF(c->softness.biasRate * s, -M3_CONTACT_PUSH_MAX_SPEED);
                massScale = c->softness.massScale;
                impulseScale = c->softness.impulseScale;
            }

            m3real vn = m3Dot3(
                m3Sub3(VelocityAt(world, c->bodyB, cp->rB), VelocityAt(world, c->bodyA, cp->rA)),
                c->normal);
            m3real impulse =
                -cp->normalMass * massScale * (vn + bias) - impulseScale * cp->normalImpulse;
            m3real newImpulse = m3MaxF(cp->normalImpulse + impulse, 0.0f);
            impulse = newImpulse - cp->normalImpulse;
            cp->normalImpulse = newImpulse;
            ApplyImpulse(world, c, m3MulSV3(impulse, c->normal), cp->rA, cp->rB);
        }

        for (int32_t k = 0; k < c->pointCount; ++k)
        {
            m3ConstraintPoint* cp = &c->points[k];
            m3Vec3 vrel =
                m3Sub3(VelocityAt(world, c->bodyB, cp->rB), VelocityAt(world, c->bodyA, cp->rA));
            m3real vt1 = m3Dot3(vrel, c->t1);
            m3real vt2 = m3Dot3(vrel, c->t2);
            m3real f1 = cp->tangentImpulse1 - cp->tangentMass1 * vt1;
            m3real f2 = cp->tangentImpulse2 - cp->tangentMass2 * vt2;
            m3real maxFriction = c->friction * cp->normalImpulse;
            m3real mag2 = f1 * f1 + f2 * f2;
            if (mag2 > maxFriction * maxFriction)
            {
                m3real mag = sqrtf(mag2);
                m3real scale = mag > 0.0f ? maxFriction / mag : 0.0f;
                f1 *= scale;
                f2 *= scale;
            }
            m3real d1 = f1 - cp->tangentImpulse1;
            m3real d2 = f2 - cp->tangentImpulse2;
            cp->tangentImpulse1 = f1;
            cp->tangentImpulse2 = f2;
            ApplyImpulse(world, c, m3Add3(m3MulSV3(d1, c->t1), m3MulSV3(d2, c->t2)), cp->rA,
                         cp->rB);
        }
    }
}
static void Restitution(m3World* world, m3ContactConstraint* constraints, int32_t count)
{
    for (int32_t i = 0; i < count; ++i)
    {
        m3ContactConstraint* c = constraints + i;
        if (c->restitution == 0.0f)
        {
            continue;
        }
        for (int32_t k = 0; k < c->pointCount; ++k)
        {
            m3ConstraintPoint* cp = &c->points[k];
            if (cp->relativeVelocity > -M3_RESTITUTION_THRESHOLD || cp->normalImpulse == 0.0f)
            {
                continue;
            }
            m3real vn = m3Dot3(
                m3Sub3(VelocityAt(world, c->bodyB, cp->rB), VelocityAt(world, c->bodyA, cp->rA)),
                c->normal);
            m3real impulse = -cp->normalMass * (vn + c->restitution * cp->relativeVelocity);
            m3real newImpulse = m3MaxF(cp->normalImpulse + impulse, 0.0f);
            impulse = newImpulse - cp->normalImpulse;
            cp->normalImpulse = newImpulse;
            ApplyImpulse(world, c, m3MulSV3(impulse, c->normal), cp->rA, cp->rB);
        }
    }
}
// Solve J * x = b for a general 3x3 via Cramer's rule. A singular
// Jacobian returns zero, which leaves omega unchanged (the safe step).
static m3Vec3 Solve3(const m3Mat3* J, m3Vec3 b)
{
    m3Vec3 cxy = m3Cross3(J->cy, J->cz);
    m3real det = m3Dot3(J->cx, cxy);
    if (det == 0.0f)
    {
        return (m3Vec3){0.0f, 0.0f, 0.0f};
    }
    m3real inv = 1.0f / det;
    m3Vec3 x;
    x.x = inv * m3Dot3(b, cxy);
    x.y = inv * m3Dot3(J->cx, m3Cross3(b, J->cz));
    x.z = inv * m3Dot3(J->cx, m3Cross3(J->cy, b));
    return x;
}

// Implicit gyroscopic torque (the reference's Newton-Raphson step on
// I*(w2 - w1) + h * cross(w2, I*w2) = 0, solved in body coordinates
// where the Jacobian is cheap). Long skinny bodies tumble correctly
// and never gain energy; the implicit form is unconditionally stable.
// Exactly isotropic tensors are gated out: cross(w, c*w) is zero in
// real arithmetic but not bit-zero in float, and spheres must keep
// their bit-identical trajectories. The gate compares are exact, so
// the branch itself is deterministic.
static m3Vec3 GyroscopicOmega(const m3World* world, int32_t body, m3Vec3 w, m3real h)
{
    const m3Mat3* inertia = &world->inertiaLocal[body];
    const m3real i00 = inertia->cx.x;
    const m3real i01 = inertia->cy.x;
    const m3real i02 = inertia->cz.x;
    const m3real i11 = inertia->cy.y;
    const m3real i12 = inertia->cz.y;
    const m3real i22 = inertia->cz.z;
    if (i01 == 0.0f && i02 == 0.0f && i12 == 0.0f && i00 == i11 && i11 == i22)
    {
        return w; // isotropic (or massless): the term vanishes
    }

    m3Quat q = world->transforms[body].q;
    m3Vec3 omega1 = m3InvRotateVec3(q, w);
    m3Vec3 omega2 = omega1;

    // One Newton iteration (the reference count): residual
    // b = I*(w2 - w1) + h * (w2 x I*w2), Jacobian
    // J = I + h * (skew(w2) * I - skew(I*w2)).
    const m3real w1 = omega2.x;
    const m3real w2 = omega2.y;
    const m3real w3 = omega2.z;
    const m3real Iw1 = i00 * w1 + i01 * w2 + i02 * w3;
    const m3real Iw2 = i01 * w1 + i11 * w2 + i12 * w3;
    const m3real Iw3 = i02 * w1 + i12 * w2 + i22 * w3;
    // omega2 - omega1 is zero on the first (only) iteration, so the
    // residual is just the gyroscopic term.
    m3Vec3 b = {
        h * (w2 * Iw3 - w3 * Iw2),
        h * (w3 * Iw1 - w1 * Iw3),
        h * (w1 * Iw2 - w2 * Iw1),
    };
    m3Mat3 J;
    J.cx = (m3Vec3){i00 + h * (w2 * i02 - w3 * i01), i01 + h * (w3 * i00 - w1 * i02 - Iw3),
                    i02 + h * (w1 * i01 - w2 * i00 + Iw2)};
    J.cy = (m3Vec3){i01 + h * (w2 * i12 - w3 * i11 + Iw3), i11 + h * (w3 * i01 - w1 * i12),
                    i12 + h * (w1 * i11 - w2 * i01 - Iw1)};
    J.cz = (m3Vec3){i02 + h * (w2 * i22 - w3 * i12 - Iw2), i12 + h * (w3 * i02 - w1 * i22 + Iw1),
                    i22 + h * (w1 * i12 - w2 * i02)};
    omega2 = m3Sub3(omega2, Solve3(&J, b));

    return m3RotateVec3(q, omega2);
}

static void StoreImpulses(m3World* world, m3ContactConstraint* constraints, int32_t count)
{
    for (int32_t i = 0; i < count; ++i)
    {
        m3ContactConstraint* c = constraints + i;
        m3Manifold* manifold = &world->manifolds[c->manifoldIndex];
        for (int32_t k = 0; k < c->pointCount; ++k)
        {
            manifold->points[k].normalImpulse = c->points[k].normalImpulse;
            manifold->points[k].tangentImpulse1 = c->points[k].tangentImpulse1;
            manifold->points[k].tangentImpulse2 = c->points[k].tangentImpulse2;
        }
    }
}
// ---------------------------------------------------------------
// Continuous collision (2b-8), modeled on the reference
// b3SolveContinuous: any fast dynamic body sweeps against statics;
// a bullet additionally sweeps against non-bullet dynamics with the
// TARGET'S true sweep in the TOI (dynamic versus dynamic moves both
// bodies). A hit pulls the fast body back to the impact pose;
// velocity stays, and next step's speculative contact resolves the
// impact. Bullet versus bullet is not resolved (the reference
// limitation, kept and documented). Bodies are processed in
// ascending index order; later bodies see earlier pull-backs, all
// deterministic.
// ---------------------------------------------------------------

typedef struct m3ContinuousContext
{
    m3World* world;
    const m3Pos3* com0;
    const m3Quat* rot0;
    int32_t fastBody;
    int32_t fastShape;
    m3Pos3 base; // re-center: TOI floats stay small
    m3Sweep fastSweep;
    m3real fraction;
} m3ContinuousContext;

static m3Sweep MakeRelativeSweep(const m3World* world, int32_t body, const m3Pos3* com0,
                                 const m3Quat* rot0, m3Pos3 base)
{
    m3Sweep sweep;
    sweep.localCenter = world->localCenters[body];
    sweep.c1 = (m3Vec3){(m3real)(com0[body].x - base.x), (m3real)(com0[body].y - base.y),
                        (m3real)(com0[body].z - base.z)};
    m3Vec3 rlc = m3RotateVec3(world->transforms[body].q, world->localCenters[body]);
    sweep.c2 = (m3Vec3){(m3real)(world->transforms[body].p.x + (double)rlc.x - base.x),
                        (m3real)(world->transforms[body].p.y + (double)rlc.y - base.y),
                        (m3real)(world->transforms[body].p.z + (double)rlc.z - base.z)};
    sweep.q1 = rot0[body];
    sweep.q2 = world->transforms[body].q;
    return sweep;
}

static bool ContinuousQueryCallback(int32_t shape, void* userContext)
{
    m3ContinuousContext* ctx = (m3ContinuousContext*)userContext;
    m3World* world = ctx->world;
    if (shape == ctx->fastShape)
    {
        return true;
    }
    int32_t body = world->shapeBody[shape];
    if (body == ctx->fastBody)
    {
        return true;
    }
    if (world->bulletFlags[body] != 0)
    {
        return true; // bullet versus bullet: skip (documented)
    }
    if (world->shapeType[shape] == (uint8_t)m3_meshShape)
    {
        // Mesh TOI (2b-9d): sweep the fast shape against every
        // candidate triangle. Each triangle is a three-point static
        // proxy in the mesh body's frame; the shared kernel does the
        // rest. Ascending triangle order, bounded candidates.
        const m3MeshData* mesh = &world->meshData[world->shapeMeshIndex[shape]];
        m3Sweep meshSweep = MakeRelativeSweep(world, body, ctx->com0, ctx->rot0, ctx->base);
        m3Vec3 scratchFast[2];
        m3DistanceProxy fastProxy = m3MakeShapeProxy(world, ctx->fastShape, scratchFast);

        // The swept bounds of the fast body in mesh-local space, a
        // conservative box from the relative sweep endpoints.
        const m3Transform* xfM = &world->transforms[body];
        m3Vec3 c1 =
            m3InvRotateVec3(xfM->q, (m3Vec3){(m3real)(ctx->com0[ctx->fastBody].x - xfM->p.x),
                                             (m3real)(ctx->com0[ctx->fastBody].y - xfM->p.y),
                                             (m3real)(ctx->com0[ctx->fastBody].z - xfM->p.z)});
        m3Vec3 rlc =
            m3RotateVec3(world->transforms[ctx->fastBody].q, world->localCenters[ctx->fastBody]);
        m3Vec3 c2 = m3InvRotateVec3(
            xfM->q,
            (m3Vec3){(m3real)(world->transforms[ctx->fastBody].p.x + (double)rlc.x - xfM->p.x),
                     (m3real)(world->transforms[ctx->fastBody].p.y + (double)rlc.y - xfM->p.y),
                     (m3real)(world->transforms[ctx->fastBody].p.z + (double)rlc.z - xfM->p.z)});
        m3real pad = world->maxExtents[ctx->fastBody] + M3_AABB_MARGIN;
        m3Vec3 lo = {m3MinF(c1.x, c2.x) - pad, m3MinF(c1.y, c2.y) - pad, m3MinF(c1.z, c2.z) - pad};
        m3Vec3 hi = {m3MaxF(c1.x, c2.x) + pad, m3MaxF(c1.y, c2.y) + pad, m3MaxF(c1.z, c2.z) + pad};

        int32_t budget = 64;
        for (int32_t t = 0; t < mesh->triangleCount && budget > 0; ++t)
        {
            m3Vec3 tv[3] = {mesh->vertices[mesh->indices[3 * t + 0]],
                            mesh->vertices[mesh->indices[3 * t + 1]],
                            mesh->vertices[mesh->indices[3 * t + 2]]};
            m3real tlx = m3MinF(tv[0].x, m3MinF(tv[1].x, tv[2].x));
            m3real thx = m3MaxF(tv[0].x, m3MaxF(tv[1].x, tv[2].x));
            m3real tly = m3MinF(tv[0].y, m3MinF(tv[1].y, tv[2].y));
            m3real thy = m3MaxF(tv[0].y, m3MaxF(tv[1].y, tv[2].y));
            m3real tlz = m3MinF(tv[0].z, m3MinF(tv[1].z, tv[2].z));
            m3real thz = m3MaxF(tv[0].z, m3MaxF(tv[1].z, tv[2].z));
            if (thx < lo.x || tlx > hi.x || thy < lo.y || tly > hi.y || thz < lo.z || tlz > hi.z)
            {
                continue;
            }
            budget -= 1;
            m3TOIInput input;
            input.proxyA.points = tv;
            input.proxyA.count = 3;
            input.proxyA.radius = 0.0f;
            input.proxyB = fastProxy;
            input.sweepA = meshSweep;
            input.sweepB = ctx->fastSweep;
            input.maxFraction = ctx->fraction;
            m3TOIOutput out = m3TimeOfImpact(&input);
            if (out.state == m3_toiStateHit && 0.0f < out.fraction && out.fraction < ctx->fraction)
            {
                ctx->fraction = out.fraction;
            }
        }
        return true;
    }
    if (world->types[body] != (uint8_t)m3_staticBody && world->bulletFlags[ctx->fastBody] == 0)
    {
        return true; // only bullets sweep against dynamics and kinematics
    }

    m3TOIInput input;
    m3Vec3 scratchA[2];
    m3Vec3 scratchB[2];
    input.proxyA = m3MakeShapeProxy(world, shape, scratchA);
    input.proxyB = m3MakeShapeProxy(world, ctx->fastShape, scratchB);
    input.sweepA = MakeRelativeSweep(world, body, ctx->com0, ctx->rot0, ctx->base);
    input.sweepB = ctx->fastSweep;
    input.maxFraction = ctx->fraction;
    m3TOIOutput out = m3TimeOfImpact(&input);
    if (out.state == m3_toiStateHit && 0.0f < out.fraction && out.fraction < ctx->fraction)
    {
        ctx->fraction = out.fraction;
    }
    return true;
}

// Plane targets are not in the tree: a bounded conservative advance
// against the analytic plane distance (support of the fast proxy at
// the swept pose, minus its radius).
static void ContinuousVersusPlane(const m3World* world, m3ContinuousContext* ctx,
                                  int32_t planeShape)
{
    m3Vec3 n = world->shapeGeom[planeShape].v;
    m3real offset =
        world->shapeGeom[planeShape].s -
        (m3real)((double)n.x * ctx->base.x + (double)n.y * ctx->base.y + (double)n.z * ctx->base.z);
    m3Vec3 scratch[2];
    m3DistanceProxy proxy = m3MakeShapeProxy(world, ctx->fastShape, scratch);

    const m3real linearSlop = 0.005f;
    m3real target = m3MaxF(linearSlop, proxy.radius - linearSlop);
    m3real tolerance = 0.25f * linearSlop;

    // Conservative rate: linear travel plus the rotation arc.
    m3Vec3 travel = m3Sub3(ctx->fastSweep.c2, ctx->fastSweep.c1);
    m3Quat dq = m3MulQuat(ctx->fastSweep.q2, (m3Quat){-ctx->fastSweep.q1.x, -ctx->fastSweep.q1.y,
                                                      -ctx->fastSweep.q1.z, ctx->fastSweep.q1.w});
    m3real arc =
        2.0f * sqrtf(dq.x * dq.x + dq.y * dq.y + dq.z * dq.z) * world->maxExtents[ctx->fastBody];
    m3real rate = sqrtf(m3Dot3(travel, travel)) + arc;
    if (!(rate > 0.0f))
    {
        return;
    }

    m3real t = 0.0f;
    for (int32_t iter = 0; iter < 25; ++iter)
    {
        m3Transform xf = m3GetSweepTransform(&ctx->fastSweep, t);
        m3real minD = 3.4e38f;
        for (int32_t k = 0; k < proxy.count; ++k)
        {
            m3Vec3 r = m3RotateVec3(xf.q, proxy.points[k]);
            m3real d = n.x * ((m3real)xf.p.x + r.x) + n.y * ((m3real)xf.p.y + r.y) +
                       n.z * ((m3real)xf.p.z + r.z);
            minD = m3MinF(minD, d);
        }
        m3real sep = minD - offset;
        if (sep <= 0.0f)
        {
            return; // started behind or overlapped: discrete owns it
        }
        if (sep <= target + tolerance)
        {
            if (t < ctx->fraction)
            {
                ctx->fraction = t;
            }
            return;
        }
        t += (sep - target) / rate;
        if (t >= ctx->fraction)
        {
            return; // no earlier hit than the current best
        }
    }
}

static void SolveContinuousPhase(m3World* world, const m3Pos3* com0, const m3Quat* rot0)
{
    int32_t maxBody = world->bodyPool.maxIndex;
    int32_t maxShape = world->shapePool.maxIndex;
    for (int32_t i = 0; i < maxBody; ++i)
    {
        if (world->bodyPool.alive[i] == 0 || world->types[i] != (uint8_t)m3_dynamicBody)
        {
            continue;
        }
        // Fast test: displacement plus rotation arc versus the
        // thinnest extent (the reference safety factor of one half).
        m3Vec3 rlc = m3RotateVec3(world->transforms[i].q, world->localCenters[i]);
        double cx = world->transforms[i].p.x + (double)rlc.x;
        double cy = world->transforms[i].p.y + (double)rlc.y;
        double cz = world->transforms[i].p.z + (double)rlc.z;
        m3Vec3 dc = {(m3real)(cx - com0[i].x), (m3real)(cy - com0[i].y), (m3real)(cz - com0[i].z)};
        m3Quat q0 = rot0[i];
        m3Quat dq = m3MulQuat(world->transforms[i].q, (m3Quat){-q0.x, -q0.y, -q0.z, q0.w});
        m3real arc = 2.0f * sqrtf(dq.x * dq.x + dq.y * dq.y + dq.z * dq.z) * world->maxExtents[i];
        m3real maxMotion = sqrtf(m3Dot3(dc, dc)) + arc;
        if (!(maxMotion > 0.5f * world->minExtents[i]))
        {
            continue;
        }

        m3ContinuousContext ctx;
        ctx.world = world;
        ctx.com0 = com0;
        ctx.rot0 = rot0;
        ctx.fastBody = i;
        ctx.base = com0[i];
        ctx.fraction = 1.0f;
        ctx.fastSweep = MakeRelativeSweep(world, i, com0, rot0, ctx.base);

        for (int32_t s = world->bodyShapeHead[i]; s != -1; s = world->shapeNext[s])
        {
            ctx.fastShape = s;
            // Swept candidate box: both COM endpoints padded by the
            // body's max extent (a coarse superset; the TOI filters).
            double pad = (double)(world->maxExtents[i] + M3_AABB_MARGIN);
            double lo[3];
            double hi[3];
            lo[0] = (com0[i].x < cx ? com0[i].x : cx) - pad;
            lo[1] = (com0[i].y < cy ? com0[i].y : cy) - pad;
            lo[2] = (com0[i].z < cz ? com0[i].z : cz) - pad;
            hi[0] = (com0[i].x > cx ? com0[i].x : cx) + pad;
            hi[1] = (com0[i].y > cy ? com0[i].y : cy) + pad;
            hi[2] = (com0[i].z > cz ? com0[i].z : cz) + pad;
            m3TreeQuery(&world->tree, lo, hi, ContinuousQueryCallback, &ctx);

            // Planes take the dedicated pass (never in the tree).
            for (int32_t p = 0; p < maxShape; ++p)
            {
                if (world->shapePool.alive[p] != 0 && world->shapeType[p] == (uint8_t)m3_planeShape)
                {
                    ContinuousVersusPlane(world, &ctx, p);
                }
            }
        }

        if (ctx.fraction < 1.0f)
        {
            // Pull the body back to the impact pose. Velocity stays:
            // next step's speculative contact turns it into impulse.
            m3Transform xf = m3GetSweepTransform(&ctx.fastSweep, ctx.fraction);
            world->transforms[i].q = xf.q;
            world->transforms[i].p.x = ctx.base.x + xf.p.x;
            world->transforms[i].p.y = ctx.base.y + xf.p.y;
            world->transforms[i].p.z = ctx.base.z + xf.p.z;
        }
    }
}

// ---------------------------------------------------------------
// Islands and sleep (2b-10, the Maul2D recipe): union-find over the
// touching dynamic pairs in canonical order. The wake pass runs
// right after contacts are built (a sleeping body touched by an
// awake one must join THIS step's solve); the sleep decision runs at
// the end of the step with the same island labels. Sleeping bodies
// are bit-frozen: velocities zeroed once, integration and solving
// skip them, and the hash stands still.
// ---------------------------------------------------------------

static int32_t IslandFind(int32_t* parent, int32_t i)
{
    while (parent[i] != i)
    {
        parent[i] = parent[parent[i]]; // halving, deterministic
        i = parent[i];
    }
    return i;
}

static void IslandUnion(int32_t* parent, int32_t a, int32_t b)
{
    int32_t ra = IslandFind(parent, a);
    int32_t rb = IslandFind(parent, b);
    if (ra != rb)
    {
        // Lower root wins: canonical labels independent of order.
        if (ra < rb)
        {
            parent[rb] = ra;
        }
        else
        {
            parent[ra] = rb;
        }
    }
}

// Build islands from the current touching pairs and wake every
// island that contains an awake member or a moving kinematic
// neighbor. Returns the parent array (scratch-allocated).
static int32_t* IslandWakePass(m3World* world)
{
    int32_t maxBody = world->bodyPool.maxIndex;
    int32_t* parent = (int32_t*)m3StackAlloc(&world->scratch,
                                             maxBody > 0 ? maxBody * (int32_t)sizeof(int32_t) : 4);
    uint8_t* forced = (uint8_t*)m3StackAlloc(&world->scratch, maxBody > 0 ? maxBody : 1);
    if (parent == NULL || forced == NULL)
    {
        return NULL;
    }
    for (int32_t i = 0; i < maxBody; ++i)
    {
        parent[i] = i;
        forced[i] = 0;
    }
    for (int32_t i = 0; i < world->pairCount; ++i)
    {
        if (world->manifolds[i].pointCount == 0)
        {
            continue;
        }
        uint64_t key = world->pairKeys[i];
        int32_t bodyA = world->shapeBody[(int32_t)(key >> 32)];
        int32_t bodyB = world->shapeBody[(int32_t)(key & 0xFFFFFFFFu)];
        int dynA = world->types[bodyA] == (uint8_t)m3_dynamicBody;
        int dynB = world->types[bodyB] == (uint8_t)m3_dynamicBody;
        if (dynA && dynB)
        {
            IslandUnion(parent, bodyA, bodyB);
        }
        else if (dynA || dynB)
        {
            // A moving kinematic neighbor forces its contact awake.
            int32_t kin = dynA ? bodyB : bodyA;
            int32_t dyn = dynA ? bodyA : bodyB;
            if (world->types[kin] == (uint8_t)m3_kinematicBody)
            {
                m3Vec3 v = world->linearVelocities[kin];
                m3Vec3 w = world->angularVelocities[kin];
                if (m3Dot3(v, v) > 0.0f || m3Dot3(w, w) > 0.0f)
                {
                    forced[dyn] = 1;
                }
            }
        }
    }
    // Aggregate: does any island member demand wakefulness?
    for (int32_t i = 0; i < maxBody; ++i)
    {
        if (world->bodyPool.alive[i] == 0 || world->types[i] != (uint8_t)m3_dynamicBody)
        {
            continue;
        }
        if (world->awake[i] != 0 || forced[i] != 0)
        {
            forced[IslandFind(parent, i)] = 1;
        }
    }
    for (int32_t i = 0; i < maxBody; ++i)
    {
        if (world->bodyPool.alive[i] == 0 || world->types[i] != (uint8_t)m3_dynamicBody)
        {
            continue;
        }
        if (world->awake[i] == 0 && forced[IslandFind(parent, i)] != 0)
        {
            world->awake[i] = 1; // woken by the island: timers restart
            world->sleepTimes[i] = 0.0f;
        }
    }
    return parent;
}

// End-of-step sleep decision: timers advance for slow awake bodies,
// and an island sleeps only when EVERY member is ready.
static void IslandSleepPass(m3World* world, int32_t* parent, const m3Pos3* com0, const m3Quat* rot0,
                            float dt)
{
    int32_t maxBody = world->bodyPool.maxIndex;
    m3real invDt = dt > 0.0f ? 1.0f / dt : 0.0f;
    for (int32_t i = 0; i < maxBody; ++i)
    {
        if (world->bodyPool.alive[i] == 0 || world->types[i] != (uint8_t)m3_dynamicBody ||
            world->awake[i] == 0)
        {
            continue;
        }
        m3Vec3 v = world->linearVelocities[i];
        m3Vec3 w = world->angularVelocities[i];
        m3real velocity = sqrtf(m3Dot3(v, v)) + sqrtf(m3Dot3(w, w)) * world->maxExtents[i];
        // Position correction counts too (the reference lesson: bias
        // pushes move bodies that report zero velocity).
        m3Vec3 rlc = m3RotateVec3(world->transforms[i].q, world->localCenters[i]);
        m3Vec3 dc = {(m3real)(world->transforms[i].p.x + (double)rlc.x - com0[i].x),
                     (m3real)(world->transforms[i].p.y + (double)rlc.y - com0[i].y),
                     (m3real)(world->transforms[i].p.z + (double)rlc.z - com0[i].z)};
        m3Quat q0 = rot0[i];
        m3Quat dq = m3MulQuat(world->transforms[i].q, (m3Quat){-q0.x, -q0.y, -q0.z, q0.w});
        m3real motion = sqrtf(m3Dot3(dc, dc)) + 2.0f *
                                                    sqrtf(dq.x * dq.x + dq.y * dq.y + dq.z * dq.z) *
                                                    world->maxExtents[i];
        m3real sleepVelocity = m3MaxF(velocity, 0.5f * invDt * motion);
        if (sleepVelocity < 0.05f)
        {
            world->sleepTimes[i] += dt;
        }
        else
        {
            world->sleepTimes[i] = 0.0f;
        }
    }
    // Island readiness: every member past the time-to-sleep bar.
    uint8_t* ready = (uint8_t*)m3StackAlloc(&world->scratch, maxBody > 0 ? maxBody : 1);
    if (ready == NULL)
    {
        return;
    }
    memset(ready, 1, (size_t)(maxBody > 0 ? maxBody : 1));
    for (int32_t i = 0; i < maxBody; ++i)
    {
        if (world->bodyPool.alive[i] == 0 || world->types[i] != (uint8_t)m3_dynamicBody ||
            world->awake[i] == 0)
        {
            continue;
        }
        if (world->sleepTimes[i] < 0.5f)
        {
            ready[IslandFind(parent, i)] = 0;
        }
    }
    for (int32_t i = 0; i < maxBody; ++i)
    {
        if (world->bodyPool.alive[i] == 0 || world->types[i] != (uint8_t)m3_dynamicBody ||
            world->awake[i] == 0)
        {
            continue;
        }
        if (ready[IslandFind(parent, i)] != 0)
        {
            // The whole island crosses together: freeze bit-solid.
            world->awake[i] = 0;
            world->linearVelocities[i] = (m3Vec3){0.0f, 0.0f, 0.0f};
            world->angularVelocities[i] = (m3Vec3){0.0f, 0.0f, 0.0f};
        }
    }
}

void m3StepInternal(m3World* world, float dt, int32_t substeps)
{
    m3StackReset(&world->scratch);

    // Stash the previous pairs and manifolds BEFORE the scan
    // overwrites them (the warm-start carry reads this).
    int32_t oldCount = world->pairCount;
    uint64_t* oldKeys = NULL;
    m3Manifold* oldManifolds = NULL;
    if (oldCount > 0)
    {
        oldKeys = (uint64_t*)m3StackAlloc(&world->scratch, oldCount * (int32_t)sizeof(uint64_t));
        oldManifolds =
            (m3Manifold*)m3StackAlloc(&world->scratch, oldCount * (int32_t)sizeof(m3Manifold));
        if (oldKeys == NULL || oldManifolds == NULL)
        {
            M3_ASSERT(false);
            return; // loud in debug; the world visibly stalls, never corrupts
        }
        memcpy(oldKeys, world->pairKeys, (size_t)oldCount * sizeof(uint64_t));
        memcpy(oldManifolds, world->manifolds, (size_t)oldCount * sizeof(m3Manifold));
    }

    // Begin-of-step COM and rotation for every body: the sweeps the
    // continuous pass (2b-8) needs. Captured before anything moves.
    int32_t sweepMax = world->bodyPool.maxIndex;
    m3Pos3* com0 =
        (m3Pos3*)m3StackAlloc(&world->scratch, sweepMax > 0 ? sweepMax * (int32_t)sizeof(m3Pos3)
                                                            : (int32_t)sizeof(m3Pos3));
    m3Quat* rot0 =
        (m3Quat*)m3StackAlloc(&world->scratch, sweepMax > 0 ? sweepMax * (int32_t)sizeof(m3Quat)
                                                            : (int32_t)sizeof(m3Quat));
    if (com0 == NULL || rot0 == NULL)
    {
        M3_ASSERT(false);
        return;
    }
    for (int32_t i = 0; i < sweepMax; ++i)
    {
        if (world->bodyPool.alive[i] == 0)
        {
            continue;
        }
        m3Vec3 rlc = m3RotateVec3(world->transforms[i].q, world->localCenters[i]);
        com0[i].x = world->transforms[i].p.x + (double)rlc.x;
        com0[i].y = world->transforms[i].p.y + (double)rlc.y;
        com0[i].z = world->transforms[i].p.z + (double)rlc.z;
        rot0[i] = world->transforms[i].q;
    }

    if (m3UpdatePairs(world) != m3_success ||
        m3UpdateContacts(world, oldKeys, oldManifolds, oldCount) != m3_success)
    {
        M3_ASSERT(false);
        return;
    }

    // Contact events: a canonical merge walk of the old and new pair
    // lists (both sorted). Serial and after the parallel narrowphase
    // on purpose: appends must happen in pair order, bit-stably.
    world->beginEventCount = 0;
    world->endEventCount = 0;
    {
        int32_t iNew = 0;
        int32_t iOld = 0;
        while (iNew < world->pairCount || iOld < oldCount)
        {
            uint64_t keyNew = iNew < world->pairCount ? world->pairKeys[iNew] : UINT64_MAX;
            uint64_t keyOld = iOld < oldCount ? oldKeys[iOld] : UINT64_MAX;
            int touchNew = 0;
            int touchOld = 0;
            uint64_t key;
            if (keyNew < keyOld)
            {
                key = keyNew;
                touchNew = world->manifolds[iNew].pointCount > 0;
                iNew += 1;
            }
            else if (keyOld < keyNew)
            {
                key = keyOld;
                touchOld = oldManifolds[iOld].pointCount > 0;
                iOld += 1;
            }
            else
            {
                key = keyNew;
                touchNew = world->manifolds[iNew].pointCount > 0;
                touchOld = oldManifolds[iOld].pointCount > 0;
                iNew += 1;
                iOld += 1;
            }
            if (touchNew == touchOld)
            {
                continue;
            }
            int32_t sA = (int32_t)(key >> 32);
            int32_t sB = (int32_t)(key & 0xFFFFFFFFu);
            // A vanished pair whose shape died emits nothing: the id
            // would be stale (documented on the API).
            if (world->shapePool.alive[sA] == 0 || world->shapePool.alive[sB] == 0)
            {
                continue;
            }
            m3ContactEvent event;
            event.shapeA =
                (m3ShapeId){sA + 1, world->worldIndex0, world->shapePool.generations[sA]};
            event.shapeB =
                (m3ShapeId){sB + 1, world->worldIndex0, world->shapePool.generations[sB]};
            if (touchNew && world->beginEventCount < world->pairCapacity)
            {
                world->beginEvents[world->beginEventCount++] = event;
            }
            else if (touchOld && world->endEventCount < world->pairCapacity)
            {
                world->endEvents[world->endEventCount++] = event;
            }
        }
    }

    // Islands and wake propagation BEFORE the solve: a sleeping body
    // touched by an awake one participates in this very step.
    int32_t* islandParent = IslandWakePass(world);
    if (islandParent == NULL)
    {
        M3_ASSERT(false);
        return;
    }

    // Solver scratch: constraints plus per-body delta accumulators.
    int32_t maxBody = world->bodyPool.maxIndex;
    m3ContactConstraint* constraints = (m3ContactConstraint*)m3StackAlloc(
        &world->scratch, world->pairCount > 0
                             ? world->pairCount * (int32_t)sizeof(m3ContactConstraint)
                             : (int32_t)sizeof(m3ContactConstraint));
    m3Vec3* deltaPos = (m3Vec3*)m3StackAlloc(
        &world->scratch, maxBody > 0 ? maxBody * (int32_t)sizeof(m3Vec3) : (int32_t)sizeof(m3Vec3));
    m3Quat* deltaRot = (m3Quat*)m3StackAlloc(
        &world->scratch, maxBody > 0 ? maxBody * (int32_t)sizeof(m3Quat) : (int32_t)sizeof(m3Quat));
    if (constraints == NULL || deltaPos == NULL || deltaRot == NULL)
    {
        M3_ASSERT(false);
        return;
    }
    for (int32_t i = 0; i < maxBody; ++i)
    {
        deltaPos[i] = (m3Vec3){0.0f, 0.0f, 0.0f};
        deltaRot[i] = m3MakeIdentityQuat();
    }

    m3real h = dt / (m3real)substeps;
    m3real invH = h > 0.0f ? 1.0f / h : 0.0f;
    int32_t constraintCount = PrepareContacts(world, constraints, h);

    for (int32_t sub = 0; sub < substeps; ++sub)
    {
        // Integrate velocities (fixed body order): gravity, damping.
        for (int32_t i = 0; i < maxBody; ++i)
        {
            if (world->bodyPool.alive[i] == 0 || world->types[i] != (uint8_t)m3_dynamicBody ||
                world->awake[i] == 0)
            {
                continue;
            }
            m3Vec3 v = world->linearVelocities[i];
            m3Vec3 w = world->angularVelocities[i];
            v = m3Add3(v, m3MulSV3(h * world->gravityScales[i], world->gravity));
            v = m3MulSV3(1.0f / (1.0f + h * world->linearDamping[i]), v);
            w = m3MulSV3(1.0f / (1.0f + h * world->angularDamping[i]), w);
            w = GyroscopicOmega(world, i, w, h);
            world->linearVelocities[i] = v;
            world->angularVelocities[i] = w;
        }

        WarmStart(world, constraints, constraintCount);
        SolveContacts(world, constraints, constraintCount, deltaPos, deltaRot, invH, true);

        // Integrate positions and accumulate the substep deltas the
        // separation tracking reads.
        for (int32_t i = 0; i < maxBody; ++i)
        {
            uint8_t type = world->types[i];
            int moves = type == (uint8_t)m3_kinematicBody ||
                        (type == (uint8_t)m3_dynamicBody && world->awake[i] != 0);
            if (world->bodyPool.alive[i] == 0 || !moves)
            {
                continue;
            }
            m3Vec3 v = world->linearVelocities[i];
            m3Vec3 w = world->angularVelocities[i];
            // Rigid bodies rotate about the center of mass: advance
            // the COM, spin, then place the origin back. A centered
            // body (lc zero) reduces to the plain origin update.
            m3Vec3 lc = world->localCenters[i];
            m3Vec3 rlcOld = m3RotateVec3(world->transforms[i].q, lc);
            double cx = world->transforms[i].p.x + (double)rlcOld.x + (double)(h * v.x);
            double cy = world->transforms[i].p.y + (double)rlcOld.y + (double)(h * v.y);
            double cz = world->transforms[i].p.z + (double)rlcOld.z + (double)(h * v.z);
            m3Vec3 dw = m3MulSV3(h, w);
            world->transforms[i].q = m3IntegrateRotation(world->transforms[i].q, dw);
            m3Vec3 rlcNew = m3RotateVec3(world->transforms[i].q, lc);
            world->transforms[i].p.x = cx - (double)rlcNew.x;
            world->transforms[i].p.y = cy - (double)rlcNew.y;
            world->transforms[i].p.z = cz - (double)rlcNew.z;
            deltaPos[i] = m3Add3(deltaPos[i], m3MulSV3(h, v));
            deltaRot[i] = m3IntegrateRotation(deltaRot[i], dw);
        }

        // Relax: remove the bias energy (reference schedule).
        SolveContacts(world, constraints, constraintCount, deltaPos, deltaRot, invH, false);
    }

    Restitution(world, constraints, constraintCount);
    StoreImpulses(world, constraints, constraintCount);

    SolveContinuousPhase(world, com0, rot0);
    IslandSleepPass(world, islandParent, com0, rot0, dt);

    world->stepCount += 1;
}

void m3World_Step(m3WorldId worldId, float dt, int32_t substeps)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || !(dt > 0.0f) || substeps < 1)
    {
        M3_ASSERT(false);
        return;
    }
    if (world->journalActive != 0)
    {
        struct
        {
            float dt;
            int32_t substeps;
        } record;
        memset(&record, 0, sizeof(record));
        record.dt = dt;
        record.substeps = substeps;
        m3JournalRecord(world, m3_opStep, &record, (int32_t)sizeof(record));
    }
    m3StepInternal(world, dt, substeps);
}
