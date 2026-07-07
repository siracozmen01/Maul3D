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

    if (m3UpdatePairs(world) != m3_success ||
        m3UpdateContacts(world, oldKeys, oldManifolds, oldCount) != m3_success)
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
            if (world->bodyPool.alive[i] == 0 || world->types[i] != (uint8_t)m3_dynamicBody)
            {
                continue;
            }
            m3Vec3 v = world->linearVelocities[i];
            m3Vec3 w = world->angularVelocities[i];
            v = m3Add3(v, m3MulSV3(h * world->gravityScales[i], world->gravity));
            v = m3MulSV3(1.0f / (1.0f + h * world->linearDamping[i]), v);
            w = m3MulSV3(1.0f / (1.0f + h * world->angularDamping[i]), w);
            world->linearVelocities[i] = v;
            world->angularVelocities[i] = w;
        }

        WarmStart(world, constraints, constraintCount);
        SolveContacts(world, constraints, constraintCount, deltaPos, deltaRot, invH, true);

        // Integrate positions and accumulate the substep deltas the
        // separation tracking reads.
        for (int32_t i = 0; i < maxBody; ++i)
        {
            if (world->bodyPool.alive[i] == 0 || world->types[i] != (uint8_t)m3_dynamicBody)
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
