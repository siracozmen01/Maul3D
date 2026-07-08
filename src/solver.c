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
static void WarmStartOne(m3World* world, m3ContactConstraint* c)
{
    for (int32_t k = 0; k < c->pointCount; ++k)
    {
        m3ConstraintPoint* cp = &c->points[k];
        m3Vec3 impulse = m3Add3(
            m3MulSV3(cp->normalImpulse, c->normal),
            m3Add3(m3MulSV3(cp->tangentImpulse1, c->t1), m3MulSV3(cp->tangentImpulse2, c->t2)));
        ApplyImpulse(world, c, impulse, cp->rA, cp->rB);
    }
}
static void SolveOneContact(m3World* world, m3ContactConstraint* c, const m3Vec3* deltaPos,
                            const m3Quat* deltaRot, m3real invH, bool useBias)
{
    {

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

// ---------------------------------------------------------------
// Joints (2c-2): the spherical point constraint in soft-step form,
// the reference schedule (joints warm start and solve BEFORE the
// contacts inside every substep pass). Few joints, serial in
// canonical index order; they join the color palette when counts
// ever justify it (noted, not needed for correctness).
// ---------------------------------------------------------------

static m3Vec3 Solve3(const m3Mat3* J, m3Vec3 b); // defined with the gyroscopic block

typedef struct m3JointConstraint
{
    int32_t joint; // world joint slot (impulses written back at store)
    uint8_t type;
    uint8_t flags; // bit0 limit, bit1 motor
    int32_t bodyA;
    int32_t bodyB;
    m3Vec3 rA; // COM-relative anchors, world-rotated at prepare
    m3Vec3 rB;
    m3Vec3 deltaCenter; // cB - cA at prepare
    m3real invMassA;
    m3real invMassB;
    m3Mat3 invIA;
    m3Mat3 invIB;
    m3Softness softness;
    m3Vec3 impulse; // linear point impulse
    // Revolute state (prepared): world joint frames, hinge axis,
    // axial mass, cached perp axes for warm start, extra impulses.
    m3Quat frameQA;
    m3Quat frameQB;
    m3Vec3 rotationAxis;
    m3real axialMass;
    m3Vec3 perpAxisX;
    m3Vec3 perpAxisY;
    m3real perpImpulseX;
    m3real perpImpulseY;
    m3real motorImpulse;
    m3real lowerImpulse;
    m3real upperImpulse;
    m3real motorSpeed;
    m3real maxMotorEffort;
    m3real lowerLimit;
    m3real upperLimit;
    m3Vec3 angularImpulse; // prismatic rotation lock (3 DOF)
} m3JointConstraint;

// Rotation vector of a relative quaternion (robust: exact angle via
// atan2, small angles fall back to the linear form).
static m3Vec3 QuatToRotationVec(m3Quat relQ)
{
    if (relQ.w < 0.0f)
    {
        relQ = (m3Quat){-relQ.x, -relQ.y, -relQ.z, -relQ.w};
    }
    m3Vec3 v = {relQ.x, relQ.y, relQ.z};
    m3real len = sqrtf(m3Dot3(v, v));
    if (len < 1.0e-9f)
    {
        return m3MulSV3(2.0f, v);
    }
    m3real angle = 2.0f * m3Atan2(len, relQ.w);
    return m3MulSV3(angle / len, v);
}

// Half-quaternion rotation of a frame axis, the reference form for
// the collinearity Jacobian columns.
static m3Vec3 PerpColumn(m3Quat qA, m3Quat relQ, m3Vec3 axis)
{
    m3Vec3 rv = {relQ.x, relQ.y, relQ.z};
    m3Vec3 inner = m3Add3(m3MulSV3(relQ.w, axis), m3Cross3(rv, axis));
    return m3MulSV3(0.5f, m3RotateVec3(qA, inner));
}

static int32_t PrepareJoints(m3World* world, m3JointConstraint* joints, m3real h)
{
    int32_t count = 0;
    int32_t maxJoint = world->jointPool.maxIndex;
    for (int32_t j = 0; j < maxJoint; ++j)
    {
        if (world->jointPool.alive[j] == 0)
        {
            continue;
        }
        int32_t bodyA = world->jointBodyA[j];
        int32_t bodyB = world->jointBodyB[j];
        int awakeDynA = world->types[bodyA] == (uint8_t)m3_dynamicBody && world->awake[bodyA];
        int awakeDynB = world->types[bodyB] == (uint8_t)m3_dynamicBody && world->awake[bodyB];
        if (!awakeDynA && !awakeDynB)
        {
            continue; // both sides frozen or immovable
        }
        m3JointConstraint* c = &joints[count];
        count += 1;
        memset(c, 0, sizeof(*c)); // every field defined for every type:
                                  // the store writes them all back
        c->joint = j;
        c->bodyA = bodyA;
        c->bodyB = bodyB;
        const m3Transform* xfA = &world->transforms[bodyA];
        const m3Transform* xfB = &world->transforms[bodyB];
        c->rA = m3RotateVec3(xfA->q, m3Sub3(world->jointLocalA[j], world->localCenters[bodyA]));
        c->rB = m3RotateVec3(xfB->q, m3Sub3(world->jointLocalB[j], world->localCenters[bodyB]));
        m3Vec3 rlcA = m3RotateVec3(xfA->q, world->localCenters[bodyA]);
        m3Vec3 rlcB = m3RotateVec3(xfB->q, world->localCenters[bodyB]);
        c->deltaCenter = (m3Vec3){(m3real)(xfB->p.x + (double)rlcB.x - xfA->p.x - (double)rlcA.x),
                                  (m3real)(xfB->p.y + (double)rlcB.y - xfA->p.y - (double)rlcA.y),
                                  (m3real)(xfB->p.z + (double)rlcB.z - xfA->p.z - (double)rlcA.z)};
        c->invMassA = world->types[bodyA] == (uint8_t)m3_dynamicBody ? world->invMass[bodyA] : 0.0f;
        c->invMassB = world->types[bodyB] == (uint8_t)m3_dynamicBody ? world->invMass[bodyB] : 0.0f;
        c->invIA = WorldInvInertia(world, bodyA);
        c->invIB = WorldInvInertia(world, bodyB);
        c->softness = MakeSoft(60.0f, 2.0f, h); // the reference joint stiffness
        c->impulse = world->jointImpulse[j];
        c->type = world->jointType[j];
        c->flags = world->jointFlags[j];
        if (c->type == (uint8_t)m3_revoluteJoint || c->type == (uint8_t)m3_prismaticJoint)
        {
            c->frameQA = m3MulQuat(xfA->q, world->jointFrameQA[j]);
            c->frameQB = m3MulQuat(xfB->q, world->jointFrameQB[j]);
            m3Vec3 axis = m3RotateVec3(c->frameQA, (m3Vec3){0.0f, 0.0f, 1.0f});
            c->rotationAxis = axis;
            m3Vec3 sum = m3Add3(m3MulMV3(c->invIA, axis), m3MulMV3(c->invIB, axis));
            m3real k = m3Dot3(axis, sum);
            c->axialMass = k > 0.0f ? 1.0f / k : 0.0f;
            m3Quat conjA = {-c->frameQA.x, -c->frameQA.y, -c->frameQA.z, c->frameQA.w};
            m3Quat relQ = m3MulQuat(conjA, c->frameQB);
            c->perpAxisX = PerpColumn(c->frameQA, relQ, (m3Vec3){1.0f, 0.0f, 0.0f});
            c->perpAxisY = PerpColumn(c->frameQA, relQ, (m3Vec3){0.0f, 1.0f, 0.0f});
            c->perpImpulseX = world->jointPerpImpulse[j].x;
            c->perpImpulseY = world->jointPerpImpulse[j].y;
            c->motorImpulse = world->jointPerpImpulse[j].z;
            c->lowerImpulse = world->jointLimitImpulse[j].x;
            c->upperImpulse = world->jointLimitImpulse[j].y;
            c->motorSpeed = world->jointMotor[j].x;
            c->maxMotorEffort = world->jointMotor[j].y;
            c->lowerLimit = world->jointLimits[j].x;
            c->upperLimit = world->jointLimits[j].y;
            c->angularImpulse = world->jointAngularImpulse[j];
        }
    }
    return count;
}

static void WarmStartJoints(m3World* world, m3JointConstraint* joints, int32_t count,
                            const m3Quat* deltaRot)
{
    for (int32_t i = 0; i < count; ++i)
    {
        m3JointConstraint* c = &joints[i];
        m3Vec3 rA = m3RotateVec3(deltaRot[c->bodyA], c->rA);
        m3Vec3 rB = m3RotateVec3(deltaRot[c->bodyB], c->rB);
        m3Vec3 angularImpulse = {0.0f, 0.0f, 0.0f};
        m3Vec3 linearExtra = {0.0f, 0.0f, 0.0f};
        if (c->type == (uint8_t)m3_revoluteJoint)
        {
            m3real axial = c->motorImpulse + c->lowerImpulse - c->upperImpulse;
            angularImpulse = m3Add3(m3MulSV3(c->perpImpulseX, c->perpAxisX),
                                    m3MulSV3(c->perpImpulseY, c->perpAxisY));
            angularImpulse = m3Add3(angularImpulse, m3MulSV3(axial, c->rotationAxis));
        }
        else if (c->type == (uint8_t)m3_prismaticJoint)
        {
            // Linear impulses along the prepared axis and perps; the
            // angular arms differ per body and re-derive in the solve,
            // so the warm start uses the prepared arms (the reference
            // does the same through its cached sA/sB).
            m3real axial = c->motorImpulse + c->lowerImpulse - c->upperImpulse;
            linearExtra = m3MulSV3(axial, c->rotationAxis);
            linearExtra = m3Add3(linearExtra, m3MulSV3(c->perpImpulseX, c->perpAxisX));
            linearExtra = m3Add3(linearExtra, m3MulSV3(c->perpImpulseY, c->perpAxisY));
            angularImpulse = c->angularImpulse;
        }
        m3Vec3 totalLinear = m3Add3(c->impulse, linearExtra);
        world->linearVelocities[c->bodyA] =
            m3Sub3(world->linearVelocities[c->bodyA], m3MulSV3(c->invMassA, totalLinear));
        world->angularVelocities[c->bodyA] =
            m3Sub3(world->angularVelocities[c->bodyA],
                   m3MulMV3(c->invIA, m3Add3(m3Cross3(rA, totalLinear), angularImpulse)));
        world->linearVelocities[c->bodyB] =
            m3Add3(world->linearVelocities[c->bodyB], m3MulSV3(c->invMassB, totalLinear));
        world->angularVelocities[c->bodyB] =
            m3Add3(world->angularVelocities[c->bodyB],
                   m3MulMV3(c->invIB, m3Add3(m3Cross3(rB, totalLinear), angularImpulse)));
    }
}

static void SolveJoints(m3World* world, m3JointConstraint* joints, int32_t count,
                        const m3Vec3* deltaPos, const m3Quat* deltaRot, m3real hSub, m3real invHSub,
                        int useBias)
{
    for (int32_t i = 0; i < count; ++i)
    {
        m3JointConstraint* c = &joints[i];
        m3Vec3 rA = m3RotateVec3(deltaRot[c->bodyA], c->rA);
        m3Vec3 rB = m3RotateVec3(deltaRot[c->bodyB], c->rB);

        m3Vec3 vA = world->linearVelocities[c->bodyA];
        m3Vec3 wA = world->angularVelocities[c->bodyA];
        m3Vec3 vB = world->linearVelocities[c->bodyB];
        m3Vec3 wB = world->angularVelocities[c->bodyB];

        if (c->type == (uint8_t)m3_revoluteJoint)
        {
            // Substep frames and the relative rotation, sign-guarded
            // so the angle lives in [-pi, pi] (the reference rule).
            m3Quat quatA = m3MulQuat(deltaRot[c->bodyA], c->frameQA);
            m3Quat quatB = m3MulQuat(deltaRot[c->bodyB], c->frameQB);
            if (quatA.x * quatB.x + quatA.y * quatB.y + quatA.z * quatB.z + quatA.w * quatB.w <
                0.0f)
            {
                quatB = (m3Quat){-quatB.x, -quatB.y, -quatB.z, -quatB.w};
            }
            m3Quat conjA = {-quatA.x, -quatA.y, -quatA.z, quatA.w};
            m3Quat relQ = m3MulQuat(conjA, quatB);
            m3Vec3 axis = c->rotationAxis;

            if ((c->flags & 2) != 0)
            {
                // Motor: velocity drive with the torque-rate cap.
                m3real cdot = m3Dot3(m3Sub3(wB, wA), axis) - c->motorSpeed;
                m3real delta = -c->axialMass * cdot;
                m3real newImpulse = c->motorImpulse + delta;
                m3real maxImpulse = c->maxMotorEffort * hSub;
                newImpulse = m3MaxF(-maxImpulse, m3MinF(maxImpulse, newImpulse));
                delta = newImpulse - c->motorImpulse;
                c->motorImpulse = newImpulse;
                wA = m3Sub3(wA, m3MulSV3(delta, m3MulMV3(c->invIA, axis)));
                wB = m3Add3(wB, m3MulSV3(delta, m3MulMV3(c->invIB, axis)));
            }

            if ((c->flags & 1) != 0)
            {
                // Twist angle about the hinge: 2 atan2(relQ.z, relQ.w).
                m3real angle = 2.0f * m3Atan2(relQ.z, relQ.w);
                // Lower limit.
                {
                    m3real cc = angle - c->lowerLimit;
                    m3real bias = 0.0f;
                    m3real massScale = 1.0f;
                    m3real impulseScale = 0.0f;
                    if (cc > 0.0f)
                    {
                        bias = cc * invHSub;
                    }
                    else if (useBias)
                    {
                        bias = c->softness.biasRate * cc;
                        massScale = c->softness.massScale;
                        impulseScale = c->softness.impulseScale;
                    }
                    m3real cdot = m3Dot3(m3Sub3(wB, wA), axis);
                    m3real old = c->lowerImpulse;
                    m3real delta = -massScale * c->axialMass * (cdot + bias) - impulseScale * old;
                    c->lowerImpulse = m3MaxF(old + delta, 0.0f);
                    delta = c->lowerImpulse - old;
                    wA = m3Sub3(wA, m3MulSV3(delta, m3MulMV3(c->invIA, axis)));
                    wB = m3Add3(wB, m3MulSV3(delta, m3MulMV3(c->invIB, axis)));
                }
                // Upper limit (signs flipped, the reference form).
                {
                    m3real cc = c->upperLimit - angle;
                    m3real bias = 0.0f;
                    m3real massScale = 1.0f;
                    m3real impulseScale = 0.0f;
                    if (cc > 0.0f)
                    {
                        bias = cc * invHSub;
                    }
                    else if (useBias)
                    {
                        bias = c->softness.biasRate * cc;
                        massScale = c->softness.massScale;
                        impulseScale = c->softness.impulseScale;
                    }
                    m3real cdot = m3Dot3(m3Sub3(wA, wB), axis);
                    m3real old = c->upperImpulse;
                    m3real delta = -massScale * c->axialMass * (cdot + bias) - impulseScale * old;
                    c->upperImpulse = m3MaxF(old + delta, 0.0f);
                    delta = c->upperImpulse - old;
                    wA = m3Add3(wA, m3MulSV3(delta, m3MulMV3(c->invIA, axis)));
                    wB = m3Sub3(wB, m3MulSV3(delta, m3MulMV3(c->invIB, axis)));
                }
            }

            // Collinearity: lock the two off-axis rotations, 2x2.
            {
                m3Vec3 perpX = PerpColumn(quatA, relQ, (m3Vec3){1.0f, 0.0f, 0.0f});
                m3Vec3 perpY = PerpColumn(quatA, relQ, (m3Vec3){0.0f, 1.0f, 0.0f});
                c->perpAxisX = perpX;
                c->perpAxisY = perpY;
                m3real biasX = 0.0f;
                m3real biasY = 0.0f;
                m3real massScale = 1.0f;
                m3real impulseScale = 0.0f;
                if (useBias)
                {
                    biasX = c->softness.biasRate * relQ.x;
                    biasY = c->softness.biasRate * relQ.y;
                    massScale = c->softness.massScale;
                    impulseScale = c->softness.impulseScale;
                }
                m3Vec3 sumX = m3Add3(m3MulMV3(c->invIA, perpX), m3MulMV3(c->invIB, perpX));
                m3Vec3 sumY = m3Add3(m3MulMV3(c->invIA, perpY), m3MulMV3(c->invIB, perpY));
                m3real kxx = m3Dot3(perpX, sumX);
                m3real kyy = m3Dot3(perpY, sumY);
                m3real kxy = m3Dot3(perpX, sumY);
                m3Vec3 wRel = m3Sub3(wB, wA);
                m3real cdotX = m3Dot3(wRel, perpX) + biasX;
                m3real cdotY = m3Dot3(wRel, perpY) + biasY;
                m3real det = kxx * kyy - kxy * kxy;
                m3real solX = 0.0f;
                m3real solY = 0.0f;
                if (det != 0.0f)
                {
                    m3real inv = 1.0f / det;
                    solX = inv * (kyy * cdotX - kxy * cdotY);
                    solY = inv * (kxx * cdotY - kxy * cdotX);
                }
                m3real deltaX = -massScale * solX - impulseScale * c->perpImpulseX;
                m3real deltaY = -massScale * solY - impulseScale * c->perpImpulseY;
                c->perpImpulseX += deltaX;
                c->perpImpulseY += deltaY;
                m3Vec3 angular = m3Add3(m3MulSV3(deltaX, perpX), m3MulSV3(deltaY, perpY));
                wA = m3Sub3(wA, m3MulMV3(c->invIA, angular));
                wB = m3Add3(wB, m3MulMV3(c->invIB, angular));
            }

            world->angularVelocities[c->bodyA] = wA;
            world->angularVelocities[c->bodyB] = wB;
            // Fall through to the shared point constraint below with
            // refreshed local copies.
            vA = world->linearVelocities[c->bodyA];
            wA = world->angularVelocities[c->bodyA];
            vB = world->linearVelocities[c->bodyB];
            wB = world->angularVelocities[c->bodyB];
        }
        else if (c->type == (uint8_t)m3_prismaticJoint)
        {
            // The reference prismatic with the FULL Jacobian arms
            // sA = cross(rA + d, axis), sB = cross(rB, axis): the
            // simplified arms are the author-flagged todo, and the
            // finite-difference test in test_joints holds the full
            // form to the numerics.
            m3Vec3 d =
                m3Add3(m3Add3(m3Sub3(deltaPos[c->bodyB], deltaPos[c->bodyA]), c->deltaCenter),
                       m3Sub3(rB, rA));
            m3Vec3 axis = m3RotateVec3(deltaRot[c->bodyA], c->rotationAxis);
            m3Vec3 sAx = m3Cross3(m3Add3(rA, d), axis);
            m3Vec3 sBx = m3Cross3(rB, axis);
            m3real translation = m3Dot3(d, axis);

            // Fresh axial mass every iteration (the reference note:
            // stale masses diverge under stress).
            m3real ka = c->invMassA + c->invMassB + m3Dot3(sAx, m3MulMV3(c->invIA, sAx)) +
                        m3Dot3(sBx, m3MulMV3(c->invIB, sBx));
            m3real axialMass = ka > 0.0f ? 1.0f / ka : 0.0f;

            if ((c->flags & 2) != 0)
            {
                // Motor: velocity drive along the axis, force cap.
                m3Vec3 vRel =
                    m3Sub3(m3Sub3(m3Add3(vB, m3Cross3(wB, rB)), vA), m3Cross3(wA, m3Add3(rA, d)));
                m3real cdot = m3Dot3(vRel, axis) - c->motorSpeed;
                m3real delta = -axialMass * cdot;
                m3real newImpulse = c->motorImpulse + delta;
                m3real maxImpulse = c->maxMotorEffort * hSub;
                newImpulse = m3MaxF(-maxImpulse, m3MinF(maxImpulse, newImpulse));
                delta = newImpulse - c->motorImpulse;
                c->motorImpulse = newImpulse;
                vA = m3Sub3(vA, m3MulSV3(c->invMassA * delta, axis));
                wA = m3Sub3(wA, m3MulMV3(c->invIA, m3MulSV3(delta, sAx)));
                vB = m3Add3(vB, m3MulSV3(c->invMassB * delta, axis));
                wB = m3Add3(wB, m3MulMV3(c->invIB, m3MulSV3(delta, sBx)));
            }

            if ((c->flags & 1) != 0)
            {
                // Lower limit.
                {
                    m3real cc = translation - c->lowerLimit;
                    m3real bias = 0.0f;
                    m3real massScale = 1.0f;
                    m3real impulseScale = 0.0f;
                    if (cc > 0.0f)
                    {
                        bias = cc * invHSub;
                    }
                    else if (useBias)
                    {
                        bias = c->softness.biasRate * cc;
                        massScale = c->softness.massScale;
                        impulseScale = c->softness.impulseScale;
                    }
                    m3Vec3 vRel = m3Sub3(m3Sub3(m3Add3(vB, m3Cross3(wB, rB)), vA),
                                         m3Cross3(wA, m3Add3(rA, d)));
                    m3real cdot = m3Dot3(vRel, axis);
                    m3real old = c->lowerImpulse;
                    m3real delta = -massScale * axialMass * (cdot + bias) - impulseScale * old;
                    c->lowerImpulse = m3MaxF(old + delta, 0.0f);
                    delta = c->lowerImpulse - old;
                    vA = m3Sub3(vA, m3MulSV3(c->invMassA * delta, axis));
                    wA = m3Sub3(wA, m3MulMV3(c->invIA, m3MulSV3(delta, sAx)));
                    vB = m3Add3(vB, m3MulSV3(c->invMassB * delta, axis));
                    wB = m3Add3(wB, m3MulMV3(c->invIB, m3MulSV3(delta, sBx)));
                }
                // Upper limit (signs flipped).
                {
                    m3real cc = c->upperLimit - translation;
                    m3real bias = 0.0f;
                    m3real massScale = 1.0f;
                    m3real impulseScale = 0.0f;
                    if (cc > 0.0f)
                    {
                        bias = cc * invHSub;
                    }
                    else if (useBias)
                    {
                        bias = c->softness.biasRate * cc;
                        massScale = c->softness.massScale;
                        impulseScale = c->softness.impulseScale;
                    }
                    m3Vec3 vRel = m3Sub3(m3Sub3(m3Add3(vB, m3Cross3(wB, rB)), vA),
                                         m3Cross3(wA, m3Add3(rA, d)));
                    m3real cdot = -m3Dot3(vRel, axis);
                    m3real old = c->upperImpulse;
                    m3real delta = -massScale * axialMass * (cdot + bias) - impulseScale * old;
                    c->upperImpulse = m3MaxF(old + delta, 0.0f);
                    m3real applied = old - c->upperImpulse;
                    vA = m3Sub3(vA, m3MulSV3(c->invMassA * applied, axis));
                    wA = m3Sub3(wA, m3MulMV3(c->invIA, m3MulSV3(applied, sAx)));
                    vB = m3Add3(vB, m3MulSV3(c->invMassB * applied, axis));
                    wB = m3Add3(wB, m3MulMV3(c->invIB, m3MulSV3(applied, sBx)));
                }
            }

            // Rotation lock: all three angular DOF held to the
            // prepared relative frame.
            {
                m3Vec3 bias = {0.0f, 0.0f, 0.0f};
                m3real massScale = 1.0f;
                m3real impulseScale = 0.0f;
                if (useBias)
                {
                    m3Quat quatA = m3MulQuat(deltaRot[c->bodyA], c->frameQA);
                    m3Quat quatB = m3MulQuat(deltaRot[c->bodyB], c->frameQB);
                    m3Quat conjA = {-quatA.x, -quatA.y, -quatA.z, quatA.w};
                    m3Quat relQ = m3MulQuat(conjA, quatB);
                    m3Vec3 rotVec = QuatToRotationVec(relQ);
                    m3Vec3 cErr = m3Neg3(m3RotateVec3(quatA, rotVec));
                    bias = m3MulSV3(c->softness.biasRate, cErr);
                    massScale = c->softness.massScale;
                    impulseScale = c->softness.impulseScale;
                }
                m3Mat3 k;
                m3Vec3 basis[3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
                m3Vec3* cols[3] = {&k.cx, &k.cy, &k.cz};
                for (int32_t a = 0; a < 3; ++a)
                {
                    *cols[a] = m3Add3(m3MulMV3(c->invIA, basis[a]), m3MulMV3(c->invIB, basis[a]));
                }
                m3Vec3 cdot = m3Sub3(wB, wA);
                m3Vec3 sol = Solve3(&k, m3Add3(cdot, bias));
                m3Vec3 delta =
                    m3Sub3(m3MulSV3(-massScale, sol), m3MulSV3(impulseScale, c->angularImpulse));
                c->angularImpulse = m3Add3(c->angularImpulse, delta);
                wA = m3Sub3(wA, m3MulMV3(c->invIA, delta));
                wB = m3Add3(wB, m3MulMV3(c->invIB, delta));
            }

            // Point-to-line: the two perpendicular translations, 2x2
            // with the full arms.
            {
                m3Vec3 perpY = m3RotateVec3(deltaRot[c->bodyA], c->perpAxisX);
                m3Vec3 perpZ = m3RotateVec3(deltaRot[c->bodyA], c->perpAxisY);
                m3real biasY = 0.0f;
                m3real biasZ = 0.0f;
                m3real massScale = 1.0f;
                m3real impulseScale = 0.0f;
                if (useBias)
                {
                    biasY = c->softness.biasRate * m3Dot3(perpY, d);
                    biasZ = c->softness.biasRate * m3Dot3(perpZ, d);
                    massScale = c->softness.massScale;
                    impulseScale = c->softness.impulseScale;
                }
                m3Vec3 sAy = m3Cross3(m3Add3(rA, d), perpY);
                m3Vec3 sBy = m3Cross3(rB, perpY);
                m3Vec3 sAz = m3Cross3(m3Add3(rA, d), perpZ);
                m3Vec3 sBz = m3Cross3(rB, perpZ);
                m3real kyy = c->invMassA + c->invMassB + m3Dot3(sAy, m3MulMV3(c->invIA, sAy)) +
                             m3Dot3(sBy, m3MulMV3(c->invIB, sBy));
                m3real kyz =
                    m3Dot3(sAy, m3MulMV3(c->invIA, sAz)) + m3Dot3(sBy, m3MulMV3(c->invIB, sBz));
                m3real kzz = c->invMassA + c->invMassB + m3Dot3(sAz, m3MulMV3(c->invIA, sAz)) +
                             m3Dot3(sBz, m3MulMV3(c->invIB, sBz));
                m3Vec3 vRel =
                    m3Sub3(m3Sub3(m3Add3(vB, m3Cross3(wB, rB)), vA), m3Cross3(wA, m3Add3(rA, d)));
                m3real cdotY = m3Dot3(vRel, perpY) + biasY;
                m3real cdotZ = m3Dot3(vRel, perpZ) + biasZ;
                m3real det = kyy * kzz - kyz * kyz;
                m3real solY = 0.0f;
                m3real solZ = 0.0f;
                if (det != 0.0f)
                {
                    m3real inv = 1.0f / det;
                    solY = inv * (kzz * cdotY - kyz * cdotZ);
                    solZ = inv * (kyy * cdotZ - kyz * cdotY);
                }
                m3real deltaY = -massScale * solY - impulseScale * c->perpImpulseX;
                m3real deltaZ = -massScale * solZ - impulseScale * c->perpImpulseY;
                c->perpImpulseX += deltaY;
                c->perpImpulseY += deltaZ;
                m3Vec3 P = m3Add3(m3MulSV3(deltaY, perpY), m3MulSV3(deltaZ, perpZ));
                m3Vec3 LA = m3Add3(m3MulSV3(deltaY, sAy), m3MulSV3(deltaZ, sAz));
                m3Vec3 LB = m3Add3(m3MulSV3(deltaY, sBy), m3MulSV3(deltaZ, sBz));
                vA = m3Sub3(vA, m3MulSV3(c->invMassA, P));
                wA = m3Sub3(wA, m3MulMV3(c->invIA, LA));
                vB = m3Add3(vB, m3MulSV3(c->invMassB, P));
                wB = m3Add3(wB, m3MulMV3(c->invIB, LB));
            }

            // The prismatic has no free point constraint: write back
            // and continue to the next joint.
            world->linearVelocities[c->bodyA] = vA;
            world->angularVelocities[c->bodyA] = wA;
            world->linearVelocities[c->bodyB] = vB;
            world->angularVelocities[c->bodyB] = wB;
            continue;
        }

        m3Vec3 cdot = m3Sub3(m3Add3(vB, m3Cross3(wB, rB)), m3Add3(vA, m3Cross3(wA, rA)));

        m3Vec3 bias = {0.0f, 0.0f, 0.0f};
        m3real massScale = 1.0f;
        m3real impulseScale = 0.0f;
        if (useBias)
        {
            // The raw anchor violation, the reference form: current
            // anchor delta = COM drift + rotated anchors + the
            // prepare-time center offset. No baseline subtraction:
            // a satisfied joint has deltaCenter = rA0 - rB0 and the
            // sum vanishes by itself.
            m3Vec3 separation =
                m3Add3(m3Add3(m3Sub3(deltaPos[c->bodyB], deltaPos[c->bodyA]), m3Sub3(rB, rA)),
                       c->deltaCenter);
            bias = m3MulSV3(c->softness.biasRate, separation);
            massScale = c->softness.massScale;
            impulseScale = c->softness.impulseScale;
        }

        // K = (mA + mB) I - skew(rA) iA skew(rA) - skew(rB) iB skew(rB),
        // built column by column by applying the operator to the basis
        // (no matrix-matrix helpers needed).
        m3Mat3 k;
        m3Vec3 basis[3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
        m3Vec3* cols[3] = {&k.cx, &k.cy, &k.cz};
        for (int32_t a = 0; a < 3; ++a)
        {
            m3Vec3 e = basis[a];
            m3Vec3 tA = m3Cross3(rA, m3MulMV3(c->invIA, m3Cross3(rA, e)));
            m3Vec3 tB = m3Cross3(rB, m3MulMV3(c->invIB, m3Cross3(rB, e)));
            m3Vec3 col = m3Sub3(m3MulSV3(c->invMassA + c->invMassB, e), m3Add3(tA, tB));
            *cols[a] = col;
        }

        m3Vec3 b = Solve3(&k, m3Add3(cdot, bias));
        m3Vec3 impulse = m3Sub3(m3MulSV3(-massScale, b), m3MulSV3(impulseScale, c->impulse));
        c->impulse = m3Add3(c->impulse, impulse);

        world->linearVelocities[c->bodyA] = m3Sub3(vA, m3MulSV3(c->invMassA, impulse));
        world->angularVelocities[c->bodyA] = m3Sub3(wA, m3MulMV3(c->invIA, m3Cross3(rA, impulse)));
        world->linearVelocities[c->bodyB] = m3Add3(vB, m3MulSV3(c->invMassB, impulse));
        world->angularVelocities[c->bodyB] = m3Add3(wB, m3MulMV3(c->invIB, m3Cross3(rB, impulse)));
    }
}

static void StoreJointImpulses(m3World* world, m3JointConstraint* joints, int32_t count)
{
    for (int32_t i = 0; i < count; ++i)
    {
        const m3JointConstraint* c = &joints[i];
        world->jointImpulse[c->joint] = c->impulse;
        world->jointPerpImpulse[c->joint] =
            (m3Vec3){c->perpImpulseX, c->perpImpulseY, c->motorImpulse};
        world->jointLimitImpulse[c->joint] = (m3Vec3){c->lowerImpulse, c->upperImpulse, 0.0f};
        world->jointAngularImpulse[c->joint] = c->angularImpulse;
    }
}

// ---------------------------------------------------------------
// Graph coloring (2b-12): constraints in one color share no awake
// dynamic body, so any schedule inside a color writes disjoint
// velocities and the bits cannot move. The greedy walk runs in
// canonical constraint order with first-free-bit colors; whatever
// cannot color inside the palette lands in the overflow bucket and
// solves serially. Wide 4-lane batching joins the profiling era
// (recorded in the plan); the parallel structure lands here.
// ---------------------------------------------------------------
#define M3_GRAPH_COLORS 16

typedef struct m3SolverColoring
{
    uint8_t* colors; // per constraint
    int32_t* lists;  // constraint indices grouped by color
    int32_t starts[M3_GRAPH_COLORS + 2];
} m3SolverColoring;

static int BuildColoring(m3World* world, m3ContactConstraint* constraints, int32_t count,
                         m3SolverColoring* out)
{
    int32_t maxBody = world->bodyPool.maxIndex;
    out->colors = (uint8_t*)m3StackAlloc(&world->scratch, count > 0 ? count : 1);
    uint32_t* bodyMasks = (uint32_t*)m3StackAlloc(
        &world->scratch, maxBody > 0 ? maxBody * (int32_t)sizeof(uint32_t) : 4);
    out->lists =
        (int32_t*)m3StackAlloc(&world->scratch, count > 0 ? count * (int32_t)sizeof(int32_t) : 4);
    if (out->colors == NULL || bodyMasks == NULL || out->lists == NULL)
    {
        return 0;
    }
    memset(bodyMasks, 0, (size_t)(maxBody > 0 ? maxBody : 1) * sizeof(uint32_t));

    int32_t counts[M3_GRAPH_COLORS + 1];
    memset(counts, 0, sizeof(counts));
    for (int32_t i = 0; i < count; ++i)
    {
        m3ContactConstraint* c = &constraints[i];
        int dynA = world->types[c->bodyA] == (uint8_t)m3_dynamicBody && world->awake[c->bodyA];
        int dynB = world->types[c->bodyB] == (uint8_t)m3_dynamicBody && world->awake[c->bodyB];
        uint32_t mask = (dynA ? bodyMasks[c->bodyA] : 0u) | (dynB ? bodyMasks[c->bodyB] : 0u);
        int32_t color = M3_GRAPH_COLORS; // overflow unless a bit frees up
        for (int32_t bit = 0; bit < M3_GRAPH_COLORS; ++bit)
        {
            if ((mask & (1u << bit)) == 0u)
            {
                color = bit;
                break;
            }
        }
        out->colors[i] = (uint8_t)color;
        if (color < M3_GRAPH_COLORS)
        {
            if (dynA)
            {
                bodyMasks[c->bodyA] |= 1u << color;
            }
            if (dynB)
            {
                bodyMasks[c->bodyB] |= 1u << color;
            }
        }
        counts[color] += 1;
    }
    int32_t cursor = 0;
    for (int32_t c = 0; c <= M3_GRAPH_COLORS; ++c)
    {
        out->starts[c] = cursor;
        cursor += counts[c];
    }
    out->starts[M3_GRAPH_COLORS + 1] = cursor;
    int32_t fill[M3_GRAPH_COLORS + 1];
    memcpy(fill, out->starts, sizeof(fill));
    for (int32_t i = 0; i < count; ++i)
    {
        out->lists[fill[out->colors[i]]++] = i; // ascending inside a color
    }
    return 1;
}

typedef struct m3SolveTaskContext
{
    m3World* world;
    m3ContactConstraint* constraints;
    const int32_t* list;
    const m3Vec3* deltaPos;
    const m3Quat* deltaRot;
    m3real invH;
    int useBias;
    int warmStartOnly;
} m3SolveTaskContext;

static void SolveRangeTask(int32_t startIndex, int32_t endIndex, void* taskContext)
{
    m3SolveTaskContext* ctx = (m3SolveTaskContext*)taskContext;
    for (int32_t k = startIndex; k < endIndex; ++k)
    {
        m3ContactConstraint* c = &ctx->constraints[ctx->list[k]];
        if (ctx->warmStartOnly)
        {
            WarmStartOne(ctx->world, c);
        }
        else
        {
            SolveOneContact(ctx->world, c, ctx->deltaPos, ctx->deltaRot, ctx->invH,
                            ctx->useBias != 0);
        }
    }
}

// Colors run in order with a barrier between them; inside a color
// the host may split any way it likes (disjoint bodies), and the
// overflow bucket always runs serial (its members may conflict).
static void RunColored(m3World* world, const m3SolverColoring* coloring,
                       m3ContactConstraint* constraints, const m3Vec3* deltaPos,
                       const m3Quat* deltaRot, m3real invH, int useBias, int warmStartOnly)
{
    for (int32_t color = 0; color <= M3_GRAPH_COLORS; ++color)
    {
        int32_t start = coloring->starts[color];
        int32_t end = coloring->starts[color + 1];
        int32_t size = end - start;
        if (size == 0)
        {
            continue;
        }
        m3SolveTaskContext ctx;
        ctx.world = world;
        ctx.constraints = constraints;
        ctx.list = coloring->lists + start;
        ctx.deltaPos = deltaPos;
        ctx.deltaRot = deltaRot;
        ctx.invH = invH;
        ctx.useBias = useBias;
        ctx.warmStartOnly = warmStartOnly;
        if (world->enqueueTask != NULL && size >= 16 && color < M3_GRAPH_COLORS)
        {
            void* task = world->enqueueTask(SolveRangeTask, size, 8, &ctx, world->userTaskContext);
            world->finishTask(task, world->userTaskContext);
        }
        else
        {
            SolveRangeTask(0, size, &ctx);
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
    // Joints couple islands exactly like touching contacts, and a
    // moving kinematic partner is a wake source through a joint too.
    int32_t maxJoint = world->jointPool.maxIndex;
    for (int32_t j = 0; j < maxJoint; ++j)
    {
        if (world->jointPool.alive[j] == 0)
        {
            continue;
        }
        int32_t bodyA = world->jointBodyA[j];
        int32_t bodyB = world->jointBodyB[j];
        int dynA = world->types[bodyA] == (uint8_t)m3_dynamicBody;
        int dynB = world->types[bodyB] == (uint8_t)m3_dynamicBody;
        if (dynA && dynB)
        {
            IslandUnion(parent, bodyA, bodyB);
        }
        else if (dynA || dynB)
        {
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

    m3SolverColoring coloring;
    if (!BuildColoring(world, constraints, constraintCount, &coloring))
    {
        M3_ASSERT(false);
        return;
    }

    int32_t maxJointSlots = world->jointPool.maxIndex;
    m3JointConstraint* jointConstraints = (m3JointConstraint*)m3StackAlloc(
        &world->scratch, maxJointSlots > 0 ? maxJointSlots * (int32_t)sizeof(m3JointConstraint)
                                           : (int32_t)sizeof(m3JointConstraint));
    if (jointConstraints == NULL)
    {
        M3_ASSERT(false);
        return;
    }
    int32_t jointCount = PrepareJoints(world, jointConstraints, h);

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

        WarmStartJoints(world, jointConstraints, jointCount, deltaRot);
        RunColored(world, &coloring, constraints, deltaPos, deltaRot, invH, 0, 1);
        SolveJoints(world, jointConstraints, jointCount, deltaPos, deltaRot, h, invH, 1);
        RunColored(world, &coloring, constraints, deltaPos, deltaRot, invH, 1, 0);

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
        SolveJoints(world, jointConstraints, jointCount, deltaPos, deltaRot, h, invH, 0);
        RunColored(world, &coloring, constraints, deltaPos, deltaRot, invH, 0, 0);
    }

    Restitution(world, constraints, constraintCount);
    StoreImpulses(world, constraints, constraintCount);
    StoreJointImpulses(world, jointConstraints, jointCount);

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
