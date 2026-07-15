// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Joint lifecycle (2c-2): the same law as bodies and shapes. Public
// functions validate and journal, internal functions mutate, replay
// drives the internals and verifies minted ids. The warm-start
// impulse lives in the persistent arena and rides the snapshot.

#include "world_internal.h"

#include <string.h>

#define M3_JOINT_COOKIE ((int32_t)(M3_COOKIE ^ ((int32_t)sizeof(m3JointDef) << 8) ^ 3))

m3JointDef m3DefaultJointDef(void)
{
    m3JointDef def;
    memset(&def, 0, sizeof(def));
    def.type = m3_sphericalJoint;
    def.localAxisA = (m3Vec3){0.0f, 0.0f, 1.0f};
    def.localAxisB = (m3Vec3){0.0f, 0.0f, 1.0f};
    def.genericMotorAxis = 255; // no motor unless chosen
    def.internalValue = M3_JOINT_COOKIE;
    return def;
}

// Quaternion from an orthonormal right-handed column basis
// (deterministic: Shepperd's branch on the largest diagonal).
static m3Quat QuatFromBasis(m3Vec3 t1, m3Vec3 t2, m3Vec3 axis)
{
    m3real m00 = t1.x, m01 = t2.x, m02 = axis.x;
    m3real m10 = t1.y, m11 = t2.y, m12 = axis.y;
    m3real m20 = t1.z, m21 = t2.z, m22 = axis.z;
    m3Quat q;
    m3real trace = m00 + m11 + m22;
    if (trace > 0.0f)
    {
        m3real s = sqrtf(trace + 1.0f) * 2.0f;
        q.w = 0.25f * s;
        q.x = (m21 - m12) / s;
        q.y = (m02 - m20) / s;
        q.z = (m10 - m01) / s;
    }
    else if (m00 > m11 && m00 > m22)
    {
        m3real s = sqrtf(1.0f + m00 - m11 - m22) * 2.0f;
        q.w = (m21 - m12) / s;
        q.x = 0.25f * s;
        q.y = (m01 + m10) / s;
        q.z = (m02 + m20) / s;
    }
    else if (m11 > m22)
    {
        m3real s = sqrtf(1.0f + m11 - m00 - m22) * 2.0f;
        q.w = (m02 - m20) / s;
        q.x = (m01 + m10) / s;
        q.y = 0.25f * s;
        q.z = (m12 + m21) / s;
    }
    else
    {
        m3real s = sqrtf(1.0f + m22 - m00 - m11) * 2.0f;
        q.w = (m10 - m01) / s;
        q.x = (m02 + m20) / s;
        q.y = (m12 + m21) / s;
        q.z = 0.25f * s;
    }
    return m3NormalizeQuat(q);
}

// Quaternion whose z-axis is the given unit axis, with the tangent
// basis rule fixing the other two columns.
static m3Quat QuatFromAxisZ(m3Vec3 axis)
{
    m3Vec3 t1;
    m3Vec3 t2;
    m3MakeTangentBasis(axis, &t1, &t2);
    return QuatFromBasis(t1, t2, axis);
}

int32_t m3JointSlot(const m3World* world, m3JointId jointId)
{
    int32_t index = jointId.index1 - 1;
    if (world == NULL || jointId.world0 != world->worldIndex0 ||
        !m3IdPoolValid(&world->jointPool, index, jointId.generation))
    {
        return -1;
    }
    return index;
}

int32_t m3CreateJointInternal(m3World* world, const m3JointDef* def, int32_t bodyA, int32_t bodyB)
{
    // The wheel frame (12-2), built BEFORE the slot is taken so a
    // refused geometry leaks nothing. Frame x = the suspension axis
    // (from A), frame z = the axle captured from B's world image and
    // snapped exactly perpendicular; real skew refuses loudly. This
    // validation lives here, not in the public wall, because replay
    // hands this function raw journal bytes.
    m3Quat wheelQA = {0.0f, 0.0f, 0.0f, 1.0f};
    m3Quat wheelQB = {0.0f, 0.0f, 0.0f, 1.0f};
    if (def->type == (int32_t)m3_wheelJoint)
    {
        const m3Transform* xfA = &world->transforms[bodyA];
        const m3Transform* xfB = &world->transforms[bodyB];
        m3Vec3 susp = m3Normalize3(def->localAxisA);
        m3Vec3 axleWorld = m3RotateVec3(xfB->q, m3Normalize3(def->localAxisB));
        m3Quat conjA = {-xfA->q.x, -xfA->q.y, -xfA->q.z, xfA->q.w};
        m3Vec3 axleA = m3RotateVec3(conjA, axleWorld);
        m3real skew = m3Dot3(axleA, susp);
        if (skew > 0.1f || skew < -0.1f)
        {
            return -1; // an axle along the strut is not a wheel
        }
        m3Vec3 z = m3Normalize3(m3Sub3(axleA, m3MulSV3(skew, susp)));
        m3Vec3 y = m3Cross3(z, susp); // x cross y = z, right-handed
        wheelQA = QuatFromBasis(susp, y, z);
        m3Quat conjB = {-xfB->q.x, -xfB->q.y, -xfB->q.z, xfB->q.w};
        wheelQB = m3NormalizeQuat(m3MulQuat(conjB, m3MulQuat(xfA->q, wheelQA)));
    }
    int32_t index = m3IdPoolAlloc(&world->jointPool);
    if (index < 0)
    {
        return -1; // exhausted: loud at the caller
    }
    world->jointType[index] = (uint8_t)def->type;
    world->jointBodyA[index] = bodyA;
    world->jointBodyB[index] = bodyB;
    world->jointLocalA[index] = def->localAnchorA;
    world->jointLocalB[index] = def->localAnchorB;
    world->jointCollide[index] = def->collideConnected ? 1 : 0;
    world->jointImpulse[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
    world->jointPerpImpulse[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
    world->jointLimitImpulse[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
    world->jointAngularImpulse[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
    if (def->type == (int32_t)m3_fixedJoint || def->type == (int32_t)m3_motorJoint)
    {
        // The weld pose (4-2): store frames so that at the create
        // pose the two world frames coincide; the solver's rotation
        // lock then drives their live relative rotation back to
        // identity. frameA = identity, frameB = conj(qB0) * qA0.
        const m3Transform* xfA = &world->transforms[bodyA];
        const m3Transform* xfB = &world->transforms[bodyB];
        m3Quat conjB = {-xfB->q.x, -xfB->q.y, -xfB->q.z, xfB->q.w};
        world->jointFrameQA[index] = (m3Quat){0.0f, 0.0f, 0.0f, 1.0f};
        world->jointFrameQB[index] = m3NormalizeQuat(m3MulQuat(conjB, xfA->q));
    }
    else if (def->type == (int32_t)m3_distanceJoint)
    {
        // Frames are unused by the axial row: identity keeps the
        // stored state canonical and the hash honest.
        world->jointFrameQA[index] = (m3Quat){0.0f, 0.0f, 0.0f, 1.0f};
        world->jointFrameQB[index] = (m3Quat){0.0f, 0.0f, 0.0f, 1.0f};
    }
    else if (def->type == (int32_t)m3_wheelJoint)
    {
        // Built above: relQ is identity at create, so the spin
        // angle and the collinearity error both start at zero.
        world->jointFrameQA[index] = wheelQA;
        world->jointFrameQB[index] = wheelQB;
    }
    else if (def->type == (int32_t)m3_filterJoint)
    {
        // A filter has no geometry at all: identity frames, and a
        // zero axis must never reach the normalizer (16-1).
        world->jointFrameQA[index] = (m3Quat){0.0f, 0.0f, 0.0f, 1.0f};
        world->jointFrameQB[index] = (m3Quat){0.0f, 0.0f, 0.0f, 1.0f};
    }
    else
    {
        world->jointFrameQA[index] = QuatFromAxisZ(m3Normalize3(def->localAxisA));
        world->jointFrameQB[index] = QuatFromAxisZ(m3Normalize3(def->localAxisB));
    }
    world->jointFlags[index] = (uint8_t)((def->enableLimit ? 1 : 0) | (def->enableMotor ? 2 : 0) |
                                         (def->enableCone ? 4 : 0));
    world->jointMotor[index] = (m3Vec3){def->motorSpeed, def->maxMotorEffort, 0.0f};
    if (def->type == (int32_t)m3_motorJoint)
    {
        // The servo's default aim is the CREATE pose (16-5): bake
        // the anchor gap (in A's frame) into the offset slot so a
        // fresh servo holds where it was built instead of snapping
        // the anchors together the moment its spring wakes.
        const m3Transform* xfA = &world->transforms[bodyA];
        const m3Transform* xfB = &world->transforms[bodyB];
        m3Vec3 aA = m3RotateVec3(xfA->q, def->localAnchorA);
        m3Vec3 aB = m3RotateVec3(xfB->q, def->localAnchorB);
        m3Vec3 gap = {(m3real)(xfB->p.x + (double)aB.x - xfA->p.x - (double)aA.x),
                      (m3real)(xfB->p.y + (double)aB.y - xfA->p.y - (double)aA.y),
                      (m3real)(xfB->p.z + (double)aB.z - xfA->p.z - (double)aA.z)};
        m3Quat conjA = {-xfA->q.x, -xfA->q.y, -xfA->q.z, xfA->q.w};
        world->jointMotor[index] = m3RotateVec3(conjA, gap);
    }
    world->jointBreak[index] = (m3Vec3){0.0f, 0.0f, 0.0f}; // unbreakable default
    world->jointSpring[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
    world->jointTargetScalar[index] = 0.0f;
    world->jointTargetQ[index] = (m3Quat){0.0f, 0.0f, 0.0f, 1.0f};
    world->jointSpringImpulse[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
    // For the spherical, z carries the cone angle (x and y stay the
    // twist range): no snapshot growth, all of it already hashed.
    world->jointLimits[index] = (m3Vec3){def->lowerLimit, def->upperLimit, def->coneAngle};
    world->jointGenericModes[index] = 0;
    world->jointGenLinLower[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
    world->jointGenLinUpper[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
    world->jointGenAngLower[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
    world->jointGenAngUpper[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
    if (def->type == (int32_t)m3_genericJoint)
    {
        // Modes pack two bits per axis: linear in bits 0..5,
        // angular in bits 6..11, the motor axis in bits 12..15.
        uint16_t packed = 0;
        for (int32_t k = 0; k < 3; ++k)
        {
            packed |= (uint16_t)(def->genericLinear[k] & 3) << (2 * k);
            packed |= (uint16_t)(def->genericAngular[k] & 3) << (6 + 2 * k);
        }
        packed |= (uint16_t)(def->genericMotorAxis == 255 ? 15 : def->genericMotorAxis) << 12;
        world->jointGenericModes[index] = packed;
        world->jointGenLinLower[index] = (m3Vec3){
            def->genericLinearLower[0], def->genericLinearLower[1], def->genericLinearLower[2]};
        world->jointGenLinUpper[index] = (m3Vec3){
            def->genericLinearUpper[0], def->genericLinearUpper[1], def->genericLinearUpper[2]};
        world->jointGenAngLower[index] = (m3Vec3){
            def->genericAngularLower[0], def->genericAngularLower[1], def->genericAngularLower[2]};
        world->jointGenAngUpper[index] = (m3Vec3){
            def->genericAngularUpper[0], def->genericAngularUpper[1], def->genericAngularUpper[2]};
    }
    // Push onto both bodies' joint lists (creation order recoverable:
    // replay recreates in the same order).
    world->jointNextA[index] = world->bodyJointHead[bodyA];
    world->bodyJointHead[bodyA] = index;
    world->jointNextB[index] = world->bodyJointHead[bodyB];
    world->bodyJointHead[bodyB] = index;
    // A new joint wakes both sides: articulation is a disturbance.
    if (world->types[bodyA] == (uint8_t)m3_dynamicBody)
    {
        world->awake[bodyA] = 1;
        world->sleepTimes[bodyA] = 0.0f;
    }
    if (world->types[bodyB] == (uint8_t)m3_dynamicBody)
    {
        world->awake[bodyB] = 1;
        world->sleepTimes[bodyB] = 0.0f;
    }
    return index;
}

// Unlink from one body's list (the list is threaded through nextA
// for joints where the body plays A and nextB where it plays B).
static void UnlinkJoint(m3World* world, int32_t body, int32_t index)
{
    int32_t* cursor = &world->bodyJointHead[body];
    while (*cursor != -1)
    {
        int32_t j = *cursor;
        if (j == index)
        {
            *cursor = world->jointBodyA[j] == body ? world->jointNextA[j] : world->jointNextB[j];
            return;
        }
        cursor = world->jointBodyA[j] == body ? &world->jointNextA[j] : &world->jointNextB[j];
    }
}

void m3DestroyJointInternal(m3World* world, int32_t index)
{
    int32_t bodyA = world->jointBodyA[index];
    int32_t bodyB = world->jointBodyB[index];
    UnlinkJoint(world, bodyA, index);
    UnlinkJoint(world, bodyB, index);
    if (world->types[bodyA] == (uint8_t)m3_dynamicBody && world->bodyPool.alive[bodyA] != 0)
    {
        world->awake[bodyA] = 1;
        world->sleepTimes[bodyA] = 0.0f;
    }
    if (world->types[bodyB] == (uint8_t)m3_dynamicBody && world->bodyPool.alive[bodyB] != 0)
    {
        world->awake[bodyB] = 1;
        world->sleepTimes[bodyB] = 0.0f;
    }
    world->jointType[index] = 0;
    world->jointBodyA[index] = -1;
    world->jointBodyB[index] = -1;
    world->jointLocalA[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
    world->jointLocalB[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
    world->jointCollide[index] = 0;
    world->jointImpulse[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
    world->jointPerpImpulse[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
    world->jointLimitImpulse[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
    world->jointAngularImpulse[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
    world->jointFrameQA[index] = m3MakeIdentityQuat();
    world->jointFrameQB[index] = m3MakeIdentityQuat();
    world->jointFlags[index] = 0;
    world->jointMotor[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
    world->jointBreak[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
    world->jointSpring[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
    world->jointTargetScalar[index] = 0.0f;
    world->jointTargetQ[index] = (m3Quat){0.0f, 0.0f, 0.0f, 1.0f};
    world->jointSpringImpulse[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
    world->jointLimits[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
    world->jointGenericModes[index] = 0;
    world->jointGenLinLower[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
    world->jointGenLinUpper[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
    world->jointGenAngLower[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
    world->jointGenAngUpper[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
    world->jointNextA[index] = -1;
    world->jointNextB[index] = -1;
    m3IdPoolFree(&world->jointPool, index);
}

m3JointId m3CreateJoint(const m3JointDef* def)
{
    if (def == NULL || def->internalValue != M3_JOINT_COOKIE ||
        (def->type != (int32_t)m3_sphericalJoint && def->type != (int32_t)m3_revoluteJoint &&
         def->type != (int32_t)m3_prismaticJoint && def->type != (int32_t)m3_fixedJoint &&
         def->type != (int32_t)m3_distanceJoint && def->type != (int32_t)m3_genericJoint &&
         def->type != (int32_t)m3_wheelJoint && def->type != (int32_t)m3_filterJoint &&
         def->type != (int32_t)m3_parallelJoint && def->type != (int32_t)m3_motorJoint))
    {
        return m3_nullJointId;
    }
    if ((def->type == (int32_t)m3_revoluteJoint || def->type == (int32_t)m3_prismaticJoint ||
         def->type == (int32_t)m3_wheelJoint || def->type == (int32_t)m3_parallelJoint) &&
        (!(m3Dot3(def->localAxisA, def->localAxisA) > 0.0f) ||
         !(m3Dot3(def->localAxisB, def->localAxisB) > 0.0f)))
    {
        return m3_nullJointId; // a hinge, slider, or wheel needs real axes
    }
    if (def->type == (int32_t)m3_genericJoint)
    {
        // The generic contract (4-3): sane modes, finite ordered
        // limits where used, one motor on a movable axis, and the
        // v1 angular-limit rule.
        int32_t limitedAngular = 0;
        int32_t lockedAngular = 0;
        int32_t freeAngular = 0;
        for (int32_t k = 0; k < 3; ++k)
        {
            if (def->genericLinear[k] > 2 || def->genericAngular[k] > 2)
            {
                return m3_nullJointId;
            }
            if (def->genericLinear[k] == (uint8_t)m3_axisLimited &&
                (!m3FiniteF(def->genericLinearLower[k]) || !m3FiniteF(def->genericLinearUpper[k]) ||
                 def->genericLinearLower[k] > def->genericLinearUpper[k]))
            {
                return m3_nullJointId;
            }
            if (def->genericAngular[k] == (uint8_t)m3_axisLimited)
            {
                limitedAngular += 1;
                if (!m3FiniteF(def->genericAngularLower[k]) ||
                    !m3FiniteF(def->genericAngularUpper[k]) ||
                    def->genericAngularLower[k] > def->genericAngularUpper[k])
                {
                    return m3_nullJointId;
                }
            }
            else if (def->genericAngular[k] == (uint8_t)m3_axisLocked)
            {
                lockedAngular += 1;
            }
            else
            {
                freeAngular += 1;
            }
        }
        if (limitedAngular > 1 || (limitedAngular == 1 && lockedAngular != 2 && freeAngular != 2))
        {
            return m3_nullJointId; // the v1 angular rule, documented
        }
        if (def->genericMotorAxis != 255)
        {
            if (def->genericMotorAxis > 5 || !m3FiniteF(def->motorSpeed) ||
                !m3FiniteF(def->maxMotorEffort) || def->maxMotorEffort < 0.0f)
            {
                return m3_nullJointId;
            }
            uint8_t mode = def->genericMotorAxis < 3
                               ? def->genericLinear[def->genericMotorAxis]
                               : def->genericAngular[def->genericMotorAxis - 3];
            if (mode == (uint8_t)m3_axisLocked)
            {
                return m3_nullJointId; // a motor on a locked axis is a
                                       // contradiction, not a request
            }
        }
        if (!(m3Dot3(def->localAxisA, def->localAxisA) > 0.0f) ||
            !(m3Dot3(def->localAxisB, def->localAxisB) > 0.0f))
        {
            return m3_nullJointId; // the joint frame needs real axes
        }
    }
    if (def->type == (int32_t)m3_distanceJoint &&
        (!def->enableLimit || def->lowerLimit < 0.0f ||
         (def->enableMotor && (!(def->motorSpeed > 0.0f) || def->maxMotorEffort < 0.0f ||
                               (def->coneAngle > 0.0f && (def->coneAngle < def->lowerLimit ||
                                                          def->coneAngle > def->upperLimit))))))
    {
        // The distance contract: an explicit range (rod = equal
        // bounds), and a spring only with a real hertz.
        return m3_nullJointId;
    }
    // Hostile-input wall (2d-1): finite fields only, ordered limits
    // only, and a cone that is a cone.
    if (!m3FiniteV3(def->localAnchorA) || !m3FiniteV3(def->localAnchorB) ||
        !m3FiniteV3(def->localAxisA) || !m3FiniteV3(def->localAxisB) ||
        !m3FiniteF(def->lowerLimit) || !m3FiniteF(def->upperLimit) || !m3FiniteF(def->motorSpeed) ||
        !m3FiniteF(def->maxMotorEffort) || !m3FiniteF(def->coneAngle) ||
        def->maxMotorEffort < 0.0f || (def->enableLimit && def->lowerLimit > def->upperLimit) ||
        (def->enableCone && def->coneAngle < 0.0f))
    {
        return m3_nullJointId;
    }
    m3World* world = m3WorldFromIndex0(def->bodyA.world0);
    if (world == NULL || def->bodyB.world0 != def->bodyA.world0)
    {
        return m3_nullJointId; // both bodies must share a world
    }
    int32_t bodyA = m3BodySlot(world, def->bodyA);
    int32_t bodyB = m3BodySlot(world, def->bodyB);
    if (bodyA < 0 || bodyB < 0 || bodyA == bodyB)
    {
        return m3_nullJointId;
    }
    if (world->types[bodyA] != (uint8_t)m3_dynamicBody &&
        world->types[bodyB] != (uint8_t)m3_dynamicBody)
    {
        return m3_nullJointId; // a joint between immovables is inert
    }
    int32_t index = m3CreateJointInternal(world, def, bodyA, bodyB);
    if (index < 0)
    {
        return m3_nullJointId;
    }
    m3JointId id = {index + 1, world->worldIndex0, world->jointPool.generations[index]};
    if (world->journalActive != 0)
    {
        m3CreateJointOp record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.expected = id;
        m3JournalRecord(world, m3_opCreateJoint, &record, (int32_t)sizeof(record));
    }
    return id;
}

void m3DestroyJoint(m3JointId jointId)
{
    m3World* world = m3WorldFromIndex0(jointId.world0);
    int32_t index = world != NULL ? m3JointSlot(world, jointId) : -1;
    if (index < 0)
    {
        return; // stale id: a quiet no-op is the destroy contract
    }
    if (world->journalActive != 0)
    {
        m3JournalRecord(world, m3_opDestroyJoint, &jointId, (int32_t)sizeof(jointId));
    }
    m3DestroyJointInternal(world, index);
}

bool m3Joint_IsValid(m3JointId jointId)
{
    m3World* world = m3WorldFromIndex0(jointId.world0);
    return world != NULL && m3JointSlot(world, jointId) >= 0;
}

// --- Runtime control and breakage (8-6a) ------------------------------------

static void JointWakeBodies(m3World* world, int32_t j)
{
    int32_t bodyA = world->jointBodyA[j];
    int32_t bodyB = world->jointBodyB[j];
    if (bodyA >= 0 && world->types[bodyA] == (uint8_t)m3_dynamicBody)
    {
        m3SetAwakeInternal(world, bodyA, 1);
    }
    if (bodyB >= 0 && world->types[bodyB] == (uint8_t)m3_dynamicBody)
    {
        m3SetAwakeInternal(world, bodyB, 1);
    }
}

void m3JointSetLimitsInternal(m3World* world, int32_t j, int32_t enable, float lower, float upper)
{
    world->jointFlags[j] = (uint8_t)((world->jointFlags[j] & ~1u) | (enable != 0 ? 1u : 0u));
    // z carries the spherical cone angle: never clobbered here.
    world->jointLimits[j].x = lower;
    world->jointLimits[j].y = upper;
    // A toggled or moved limit invalidates the stored row impulses
    // (x = lower, y = upper; the spherical's swing rides z and
    // survives only when untouched by this setter's rows).
    world->jointLimitImpulse[j].x = 0.0f;
    world->jointLimitImpulse[j].y = 0.0f;
    JointWakeBodies(world, j);
}

void m3JointSetMotorInternal(m3World* world, int32_t j, int32_t enable, float speed, float effort)
{
    world->jointFlags[j] = (uint8_t)((world->jointFlags[j] & ~2u) | (enable != 0 ? 2u : 0u));
    world->jointMotor[j].x = speed;
    world->jointMotor[j].y = effort;
    if (world->jointType[j] == (uint8_t)m3_genericJoint)
    {
        world->jointLimitImpulse[j].y = 0.0f; // the generic slot map
    }
    else
    {
        world->jointPerpImpulse[j].z = 0.0f;
    }
    JointWakeBodies(world, j);
}

void m3JointSetSteerInternal(m3World* world, int32_t j, int32_t enable, float target, float hertz,
                             float zeta, float maxEffort)
{
    // The wheel slot map (16-3): flags bit 4 (the cone bit, unused
    // on wheels), target in jointMotor.z, softness and budget in
    // the spherical target slots, warm impulse in spring slot y.
    world->jointFlags[j] = (uint8_t)((world->jointFlags[j] & ~4u) | (enable != 0 ? 4u : 0u));
    world->jointMotor[j].z = target;
    world->jointTargetQ[j].x = hertz;
    world->jointTargetQ[j].y = zeta;
    world->jointTargetQ[j].z = maxEffort;
    if (enable == 0)
    {
        world->jointSpringImpulse[j].y = 0.0f; // the warm slot dies too
    }
    JointWakeBodies(world, j);
}

void m3JointSetMotorPoseInternal(m3World* world, int32_t j, m3Vec3 offset, m3Quat rotation)
{
    // The motor slot map (16-5): the target offset rides jointMotor
    // (the servo has no velocity motor), the rotation rides the
    // spherical target slot.
    world->jointMotor[j] = offset;
    world->jointTargetQ[j] = m3NormalizeQuat(rotation);
    JointWakeBodies(world, j);
}

void m3JointSetCollideInternal(m3World* world, int32_t j, int32_t on)
{
    world->jointCollide[j] = on != 0 ? 1 : 0;
    JointWakeBodies(world, j);
}

void m3JointSetBreakInternal(m3World* world, int32_t j, float maxForce, float maxTorque)
{
    world->jointBreak[j] = (m3Vec3){maxForce, maxTorque, 0.0f};
    JointWakeBodies(world, j);
}

// Reaction magnitudes, the per-type assembly documented on the API:
// linear rows fold into force, angular rows into torque, and the
// generic joint reports a conservative sum of its slot map.
void m3JointReactionMagnitudes(const m3World* world, int32_t j, m3real invH, m3real* outForce,
                               m3real* outTorque)
{
    m3Vec3 lin = world->jointImpulse[j];
    m3Vec3 perp = world->jointPerpImpulse[j];
    m3Vec3 lim = world->jointLimitImpulse[j];
    m3Vec3 ang = world->jointAngularImpulse[j];
    m3real force = m3Length3(lin);
    m3real torque = 0.0f;
    switch (world->jointType[j])
    {
    case (uint8_t)m3_parallelJoint:
        // Two angular locks, nothing else (16-1).
        *outForce = 0.0f;
        *outTorque = invH * m3Length3((m3Vec3){perp.x, perp.y, 0.0f});
        return;
    case (uint8_t)m3_filterJoint:
        // No rows, no reactions, ever (16-1).
        *outForce = 0.0f;
        *outTorque = 0.0f;
        return;
    case (uint8_t)m3_revoluteJoint:
        // Perp x, y are the hinge's cross-axis ANGULAR locks; the
        // motor rides perp.z and the limits ride lim.x, lim.y, all
        // about the hinge axis.
        torque =
            sqrtf(perp.x * perp.x + perp.y * perp.y) + fabsf(perp.z) + fabsf(lim.x) + fabsf(lim.y);
        break;
    case (uint8_t)m3_prismaticJoint:
        // Perp x, y are LINEAR translation locks, the motor and the
        // limits act along the slide axis: all force. The 3-DOF
        // rotation lock is the torque.
        force +=
            sqrtf(perp.x * perp.x + perp.y * perp.y) + fabsf(perp.z) + fabsf(lim.x) + fabsf(lim.y);
        torque = m3Length3(ang);
        break;
    case (uint8_t)m3_sphericalJoint:
        // Twist rows ride lim.x, lim.y; the swing row rides lim.z.
        torque = fabsf(lim.x) + fabsf(lim.y) + fabsf(lim.z);
        break;
    case (uint8_t)m3_distanceJoint:
        force += fabsf(lim.x) + fabsf(lim.y) + fabsf(perp.z);
        break;
    case (uint8_t)m3_genericJoint:
        // The slot map (4-3): linear uppers ride perp x, y; the
        // angular upper and the motor ride lim.x, lim.y. The motor
        // may be either kind: fold it into BOTH sums, conservative
        // by construction, documented.
        force += sqrtf(perp.x * perp.x + perp.y * perp.y) + fabsf(lim.x) + fabsf(lim.y);
        torque = m3Length3(ang) + fabsf(lim.x) + fabsf(lim.y);
        break;
    case (uint8_t)m3_wheelJoint:
        // The composed split (12-2): lin carries the two point-to-
        // line rows and the suspension limits are linear (force);
        // perp x, y are the axle collinearity locks and the spin
        // motor rides perp.z (torque). The BREAK CONTRACT for an
        // axle: the force cap snaps a wheel torn sideways, the
        // torque cap snaps a drive axle over-driven.
        force += fabsf(lim.x) + fabsf(lim.y);
        torque = sqrtf(perp.x * perp.x + perp.y * perp.y) + fabsf(perp.z);
        break;
    default: // fixed: weld rows
        torque = m3Length3(ang);
        break;
    }
    *outForce = force * invH;
    *outTorque = torque * invH;
}

static m3World* ResolveJoint(m3JointId jointId, int32_t* outSlot)
{
    m3World* world = m3WorldFromIndex0(jointId.world0);
    if (world == NULL)
    {
        return NULL;
    }
    int32_t slot = m3JointSlot(world, jointId);
    if (slot < 0)
    {
        return NULL;
    }
    *outSlot = slot;
    return world;
}

void m3Joint_SetLimits(m3JointId jointId, bool enable, float lower, float upper)
{
    int32_t slot;
    m3World* world = ResolveJoint(jointId, &slot);
    if (world == NULL || !m3FiniteF(lower) || !m3FiniteF(upper))
    {
        return;
    }
    if (world->jointType[slot] == (uint8_t)m3_motorJoint)
    {
        // The servo budgets (16-5): lower = maxForce, upper =
        // maxTorque, independent allowances rather than a range, so
        // the range order rule does not apply but negatives do.
        if (lower < 0.0f || upper < 0.0f)
        {
            return;
        }
    }
    else if (lower > upper)
    {
        return;
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m3JointId id;
            int32_t enable;
            float a;
            float b;
        } record;
        memset(&record, 0, sizeof(record));
        record.id = jointId;
        record.enable = enable ? 1 : 0;
        record.a = lower;
        record.b = upper;
        m3JournalRecord(world, m3_opJointSetLimits, &record, (int32_t)sizeof(record));
    }
    m3JointSetLimitsInternal(world, slot, enable ? 1 : 0, lower, upper);
}

void m3Joint_SetSteer(m3JointId jointId, bool enable, float targetAngle, float hertz, float zeta,
                      float maxEffort)
{
    int32_t slot;
    m3World* world = ResolveJoint(jointId, &slot);
    if (world == NULL || world->jointType[slot] != (uint8_t)m3_wheelJoint ||
        !m3FiniteF(targetAngle) || m3AbsF(targetAngle) > 1.0f || !m3FiniteF(hertz) ||
        !m3FiniteF(zeta) || !m3FiniteF(maxEffort) || zeta < 0.0f || maxEffort < 0.0f ||
        (enable && !(hertz > 0.0f)))
    {
        return; // steering is a wheel contract, refused loudly
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m3JointId id;
            int32_t enable;
            float target;
            float hertz;
            float zeta;
            float effort;
        } record;
        memset(&record, 0, sizeof(record));
        record.id = jointId;
        record.enable = enable ? 1 : 0;
        record.target = targetAngle;
        record.hertz = hertz;
        record.zeta = zeta;
        record.effort = maxEffort;
        m3JournalRecord(world, m3_opJointSetSteer, &record, (int32_t)sizeof(record));
    }
    m3JointSetSteerInternal(world, slot, enable ? 1 : 0, targetAngle, hertz, zeta, maxEffort);
}

float m3Joint_GetSteerAngle(m3JointId jointId)
{
    int32_t slot;
    m3World* world = ResolveJoint(jointId, &slot);
    if (world == NULL || world->jointType[slot] != (uint8_t)m3_wheelJoint)
    {
        return 0.0f;
    }
    int32_t bodyA = world->jointBodyA[slot];
    int32_t bodyB = world->jointBodyB[slot];
    m3Quat quatA = m3MulQuat(world->transforms[bodyA].q, world->jointFrameQA[slot]);
    m3Quat quatB = m3MulQuat(world->transforms[bodyB].q, world->jointFrameQB[slot]);
    if (quatA.x * quatB.x + quatA.y * quatB.y + quatA.z * quatB.z + quatA.w * quatB.w < 0.0f)
    {
        quatB = (m3Quat){-quatB.x, -quatB.y, -quatB.z, -quatB.w};
    }
    m3Quat conjA = {-quatA.x, -quatA.y, -quatA.z, quatA.w};
    m3Quat relQ = m3MulQuat(conjA, quatB);
    return 2.0f * m3Atan2(relQ.x, relQ.w);
}

void m3Joint_SetMotorPose(m3JointId jointId, m3Vec3 offset, m3Quat rotation)
{
    int32_t slot;
    m3World* world = ResolveJoint(jointId, &slot);
    m3real q2 = rotation.x * rotation.x + rotation.y * rotation.y + rotation.z * rotation.z +
                rotation.w * rotation.w;
    if (world == NULL || world->jointType[slot] != (uint8_t)m3_motorJoint || !m3FiniteV3(offset) ||
        !m3FiniteQuat(rotation) || q2 < 0.81f || q2 > 1.21f)
    {
        return; // the servo aim is a motor-joint contract
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m3JointId id;
            m3Vec3 offset;
            m3Quat rotation;
        } record;
        memset(&record, 0, sizeof(record));
        record.id = jointId;
        record.offset = offset;
        record.rotation = rotation;
        m3JournalRecord(world, m3_opJointSetMotorPose, &record, (int32_t)sizeof(record));
    }
    m3JointSetMotorPoseInternal(world, slot, offset, rotation);
}

void m3Joint_SetMotor(m3JointId jointId, bool enable, float speed, float maxEffort)
{
    int32_t slot;
    m3World* world = ResolveJoint(jointId, &slot);
    if (world == NULL || !m3FiniteF(speed) || !m3FiniteF(maxEffort) || maxEffort < 0.0f)
    {
        return;
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m3JointId id;
            int32_t enable;
            float a;
            float b;
        } record;
        memset(&record, 0, sizeof(record));
        record.id = jointId;
        record.enable = enable ? 1 : 0;
        record.a = speed;
        record.b = maxEffort;
        m3JournalRecord(world, m3_opJointSetMotor, &record, (int32_t)sizeof(record));
    }
    m3JointSetMotorInternal(world, slot, enable ? 1 : 0, speed, maxEffort);
}

void m3Joint_SetCollideConnected(m3JointId jointId, bool collide)
{
    int32_t slot;
    m3World* world = ResolveJoint(jointId, &slot);
    if (world == NULL)
    {
        return;
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m3JointId id;
            int32_t on;
        } record;
        memset(&record, 0, sizeof(record));
        record.id = jointId;
        record.on = collide ? 1 : 0;
        m3JournalRecord(world, m3_opJointSetCollide, &record, (int32_t)sizeof(record));
    }
    m3JointSetCollideInternal(world, slot, collide ? 1 : 0);
}

bool m3Joint_GetCollideConnected(m3JointId jointId)
{
    int32_t slot;
    m3World* world = ResolveJoint(jointId, &slot);
    return world != NULL && world->jointCollide[slot] != 0;
}

void m3Joint_SetBreakThresholds(m3JointId jointId, float maxForce, float maxTorque)
{
    int32_t slot;
    m3World* world = ResolveJoint(jointId, &slot);
    if (world == NULL || !m3FiniteF(maxForce) || maxForce < 0.0f || !m3FiniteF(maxTorque) ||
        maxTorque < 0.0f)
    {
        return;
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m3JointId id;
            float maxForce;
            float maxTorque;
        } record;
        memset(&record, 0, sizeof(record));
        record.id = jointId;
        record.maxForce = maxForce;
        record.maxTorque = maxTorque;
        m3JournalRecord(world, m3_opJointSetBreak, &record, (int32_t)sizeof(record));
    }
    m3JointSetBreakInternal(world, slot, maxForce, maxTorque);
}

m3real m3Joint_GetConstraintForce(m3JointId jointId)
{
    int32_t slot;
    m3World* world = ResolveJoint(jointId, &slot);
    if (world == NULL || world->lastInvH == 0.0f)
    {
        return 0.0f;
    }
    m3real force;
    m3real torque;
    m3JointReactionMagnitudes(world, slot, world->lastInvH, &force, &torque);
    return force;
}

m3real m3Joint_GetConstraintTorque(m3JointId jointId)
{
    int32_t slot;
    m3World* world = ResolveJoint(jointId, &slot);
    if (world == NULL || world->lastInvH == 0.0f)
    {
        return 0.0f;
    }
    m3real force;
    m3real torque;
    m3JointReactionMagnitudes(world, slot, world->lastInvH, &force, &torque);
    return torque;
}

m3real m3Joint_GetAngle(m3JointId jointId)
{
    int32_t slot;
    m3World* world = ResolveJoint(jointId, &slot);
    if (world == NULL || (world->jointType[slot] != (uint8_t)m3_revoluteJoint &&
                          world->jointType[slot] != (uint8_t)m3_wheelJoint))
    {
        return 0.0f; // the wheel's frame z is its axle: same twist read
    }
    m3Quat qA = m3MulQuat(world->transforms[world->jointBodyA[slot]].q, world->jointFrameQA[slot]);
    m3Quat qB = m3MulQuat(world->transforms[world->jointBodyB[slot]].q, world->jointFrameQB[slot]);
    m3Quat conjA = {-qA.x, -qA.y, -qA.z, qA.w};
    m3Quat relQ = m3MulQuat(conjA, qB);
    m3real twist = relQ.w < 0.0f ? m3Atan2(-relQ.z, -relQ.w) : m3Atan2(relQ.z, relQ.w);
    return 2.0f * twist;
}

m3real m3Joint_GetTranslation(m3JointId jointId)
{
    int32_t slot;
    m3World* world = ResolveJoint(jointId, &slot);
    if (world == NULL || (world->jointType[slot] != (uint8_t)m3_prismaticJoint &&
                          world->jointType[slot] != (uint8_t)m3_wheelJoint))
    {
        return 0.0f;
    }
    int32_t bodyA = world->jointBodyA[slot];
    int32_t bodyB = world->jointBodyB[slot];
    const m3Transform* xfA = &world->transforms[bodyA];
    const m3Transform* xfB = &world->transforms[bodyB];
    m3Vec3 pA = m3RotateVec3(xfA->q, world->jointLocalA[slot]);
    m3Vec3 pB = m3RotateVec3(xfB->q, world->jointLocalB[slot]);
    m3Vec3 d = {(m3real)(xfB->p.x + (double)pB.x - xfA->p.x - (double)pA.x),
                (m3real)(xfB->p.y + (double)pB.y - xfA->p.y - (double)pA.y),
                (m3real)(xfB->p.z + (double)pB.z - xfA->p.z - (double)pA.z)};
    m3Quat frameQ = m3MulQuat(xfA->q, world->jointFrameQA[slot]);
    // The slide axis is frame z for the prismatic and frame x for
    // the wheel (whose z is the axle).
    m3Vec3 local = world->jointType[slot] == (uint8_t)m3_wheelJoint ? (m3Vec3){1.0f, 0.0f, 0.0f}
                                                                    : (m3Vec3){0.0f, 0.0f, 1.0f};
    m3Vec3 axis = m3RotateVec3(frameQ, local);
    return m3Dot3(d, axis);
}

// --- Position drive (8-6b) --------------------------------------------------

void m3JointSetSpringInternal(m3World* world, int32_t j, int32_t enable, float hertz, float zeta)
{
    world->jointFlags[j] = (uint8_t)((world->jointFlags[j] & ~8u) | (enable != 0 ? 8u : 0u));
    world->jointSpring[j] = (m3Vec3){hertz, zeta, 0.0f};
    // Any change invalidates the stored spring impulse: a stale
    // warm start toward an old target kicks.
    world->jointSpringImpulse[j] = (m3Vec3){0.0f, 0.0f, 0.0f};
    JointWakeBodies(world, j);
}

void m3JointSetTargetInternal(m3World* world, int32_t j, float scalar, m3Quat q)
{
    world->jointTargetScalar[j] = scalar;
    world->jointTargetQ[j] = q;
    JointWakeBodies(world, j);
}

static int JointTypeDrives(uint8_t type)
{
    return type == (uint8_t)m3_revoluteJoint || type == (uint8_t)m3_prismaticJoint ||
           type == (uint8_t)m3_sphericalJoint || type == (uint8_t)m3_wheelJoint ||
           type == (uint8_t)m3_motorJoint;
}

void m3Joint_SetSpring(m3JointId jointId, bool enable, float hertz, float dampingRatio)
{
    int32_t slot;
    m3World* world = ResolveJoint(jointId, &slot);
    if (world == NULL || !JointTypeDrives(world->jointType[slot]) || !m3FiniteF(hertz) ||
        hertz <= 0.0f || !m3FiniteF(dampingRatio) || dampingRatio < 0.0f)
    {
        return;
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m3JointId id;
            int32_t enable;
            float hertz;
            float zeta;
        } record;
        memset(&record, 0, sizeof(record));
        record.id = jointId;
        record.enable = enable ? 1 : 0;
        record.hertz = hertz;
        record.zeta = dampingRatio;
        m3JournalRecord(world, m3_opJointSetSpring, &record, (int32_t)sizeof(record));
    }
    m3JointSetSpringInternal(world, slot, enable ? 1 : 0, hertz, dampingRatio);
}

static void JointTargetOp(m3World* world, m3JointId jointId, int32_t slot, float scalar, m3Quat q)
{
    if (world->journalActive != 0)
    {
        struct
        {
            m3JointId id;
            float scalar;
            m3Quat q;
        } record;
        memset(&record, 0, sizeof(record));
        record.id = jointId;
        record.scalar = scalar;
        record.q = q;
        m3JournalRecord(world, m3_opJointSetTarget, &record, (int32_t)sizeof(record));
    }
    m3JointSetTargetInternal(world, slot, scalar, q);
}

void m3Joint_SetTargetAngle(m3JointId jointId, float radians)
{
    int32_t slot;
    m3World* world = ResolveJoint(jointId, &slot);
    if (world == NULL || world->jointType[slot] != (uint8_t)m3_revoluteJoint ||
        !m3FiniteF(radians) || radians < -M3_PI || radians > M3_PI)
    {
        return;
    }
    JointTargetOp(world, jointId, slot, radians, (m3Quat){0.0f, 0.0f, 0.0f, 1.0f});
}

void m3Joint_SetTargetTranslation(m3JointId jointId, float meters)
{
    int32_t slot;
    m3World* world = ResolveJoint(jointId, &slot);
    if (world == NULL ||
        (world->jointType[slot] != (uint8_t)m3_prismaticJoint &&
         world->jointType[slot] != (uint8_t)m3_wheelJoint) ||
        !m3FiniteF(meters))
    {
        return; // the wheel's drive is its suspension spring (12-2)
    }
    JointTargetOp(world, jointId, slot, meters, (m3Quat){0.0f, 0.0f, 0.0f, 1.0f});
}

void m3Joint_SetTargetRotation(m3JointId jointId, m3Quat target)
{
    int32_t slot;
    m3World* world = ResolveJoint(jointId, &slot);
    if (world == NULL || world->jointType[slot] != (uint8_t)m3_sphericalJoint ||
        !m3FiniteQuat(target))
    {
        return;
    }
    m3real len2 =
        target.x * target.x + target.y * target.y + target.z * target.z + target.w * target.w;
    if (len2 < 0.99f || len2 > 1.01f)
    {
        return; // not a unit rotation: refuse by doing nothing
    }
    JointTargetOp(world, jointId, slot, 0.0f, target);
}
