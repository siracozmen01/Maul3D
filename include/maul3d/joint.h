// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Joints (2c-2): articulation with snapshot-riding warm starts. The
// first joint is the spherical (ball) point constraint; hinges,
// sliders, and shoulder limits follow in their own slices.

#ifndef MAUL3D_JOINT_H
#define MAUL3D_JOINT_H

#include "body.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum m3JointType
    {
        m3_sphericalJoint = 0, // ball: pins two body-frame points together
        m3_revoluteJoint = 1,  // hinge: one rotation axis, limits, motor
        m3_prismaticJoint = 2, // slider: one translation axis, limits, motor
        m3_fixedJoint = 3,     // weld: full 6-DOF lock at the create pose
        m3_distanceJoint = 4,  // rope/rod: anchor distance in [lower, upper]
        m3_genericJoint = 5,   // 6-DOF: per-axis lock/free/limit + one motor
    } m3JointType;

    /// Per-axis behavior of the generic joint, in the joint frame
    /// built from localAxisA/localAxisB (the frame's z axis).
    typedef enum m3AxisMode
    {
        m3_axisLocked = 0,
        m3_axisFree = 1,
        m3_axisLimited = 2,
    } m3AxisMode;

    /// Build with m3DefaultJointDef; hand-rolled defs are rejected
    /// loudly. Anchors are body-frame points on each body.
    typedef struct m3JointDef
    {
        int32_t type; // m3JointType
        m3BodyId bodyA;
        m3BodyId bodyB;
        m3Vec3 localAnchorA;
        m3Vec3 localAnchorB;
        /// Hinge axis per body frame (unit; revolute only). The two
        /// axes must map to the same world direction at create time
        /// for a well-posed hinge.
        m3Vec3 localAxisA;
        m3Vec3 localAxisB;
        /// Limits and motor share names across joint types: for the
        /// revolute they are angles (radians) and a torque cap, for
        /// the prismatic translations (meters) and a force cap.
        bool enableLimit;
        m3real lowerLimit;
        m3real upperLimit;
        bool enableMotor;
        m3real motorSpeed; // rad/s or m/s by joint type
        m3real maxMotorEffort;
        /// Spherical only: the swing cone (the shoulder). enableLimit
        /// doubles as the twist range for the spherical, using
        /// lowerLimit and upperLimit as twist angles.
        bool enableCone;
        m3real coneAngle; // radians from the A frame's z-axis
        /// Distance joints (4-2) REQUIRE enableLimit with
        /// 0 <= lowerLimit <= upperLimit meters (a rod is lower ==
        /// upper; a rope is a range). Their optional spring reuses
        /// the motor fields, documented reuse: enableMotor turns it
        /// on, motorSpeed is the spring hertz (> 0), maxMotorEffort
        /// is the damping ratio (>= 0); coneAngle (a further
        /// documented reuse) is the spring's rest length when
        /// positive and must sit inside [lower, upper]; when zero,
        /// the rest length is the upper limit. Fixed joints ignore axes
        /// and limits entirely: they weld the create-time pose.
        /// Jointed bodies do not collide with each other unless this
        /// is set (the classic chain-fight guard).
        bool collideConnected;
        /// The generic joint (4-3). Modes are m3AxisMode per joint
        /// frame axis; limits are meters (linear) and radians
        /// (angular), used only where the mode is limited. One
        /// motor: genericMotorAxis picks 0..2 (linear) or 3..5
        /// (angular), 255 for none; it drives motorSpeed with the
        /// maxMotorEffort cap on a free or limited axis. Angular
        /// limits come with a v1 contract: at most ONE angular axis
        /// may be limited, and the other two must be BOTH locked
        /// (a hinge with range) or BOTH free (a twist-limited
        /// ball); anything else refuses loudly. Growing this def
        /// bumps the cookie: stale-compiled callers are refused
        /// loudly instead of misread (the freeze's mechanism).
        uint8_t genericLinear[3];
        uint8_t genericAngular[3];
        uint8_t genericMotorAxis;
        uint8_t genericReserved;
        m3real genericLinearLower[3];
        m3real genericLinearUpper[3];
        m3real genericAngularLower[3];
        m3real genericAngularUpper[3];
        int32_t internalValue;
    } m3JointDef;

    /// The pinned joint defaults: spherical type, zero anchors, unit
    /// z axes, everything disabled, collideConnected off, and the
    /// def cookie every create demands.
    M3_API m3JointDef m3DefaultJointDef(void);

    /// Create a joint between two distinct bodies of the same world
    /// (at least one dynamic). Returns the null id on a bad def, a
    /// stale body, or an exhausted pool. Journaled; replay verifies
    /// the minted id. Destroying either body destroys the joint.
    M3_API m3JointId m3CreateJoint(const m3JointDef* def);
    M3_API void m3DestroyJoint(m3JointId jointId);
    M3_API bool m3Joint_IsValid(m3JointId jointId);

    /// Runtime joint control (8-6a). All journaled; both bodies wake
    /// on any change. Limits and motor reuse the def semantics per
    /// type (angles for revolute and spherical twist, meters for
    /// prismatic and distance); toggling zeroes the row's stored
    /// impulse so a stale warm start cannot kick.
    M3_API void m3Joint_SetLimits(m3JointId jointId, bool enable, float lower, float upper);
    M3_API void m3Joint_SetMotor(m3JointId jointId, bool enable, float speed, float maxEffort);

    /// Let (or forbid) the two jointed bodies collide with each
    /// other. Journaled; binds at the next step's pair scan.
    M3_API void m3Joint_SetCollideConnected(m3JointId jointId, bool collide);
    M3_API bool m3Joint_GetCollideConnected(m3JointId jointId);

    /// Breakage (8-6a), a deliberate addition over the reference:
    /// rollback games need breakage as a deterministic in-step
    /// state transition, not a host poll racing the journal. When
    /// either reaction magnitude exceeds its cap at the end of a
    /// step, the joint destroys itself and emits the joint break
    /// event (see world.h). Zero disables a cap; both zero (the
    /// default) means unbreakable.
    M3_API void m3Joint_SetBreakThresholds(m3JointId jointId, float maxForce, float maxTorque);

    /// Reaction readback (8-6a): MAGNITUDES of the last step's
    /// constraint reactions, assembled per type from the stored
    /// solver rows (linear rows into force, angular rows into
    /// torque; the generic joint reports a conservative sum). Reads
    /// 0 before the first step after a restore (documented
    /// transient). The reference's vector form waits for a consumer
    /// with a direction to point at (argued in the plan).
    M3_API m3real m3Joint_GetConstraintForce(m3JointId jointId);
    M3_API m3real m3Joint_GetConstraintTorque(m3JointId jointId);

    /// Geometry reads: the revolute twist angle about the hinge
    /// (radians) and the prismatic translation along the slide axis
    /// (meters). Wrong-type calls read 0.
    M3_API m3real m3Joint_GetAngle(m3JointId jointId);
    M3_API m3real m3Joint_GetTranslation(m3JointId jointId);

    /// Position drive (8-6b), the reference spring rows: a soft
    /// constraint with the given frequency and damping ratio pulls
    /// the joint toward its target. Revolute drives the hinge angle,
    /// prismatic the translation, spherical the relative rotation;
    /// other types refuse. Toggling zeroes the spring's stored
    /// impulse. All journaled; both bodies wake.
    M3_API void m3Joint_SetSpring(m3JointId jointId, bool enable, float hertz, float dampingRatio);
    M3_API void m3Joint_SetTargetAngle(m3JointId jointId, float radians);
    M3_API void m3Joint_SetTargetTranslation(m3JointId jointId, float meters);
    /// The target must be a unit rotation; garbage refuses loudly by
    /// doing nothing. The target lives in the JOINT FRAMES (frame z
    /// is the create-time local axis): it is the desired rotation of
    /// frame B relative to frame A, the reference semantic. Pick
    /// your axes at create time so the frame reads naturally.
    M3_API void m3Joint_SetTargetRotation(m3JointId jointId, m3Quat target);

#ifdef __cplusplus
}
#endif

#endif
