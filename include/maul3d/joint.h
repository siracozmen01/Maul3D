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
    } m3JointType;

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
        /// Jointed bodies do not collide with each other unless this
        /// is set (the classic chain-fight guard).
        bool collideConnected;
        int32_t internalValue;
    } m3JointDef;

    M3_API m3JointDef m3DefaultJointDef(void);

    /// Create a joint between two distinct bodies of the same world
    /// (at least one dynamic). Returns the null id on a bad def, a
    /// stale body, or an exhausted pool. Journaled; replay verifies
    /// the minted id. Destroying either body destroys the joint.
    M3_API m3JointId m3CreateJoint(const m3JointDef* def);
    M3_API void m3DestroyJoint(m3JointId jointId);
    M3_API bool m3Joint_IsValid(m3JointId jointId);

#ifdef __cplusplus
}
#endif

#endif
