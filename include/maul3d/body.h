// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen

#ifndef MAUL3D_BODY_H
#define MAUL3D_BODY_H

#include "maul3d/world.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum m3BodyType
    {
        m3_staticBody = 0,
        m3_kinematicBody = 1, // moved by velocity, immovable by contact
        m3_dynamicBody = 2,
    } m3BodyType;

    typedef struct m3BodyDef
    {
        int32_t type;           // m3BodyType
        m3Pos3 position;        // world space, double (hybrid precision)
        m3Quat rotation;        // unit quaternion
        m3Vec3 linearVelocity;  // m/s
        m3Vec3 angularVelocity; // rad/s, world frame
        float gravityScale;
        float linearDamping;
        float angularDamping;
        uint64_t userData; // opaque, carried verbatim
        /// A high speed body that gets the full continuous pass
        /// against static AND dynamic targets (2b-8). Every fast
        /// dynamic body already sweeps against statics; the bullet
        /// flag buys the dynamic-target sweep. Bullet versus bullet
        /// is not resolved (the reference limitation, documented).
        bool isBullet;
        int32_t internalValue;
    } m3BodyDef;

    /// Returns a def with pinned defaults (identity rotation, gravity
    /// scale one) and a valid cookie.
    M3_API m3BodyDef m3DefaultBodyDef(void);

    /// Create a body. Returns the null id on an invalid def, a stale
    /// world, or an exhausted body pool (loud in debug builds). A
    /// shapeless dynamic body has unit mass and zero inertia until a
    /// shape provides the real values (task 7).
    M3_API m3BodyId m3CreateBody(m3WorldId worldId, const m3BodyDef* def);

    /// Destroy a body. The id goes stale; the slot recycles FIFO with a
    /// generation bump and retires instead of wrapping.
    M3_API void m3DestroyBody(m3BodyId bodyId);

    M3_API bool m3Body_IsValid(m3BodyId bodyId);
    M3_API m3Pos3 m3Body_GetPosition(m3BodyId bodyId);
    M3_API m3Quat m3Body_GetRotation(m3BodyId bodyId);
    M3_API m3Vec3 m3Body_GetLinearVelocity(m3BodyId bodyId);
    M3_API m3Vec3 m3Body_GetAngularVelocity(m3BodyId bodyId);
    M3_API uint64_t m3Body_GetUserData(m3BodyId bodyId);
    M3_API m3BodyType m3Body_GetType(m3BodyId bodyId);

    /// Journaled setters: every mutation is a discrete op.
    /// Forces and impulses (8-2), journaled like every mutation.
    /// Forces and torques ACCUMULATE and act over the next step,
    /// then clear; impulses change velocity immediately. Only
    /// awake-able dynamic bodies respond: static and kinematic
    /// targets and non-finite values are documented no-ops. A
    /// nonzero application wakes the body. Points are world-space;
    /// an off-center application adds the matching angular part.
    M3_API void m3Body_ApplyForce(m3BodyId bodyId, m3Vec3 force);
    M3_API void m3Body_ApplyTorque(m3BodyId bodyId, m3Vec3 torque);
    M3_API void m3Body_ApplyLinearImpulse(m3BodyId bodyId, m3Vec3 impulse);
    M3_API void m3Body_ApplyAngularImpulse(m3BodyId bodyId, m3Vec3 impulse);
    M3_API void m3Body_ApplyForceAtPoint(m3BodyId bodyId, m3Vec3 force, m3Pos3 point);
    M3_API void m3Body_ApplyLinearImpulseAtPoint(m3BodyId bodyId, m3Vec3 impulse, m3Pos3 point);

    M3_API void m3Body_SetLinearVelocity(m3BodyId bodyId, m3Vec3 velocity);
    M3_API void m3Body_SetAngularVelocity(m3BodyId bodyId, m3Vec3 velocity);

    static const m3BodyId m3_nullBodyId = {0, 0, 0};

#ifdef __cplusplus
}
#endif

#endif // MAUL3D_BODY_H
