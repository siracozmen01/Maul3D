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
        m3_dynamicBody = 2, // kinematic (1) arrives in phase 2b
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
    M3_API void m3Body_SetLinearVelocity(m3BodyId bodyId, m3Vec3 velocity);
    M3_API void m3Body_SetAngularVelocity(m3BodyId bodyId, m3Vec3 velocity);

    static const m3BodyId m3_nullBodyId = {0, 0, 0};

#ifdef __cplusplus
}
#endif

#endif // MAUL3D_BODY_H
