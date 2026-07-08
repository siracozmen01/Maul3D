// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The character controller (4-4): a kinematic capsule driven by
// collide-and-slide over the engine's convex casts. Never a dynamic
// body: controllers want exactness, not bounce. The character's
// body has ZERO velocity by contract and never moves during
// m3World_Step; every displacement comes from m3Character_Move,
// which is a journaled, deterministic, rollback-covered command
// (gravity is the host's job: add it to the move each tick).

#ifndef MAUL3D_CHARACTER_H
#define MAUL3D_CHARACTER_H

#include "world.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct m3CharacterId
    {
        int32_t index1;
        uint16_t world0;
        uint16_t generation;
    } m3CharacterId;

    typedef struct m3CharacterDef
    {
        m3Pos3 position; // the capsule CENTER at spawn
        m3real radius;
        m3real halfHeight;    // segment half length (total height =
                              // 2 * (halfHeight + radius))
        m3real maxSlopeAngle; // radians from horizontal; steeper
                              // ground never counts as walkable
        m3real snapDistance;  // how far down a grounded character
                              // reaches to stay glued on descents
        m3real skin;          // the standing gap casts preserve
        m3real stepHeight;    // the tallest riser a walking
                              // character mounts in one move (4-5)
        uint64_t userData;
        int32_t internalValue;
    } m3CharacterDef;

    M3_API m3CharacterDef m3DefaultCharacterDef(void);

    /// Creates the character and its internal kinematic capsule
    /// body (destroyed together). The body collides like any
    /// kinematic: dynamics bounce off it; it walks through nothing.
    M3_API m3CharacterId m3CreateCharacter(m3WorldId worldId, const m3CharacterDef* def);
    M3_API void m3DestroyCharacter(m3CharacterId characterId);
    M3_API bool m3Character_IsValid(m3CharacterId characterId);

    /// Collide-and-slide by `translation`, immediately: cast the
    /// capsule, advance to the first hit, slide the remainder along
    /// the surface, repeat (bounded), then snap to walkable ground
    /// within snapDistance when descending. Walkable is measured
    /// against maxSlopeAngle; ceilings and steep walls only ever
    /// block and slide. Journaled; hostile translations no-op.
    M3_API void m3Character_Move(m3CharacterId characterId, m3Vec3 translation);

    M3_API m3Pos3 m3Character_GetPosition(m3CharacterId characterId);
    M3_API bool m3Character_IsGrounded(m3CharacterId characterId);
    /// The last walkable surface normal (zeros while airborne).
    M3_API m3Vec3 m3Character_GetGroundNormal(m3CharacterId characterId);

    static const m3CharacterId m3_nullCharacterId = {0, 0, 0};

#ifdef __cplusplus
}
#endif

#endif
