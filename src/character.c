// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The character controller core (4-4): collide-and-slide over the
// engine's own convex casts, deterministic front to back. The
// character owns a zero-velocity kinematic capsule body: the solver
// never moves it, dynamics collide with it, and every displacement
// is a journaled command that replays and rolls back like any other
// mutation.

#include "maul3d/character.h"

#include "world_internal.h"

#include <string.h>

#define M3_CHARACTER_COOKIE ((int32_t)(M3_COOKIE ^ ((int32_t)sizeof(m3CharacterDef) << 8) ^ 7))
#define M3_CHARACTER_SLIDE_ITERATIONS 4

m3CharacterDef m3DefaultCharacterDef(void)
{
    m3CharacterDef def;
    memset(&def, 0, sizeof(def));
    def.radius = 0.4f;
    def.halfHeight = 0.5f;
    def.maxSlopeAngle = 0.8f; // ~46 degrees
    def.snapDistance = 0.3f;
    def.skin = 0.02f;
    def.internalValue = M3_CHARACTER_COOKIE;
    return def;
}

int32_t m3CharacterSlot(const m3World* world, m3CharacterId characterId)
{
    int32_t index = characterId.index1 - 1;
    if (world == NULL || characterId.world0 != world->worldIndex0 ||
        !m3IdPoolValid(&world->charPool, index, characterId.generation))
    {
        return -1;
    }
    return index;
}

int32_t m3CreateCharacterInternal(m3World* world, const m3CharacterDef* def)
{
    int32_t slot = m3IdPoolAlloc(&world->charPool);
    if (slot < 0)
    {
        return -1;
    }
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_kinematicBody;
    bd.position = def->position;
    int32_t body = m3CreateBodyInternal(world, &bd);
    if (body < 0)
    {
        m3IdPoolFree(&world->charPool, slot);
        return -1;
    }
    m3ShapeDef sd = m3DefaultShapeDef();
    m3ShapeGeom geom;
    memset(&geom, 0, sizeof(geom));
    geom.v = (m3Vec3){0.0f, def->halfHeight, 0.0f};
    geom.v2 = (m3Vec3){0.0f, -def->halfHeight, 0.0f};
    geom.s = def->radius;
    int32_t shape =
        m3CreateShapeInternal(world, body, (uint8_t)m3_capsuleShape, &geom, &sd, NULL, NULL, NULL);
    if (shape < 0)
    {
        m3DestroyBodyInternal(world, body);
        m3IdPoolFree(&world->charPool, slot);
        return -1;
    }
    world->charBody[slot] = body;
    world->charRadius[slot] = def->radius;
    world->charHalfHeight[slot] = def->halfHeight;
    world->charCosSlope[slot] = cosf(def->maxSlopeAngle);
    world->charSnap[slot] = def->snapDistance;
    world->charSkin[slot] = def->skin;
    world->charGrounded[slot] = 0;
    world->charGroundNormal[slot] = (m3Vec3){0.0f, 0.0f, 0.0f};
    return slot;
}

void m3DestroyCharacterInternal(m3World* world, int32_t slot)
{
    int32_t body = world->charBody[slot];
    if (body >= 0 && world->bodyPool.alive[body] != 0)
    {
        m3DestroyBodyInternal(world, body);
    }
    world->charBody[slot] = -1;
    world->charRadius[slot] = 0.0f;
    world->charHalfHeight[slot] = 0.0f;
    world->charCosSlope[slot] = 0.0f;
    world->charSnap[slot] = 0.0f;
    world->charSkin[slot] = 0.0f;
    world->charGrounded[slot] = 0;
    world->charGroundNormal[slot] = (m3Vec3){0.0f, 0.0f, 0.0f};
    m3IdPoolFree(&world->charPool, slot);
}

// One capsule cast from the character's center, excluding its own
// body (the internal caster's ignore hook exists for exactly this).
static m3RayHit CharacterCast(m3World* world, int32_t slot, m3Pos3 center, m3Vec3 translation)
{
    m3Vec3 points[2] = {{0.0f, world->charHalfHeight[slot], 0.0f},
                        {0.0f, -world->charHalfHeight[slot], 0.0f}};
    return m3CastConvexClosestEx(world, center, points, 2, world->charRadius[slot], translation,
                                 world->charBody[slot]);
}

void m3CharacterMoveInternal(m3World* world, int32_t slot, m3Vec3 translation)
{
    int32_t body = world->charBody[slot];
    m3Pos3 pos = world->transforms[body].p;
    m3Vec3 remaining = translation;
    m3real skin = world->charSkin[slot];
    m3real cosSlope = world->charCosSlope[slot];
    int wasDescending = translation.y < 0.0f;
    world->charGrounded[slot] = 0;
    world->charGroundNormal[slot] = (m3Vec3){0.0f, 0.0f, 0.0f};

    for (int32_t iteration = 0; iteration < M3_CHARACTER_SLIDE_ITERATIONS; ++iteration)
    {
        m3real len2 = m3Dot3(remaining, remaining);
        if (len2 < 1.0e-10f)
        {
            break;
        }
        m3RayHit hit = CharacterCast(world, slot, pos, remaining);
        if (!hit.hit)
        {
            pos.x += (double)remaining.x;
            pos.y += (double)remaining.y;
            pos.z += (double)remaining.z;
            break;
        }
        // Advance to the hit, keeping the skin along the direction
        // of travel (never park flush: the next cast would start
        // overlapped and report zero forever). The skin retreat is
        // NOT lost: it returns to the slide budget, or a hundred
        // grounded ticks would each quietly eat a skin of distance.
        m3real len = sqrtf(len2);
        m3real advance = hit.fraction * len - skin;
        if (advance < 0.0f)
        {
            advance = 0.0f;
        }
        m3real inv = 1.0f / len;
        pos.x += (double)(remaining.x * inv * advance);
        pos.y += (double)(remaining.y * inv * advance);
        pos.z += (double)(remaining.z * inv * advance);
        m3Vec3 n = hit.normal;
        if (n.y >= cosSlope)
        {
            world->charGrounded[slot] = 1;
            world->charGroundNormal[slot] = n;
        }
        // Slide: everything not actually traveled stays in the
        // budget, minus its component into the surface.
        m3Vec3 leftover = m3MulSV3(1.0f - advance * inv, remaining);
        remaining = m3Sub3(leftover, m3MulSV3(m3Dot3(leftover, n), n));
    }

    // Snap to ground: a descending character glues to walkable
    // floor within snapDistance (stairs and edges stay under foot);
    // no floor in reach means airborne, honestly.
    if (wasDescending || world->charGrounded[slot] != 0)
    {
        m3RayHit down =
            CharacterCast(world, slot, pos, (m3Vec3){0.0f, -world->charSnap[slot], 0.0f});
        if (down.hit && down.normal.y >= cosSlope)
        {
            m3real drop = down.fraction * world->charSnap[slot] - skin;
            if (drop > 0.0f)
            {
                pos.y -= (double)drop;
            }
            world->charGrounded[slot] = 1;
            world->charGroundNormal[slot] = down.normal;
        }
        else if (!down.hit)
        {
            world->charGrounded[slot] = 0;
            world->charGroundNormal[slot] = (m3Vec3){0.0f, 0.0f, 0.0f};
        }
    }

    // Write the kinematic body's new pose (rotation stays identity:
    // characters translate; facing is the host's render concern).
    world->transforms[body].p = pos;
    world->sleepTimes[body] = 0.0f;
}

m3CharacterId m3CreateCharacter(m3WorldId worldId, const m3CharacterDef* def)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || def == NULL || def->internalValue != M3_CHARACTER_COOKIE ||
        !m3FinitePos3(def->position) || !m3FiniteF(def->radius) || !(def->radius > 0.0f) ||
        !m3FiniteF(def->halfHeight) || def->halfHeight < 0.0f || !m3FiniteF(def->maxSlopeAngle) ||
        def->maxSlopeAngle <= 0.0f || def->maxSlopeAngle > 1.5f || !m3FiniteF(def->snapDistance) ||
        def->snapDistance < 0.0f || !m3FiniteF(def->skin) || !(def->skin > 0.0f))
    {
        return m3_nullCharacterId;
    }
    int32_t slot = m3CreateCharacterInternal(world, def);
    if (slot < 0)
    {
        return m3_nullCharacterId;
    }
    m3CharacterId id = {slot + 1, world->worldIndex0, world->charPool.generations[slot]};
    if (world->journalActive != 0)
    {
        struct
        {
            m3CharacterDef def;
            m3CharacterId expected;
        } record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.expected = id;
        m3JournalRecord(world, m3_opCreateCharacter, &record, (int32_t)sizeof(record));
    }
    return id;
}

void m3DestroyCharacter(m3CharacterId characterId)
{
    m3World* world = m3WorldFromIndex0(characterId.world0);
    int32_t slot = world != NULL ? m3CharacterSlot(world, characterId) : -1;
    if (slot < 0)
    {
        return; // stale: the quiet destroy contract
    }
    if (world->journalActive != 0)
    {
        m3JournalRecord(world, m3_opDestroyCharacter, &characterId, (int32_t)sizeof(characterId));
    }
    m3DestroyCharacterInternal(world, slot);
}

bool m3Character_IsValid(m3CharacterId characterId)
{
    m3World* world = m3WorldFromIndex0(characterId.world0);
    return world != NULL && m3CharacterSlot(world, characterId) >= 0;
}

void m3Character_Move(m3CharacterId characterId, m3Vec3 translation)
{
    m3World* world = m3WorldFromIndex0(characterId.world0);
    int32_t slot = world != NULL ? m3CharacterSlot(world, characterId) : -1;
    if (slot < 0 || !m3FiniteV3(translation))
    {
        return; // stale id or hostile move: a documented no-op
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m3CharacterId id;
            m3Vec3 translation;
        } record;
        memset(&record, 0, sizeof(record));
        record.id = characterId;
        record.translation = translation;
        m3JournalRecord(world, m3_opCharacterMove, &record, (int32_t)sizeof(record));
    }
    m3CharacterMoveInternal(world, slot, translation);
}

m3Pos3 m3Character_GetPosition(m3CharacterId characterId)
{
    m3World* world = m3WorldFromIndex0(characterId.world0);
    int32_t slot = world != NULL ? m3CharacterSlot(world, characterId) : -1;
    if (slot < 0)
    {
        return (m3Pos3){0.0, 0.0, 0.0};
    }
    return world->transforms[world->charBody[slot]].p;
}

bool m3Character_IsGrounded(m3CharacterId characterId)
{
    m3World* world = m3WorldFromIndex0(characterId.world0);
    int32_t slot = world != NULL ? m3CharacterSlot(world, characterId) : -1;
    return slot >= 0 && world->charGrounded[slot] != 0;
}

m3Vec3 m3Character_GetGroundNormal(m3CharacterId characterId)
{
    m3World* world = m3WorldFromIndex0(characterId.world0);
    int32_t slot = world != NULL ? m3CharacterSlot(world, characterId) : -1;
    if (slot < 0)
    {
        return (m3Vec3){0.0f, 0.0f, 0.0f};
    }
    return world->charGroundNormal[slot];
}
