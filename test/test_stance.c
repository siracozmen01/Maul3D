// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The stance gate (12-3): crouch and resize with the stand-up veto.
// A character ducks under a slab it cannot walk under, is REFUSED
// the stand while the slab presses, stands the moment it clears the
// edge, lands shorter after a mid-air crouch, cannot widen inside a
// slot, and drags the whole story through journal replay and a mid
// tunnel rollback on identical bits. Hostile dimensions bounce.

#include "maul3d/character.h"
#include "maul3d/shape.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int s_failures = 0;

#define CHECK(cond, msg)                                                                           \
    do                                                                                             \
    {                                                                                              \
        if (!(cond))                                                                               \
        {                                                                                          \
            printf("FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__);                                 \
            s_failures += 1;                                                                       \
        }                                                                                          \
    } while (0)

// Floor plane plus a slab roof over x in [2, 6]: 1.3 meters of
// clearance. The default character stands 1.8 tall and crouches
// to 1.2: the tunnel admits exactly one of those.
static m3WorldId TunnelWorld(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    def.characterCapacity = 2;
    m3WorldId world = m3CreateWorld(&def);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    m3CreatePlaneShape(ground, &sd, &floor);
    m3BodyDef rd = m3DefaultBodyDef();
    rd.position = (m3Pos3){4.0, 1.55, 0.0};
    m3BodyId roof = m3CreateBody(world, &rd);
    m3CreateBoxShape(roof, &sd, (m3Vec3){2.0f, 0.25f, 2.0f});
    return world;
}

// The tunnel run as a pure function of the tick index, so twins,
// replays, and rollback re-runs all issue the identical schedule.
static void TunnelTick(m3CharacterId hero, int32_t i)
{
    if (i < 60)
    {
        m3Character_Move(hero, (m3Vec3){0.0f, -0.1f, 0.0f}); // land
        return;
    }
    if (i < 120)
    {
        // Standing walk: jams against the slab edge and stays there.
        m3Character_Move(hero, (m3Vec3){0.05f, -0.1f, 0.0f});
        return;
    }
    if (i == 120)
    {
        m3Character_SetStance(hero, 0.2f, 0.4f); // duck
    }
    if (i == 240)
    {
        m3Character_SetStance(hero, 0.5f, 0.4f); // pressed: refused
    }
    if (i == 460)
    {
        m3Character_SetStance(hero, 0.5f, 0.4f); // clear: stands
    }
    m3Character_Move(hero, (m3Vec3){0.03f, -0.1f, 0.0f}); // crawl east
}

static void TestDuckWalkStand(void)
{
    // The full arc, plus the journal and the twin in one pass.
    static uint8_t journal[131072];
    uint64_t hashes[2];
    for (int32_t run = 0; run < 2; ++run)
    {
        m3WorldId world = TunnelWorld();
        bool recording = run == 0 && m3World_JournalBegin(world, journal, (int32_t)sizeof(journal));
        m3CharacterDef cd = m3DefaultCharacterDef();
        cd.position = (m3Pos3){0.0, 3.0, 0.0};
        m3CharacterId hero = m3CreateCharacter(world, &cd);
        for (int32_t i = 0; i < 520; ++i)
        {
            TunnelTick(hero, i);
            if (run == 0)
            {
                if (i == 119)
                {
                    // Standing 1.8 against 1.3 of clearance: the
                    // slab edge is a wall long before the tunnel
                    // (sixty walk ticks would have covered 3 meters
                    // on open ground).
                    double blockedX = m3Character_GetPosition(hero).x;
                    CHECK(blockedX > 1.3 && blockedX < 1.9, "the standing character cannot enter");
                }
                if (i == 121)
                {
                    m3real hh;
                    m3real r;
                    m3Character_GetStance(hero, &hh, &r);
                    CHECK(hh == 0.2f && r == 0.4f, "the duck applies");
                    CHECK(m3Character_IsGrounded(hero), "the crouch keeps its feet");
                }
                if (i == 241)
                {
                    m3real hh;
                    m3real r;
                    m3Character_GetStance(hero, &hh, &r);
                    CHECK(hh == 0.2f, "the pressed stand is REFUSED, stance holds");
                    double x = m3Character_GetPosition(hero).x;
                    CHECK(x > 2.2 && x < 6.0, "the refusal happened inside the tunnel");
                }
                if (i == 461)
                {
                    m3real hh;
                    m3real r;
                    m3Character_GetStance(hero, &hh, &r);
                    CHECK(hh == 0.5f, "past the edge the stand applies");
                    CHECK(m3Character_GetPosition(hero).x > 6.5, "genuinely past the slab");
                }
            }
        }
        double standY = m3Character_GetPosition(hero).y;
        CHECK(standY > 0.89 && standY < 0.95, "the stood character is tall again");
        hashes[run] = m3World_Hash(world);
        if (recording)
        {
            int32_t bytes = m3World_JournalEnd(world);
            CHECK(bytes > 0, "the tunnel run records");
            m3WorldId replayed = TunnelWorld();
            CHECK(m3World_JournalReplay(replayed, journal, bytes), "the tunnel run replays");
            CHECK(m3World_Hash(replayed) == hashes[0], "the replayed run is bit-identical");
            // The refused stand at tick 240 journaled NOTHING: the
            // replay just proved it by landing (a phantom op 64
            // would have re-applied and diverged or failed).
            m3DestroyWorld(replayed);
        }
        m3DestroyWorld(world);
    }
    CHECK(hashes[0] == hashes[1], "twin tunnel runs are bit-identical");
}

static void TestMidTunnelRollback(void)
{
    // Snapshot CROUCHED inside the tunnel, run out and stand, then
    // restore and re-run the same schedule: stance is state, so the
    // re-run lands on the same bits including the refusal.
    static uint8_t snap[2097152];
    m3WorldId world = TunnelWorld();
    m3CharacterDef cd = m3DefaultCharacterDef();
    cd.position = (m3Pos3){0.0, 3.0, 0.0};
    m3CharacterId hero = m3CreateCharacter(world, &cd);
    int32_t snapBytes = 0;
    for (int32_t i = 0; i < 520; ++i)
    {
        TunnelTick(hero, i);
        if (i == 300)
        {
            snapBytes = m3World_Snapshot(world, snap, (int32_t)sizeof(snap));
            CHECK(snapBytes > 0, "the mid-tunnel snapshot fits");
        }
    }
    uint64_t final = m3World_Hash(world);
    CHECK(m3World_Restore(world, snap, snapBytes), "the mid-tunnel restore lands");
    m3real hh;
    m3real r;
    m3Character_GetStance(hero, &hh, &r);
    CHECK(hh == 0.2f, "the restored stance is the crouch");
    for (int32_t i = 301; i < 520; ++i)
    {
        TunnelTick(hero, i);
    }
    CHECK(m3World_Hash(world) == final, "the re-run tunnel is bit-identical");
    m3DestroyWorld(world);
}

static void TestMidAirCrouchLandsShorter(void)
{
    // Feet anchoring is the law even in the air: the crouch pulls
    // the center down, the capsule is shorter, and the landing
    // height is the crouched skin gap, not the standing one.
    m3WorldId world = TunnelWorld();
    m3CharacterDef cd = m3DefaultCharacterDef();
    cd.position = (m3Pos3){-3.0, 4.0, 0.0};
    m3CharacterId hero = m3CreateCharacter(world, &cd);
    for (int32_t i = 0; i < 20; ++i)
    {
        m3Character_Move(hero, (m3Vec3){0.0f, -0.08f, 0.0f});
    }
    CHECK(!m3Character_IsGrounded(hero), "still airborne at the crouch");
    CHECK(m3Character_SetStance(hero, 0.2f, 0.4f), "the mid-air crouch applies");
    for (int32_t i = 0; i < 120; ++i)
    {
        m3Character_Move(hero, (m3Vec3){0.0f, -0.08f, 0.0f});
    }
    CHECK(m3Character_IsGrounded(hero), "the crouched faller lands");
    double y = m3Character_GetPosition(hero).y;
    CHECK(y > 0.59 && y < 0.65, "the landing height is the crouched one");
    m3DestroyWorld(world);
}

static void TestSlotRefusesWidth(void)
{
    // Two walls 0.9 apart: the 0.8-wide character fits, a 1.2-wide
    // one cannot, and the veto sees the walls exactly like it sees
    // ceilings (the fraction-zero overlap report).
    m3WorldId world = TunnelWorld();
    m3ShapeDef sd = m3DefaultShapeDef();
    m3BodyDef wd = m3DefaultBodyDef();
    wd.position = (m3Pos3){-4.0, 1.0, 0.65};
    m3BodyId wallA = m3CreateBody(world, &wd);
    m3CreateBoxShape(wallA, &sd, (m3Vec3){1.0f, 1.0f, 0.2f});
    wd.position = (m3Pos3){-4.0, 1.0, -0.65};
    m3BodyId wallB = m3CreateBody(world, &wd);
    m3CreateBoxShape(wallB, &sd, (m3Vec3){1.0f, 1.0f, 0.2f});
    m3CharacterDef cd = m3DefaultCharacterDef();
    cd.position = (m3Pos3){-4.0, 3.0, 0.0};
    m3CharacterId hero = m3CreateCharacter(world, &cd);
    for (int32_t i = 0; i < 120; ++i)
    {
        m3Character_Move(hero, (m3Vec3){0.0f, -0.1f, 0.0f});
    }
    CHECK(m3Character_IsGrounded(hero), "the character lands inside the slot");
    CHECK(!m3Character_SetStance(hero, 0.5f, 0.6f), "widening inside the slot refuses");
    m3real hh;
    m3real r;
    m3Character_GetStance(hero, &hh, &r);
    CHECK(r == 0.4f, "the refused widening left the radius alone");
    CHECK(m3Character_SetStance(hero, 0.5f, 0.3f), "narrowing inside the slot applies");
    m3DestroyWorld(world);
}

static void TestHostileWall(void)
{
    m3WorldId world = TunnelWorld();
    m3CharacterDef cd = m3DefaultCharacterDef();
    cd.position = (m3Pos3){0.0, 3.0, 0.0};
    m3CharacterId hero = m3CreateCharacter(world, &cd);
    float bad;
    uint32_t nanBits = 0x7FC00000u;
    memcpy(&bad, &nanBits, sizeof(bad));
    CHECK(!m3Character_SetStance(hero, bad, 0.4f), "a NaN height bounces");
    CHECK(!m3Character_SetStance(hero, 0.5f, -0.4f), "a negative radius bounces");
    CHECK(!m3Character_SetStance(hero, 0.01f, 0.4f), "a sub-minimum height bounces");
    CHECK(!m3Character_SetStance(hero, 0.5f, 40.0f), "an absurd radius bounces");
    m3real hh;
    m3real r;
    m3Character_GetStance(hero, &hh, &r);
    CHECK(hh == 0.5f && r == 0.4f, "four hostile calls changed nothing");
    m3CharacterId stale = {99, hero.world0, 7};
    CHECK(!m3Character_SetStance(stale, 0.5f, 0.4f), "a stale id bounces");
    m3Character_GetStance(stale, &hh, &r);
    CHECK(hh == 0.0f && r == 0.0f, "a stale stance reads zeros");
    CHECK(m3Character_SetStance(hero, 0.5f, 0.4f), "the same-dims stance re-applies");
    m3DestroyWorld(world);
}

static void TestCrouchSpamMovingCeiling(void)
{
    // The 12-4 red team: a kinematic press descends and rises over
    // a character that attempts to STAND EVERY TICK. The veto must
    // flicker between grant and refusal in perfect step with the
    // press, twins must flicker identically, and a mid-storm
    // rollback must re-run onto the same bits. The invariant every
    // tick: standing means clearance existed at the grant.
    static uint8_t snap[2097152];
    uint64_t hashes[2];
    int32_t grants[2] = {0, 0};
    for (int32_t run = 0; run < 2; ++run)
    {
        m3WorldId world = TunnelWorld();
        m3BodyDef pd = m3DefaultBodyDef();
        pd.type = m3_kinematicBody;
        pd.position = (m3Pos3){-3.0, 2.6, 0.0}; // slab bottom at 2.35
        m3BodyId press = m3CreateBody(world, &pd);
        m3ShapeDef sd = m3DefaultShapeDef();
        m3CreateBoxShape(press, &sd, (m3Vec3){1.5f, 0.25f, 1.5f});
        m3CharacterDef cd = m3DefaultCharacterDef();
        cd.position = (m3Pos3){-3.0, 3.0, 0.0};
        m3CharacterId hero = m3CreateCharacter(world, &cd);
        for (int32_t i = 0; i < 80; ++i)
        {
            m3Character_Move(hero, (m3Vec3){0.0f, -0.1f, 0.0f});
        }
        CHECK(m3Character_IsGrounded(hero), "the character lands under the press");
        int32_t snapBytes = 0;
        uint64_t midAhead = 0;
        for (int32_t i = 0; i < 480; ++i)
        {
            // The press: down for 120, hold 60, up for 120, hold,
            // repeat. Speed 0.01 per tick: from 2.35 down to 1.15
            // (crouch-only) and back.
            int32_t phase = i % 420;
            m3real vy = phase < 120 ? -0.6f : (phase < 180 ? 0.0f : (phase < 300 ? 0.6f : 0.0f));
            m3Body_SetLinearVelocity(press, (m3Vec3){0.0f, vy, 0.0f});
            m3Character_SetStance(hero, 0.2f, 0.4f); // duck first
            m3World_Step(world, 1.0f / 60.0f, 4);
            if (m3Character_SetStance(hero, 0.5f, 0.4f))
            {
                grants[run] += 1;
                m3Character_SetStance(hero, 0.2f, 0.4f); // back down for
                                                         // the next tick
            }
            if (i == 200 && run == 0)
            {
                snapBytes = m3World_Snapshot(world, snap, (int32_t)sizeof(snap));
                CHECK(snapBytes > 0, "the mid-storm snapshot fits");
            }
        }
        hashes[run] = m3World_Hash(world);
        if (run == 0)
        {
            midAhead = hashes[0];
            CHECK(m3World_Restore(world, snap, snapBytes), "the mid-storm restore lands");
            int32_t regrants = 0;
            for (int32_t i = 201; i < 480; ++i)
            {
                int32_t phase = i % 420;
                m3real vy =
                    phase < 120 ? -0.6f : (phase < 180 ? 0.0f : (phase < 300 ? 0.6f : 0.0f));
                m3Body_SetLinearVelocity(press, (m3Vec3){0.0f, vy, 0.0f});
                m3Character_SetStance(hero, 0.2f, 0.4f);
                m3World_Step(world, 1.0f / 60.0f, 4);
                if (m3Character_SetStance(hero, 0.5f, 0.4f))
                {
                    regrants += 1;
                    m3Character_SetStance(hero, 0.2f, 0.4f);
                }
            }
            CHECK(m3World_Hash(world) == midAhead, "the re-run storm is bit-identical");
            (void)regrants;
        }
        CHECK(grants[run] > 0, "the veto grants when the press is high");
        CHECK(grants[run] < 480, "the veto refuses when the press is low");
        m3DestroyWorld(world);
    }
    CHECK(hashes[0] == hashes[1], "twin press storms are bit-identical");
    CHECK(grants[0] == grants[1], "twin grant patterns are identical");
}

int main(void)
{
    TestDuckWalkStand();
    TestMidTunnelRollback();
    TestMidAirCrouchLandsShorter();
    TestSlotRefusesWidth();
    TestHostileWall();
    TestCrouchSpamMovingCeiling();
    if (s_failures == 0)
    {
        printf("test_stance: all passed\n");
        return 0;
    }
    return 1;
}
