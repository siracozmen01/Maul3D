// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The explosion gate (13-2): one journaled call pushes every dynamic
// convex shape in range. A symmetric ring flies outward on mirrored
// speeds, the falloff band grades the push, sleepers in range wake
// while sleepers beyond it nap on, a negative impulse implodes, the
// filter and static bodies stand aside, an off-center hit spins its
// target, and the whole storm rides twins, replay, rollback, and a
// hostile tape without losing a bit.

#include "maul3d/body.h"
#include "maul3d/shape.h"

#include <math.h>
#include <stdio.h>
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

static m3WorldId BlastWorld(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
    def.bodyCapacity = 64;
    def.shapeCapacity = 64;
    return m3CreateWorld(&def);
}

static m3BodyId Crate(m3WorldId world, double x, double y, double z)
{
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){x, y, z};
    m3BodyId body = m3CreateBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3CreateBoxShape(body, &sd, (m3Vec3){0.5f, 0.5f, 0.5f});
    return body;
}

static float Speed(m3BodyId body)
{
    m3Vec3 v = m3Body_GetLinearVelocity(body);
    return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

static void TestRingAndFalloff(void)
{
    m3WorldId world = BlastWorld();
    // Four crates on the axes at 4 m, one in the falloff band at
    // 12 m, one beyond every reach at 30 m.
    m3BodyId east = Crate(world, 4.0, 0.0, 0.0);
    m3BodyId west = Crate(world, -4.0, 0.0, 0.0);
    m3BodyId north = Crate(world, 0.0, 0.0, 4.0);
    m3BodyId south = Crate(world, 0.0, 0.0, -4.0);
    m3BodyId graze = Crate(world, 12.0, 0.0, 0.0);
    m3BodyId far = Crate(world, 30.0, 0.0, 0.0);
    m3ExplosionDef def = m3DefaultExplosionDef();
    def.position = (m3Pos3){0.0, 0.0, 0.0};
    def.radius = 10.0f;
    def.falloff = 5.0f;
    def.impulsePerArea = 4.0f;
    m3World_Explode(world, &def);
    CHECK(Speed(east) > 1.0f, "the blast moves the ring");
    CHECK(fabsf(Speed(east) - Speed(west)) < 1e-6f, "mirrored crates get mirrored speeds");
    CHECK(fabsf(Speed(north) - Speed(south)) < 1e-6f, "the mirror holds on both axes");
    CHECK(m3Body_GetLinearVelocity(east).x > 0.0f, "east flies east");
    CHECK(m3Body_GetLinearVelocity(west).x < 0.0f, "west flies west");
    CHECK(Speed(graze) > 0.0f, "the falloff band still pushes");
    CHECK(Speed(graze) < Speed(east), "the falloff band pushes less");
    CHECK(Speed(far) == 0.0f, "beyond radius plus falloff nothing moves");
    // An implosion pulls the same ring inward.
    def.impulsePerArea = -4.0f;
    m3World_Explode(world, &def);
    CHECK(fabsf(Speed(east)) < 1e-5f, "the implosion refunds the blast exactly");
    m3DestroyWorld(world);
}

static void TestWakeFilterAndStatics(void)
{
    m3WorldId world = BlastWorld();
    m3BodyId sleeper = Crate(world, 3.0, 0.0, 0.0);
    m3BodyId dreamer = Crate(world, 40.0, 0.0, 0.0);
    m3Body_SetAwake(sleeper, false);
    m3Body_SetAwake(dreamer, false);
    // A crate the filter excludes: category 2, blast masks it out.
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){-3.0, 0.0, 0.0};
    m3BodyId ghost = m3CreateBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.categoryBits = 2ull;
    m3CreateBoxShape(ghost, &sd, (m3Vec3){0.5f, 0.5f, 0.5f});
    // A static wall inside the blast.
    m3BodyDef wd = m3DefaultBodyDef();
    wd.position = (m3Pos3){0.0, -3.0, 0.0};
    m3BodyId wall = m3CreateBody(world, &wd);
    m3ShapeDef wsd = m3DefaultShapeDef();
    m3CreateBoxShape(wall, &wsd, (m3Vec3){2.0f, 0.5f, 2.0f});
    m3ExplosionDef def = m3DefaultExplosionDef();
    def.position = (m3Pos3){0.0, 0.0, 0.0};
    def.radius = 8.0f;
    def.falloff = 2.0f;
    def.impulsePerArea = 3.0f;
    def.filter.maskBits = ~2ull;
    m3World_Explode(world, &def);
    CHECK(m3Body_IsAwake(sleeper), "a sleeper in range wakes");
    CHECK(Speed(sleeper) > 0.0f, "the woken sleeper moves");
    CHECK(!m3Body_IsAwake(dreamer), "a sleeper beyond range naps on");
    CHECK(Speed(ghost) == 0.0f, "the filtered crate never feels the blast");
    CHECK(Speed(wall) == 0.0f, "the static wall stands");
    m3Vec3 wv = m3Body_GetLinearVelocity(wall);
    CHECK(wv.x == 0.0f && wv.y == 0.0f && wv.z == 0.0f, "statics take no velocity at all");
    m3DestroyWorld(world);
}

static void TestOffCenterSpin(void)
{
    // The blast center level with the crate's lower edge: the
    // closest point is off the center of mass, the lever arm turns
    // the push into spin.
    m3WorldId world = BlastWorld();
    m3BodyId crate = Crate(world, 3.0, 0.0, 0.0);
    m3ExplosionDef def = m3DefaultExplosionDef();
    def.position = (m3Pos3){0.0, -0.45, 0.0};
    def.radius = 6.0f;
    def.falloff = 2.0f;
    def.impulsePerArea = 3.0f;
    m3World_Explode(world, &def);
    m3Vec3 w = m3Body_GetAngularVelocity(crate);
    CHECK(Speed(crate) > 0.0f, "the off-center blast still pushes");
    CHECK(sqrtf(w.x * w.x + w.y * w.y + w.z * w.z) > 0.01f, "the lever arm spins the crate");
    m3DestroyWorld(world);
}

static void TestHostileDefWall(void)
{
    m3WorldId world = BlastWorld();
    m3BodyId crate = Crate(world, 3.0, 0.0, 0.0);
    uint64_t before = m3World_Hash(world);
    m3ExplosionDef def = m3DefaultExplosionDef();
    def.position = (m3Pos3){0.0, 0.0, 0.0};
    def.impulsePerArea = 5.0f;
    def.radius = -1.0f;
    m3World_Explode(world, &def);
    def = m3DefaultExplosionDef();
    def.impulsePerArea = 5.0f;
    def.falloff = -0.5f;
    m3World_Explode(world, &def);
    def = m3DefaultExplosionDef();
    float bad;
    uint32_t nanBits = 0x7FC00000u;
    memcpy(&bad, &nanBits, sizeof(bad));
    def.impulsePerArea = bad;
    m3World_Explode(world, &def);
    def = m3DefaultExplosionDef();
    def.position.x = (double)bad;
    def.impulsePerArea = 5.0f;
    m3World_Explode(world, &def);
    def = m3DefaultExplosionDef();
    def.impulsePerArea = 5.0f;
    def.internalValue = 0; // a dead cookie bounces at the door
    m3World_Explode(world, &def);
    CHECK(m3World_Hash(world) == before, "five hostile blasts changed nothing");
    CHECK(Speed(crate) == 0.0f, "the crate never moved");
    m3DestroyWorld(world);
}

static void TestTwinsReplayRollback(void)
{
    static uint8_t journal[65536];
    static uint8_t snap[2097152];
    uint64_t hashes[2];
    for (int32_t run = 0; run < 2; ++run)
    {
        m3WorldId world = BlastWorld();
        bool recording =
            run == 0 && m3World_JournalBegin(world, journal, (int32_t)sizeof(journal));
        m3BodyId a = Crate(world, 4.0, 0.0, 0.0);
        m3BodyId b = Crate(world, -4.0, 0.0, 1.0);
        Crate(world, 0.0, 4.0, -2.0);
        int32_t snapBytes = 0;
        for (int32_t i = 0; i < 120; ++i)
        {
            if (i == 30)
            {
                m3ExplosionDef def = m3DefaultExplosionDef();
                def.position = (m3Pos3){0.0, 0.0, 0.0};
                def.radius = 8.0f;
                def.falloff = 4.0f;
                def.impulsePerArea = 2.5f;
                m3World_Explode(world, &def);
            }
            if (i == 60 && run == 0)
            {
                snapBytes = m3World_Snapshot(world, snap, (int32_t)sizeof(snap));
                CHECK(snapBytes > 0, "the mid-storm snapshot fits");
            }
            if (i == 90)
            {
                m3ExplosionDef def = m3DefaultExplosionDef();
                def.position = (m3Pos3){2.0, 0.0, 0.0};
                def.radius = 5.0f;
                def.falloff = 5.0f;
                def.impulsePerArea = -1.5f; // the implosion pass
                m3World_Explode(world, &def);
            }
            m3World_Step(world, 1.0f / 60.0f, 4);
        }
        CHECK(Speed(a) > 0.0f && Speed(b) > 0.0f, "the storm moved the crates");
        hashes[run] = m3World_Hash(world);
        if (recording)
        {
            int32_t bytes = m3World_JournalEnd(world);
            CHECK(bytes > 0, "the storm records");
            m3WorldId replayed = BlastWorld();
            CHECK(m3World_JournalReplay(replayed, journal, bytes), "the storm replays");
            CHECK(m3World_Hash(replayed) == hashes[0], "the replay is bit-identical");
            m3DestroyWorld(replayed);
            // Rollback: restore to tick 60 and re-run through the
            // implosion; the re-run must land on the same bits.
            CHECK(m3World_Restore(world, snap, snapBytes), "the mid-storm restore lands");
            for (int32_t i = 60; i < 120; ++i)
            {
                if (i == 90)
                {
                    m3ExplosionDef def = m3DefaultExplosionDef();
                    def.position = (m3Pos3){2.0, 0.0, 0.0};
                    def.radius = 5.0f;
                    def.falloff = 5.0f;
                    def.impulsePerArea = -1.5f;
                    m3World_Explode(world, &def);
                }
                m3World_Step(world, 1.0f / 60.0f, 4);
            }
            CHECK(m3World_Hash(world) == hashes[0], "the re-run storm is bit-identical");
        }
        m3DestroyWorld(world);
    }
    CHECK(hashes[0] == hashes[1], "twin storms are bit-identical");
}

static void TestHostileTape(void)
{
    // One explode op on tape, then surgical damage: a NaN center and
    // a negative radius must fail the replay loudly.
    static uint8_t tape[256];
    m3WorldId world = BlastWorld();
    Crate(world, 3.0, 0.0, 0.0);
    CHECK(m3World_JournalBegin(world, tape, (int32_t)sizeof(tape)), "the tape opens");
    m3ExplosionDef def = m3DefaultExplosionDef();
    def.position = (m3Pos3){0.0, 0.0, 0.0};
    def.impulsePerArea = 2.0f;
    m3World_Explode(world, &def);
    int32_t bytes = m3World_JournalEnd(world);
    CHECK(bytes == 8 + (int32_t)sizeof(m3ExplosionDef), "one blast op is header plus def");
    m3DestroyWorld(world);

    m3WorldId twin = BlastWorld();
    m3BodyId crate = Crate(twin, 3.0, 0.0, 0.0);
    uint8_t hostile[256];
    memcpy(hostile, tape, (size_t)bytes);
    double nanD;
    uint64_t nanBits = 0x7FF8000000000000ull;
    memcpy(&nanD, &nanBits, sizeof(nanD));
    memcpy(hostile + 8, &nanD, sizeof(nanD)); // position.x
    CHECK(!m3World_JournalReplay(twin, hostile, bytes), "a NaN center bounces");
    memcpy(hostile, tape, (size_t)bytes);
    float negative = -2.0f;
    memcpy(hostile + 8 + 40, &negative, sizeof(negative)); // radius
    CHECK(!m3World_JournalReplay(twin, hostile, bytes), "a negative radius bounces");
    CHECK(Speed(crate) == 0.0f, "the bounced tapes moved nothing");
    CHECK(m3World_JournalReplay(twin, tape, bytes), "the clean tape lands");
    CHECK(Speed(crate) > 0.0f, "the replayed blast pushes the twin crate");
    m3DestroyWorld(twin);
}

int main(void)
{
    TestRingAndFalloff();
    TestWakeFilterAndStatics();
    TestOffCenterSpin();
    TestHostileDefWall();
    TestTwinsReplayRollback();
    TestHostileTape();
    if (s_failures == 0)
    {
        printf("test_explosion: all passed\n");
        return 0;
    }
    return 1;
}
