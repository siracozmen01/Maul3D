// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The mover core gate (4-4): a kinematic capsule walks the world by
// collide-and-slide, deterministically. Flat ground carries it at
// exact height, walls strip the blocked component and keep the
// rest, ceilings stop it, ledges honestly drop it, and every move
// is a journaled command that replays and rolls back bit for bit.

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

static m3WorldId ArenaWorld(void)
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
    return world;
}

// The standing height for the default character: capsule bottom
// (center - halfHeight - radius) rides one skin above the floor.
static double StandingY(const m3CharacterDef* def)
{
    return (double)(def->halfHeight + def->radius + def->skin);
}

static void TestFlatWalk(void)
{
    m3WorldId world = ArenaWorld();
    m3CharacterDef cd = m3DefaultCharacterDef();
    cd.position = (m3Pos3){0.0, 3.0, 0.0};
    m3CharacterId hero = m3CreateCharacter(world, &cd);
    CHECK(m3Character_IsValid(hero), "the character creates");
    CHECK(!m3Character_IsGrounded(hero), "spawned in the air, honestly airborne");

    // Fall to the floor under host gravity, then walk.
    for (int32_t i = 0; i < 120; ++i)
    {
        m3Character_Move(hero, (m3Vec3){0.0f, -0.1f, 0.0f});
    }
    CHECK(m3Character_IsGrounded(hero), "the character lands");
    m3Pos3 p = m3Character_GetPosition(hero);
    double stand = StandingY(&cd);
    CHECK(p.y > stand - 0.03 && p.y < stand + 0.03, "standing height is the analytic skin gap");
    m3Vec3 n = m3Character_GetGroundNormal(hero);
    CHECK(n.y > 0.99f, "flat ground reports a vertical normal");

    // One hundred forward ticks: distance adds up exactly, the
    // height never wanders, grounded never flickers.
    double y0 = p.y;
    for (int32_t i = 0; i < 100; ++i)
    {
        m3Character_Move(hero, (m3Vec3){0.05f, -0.05f, 0.0f});
        CHECK(m3Character_IsGrounded(hero), "grounded holds through the walk");
    }
    p = m3Character_GetPosition(hero);
    CHECK(p.x > 4.9 && p.x < 5.1, "the walk covers the commanded distance");
    CHECK(fabs(p.y - y0) < 0.02, "the walk keeps its height");
    m3DestroyWorld(world);
}

static void TestWallSlideAndCeiling(void)
{
    m3WorldId world = ArenaWorld();
    m3ShapeDef sd = m3DefaultShapeDef();
    m3BodyDef wd = m3DefaultBodyDef();
    wd.position = (m3Pos3){3.0, 2.0, 0.0};
    m3BodyId wallBody = m3CreateBody(world, &wd);
    m3CreateBoxShape(wallBody, &sd, (m3Vec3){0.2f, 2.0f, 4.0f}); // a wall at x = 2.8

    wd.position = (m3Pos3){0.0, 3.0, 0.0};
    m3BodyId lidBody = m3CreateBody(world, &wd);
    m3CreateBoxShape(lidBody, &sd, (m3Vec3){1.5f, 0.2f, 1.5f}); // a lid at y = 2.8

    m3CharacterDef cd = m3DefaultCharacterDef();
    cd.position = (m3Pos3){0.0, 1.0, 0.0};
    m3CharacterId hero = m3CreateCharacter(world, &cd);
    for (int32_t i = 0; i < 30; ++i)
    {
        m3Character_Move(hero, (m3Vec3){0.0f, -0.1f, 0.0f});
    }

    // Charge the wall diagonally: x stops at the wall minus the
    // capsule radius and skin, z keeps flowing (the slide).
    for (int32_t i = 0; i < 100; ++i)
    {
        m3Character_Move(hero, (m3Vec3){0.06f, -0.02f, 0.03f});
    }
    m3Pos3 p = m3Character_GetPosition(hero);
    CHECK(p.x < 2.8 - 0.4 + 0.01, "the wall stops the blocked axis");
    CHECK(p.x > 2.8 - 0.4 - 0.1, "the character presses close to the wall");
    CHECK(p.z > 2.0, "the slide keeps the free axis flowing");
    CHECK(m3Character_IsGrounded(hero), "sliding along a wall stays grounded");

    // The ceiling: jump straight up under the lid; the head stops
    // below it and never penetrates.
    m3CharacterDef cd2 = m3DefaultCharacterDef();
    cd2.position = (m3Pos3){0.0, 1.0, 0.0};
    m3CharacterId jumper = m3CreateCharacter(world, &cd2);
    for (int32_t i = 0; i < 60; ++i)
    {
        m3Character_Move(jumper, (m3Vec3){0.0f, 0.08f, 0.0f});
    }
    p = m3Character_GetPosition(jumper);
    // Lid center y = 3 with half 0.2: the bottom face is 2.8, and
    // the head (center + halfHeight + radius) parks one skin below.
    double headroom = 2.8 - (double)(cd2.halfHeight + cd2.radius);
    CHECK(p.y < headroom + 0.01, "the ceiling stops the rise");
    CHECK(p.y > headroom - 0.1, "the character reaches the ceiling before stopping");
    m3DestroyWorld(world);
}

static void TestLedgeDropAndSnap(void)
{
    // A raised slab: walking off its edge must transition to
    // airborne (the snap gives up beyond snapDistance), and small
    // descents stay glued.
    m3WorldId world = ArenaWorld();
    m3ShapeDef sd = m3DefaultShapeDef();
    m3BodyDef pd = m3DefaultBodyDef();
    pd.position = (m3Pos3){0.0, 0.5, 0.0};
    m3BodyId slabBody = m3CreateBody(world, &pd);
    m3CreateBoxShape(slabBody, &sd, (m3Vec3){2.0f, 0.5f, 2.0f}); // top at y = 1

    m3CharacterDef cd = m3DefaultCharacterDef();
    cd.position = (m3Pos3){0.0, 3.0, 0.0};
    m3CharacterId hero = m3CreateCharacter(world, &cd);
    for (int32_t i = 0; i < 60; ++i)
    {
        m3Character_Move(hero, (m3Vec3){0.0f, -0.1f, 0.0f});
    }
    m3Pos3 p = m3Character_GetPosition(hero);
    CHECK(m3Character_IsGrounded(hero) && p.y > 1.8 && p.y < 2.0,
          "the character stands on the slab");

    // March off the edge with gravity-free forward ticks: past the
    // rim the snap has nothing walkable within reach, so grounded
    // must drop even though nothing pushed us down yet.
    bool wentAirborne = false;
    for (int32_t i = 0; i < 80; ++i)
    {
        m3Character_Move(hero, (m3Vec3){0.08f, 0.0f, 0.0f});
        if (!m3Character_IsGrounded(hero))
        {
            wentAirborne = true;
            break;
        }
    }
    CHECK(wentAirborne, "walking off the ledge reports airborne");
    m3DestroyWorld(world);
}

static void TestMoveDeterminismAndJournal(void)
{
    // Twin worlds walk the identical script and land on identical
    // bits; the journaled session replays; a rollback re-walks.
    static uint8_t journal[131072];
    uint64_t hashes[2];
    for (int32_t run = 0; run < 2; ++run)
    {
        m3WorldId world = ArenaWorld();
        bool recording = run == 0 && m3World_JournalBegin(world, journal, (int32_t)sizeof(journal));
        m3CharacterDef cd = m3DefaultCharacterDef();
        cd.position = (m3Pos3){0.0, 2.0, 0.0};
        m3CharacterId hero = m3CreateCharacter(world, &cd);
        // A dynamic crate shares the arena so the character's body
        // participates in real simulation.
        m3BodyDef bd = m3DefaultBodyDef();
        bd.type = m3_dynamicBody;
        bd.position = (m3Pos3){2.0, 2.0, 0.0};
        m3BodyId crate = m3CreateBody(world, &bd);
        m3ShapeDef sd = m3DefaultShapeDef();
        m3CreateBoxShape(crate, &sd, (m3Vec3){0.3f, 0.3f, 0.3f});
        for (int32_t i = 0; i < 150; ++i)
        {
            float side = (i / 25) % 2 == 0 ? 1.0f : -1.0f;
            m3Character_Move(hero, (m3Vec3){0.04f * side, -0.06f, 0.02f});
            m3World_Step(world, 1.0f / 60.0f, 4);
        }
        hashes[run] = m3World_Hash(world);
        if (recording)
        {
            int32_t bytes = m3World_JournalEnd(world);
            CHECK(bytes > 0, "the walk session records");
            m3WorldId replayed = ArenaWorld();
            CHECK(m3World_JournalReplay(replayed, journal, bytes), "the walk session replays");
            CHECK(m3World_Hash(replayed) == hashes[0], "the replayed walk is bit-identical");
            m3DestroyWorld(replayed);

            // Rollback: rewind mid-walk and re-walk the same tail.
            int32_t snapBytes = m3World_SnapshotSize(world);
            uint8_t* snap = (uint8_t*)malloc((size_t)snapBytes);
            CHECK(m3World_Snapshot(world, snap, snapBytes) == snapBytes, "the walk snapshot");
            for (int32_t i = 0; i < 40; ++i)
            {
                m3Character_Move(hero, (m3Vec3){0.05f, -0.05f, 0.0f});
                m3World_Step(world, 1.0f / 60.0f, 4);
            }
            uint64_t ahead = m3World_Hash(world);
            CHECK(m3World_Restore(world, snap, snapBytes), "the walk rollback lands");
            for (int32_t i = 0; i < 40; ++i)
            {
                m3Character_Move(hero, (m3Vec3){0.05f, -0.05f, 0.0f});
                m3World_Step(world, 1.0f / 60.0f, 4);
            }
            CHECK(m3World_Hash(world) == ahead, "the re-walked timeline is bit-identical");
            free(snap);
        }
        m3DestroyWorld(world);
    }
    CHECK(hashes[0] == hashes[1], "twin walks agree bit for bit");
}

static void TestCharacterContracts(void)
{
    m3WorldId world = ArenaWorld();
    m3CharacterDef cd = m3DefaultCharacterDef();
    cd.radius = 0.0f;
    CHECK(!m3Character_IsValid(m3CreateCharacter(world, &cd)), "a zero radius refuses");
    cd = m3DefaultCharacterDef();
    cd.skin = 0.0f;
    CHECK(!m3Character_IsValid(m3CreateCharacter(world, &cd)), "a zero skin refuses");
    cd = m3DefaultCharacterDef();
    cd.maxSlopeAngle = 0.0f;
    CHECK(!m3Character_IsValid(m3CreateCharacter(world, &cd)), "a zero slope refuses");

    cd = m3DefaultCharacterDef();
    cd.position = (m3Pos3){0.0, 2.0, 0.0};
    m3CharacterId one = m3CreateCharacter(world, &cd);
    cd.position = (m3Pos3){3.0, 2.0, 0.0};
    m3CharacterId two = m3CreateCharacter(world, &cd);
    CHECK(m3Character_IsValid(one) && m3Character_IsValid(two), "the pool fills");
    CHECK(!m3Character_IsValid(m3CreateCharacter(world, &cd)),
          "the character pool refuses past capacity");

    m3DestroyCharacter(one);
    CHECK(!m3Character_IsValid(one), "the destroyed character is stale");
    m3Character_Move(one, (m3Vec3){1.0f, 0.0f, 0.0f}); // stale: no-op
    m3Pos3 p = m3Character_GetPosition(one);
    CHECK(p.x == 0.0 && p.y == 0.0, "a stale getter returns zeros");
    float bad;
    uint32_t nanBits = 0x7FC00000u; // bit-built NaN: no compiler or
                                    // sanitizer gets an opinion
    memcpy(&bad, &nanBits, sizeof(bad));
    m3Character_Move(two, (m3Vec3){bad, 0.0f, 0.0f}); // NaN: no-op
    p = m3Character_GetPosition(two);
    CHECK(p.x == 3.0, "a hostile move never lands");
    m3World_Step(world, 1.0f / 60.0f, 4);
    CHECK(m3World_Hash(world) != 0, "the world shrugs it off");
    m3DestroyWorld(world);
}

// 4-5: stairs, slopes, the voxel floor, and the welded seam.
static void TestVoxelStairs(void)
{
    // Four quarter-meter risers climb; a half-meter riser refuses
    // (stepHeight is 0.35). The staircase is a voxel chunk, so the
    // whole exercise runs on the merged-box surface.
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    def.voxelCapacity = 1;
    def.characterCapacity = 1;
    m3WorldId world = m3CreateWorld(&def);
    m3ShapeDef sd = m3DefaultShapeDef();
    static uint8_t voxels[16 * 16 * 16];
    memset(voxels, 0, sizeof(voxels));
    // cellSize 0.25: floor at layer 0; stairs rise one layer per
    // two cells of x from x = 4; a double riser at x = 12.
    for (int32_t z = 0; z < 16; ++z)
    {
        for (int32_t x = 0; x < 16; ++x)
        {
            int32_t top = 0;
            if (x >= 4)
            {
                top = (x - 4) / 2 + 1;
            }
            if (x >= 12)
            {
                top += 2; // the illegal cliff: two layers at once
            }
            if (top > 15)
            {
                top = 15;
            }
            for (int32_t y = 0; y <= top; ++y)
            {
                voxels[x + 16 * (y + 16 * z)] = 1;
            }
        }
    }
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    CHECK(m3Shape_IsValid(m3CreateVoxelChunkShape(ground, &sd, voxels, NULL, 0.25f)),
          "the staircase creates");

    m3CharacterDef cd = m3DefaultCharacterDef();
    cd.radius = 0.2f;
    cd.halfHeight = 0.3f;
    cd.position = (m3Pos3){0.5, 2.0, 2.0};
    m3CharacterId hero = m3CreateCharacter(world, &cd);
    for (int32_t i = 0; i < 40; ++i)
    {
        m3Character_Move(hero, (m3Vec3){0.0f, -0.1f, 0.0f});
    }
    CHECK(m3Character_IsGrounded(hero), "the character lands at the stair base");

    // March up the stairs.
    for (int32_t i = 0; i < 120; ++i)
    {
        m3Character_Move(hero, (m3Vec3){0.03f, -0.03f, 0.0f});
    }
    m3Pos3 p = m3Character_GetPosition(hero);
    CHECK(p.x > 2.6 && p.x < 3.05, "the climb is stopped only by the double riser");
    // Feet on the step below the cliff: the top under x ~ 2.9 is
    // layer (2.9/0.25 - 4)/2 + 1 = 4 -> surface y = 5 * 0.25 = 1.25.
    double stand = 1.25 + (double)(cd.halfHeight + cd.radius + cd.skin);
    CHECK(p.y > stand - 0.05 && p.y < stand + 0.05, "the character stands on the fourth step");
    CHECK(m3Character_IsGrounded(hero), "stairs never break grounding");
    m3DestroyWorld(world);
}

static void TestSteepSlopeRefusal(void)
{
    // A sixty-degree ramp is a wall in walking terms: pressing into
    // it must not mint height, tick after tick.
    m3WorldId world = ArenaWorld();
    m3ShapeDef sd = m3DefaultShapeDef();
    m3BodyDef rd = m3DefaultBodyDef();
    rd.position = (m3Pos3){4.0, 0.0, 0.0};
    // Yaw the box about z by sixty degrees to make the steep face.
    float half = 0.5235988f; // 60 degrees / 2
    rd.rotation = (m3Quat){0.0f, 0.0f, sinf(half), cosf(half)};
    m3BodyId rampBody = m3CreateBody(world, &rd);
    m3CreateBoxShape(rampBody, &sd, (m3Vec3){3.0f, 3.0f, 3.0f});

    m3CharacterDef cd = m3DefaultCharacterDef();
    cd.position = (m3Pos3){0.0, 2.0, 0.0};
    m3CharacterId hero = m3CreateCharacter(world, &cd);
    for (int32_t i = 0; i < 40; ++i)
    {
        m3Character_Move(hero, (m3Vec3){0.0f, -0.1f, 0.0f});
    }
    double y0 = m3Character_GetPosition(hero).y;
    for (int32_t i = 0; i < 150; ++i)
    {
        m3Character_Move(hero, (m3Vec3){0.05f, -0.02f, 0.0f});
    }
    m3Pos3 p = m3Character_GetPosition(hero);
    CHECK(p.y - y0 < 0.08, "the steep face mints no height");
    m3DestroyWorld(world);
}

static void TestCarvedFloorDrop(void)
{
    // The interplay nobody else can pose: the floor is carved out
    // from under a grounded character, and grounding drops THE SAME
    // step, before any move.
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    def.voxelCapacity = 1;
    def.characterCapacity = 2; // the hero AND the bystander
    m3WorldId world = m3CreateWorld(&def);
    m3ShapeDef sd = m3DefaultShapeDef();
    static uint8_t voxels[16 * 16 * 16];
    memset(voxels, 0, sizeof(voxels));
    for (int32_t z = 0; z < 16; ++z)
    {
        for (int32_t x = 0; x < 16; ++x)
        {
            voxels[x + 16 * (0 + 16 * z)] = 1; // a one-layer floor
        }
    }
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeId slab = m3CreateVoxelChunkShape(ground, &sd, voxels, NULL, 1.0f);

    m3CharacterDef cd = m3DefaultCharacterDef();
    cd.position = (m3Pos3){8.5, 4.0, 8.5};
    m3CharacterId hero = m3CreateCharacter(world, &cd);
    for (int32_t i = 0; i < 60; ++i)
    {
        m3Character_Move(hero, (m3Vec3){0.0f, -0.1f, 0.0f});
    }
    CHECK(m3Character_IsGrounded(hero), "the character stands on the voxel floor");

    // Carve the tiles under foot: NO move happens after this; the
    // grounding must drop from the edit alone.
    int32_t lo[3] = {7, 0, 7};
    int32_t hi[3] = {10, 0, 10};
    CHECK(m3VoxelChunk_ClearBox(slab, lo, hi) == 16, "the floor vanishes under foot");
    CHECK(!m3Character_IsGrounded(hero), "grounding drops the same step the floor goes");

    // An edit elsewhere never touches a grounded character.
    m3CharacterDef cd2 = m3DefaultCharacterDef();
    cd2.position = (m3Pos3){2.5, 4.0, 2.5};
    m3CharacterId bystander = m3CreateCharacter(world, &cd2);
    for (int32_t i = 0; i < 60; ++i)
    {
        m3Character_Move(bystander, (m3Vec3){0.0f, -0.1f, 0.0f});
    }
    CHECK(m3Character_IsGrounded(bystander), "the bystander stands");
    int32_t lo2[3] = {13, 0, 13};
    int32_t hi2[3] = {14, 0, 14};
    CHECK(m3VoxelChunk_ClearBox(slab, lo2, hi2) == 4, "the far carve lands");
    CHECK(m3Character_IsGrounded(bystander), "a far edit never shakes the bystander");
    m3DestroyWorld(world);
}

static void TestWeldedSeamWalk(void)
{
    // A character crosses the chunk border of two welded slabs:
    // height wander stays inside the skin and grounding never
    // flickers (the 3-4 promise, now on foot).
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    def.voxelCapacity = 2;
    def.characterCapacity = 1;
    m3WorldId world = m3CreateWorld(&def);
    m3ShapeDef sd = m3DefaultShapeDef();
    static uint8_t voxels[16 * 16 * 16];
    memset(voxels, 0, sizeof(voxels));
    for (int32_t z = 0; z < 16; ++z)
    {
        for (int32_t x = 0; x < 16; ++x)
        {
            for (int32_t y = 0; y < 2; ++y)
            {
                voxels[x + 16 * (y + 16 * z)] = 1;
            }
        }
    }
    m3BodyDef gd = m3DefaultBodyDef();
    gd.position = (m3Pos3){0.0, 0.0, 0.0};
    m3BodyId groundA = m3CreateBody(world, &gd);
    CHECK(m3Shape_IsValid(m3CreateVoxelChunkShape(groundA, &sd, voxels, NULL, 1.0f)),
          "slab A creates");
    gd.position = (m3Pos3){16.0, 0.0, 0.0};
    m3BodyId groundB = m3CreateBody(world, &gd);
    CHECK(m3Shape_IsValid(m3CreateVoxelChunkShape(groundB, &sd, voxels, NULL, 1.0f)),
          "slab B creates and welds");

    m3CharacterDef cd = m3DefaultCharacterDef();
    cd.position = (m3Pos3){12.0, 4.0, 8.0};
    m3CharacterId hero = m3CreateCharacter(world, &cd);
    for (int32_t i = 0; i < 40; ++i)
    {
        m3Character_Move(hero, (m3Vec3){0.0f, -0.1f, 0.0f});
    }
    double y0 = m3Character_GetPosition(hero).y;
    double maxWander = 0.0;
    for (int32_t i = 0; i < 160; ++i)
    {
        m3Character_Move(hero, (m3Vec3){0.06f, -0.04f, 0.0f});
        CHECK(m3Character_IsGrounded(hero), "grounding never flickers at the seam");
        double w = m3Character_GetPosition(hero).y - y0;
        if (w < 0.0)
        {
            w = -w;
        }
        if (w > maxWander)
        {
            maxWander = w;
        }
    }
    m3Pos3 p = m3Character_GetPosition(hero);
    CHECK(p.x > 20.0, "the walk crossed the welded border");
    CHECK(maxWander < 0.03, "the seam never lifts or drops the walker");
    m3DestroyWorld(world);
}

int main(void)
{
    TestFlatWalk();
    TestWallSlideAndCeiling();
    TestLedgeDropAndSnap();
    TestMoveDeterminismAndJournal();
    TestCharacterContracts();
    TestVoxelStairs();
    TestSteepSlopeRefusal();
    TestCarvedFloorDrop();
    TestWeldedSeamWalk();
    if (s_failures == 0)
    {
        printf("test_character: all green\n");
        return 0;
    }
    printf("test_character: %d failure(s)\n", s_failures);
    return 1;
}
