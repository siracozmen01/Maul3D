// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The vehicle gate (5-1): the raycast suspension. A dropped chassis
// settles with every spring at the analytic balance, the whole
// bounce twins and rolls back to the bit, the pool churns and
// refuses honestly, and a destroyed chassis takes its vehicle with
// it. Wheels are casts, not bodies: nothing here touches contacts.

#include "maul3d/shape.h"
#include "maul3d/vehicle.h"

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

static m3WorldId PlaneWorld(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    def.vehicleCapacity = 2;
    m3WorldId world = m3CreateWorld(&def);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    m3CreatePlaneShape(ground, &sd, &floor);
    return world;
}

// A one-box car: half extents (1.0, 0.25, 0.5), four corner wheels.
static m3VehicleId MakeCar(m3WorldId world, m3Pos3 at, m3BodyId* outChassis)
{
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = at;
    m3BodyId chassis = m3CreateBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.density = 300.0f; // mass 300 over 1 m^3
    m3CreateBoxShape(chassis, &sd, (m3Vec3){1.0f, 0.25f, 0.5f});
    if (outChassis != NULL)
    {
        *outChassis = chassis;
    }
    m3VehicleDef vd = m3DefaultVehicleDef();
    vd.chassis = chassis;
    vd.wheelCount = 4;
    for (int32_t w = 0; w < 4; ++w)
    {
        vd.wheels[w].anchor =
            (m3Vec3){(w & 1) != 0 ? 0.8f : -0.8f, -0.25f, (w & 2) != 0 ? 0.45f : -0.45f};
        vd.wheels[w].driven = true;
        vd.wheels[w].steerable = (w & 1) != 0; // the +x pair steers
    }
    return m3CreateVehicle(world, &vd);
}

static void TestStraightLine(void)
{
    // Full throttle on a settled car: four driven wheels at 800 N
    // on 300 kg accelerate inside the analytic band, dead straight,
    // and the wheels visibly spin. Setting commands wakes a
    // sleeping chassis (the settle is long enough to sleep one).
    m3WorldId world = PlaneWorld();
    m3BodyId chassis;
    m3VehicleId car = MakeCar(world, (m3Pos3){0.0, 1.0, 0.0}, &chassis);
    for (int32_t i = 0; i < 300; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3Pos3 start = m3Body_GetPosition(chassis);
    m3Vehicle_SetCommands(car, 1.0f, 0.0f, 0.0f);
    for (int32_t i = 0; i < 120; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3Vec3 v = m3Body_GetLinearVelocity(chassis);
    m3Pos3 p = m3Body_GetPosition(chassis);
    // 3200 N / 300 kg for two seconds is 21.3 m/s in vacuum; the
    // damper phase and tire losses eat some of it.
    CHECK(v.x > 10.0f && v.x < 23.0f, "the car accelerates inside the analytic band");
    CHECK(fabsf(v.z) < 0.01f, "the run is dead straight in velocity");
    CHECK(fabs(p.z - start.z) < 0.01, "the run is dead straight in position");
    CHECK(m3Vehicle_GetWheelSpin(car, 0) > 10.0f, "the wheels spin as they roll");
    m3DestroyWorld(world);
}

static void TestSteerCircle(void)
{
    // A steady steer at modest speed circles with the turning
    // radius in the Ackermann neighborhood: R is wheelbase over
    // tan(steer angle), 1.6 / tan(0.6) = 2.3 m, and slip widens it.
    m3WorldId world = PlaneWorld();
    m3BodyId chassis;
    m3VehicleId car = MakeCar(world, (m3Pos3){0.0, 1.0, 0.0}, &chassis);
    for (int32_t i = 0; i < 120; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3Vehicle_SetCommands(car, 0.4f, 1.0f, 0.0f);
    for (int32_t i = 0; i < 600; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3Vec3 v = m3Body_GetLinearVelocity(chassis);
    m3Vec3 w = m3Body_GetAngularVelocity(chassis);
    m3real speed = sqrtf(v.x * v.x + v.z * v.z);
    CHECK(speed > 1.0f, "the circling car keeps real speed");
    CHECK(fabsf(w.y) > 0.3f, "the steer turns the car");
    m3real radius = speed / fabsf(w.y);
    CHECK(radius > 1.4f && radius < 5.0f, "the turning radius sits near Ackermann");
    m3DestroyWorld(world);
}

static void TestBrakeStop(void)
{
    // Full brake from speed: 1600 N on 300 kg stops a 15 m/s car
    // in under three seconds, and a stopped car STAYS stopped (the
    // brake clamp never reverses rolling).
    m3WorldId world = PlaneWorld();
    m3BodyId chassis;
    m3VehicleId car = MakeCar(world, (m3Pos3){0.0, 1.0, 0.0}, &chassis);
    for (int32_t i = 0; i < 120; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3Vehicle_SetCommands(car, 1.0f, 0.0f, 0.0f);
    for (int32_t i = 0; i < 120; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m3Body_GetLinearVelocity(chassis).x > 8.0f, "the car reaches braking speed");
    m3Vehicle_SetCommands(car, 0.0f, 0.0f, 1.0f);
    for (int32_t i = 0; i < 240; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3Vec3 v = m3Body_GetLinearVelocity(chassis);
    CHECK(fabsf(v.x) < 0.2f, "full brake stops the car");
    m3Pos3 before = m3Body_GetPosition(chassis);
    for (int32_t i = 0; i < 60; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3Pos3 after = m3Body_GetPosition(chassis);
    CHECK(fabs(after.x - before.x) < 0.05, "the stopped car stays stopped");
    m3DestroyWorld(world);
}

static void TestDriveDeterminism(void)
{
    // Twin worlds drive the same script (accelerate, corner, brake,
    // hostile and out-of-range commands included) onto identical
    // bits; a mid-drive rollback re-drives the tail bit-exact; the
    // journaled session replays with every command in the record.
    static uint8_t journal[65536];
    static uint8_t snap[393216];
    uint64_t hashes[2];
    for (int32_t run = 0; run < 2; ++run)
    {
        m3WorldId world = PlaneWorld();
        bool recording = run == 0 && m3World_JournalBegin(world, journal, (int32_t)sizeof(journal));
        m3VehicleId car = MakeCar(world, (m3Pos3){0.0, 1.0, 0.0}, NULL);
        float bad;
        uint32_t nanBits = 0x7FC00000u;
        memcpy(&bad, &nanBits, sizeof(bad));
        int32_t snapBytes = 0;
        for (int32_t i = 0; i < 240; ++i)
        {
            if (i == 30)
            {
                m3Vehicle_SetCommands(car, 5.0f, 0.0f, 0.0f); // clamps to 1
            }
            if (i == 90)
            {
                m3Vehicle_SetCommands(car, 0.5f, -0.7f, 0.0f);
            }
            if (i == 120)
            {
                m3Vehicle_SetCommands(car, bad, 0.0f, 0.0f); // hostile no-op
            }
            if (i == 180)
            {
                m3Vehicle_SetCommands(car, 0.0f, 0.0f, 1.0f);
            }
            m3World_Step(world, 1.0f / 60.0f, 4);
            if (i == 100 && run == 0)
            {
                snapBytes = m3World_Snapshot(world, snap, (int32_t)sizeof(snap));
                CHECK(snapBytes > 0, "the mid-drive snapshot fits");
            }
        }
        hashes[run] = m3World_Hash(world);
        if (recording)
        {
            int32_t bytes = m3World_JournalEnd(world);
            CHECK(bytes > 0, "the drive session records");
            m3WorldId replayed = PlaneWorld();
            CHECK(m3World_JournalReplay(replayed, journal, bytes), "the drive session replays");
            CHECK(m3World_Hash(replayed) == hashes[0], "the replayed drive is bit-identical");
            m3DestroyWorld(replayed);

            CHECK(m3World_Restore(world, snap, snapBytes), "the mid-drive restore lands");
            for (int32_t i = 101; i < 240; ++i)
            {
                if (i == 120)
                {
                    m3Vehicle_SetCommands(car, bad, 0.0f, 0.0f);
                }
                if (i == 180)
                {
                    m3Vehicle_SetCommands(car, 0.0f, 0.0f, 1.0f);
                }
                m3World_Step(world, 1.0f / 60.0f, 4);
            }
            CHECK(m3World_Hash(world) == hashes[0], "the re-drive is bit-identical");
        }
        m3DestroyWorld(world);
    }
    CHECK(hashes[0] == hashes[1], "twin drives are bit-identical");
}

static void TestSuspensionSettle(void)
{
    // At rest each spring holds a quarter of the weight, and with
    // stiffness = (m / 4) * omega^2 the balance compression is
    // g / omega^2 for ANY mass: hertz 1.5 and gravity 10 give
    // 10 / (2 pi 1.5)^2 = 0.1126 m. The suite reads it back.
    m3WorldId world = PlaneWorld();
    m3VehicleId car = MakeCar(world, (m3Pos3){0.0, 1.0, 0.0}, NULL);
    CHECK(m3Vehicle_IsValid(car), "the car creates");
    for (int32_t i = 0; i < 360; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    float omega = 2.0f * 3.14159265f * 1.5f;
    float balance = 10.0f / (omega * omega);
    float sum = 0.0f;
    for (int32_t w = 0; w < 4; ++w)
    {
        float x = m3Vehicle_GetCompression(car, w);
        CHECK(m3Vehicle_IsWheelGrounded(car, w), "every wheel finds the plane");
        CHECK(x > 0.8f * balance && x < 1.2f * balance,
              "the compression sits at the analytic balance");
        sum += x;
    }
    for (int32_t w = 1; w < 4; ++w)
    {
        CHECK(fabsf(m3Vehicle_GetCompression(car, w) - m3Vehicle_GetCompression(car, 0)) < 0.01f,
              "the symmetric drop loads all four corners alike");
    }
    CHECK(sum > 0.0f, "the suspension carries weight");
    m3DestroyWorld(world);
}

static void TestBounceTwinsAndRollback(void)
{
    // Twin drops hash identical; a mid-bounce snapshot re-bounces
    // onto the same bits. Journaled create replays with the id
    // verified.
    static uint8_t journal[65536];
    static uint8_t snap[393216];
    uint64_t hashes[2];
    for (int32_t run = 0; run < 2; ++run)
    {
        m3WorldId world = PlaneWorld();
        bool recording = run == 0 && m3World_JournalBegin(world, journal, (int32_t)sizeof(journal));
        m3VehicleId car = MakeCar(world, (m3Pos3){0.0, 2.5, 0.0}, NULL);
        CHECK(m3Vehicle_IsValid(car), "the twin car creates");
        int32_t snapBytes = 0;
        for (int32_t i = 0; i < 90; ++i)
        {
            m3World_Step(world, 1.0f / 60.0f, 4);
            if (i == 30 && run == 0)
            {
                snapBytes = m3World_Snapshot(world, snap, (int32_t)sizeof(snap));
                CHECK(snapBytes > 0, "the mid-bounce snapshot fits");
            }
        }
        hashes[run] = m3World_Hash(world);
        if (recording)
        {
            int32_t bytes = m3World_JournalEnd(world);
            CHECK(bytes > 0, "the drop session records");
            m3WorldId replayed = PlaneWorld();
            CHECK(m3World_JournalReplay(replayed, journal, bytes), "the drop session replays");
            CHECK(m3World_Hash(replayed) == hashes[0], "the replayed drop is bit-identical");
            m3DestroyWorld(replayed);

            CHECK(m3World_Restore(world, snap, snapBytes), "the mid-bounce restore lands");
            for (int32_t i = 31; i < 90; ++i)
            {
                m3World_Step(world, 1.0f / 60.0f, 4);
            }
            CHECK(m3World_Hash(world) == hashes[0], "the re-bounce is bit-identical");
        }
        m3DestroyWorld(world);
    }
    CHECK(hashes[0] == hashes[1], "twin drops are bit-identical");
}

static void TestVoxelDeckAndSeam(void)
{
    // Two welded chunks make one long deck: the car drives across
    // the seam without a hitch, dead straight, at speed.
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    def.voxelCapacity = 2;
    def.vehicleCapacity = 1;
    m3WorldId world = m3CreateWorld(&def);
    static uint8_t deck[16 * 16 * 16];
    memset(deck, 0, sizeof(deck));
    for (int32_t z = 0; z < 16; ++z)
    {
        for (int32_t x = 0; x < 16; ++x)
        {
            deck[x + 16 * (0 + 16 * z)] = 1;
            deck[x + 16 * (1 + 16 * z)] = 1;
        }
    }
    m3ShapeDef sd = m3DefaultShapeDef();
    m3BodyDef ga = m3DefaultBodyDef();
    ga.position = (m3Pos3){-8.0, 0.0, -4.0};
    CHECK(m3Shape_IsValid(m3CreateVoxelChunkShape(m3CreateBody(world, &ga), &sd, deck, NULL, 0.5f)),
          "the west deck creates");
    m3BodyDef gb = m3DefaultBodyDef();
    gb.position = (m3Pos3){0.0, 0.0, -4.0}; // exactly one extent east: welded
    CHECK(m3Shape_IsValid(m3CreateVoxelChunkShape(m3CreateBody(world, &gb), &sd, deck, NULL, 0.5f)),
          "the east deck creates");

    m3BodyId chassis;
    m3VehicleId car = MakeCar(world, (m3Pos3){-5.0, 2.0, 0.0}, &chassis);
    CHECK(m3Vehicle_IsValid(car), "the deck car creates");
    for (int32_t i = 0; i < 90; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3Vehicle_SetCommands(car, 0.5f, 0.0f, 0.0f);
    for (int32_t i = 0; i < 300; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
        m3Pos3 q = m3Body_GetPosition(chassis);
        CHECK(q.y > 1.3 && q.y < 2.2, "the deck ride never hitches");
        if (q.x > 5.0)
        {
            break;
        }
    }
    m3Pos3 p = m3Body_GetPosition(chassis);
    CHECK(p.x > 3.0, "the car crosses the welded seam");
    CHECK(fabs(p.z) < 0.05, "the seam crossing stays dead straight");
    m3DestroyWorld(world);
}

static void TestRampClimbAndWall(void)
{
    // A quarter-cell voxel ramp climbs like rough road; a two-meter
    // voxel wall is a wall. The suspension casts do not care about
    // grade, the chassis contact does.
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    def.voxelCapacity = 2;
    def.vehicleCapacity = 1;
    m3WorldId world = m3CreateWorld(&def);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    m3CreatePlaneShape(ground, &sd, &floor);

    // The ramp chunk: one layer per two cells from x = 4.
    static uint8_t ramp[16 * 16 * 16];
    memset(ramp, 0, sizeof(ramp));
    for (int32_t z = 0; z < 16; ++z)
    {
        for (int32_t x = 0; x < 16; ++x)
        {
            int32_t top = x < 4 ? 0 : (x - 4) / 2;
            for (int32_t y = 0; y <= top; ++y)
            {
                ramp[x + 16 * (y + 16 * z)] = 1;
            }
        }
    }
    m3BodyDef rd = m3DefaultBodyDef();
    rd.position = (m3Pos3){0.0, 0.0, -2.0};
    CHECK(
        m3Shape_IsValid(m3CreateVoxelChunkShape(m3CreateBody(world, &rd), &sd, ramp, NULL, 0.25f)),
        "the ramp creates");

    // The wall chunk: a two-meter cliff just past the ramp.
    static uint8_t wall[16 * 16 * 16];
    memset(wall, 0, sizeof(wall));
    for (int32_t z = 0; z < 16; ++z)
    {
        for (int32_t y = 0; y < 8; ++y)
        {
            for (int32_t x = 0; x < 2; ++x)
            {
                wall[x + 16 * (y + 16 * z)] = 1;
            }
        }
    }
    m3BodyDef wd = m3DefaultBodyDef();
    wd.position = (m3Pos3){6.0, 1.5, -2.0};
    CHECK(
        m3Shape_IsValid(m3CreateVoxelChunkShape(m3CreateBody(world, &wd), &sd, wall, NULL, 0.25f)),
        "the wall creates");

    m3BodyId chassis;
    m3VehicleId car = MakeCar(world, (m3Pos3){-2.5, 1.0, 0.0}, &chassis);
    for (int32_t i = 0; i < 90; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    double y0 = m3Body_GetPosition(chassis).y;
    m3Vehicle_SetCommands(car, 0.6f, 0.0f, 0.0f);
    double peak = y0;
    for (int32_t i = 0; i < 420; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
        double y = m3Body_GetPosition(chassis).y;
        peak = y > peak ? y : peak;
    }
    m3Pos3 p = m3Body_GetPosition(chassis);
    CHECK(peak - y0 > 0.8, "the quarter-cell ramp climbs like road");
    CHECK(p.x < 6.2, "the two-meter wall is a wall");
    CHECK(isfinite(p.x) && isfinite(p.y) && isfinite(p.z), "the blocked car stays finite");
    m3DestroyWorld(world);
}

static void TestCarveUnderParkedCar(void)
{
    // A car parked long enough to sleep drops the SAME step its
    // deck is carved away: the wheel rays overlap the region, the
    // chassis wakes, and the suspension reads the void. Without
    // the wheel-ray wake the sleeper hovers on vanished floor.
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    def.voxelCapacity = 1;
    def.vehicleCapacity = 1;
    m3WorldId world = m3CreateWorld(&def);
    static uint8_t deck[16 * 16 * 16];
    memset(deck, 0, sizeof(deck));
    for (int32_t z = 0; z < 16; ++z)
    {
        for (int32_t x = 0; x < 16; ++x)
        {
            deck[x + 16 * (0 + 16 * z)] = 1;
            deck[x + 16 * (1 + 16 * z)] = 1;
        }
    }
    m3BodyDef gd = m3DefaultBodyDef();
    gd.position = (m3Pos3){-4.0, 0.0, -4.0};
    m3ShapeDef sd = m3DefaultShapeDef();
    m3ShapeId chunk = m3CreateVoxelChunkShape(m3CreateBody(world, &gd), &sd, deck, NULL, 0.5f);
    CHECK(m3Shape_IsValid(chunk), "the parking deck creates");

    m3BodyId chassis;
    m3VehicleId car = MakeCar(world, (m3Pos3){0.0, 2.2, 0.0}, &chassis);
    CHECK(m3Vehicle_IsValid(car), "the parked car creates");
    for (int32_t i = 0; i < 600; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4); // long enough to sleep
    }
    double parked = m3Body_GetPosition(chassis).y;
    CHECK(parked > 1.4 && parked < 2.0, "the car parks on the deck");

    // Carve everything under and around the car.
    int32_t lo[3] = {4, 0, 4};
    int32_t hi[3] = {12, 1, 12};
    CHECK(m3VoxelChunk_ClearBox(chunk, lo, hi) > 0, "the carve clears the deck");
    for (int32_t i = 0; i < 120; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3Pos3 p = m3Body_GetPosition(chassis);
    CHECK(parked - p.y > 0.5, "the sleeping car drops the step its floor vanishes");
    m3DestroyWorld(world);
}

static void TestStormUnderMovingCar(void)
{
    // Columns vanish ahead of a moving car, fragments rain, and
    // twin worlds plus the wheel books agree to the bit.
    uint64_t hashes[2];
    for (int32_t run = 0; run < 2; ++run)
    {
        m3WorldDef def = m3DefaultWorldDef();
        def.bodyCapacity = 16;
        def.shapeCapacity = 16;
        def.voxelCapacity = 1;
        def.vehicleCapacity = 1;
        m3WorldId world = m3CreateWorld(&def);
        static uint8_t deck[16 * 16 * 16];
        memset(deck, 0, sizeof(deck));
        for (int32_t z = 0; z < 16; ++z)
        {
            for (int32_t x = 0; x < 16; ++x)
            {
                deck[x + 16 * (0 + 16 * z)] = 1;
                deck[x + 16 * (1 + 16 * z)] = 1;
            }
        }
        m3BodyDef gd = m3DefaultBodyDef();
        gd.position = (m3Pos3){-4.0, 0.0, -4.0};
        m3ShapeDef sd = m3DefaultShapeDef();
        m3ShapeId chunk = m3CreateVoxelChunkShape(m3CreateBody(world, &gd), &sd, deck, NULL, 0.5f);
        m3BodyId chassis;
        m3VehicleId car = MakeCar(world, (m3Pos3){-2.0, 2.2, 0.0}, &chassis);
        for (int32_t i = 0; i < 60; ++i)
        {
            m3World_Step(world, 1.0f / 60.0f, 4);
        }
        m3Vehicle_SetCommands(car, 0.5f, 0.0f, 0.0f);
        for (int32_t i = 0; i < 90; ++i)
        {
            if (i % 6 == 3)
            {
                int32_t cx = 6 + (i / 6);
                if (cx <= 15)
                {
                    int32_t lo[3] = {cx, 1, 6};
                    int32_t hi[3] = {cx, 1, 10};
                    m3VoxelChunk_ClearBox(chunk, lo, hi);
                }
            }
            m3World_Step(world, 1.0f / 60.0f, 4);
            int32_t count = 0;
            (void)m3World_FragmentEvents(world, &count);
            m3Pos3 p = m3Body_GetPosition(chassis);
            CHECK(isfinite(p.x) && isfinite(p.y) && isfinite(p.z), "the storm drive stays finite");
        }
        hashes[run] = m3World_Hash(world);
        m3DestroyWorld(world);
    }
    CHECK(hashes[0] == hashes[1], "twin storm drives are bit-identical");
}

static void TestHeightfieldDrive(void)
{
    // Rolling terrain through the heightfield path: the car crosses
    // the swells without losing its line.
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    def.vehicleCapacity = 1;
    m3WorldId world = m3CreateWorld(&def);
    static float heights[32 * 32];
    for (int32_t z = 0; z < 32; ++z)
    {
        for (int32_t x = 0; x < 32; ++x)
        {
            heights[x + 32 * z] = 0.25f * sinf((float)x * 0.6f);
        }
    }
    m3BodyDef gd = m3DefaultBodyDef();
    gd.position = (m3Pos3){-8.0, 0.0, -8.0};
    m3ShapeDef sd = m3DefaultShapeDef();
    CHECK(m3Shape_IsValid(
              m3CreateHeightFieldShape(m3CreateBody(world, &gd), &sd, heights, 32, 32, 0.5f)),
          "the terrain creates");

    m3BodyId chassis;
    m3VehicleId car = MakeCar(world, (m3Pos3){-5.0, 1.5, 0.0}, &chassis);
    for (int32_t i = 0; i < 90; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3Vehicle_SetCommands(car, 0.5f, 0.0f, 0.0f);
    for (int32_t i = 0; i < 300; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
        if (m3Body_GetPosition(chassis).x > 4.0)
        {
            break; // stop well before the terrain's east edge
        }
    }
    m3Pos3 p = m3Body_GetPosition(chassis);
    CHECK(p.x > -1.0, "the car crosses the swells");
    CHECK(fabs(p.z) < 0.2, "the swells do not steal the line");
    CHECK(p.y > 0.5 && p.y < 2.5, "the ride height stays sane");
    m3DestroyWorld(world);
}

static void TestPoolAndCascade(void)
{
    // Capacity two: the third car refuses; destroying a chassis
    // destroys its vehicle (the cascade rule); stale handles no-op
    // and read zeros.
    m3WorldId world = PlaneWorld();
    m3BodyId chassisA;
    m3VehicleId a = MakeCar(world, (m3Pos3){0.0, 1.0, 0.0}, &chassisA);
    m3VehicleId b = MakeCar(world, (m3Pos3){4.0, 1.0, 0.0}, NULL);
    CHECK(m3Vehicle_IsValid(a) && m3Vehicle_IsValid(b), "the pool serves two");
    m3VehicleId c = MakeCar(world, (m3Pos3){8.0, 1.0, 0.0}, NULL);
    CHECK(!m3Vehicle_IsValid(c), "the full pool refuses the third");

    m3DestroyBody(chassisA);
    CHECK(!m3Vehicle_IsValid(a), "the chassis takes its vehicle with it");
    CHECK(m3Vehicle_GetCompression(a, 0) == 0.0f, "a stale handle reads zero");
    CHECK(!m3Vehicle_IsWheelGrounded(a, 0), "a stale handle grounds nothing");
    m3DestroyVehicle(a); // stale: no-op
    m3World_Step(world, 1.0f / 60.0f, 4);
    CHECK(m3Vehicle_IsValid(b), "the survivor drives on");

    m3VehicleId d = MakeCar(world, (m3Pos3){8.0, 1.0, 0.0}, NULL);
    CHECK(m3Vehicle_IsValid(d), "the freed slot serves again");
    m3DestroyWorld(world);
}

static void TestFerryRideAndChurn(void)
{
    // Red team (5-4): a car parked on a moving kinematic ferry
    // rides it (the tire works in surface-relative velocity), and
    // stops with it; then vehicle churn twins prove the pool under
    // create/destroy load.
    m3WorldId world = PlaneWorld();
    m3ShapeDef sd = m3DefaultShapeDef();
    m3BodyDef fd = m3DefaultBodyDef();
    fd.type = m3_kinematicBody;
    fd.position = (m3Pos3){0.0, 1.0, 0.0};
    m3BodyId ferry = m3CreateBody(world, &fd);
    m3CreateBoxShape(ferry, &sd, (m3Vec3){3.0f, 0.25f, 3.0f}); // top at 1.25

    m3BodyId chassis;
    m3VehicleId car = MakeCar(world, (m3Pos3){0.0, 2.5, 0.0}, &chassis);
    CHECK(m3Vehicle_IsValid(car), "the ferry car creates");
    for (int32_t i = 0; i < 120; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    double x0 = m3Body_GetPosition(chassis).x;
    // Handbrake on: free-rolling wheels have no longitudinal grip
    // by design (that is what wheels are), so parking on a ferry
    // that sails along the car's forward axis takes the brake.
    m3Vehicle_SetCommands(car, 0.0f, 0.0f, 1.0f);
    m3Body_SetLinearVelocity(ferry, (m3Vec3){0.6f, 0.0f, 0.0f});
    for (int32_t i = 0; i < 120; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3Pos3 p = m3Body_GetPosition(chassis);
    CHECK(p.x - x0 > 0.9 && p.x - x0 < 1.3, "the parked car rides the ferry");
    m3Body_SetLinearVelocity(ferry, (m3Vec3){0.0f, 0.0f, 0.0f});
    for (int32_t i = 0; i < 90; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3Vec3 v = m3Body_GetLinearVelocity(chassis);
    CHECK(fabsf(v.x) < 0.05f, "the car stops with its ferry");
    m3DestroyWorld(world);

    // Churn twins: create, drive, destroy, repeat, on two worlds.
    uint64_t hashes[2];
    for (int32_t run = 0; run < 2; ++run)
    {
        m3WorldId sea = PlaneWorld();
        for (int32_t cycle = 0; cycle < 6; ++cycle)
        {
            m3BodyId body;
            m3VehicleId veh = MakeCar(sea, (m3Pos3){(double)(cycle % 3) * 3.0, 1.0, 0.0}, &body);
            CHECK(m3Vehicle_IsValid(veh), "the churn car creates");
            m3Vehicle_SetCommands(veh, 0.7f, cycle % 2 == 0 ? 0.4f : -0.4f, 0.0f);
            for (int32_t i = 0; i < 30; ++i)
            {
                m3World_Step(sea, 1.0f / 60.0f, 4);
            }
            if (cycle % 2 == 0)
            {
                m3DestroyVehicle(veh); // the chassis remains a plain body
            }
            else
            {
                m3DestroyBody(body); // the cascade path
            }
        }
        hashes[run] = m3World_Hash(sea);
        m3DestroyWorld(sea);
    }
    CHECK(hashes[0] == hashes[1], "twin churns are bit-identical");
}

static void TestDriveOnFragment(void)
{
    // Newton holds on loose ground: a car driving across a
    // free-sliding slab (a raft on a frictionless table: held up,
    // free sideways, no outside friction to hide the books) kicks the slab backward
    // while the car goes forward. The first draft parked the slab
    // on an icy plane, which proved nothing once the rev-20
    // friction fix landed: HONEST static friction holds a slab
    // that diseased frictionless hovering used to let slide.
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    def.vehicleCapacity = 1;
    m3WorldId world = m3CreateWorld(&def);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef gs = m3DefaultShapeDef();
    gs.friction = 0.0f; // a frictionless table: it holds the raft
                        // up and hides nothing sideways (geometric
                        // mixing makes the pair friction zero)
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    m3CreatePlaneShape(ground, &gs, &floor);

    m3BodyDef slabDef = m3DefaultBodyDef();
    slabDef.type = m3_dynamicBody;
    slabDef.position = (m3Pos3){0.0, 0.15, 0.0};
    m3BodyId slab = m3CreateBody(world, &slabDef);
    m3ShapeDef ss = m3DefaultShapeDef();
    ss.density = 500.0f; // 4 x 0.3 x 4 slab: 2400 kg
    m3CreateBoxShape(slab, &ss, (m3Vec3){2.0f, 0.15f, 2.0f});

    m3BodyId chassis;
    m3VehicleId car = MakeCar(world, (m3Pos3){-1.0, 1.2, 0.0}, &chassis);
    for (int32_t i = 0; i < 90; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3Vehicle_SetCommands(car, 1.0f, 0.0f, 0.0f);
    for (int32_t i = 0; i < 40; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3Vec3 vCar = m3Body_GetLinearVelocity(chassis);
    m3Vec3 vSlab = m3Body_GetLinearVelocity(slab);
    CHECK(vCar.x > 0.5f, "the car drives forward off the raft");
    CHECK(vSlab.x < -0.05f, "the raft takes the drive reaction backward");
    m3DestroyWorld(world);
}

static void TestVehicleContracts(void)
{
    // Hostile defs refuse with the null id, loudly documented.
    m3WorldId world = PlaneWorld();
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 1.0, 0.0};
    m3BodyId chassis = m3CreateBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3CreateBoxShape(chassis, &sd, (m3Vec3){1.0f, 0.25f, 0.5f});

    m3VehicleDef vd = m3DefaultVehicleDef();
    vd.chassis = chassis;
    vd.wheelCount = 0;
    CHECK(!m3Vehicle_IsValid(m3CreateVehicle(world, &vd)), "zero wheels refuse");
    vd.wheelCount = M3_VEHICLE_MAX_WHEELS + 1;
    CHECK(!m3Vehicle_IsValid(m3CreateVehicle(world, &vd)), "nine wheels refuse");
    vd.wheelCount = 4;
    vd.wheels[2].radius = 0.0f;
    CHECK(!m3Vehicle_IsValid(m3CreateVehicle(world, &vd)), "a zero radius refuses");
    vd = m3DefaultVehicleDef();
    vd.chassis = chassis;
    vd.wheelCount = 4;
    vd.wheels[1].travel = vd.wheels[1].restLength + 0.1f;
    CHECK(!m3Vehicle_IsValid(m3CreateVehicle(world, &vd)), "travel past rest length refuses");
    vd = m3DefaultVehicleDef();
    vd.chassis = chassis;
    vd.wheelCount = 4;
    vd.wheels[0].direction = (m3Vec3){0.0f, 0.0f, 0.0f};
    CHECK(!m3Vehicle_IsValid(m3CreateVehicle(world, &vd)), "a zero direction refuses");
    vd = m3DefaultVehicleDef();
    vd.chassis = chassis;
    vd.wheelCount = 4;
    float bad;
    uint32_t nanBits = 0x7FC00000u;
    memcpy(&bad, &nanBits, sizeof(bad));
    vd.wheels[3].hertz = bad;
    CHECK(!m3Vehicle_IsValid(m3CreateVehicle(world, &vd)), "a NaN hertz refuses");

    // A static chassis refuses (vehicles spring against dynamics).
    m3BodyDef st = m3DefaultBodyDef();
    st.position = (m3Pos3){4.0, 1.0, 0.0};
    m3BodyId wallBody = m3CreateBody(world, &st);
    vd = m3DefaultVehicleDef();
    vd.chassis = wallBody;
    vd.wheelCount = 4;
    CHECK(!m3Vehicle_IsValid(m3CreateVehicle(world, &vd)), "a static chassis refuses");

    // The airborne contract: a car in the air reads zero compression
    // and no contact on every wheel.
    m3VehicleDef good = m3DefaultVehicleDef();
    good.chassis = chassis;
    good.wheelCount = 4;
    for (int32_t w = 0; w < 4; ++w)
    {
        good.wheels[w].anchor =
            (m3Vec3){(w & 1) != 0 ? 0.8f : -0.8f, -0.25f, (w & 2) != 0 ? 0.45f : -0.45f};
    }
    m3VehicleId car = m3CreateVehicle(world, &good);
    CHECK(m3Vehicle_IsValid(car), "the good def creates");
    m3Body_SetLinearVelocity(chassis, (m3Vec3){0.0f, 10.0f, 0.0f});
    m3World_Step(world, 1.0f / 60.0f, 4);
    for (int32_t i = 0; i < 30; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m3Vehicle_GetCompression(car, 5) == 0.0f, "an out-of-range wheel reads zero");
    m3DestroyWorld(world);
}

int main(void)
{
    TestSuspensionSettle();
    TestStraightLine();
    TestSteerCircle();
    TestBrakeStop();
    TestDriveDeterminism();
    TestVoxelDeckAndSeam();
    TestRampClimbAndWall();
    TestCarveUnderParkedCar();
    TestStormUnderMovingCar();
    TestHeightfieldDrive();
    TestFerryRideAndChurn();
    TestDriveOnFragment();
    TestBounceTwinsAndRollback();
    TestPoolAndCascade();
    TestVehicleContracts();
    if (s_failures == 0)
    {
        printf("test_vehicle: all green\n");
        return 0;
    }
    printf("test_vehicle: %d failure(s)\n", s_failures);
    return 1;
}
