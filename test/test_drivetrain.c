// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The drivetrain gate (12-1): the engine curve and gearbox. A
// geared car climbs a slope that bogs it in top gear (the whole
// point of gears), auto shift walks up through a flat sprint at
// the pinned RPMs on identical bits, a rollback lands mid-shift
// with the clutch open and re-runs to the same hash, the journal
// replays every def and gear select, and hostile defs bounce off
// the validation wall without touching state.

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

// A plane tilted about z: uphill is +x. Fifteen degrees is enough
// for gravity to beat top gear and lose to first (the analytic
// heart of the bog-or-climb test).
static m3WorldId RampWorld(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    def.vehicleCapacity = 2;
    m3WorldId world = m3CreateWorld(&def);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane slope = {{-0.25881904510252076f, 0.96592582628906829f, 0.0f}, 0.0f};
    m3CreatePlaneShape(ground, &sd, &slope);
    return world;
}

// The same one-box car the vehicle gate drives: half extents
// (1.0, 0.25, 0.5), 300 kg, four corner wheels, all driven.
static m3VehicleId MakeCar(m3WorldId world, m3Pos3 at, m3BodyId* outChassis)
{
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = at;
    m3BodyId chassis = m3CreateBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.density = 300.0f;
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
        vd.wheels[w].steerable = (w & 1) != 0;
    }
    return m3CreateVehicle(world, &vd);
}

static void TestUphillBogAndClimb(void)
{
    // Same slope, same car, same throttle; only the gear differs.
    // Top gear multiplies idle torque below the gravity component
    // and the car bogs; first gear multiplies it past and the car
    // climbs. Flat driveForce could only ever do one of the two.
    double progress[2];
    for (int32_t attempt = 0; attempt < 2; ++attempt)
    {
        m3WorldId world = RampWorld();
        m3BodyId chassis;
        m3VehicleId car = MakeCar(world, (m3Pos3){0.0, 1.0, 0.0}, &chassis);
        m3DrivetrainDef dt = m3DefaultDrivetrainDef();
        dt.autoShift = false; // the gear is the experiment variable
        m3Vehicle_SetDrivetrain(car, &dt);
        m3Vehicle_SelectGear(car, attempt == 0 ? 5 : 1);
        for (int32_t i = 0; i < 120; ++i)
        {
            m3World_Step(world, 1.0f / 60.0f, 4); // settle on the slope
        }
        double startX = m3Body_GetPosition(chassis).x;
        m3Vehicle_SetCommands(car, 1.0f, 0.0f, 0.0f);
        for (int32_t i = 0; i < 300; ++i)
        {
            m3World_Step(world, 1.0f / 60.0f, 4);
        }
        progress[attempt] = m3Body_GetPosition(chassis).x - startX;
        CHECK(m3Vehicle_GetGear(car) == (attempt == 0 ? 5 : 1), "the manual gear holds");
        m3DestroyWorld(world);
    }
    CHECK(progress[0] < 0.5, "top gear bogs on the slope");
    CHECK(progress[1] > 2.0, "first gear climbs the same slope");
    CHECK(progress[1] > progress[0] + 2.0, "the gearbox is the difference");
}

static void TestAutoShiftSprintTwins(void)
{
    // A flat full-throttle sprint climbs the gearbox at the pinned
    // shift RPM, never downshifts, and twin runs land on one hash.
    uint64_t hashes[2];
    for (int32_t run = 0; run < 2; ++run)
    {
        m3WorldId world = PlaneWorld();
        m3BodyId chassis;
        m3VehicleId car = MakeCar(world, (m3Pos3){0.0, 1.0, 0.0}, &chassis);
        m3DrivetrainDef dt = m3DefaultDrivetrainDef();
        m3Vehicle_SetDrivetrain(car, &dt);
        for (int32_t i = 0; i < 120; ++i)
        {
            m3World_Step(world, 1.0f / 60.0f, 4);
        }
        m3Vehicle_SetCommands(car, 1.0f, 0.0f, 0.0f);
        int32_t lastGear = m3Vehicle_GetGear(car);
        int32_t shifts = 0;
        m3real rpmBefore = 0.0f;
        for (int32_t i = 0; i < 1500; ++i)
        {
            m3real rpmPrev = m3Vehicle_GetEngineRpm(car);
            m3World_Step(world, 1.0f / 60.0f, 4);
            int32_t gear = m3Vehicle_GetGear(car);
            if (run == 0)
            {
                CHECK(gear >= lastGear, "a flat sprint never downshifts");
                if (gear > lastGear)
                {
                    shifts += 1;
                    if (shifts == 1)
                    {
                        rpmBefore = rpmPrev;
                    }
                }
            }
            lastGear = gear;
        }
        if (run == 0)
        {
            CHECK(lastGear >= 4, "the sprint climbs at least to fourth");
            CHECK(shifts >= 3, "at least three upshifts happen");
            CHECK(rpmBefore > 5300.0f, "the first upshift waits for the pinned RPM");
            m3Vec3 v = m3Body_GetLinearVelocity(chassis);
            CHECK(v.x > 20.0f, "the geared sprint is genuinely fast");
        }
        hashes[run] = m3World_Hash(world);
        m3DestroyWorld(world);
    }
    CHECK(hashes[0] == hashes[1], "twin sprints are bit-identical");
}

static void TestMidShiftRollback(void)
{
    // Snapshot with the clutch OPEN (right after an upshift) and
    // prove the re-run lands on the same bits: gear, countdown, and
    // tachometer are all state, so rollback resumes the shift
    // mid-motion instead of inventing a closed clutch.
    static uint8_t snap[2097152];
    m3WorldId world = PlaneWorld();
    m3VehicleId car = MakeCar(world, (m3Pos3){0.0, 1.0, 0.0}, NULL);
    m3DrivetrainDef dt = m3DefaultDrivetrainDef();
    m3Vehicle_SetDrivetrain(car, &dt);
    for (int32_t i = 0; i < 120; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3Vehicle_SetCommands(car, 1.0f, 0.0f, 0.0f);
    int32_t snapBytes = 0;
    int32_t lastGear = m3Vehicle_GetGear(car);
    for (int32_t i = 0; i < 1200 && snapBytes == 0; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
        int32_t gear = m3Vehicle_GetGear(car);
        if (gear > lastGear)
        {
            // The clutch countdown is 12 steps; this snapshot sits
            // squarely inside the torque cut.
            m3World_Step(world, 1.0f / 60.0f, 4);
            m3World_Step(world, 1.0f / 60.0f, 4);
            snapBytes = m3World_Snapshot(world, snap, (int32_t)sizeof(snap));
            CHECK(snapBytes > 0, "the mid-shift snapshot fits");
        }
        lastGear = gear;
    }
    CHECK(snapBytes > 0, "an upshift happened to snapshot");
    for (int32_t i = 0; i < 200; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    uint64_t after = m3World_Hash(world);
    CHECK(m3World_Restore(world, snap, snapBytes), "the mid-shift restore lands");
    for (int32_t i = 0; i < 200; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m3World_Hash(world) == after, "the re-run through the shift is bit-identical");
    m3DestroyWorld(world);
}

static void TestJournalReplay(void)
{
    // A manual-gearbox session: attach, drive through two upshifts,
    // brake, back into reverse. Two hostile calls sit mid-session
    // (a garbage def, an out-of-range gear) and journal NOTHING.
    static uint8_t journal[65536];
    m3WorldId world = PlaneWorld();
    bool recording = m3World_JournalBegin(world, journal, (int32_t)sizeof(journal));
    CHECK(recording, "the journal opens");
    m3VehicleId car = MakeCar(world, (m3Pos3){0.0, 1.0, 0.0}, NULL);
    m3DrivetrainDef dt = m3DefaultDrivetrainDef();
    dt.autoShift = false;
    m3Vehicle_SetDrivetrain(car, &dt);
    m3DrivetrainDef bad = m3DefaultDrivetrainDef();
    bad.curveRpm[2] = bad.curveRpm[1]; // not strictly ascending
    for (int32_t i = 0; i < 420; ++i)
    {
        if (i == 30)
        {
            m3Vehicle_SetCommands(car, 1.0f, 0.0f, 0.0f);
        }
        if (i == 120)
        {
            m3Vehicle_SelectGear(car, 2);
        }
        if (i == 150)
        {
            m3Vehicle_SetDrivetrain(car, &bad); // hostile no-op
            m3Vehicle_SelectGear(car, 9);       // hostile no-op
        }
        if (i == 240)
        {
            m3Vehicle_SelectGear(car, 3);
        }
        if (i == 300)
        {
            m3Vehicle_SetCommands(car, 0.0f, 0.0f, 1.0f);
        }
        if (i == 390)
        {
            m3Vehicle_SetCommands(car, 0.6f, 0.0f, 0.0f);
            m3Vehicle_SelectGear(car, -1);
        }
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m3Vehicle_GetGear(car) == -1, "the session ends in reverse");
    uint64_t final = m3World_Hash(world);
    int32_t bytes = m3World_JournalEnd(world);
    CHECK(bytes > 0, "the session records");
    m3WorldId replayed = PlaneWorld();
    CHECK(m3World_JournalReplay(replayed, journal, bytes), "the session replays");
    CHECK(m3World_Hash(replayed) == final, "the replayed session is bit-identical");
    m3DestroyWorld(replayed);
    m3DestroyWorld(world);
}

static void TestNeutralAndReverse(void)
{
    // Neutral holds the car still under full throttle; reverse
    // drives it backward off throttle magnitude alone (with a
    // drivetrain, direction belongs to the gear, not the pedal).
    m3WorldId world = PlaneWorld();
    m3BodyId chassis;
    m3VehicleId car = MakeCar(world, (m3Pos3){0.0, 1.0, 0.0}, &chassis);
    m3DrivetrainDef dt = m3DefaultDrivetrainDef();
    dt.autoShift = false;
    m3Vehicle_SetDrivetrain(car, &dt);
    m3Vehicle_SelectGear(car, 0);
    for (int32_t i = 0; i < 120; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    double startX = m3Body_GetPosition(chassis).x;
    m3Vehicle_SetCommands(car, 1.0f, 0.0f, 0.0f);
    for (int32_t i = 0; i < 120; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(fabs(m3Body_GetPosition(chassis).x - startX) < 0.1, "neutral drives nothing");
    m3Vehicle_SelectGear(car, -1);
    for (int32_t i = 0; i < 180; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m3Body_GetPosition(chassis).x < startX - 1.0, "reverse drives backward");
    m3DestroyWorld(world);
}

static void TestHostileWall(void)
{
    // Every malformed def and gear select is a documented no-op:
    // the car keeps its flat model (gear reads 0) and the world
    // keeps stepping. The validation wall is the same one replay
    // uses against fuzzed journal bytes.
    m3WorldId world = PlaneWorld();
    m3VehicleId car = MakeCar(world, (m3Pos3){0.0, 1.0, 0.0}, NULL);

    m3DrivetrainDef dt = m3DefaultDrivetrainDef();
    dt.internalValue = 0; // bad cookie
    m3Vehicle_SetDrivetrain(car, &dt);
    CHECK(m3Vehicle_GetGear(car) == 0, "a bad cookie bounces");

    dt = m3DefaultDrivetrainDef();
    dt.curveCount = 1;
    m3Vehicle_SetDrivetrain(car, &dt);
    dt = m3DefaultDrivetrainDef();
    dt.curveCount = 99;
    m3Vehicle_SetDrivetrain(car, &dt);
    dt = m3DefaultDrivetrainDef();
    dt.curveRpm[1] = dt.curveRpm[0]; // not ascending
    m3Vehicle_SetDrivetrain(car, &dt);
    dt = m3DefaultDrivetrainDef();
    uint32_t nanBits = 0x7FC00000u;
    memcpy(&dt.curveTorque[1], &nanBits, sizeof(float));
    m3Vehicle_SetDrivetrain(car, &dt);
    dt = m3DefaultDrivetrainDef();
    dt.gearCount = 0;
    m3Vehicle_SetDrivetrain(car, &dt);
    dt = m3DefaultDrivetrainDef();
    dt.gearRatio[2] = -1.0f;
    m3Vehicle_SetDrivetrain(car, &dt);
    dt = m3DefaultDrivetrainDef();
    dt.shiftUpRpm = dt.shiftDownRpm; // shift band collapses
    m3Vehicle_SetDrivetrain(car, &dt);
    dt = m3DefaultDrivetrainDef();
    dt.clutchSteps = -5;
    m3Vehicle_SetDrivetrain(car, &dt);
    dt = m3DefaultDrivetrainDef();
    dt.finalDrive = 0.0f;
    m3Vehicle_SetDrivetrain(car, &dt);
    CHECK(m3Vehicle_GetGear(car) == 0, "nine garbage defs all bounce");
    CHECK(m3Vehicle_GetEngineRpm(car) == 0.0f, "no drivetrain, no tachometer");

    m3Vehicle_SelectGear(car, 3); // no drivetrain attached
    CHECK(m3Vehicle_GetGear(car) == 0, "gear select without a drivetrain bounces");

    dt = m3DefaultDrivetrainDef();
    dt.reverseRatio = 0.0f; // this car forbids reverse
    m3Vehicle_SetDrivetrain(car, &dt);
    CHECK(m3Vehicle_GetGear(car) == 1, "the valid def lands in first");
    m3Vehicle_SelectGear(car, -1);
    CHECK(m3Vehicle_GetGear(car) == 1, "reverse without a reverse ratio bounces");
    m3Vehicle_SelectGear(car, 6); // gearCount is 5
    CHECK(m3Vehicle_GetGear(car) == 1, "an out-of-range gear bounces");

    m3Vehicle_SetDrivetrain(m3_nullVehicleId, &dt); // stale id no-op
    m3Vehicle_SelectGear(m3_nullVehicleId, 1);
    for (int32_t i = 0; i < 60; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m3Vehicle_GetGear(car) == 1, "the world steps on through all of it");
    m3DestroyWorld(world);
}

int main(void)
{
    TestUphillBogAndClimb();
    TestAutoShiftSprintTwins();
    TestMidShiftRollback();
    TestJournalReplay();
    TestNeutralAndReverse();
    TestHostileWall();
    if (s_failures == 0)
    {
        printf("test_drivetrain: all passed\n");
        return 0;
    }
    return 1;
}
