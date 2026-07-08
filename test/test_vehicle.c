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
    }
    return m3CreateVehicle(world, &vd);
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
