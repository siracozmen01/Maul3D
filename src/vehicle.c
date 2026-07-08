// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The raycast vehicle (5-1): the suspension pass. Wheels cast rays
// from the chassis (the chassis ignores itself through the ray
// hook), springs push back through hertz and zeta scaled by a
// quarter of the chassis mass per wheel, and every impulse lands
// before the solver prepares contacts, so the solver sees a sprung
// chassis exactly the way it sees gravity. Serial, slot order,
// canonical: twin worlds drive on identical bits.

#include "maul3d/vehicle.h"

#include "world_internal.h"

#include <math.h>
#include <string.h>

#define M3_VEHICLE_COOKIE ((int32_t)(M3_COOKIE ^ ((int32_t)sizeof(m3VehicleDef) << 8) ^ 8))

m3VehicleDef m3DefaultVehicleDef(void)
{
    m3VehicleDef def;
    memset(&def, 0, sizeof(def));
    for (int32_t w = 0; w < M3_VEHICLE_MAX_WHEELS; ++w)
    {
        def.wheels[w].direction = (m3Vec3){0.0f, -1.0f, 0.0f};
        def.wheels[w].restLength = 0.4f;
        def.wheels[w].travel = 0.25f;
        def.wheels[w].hertz = 1.5f;
        def.wheels[w].zeta = 0.6f;
        def.wheels[w].radius = 0.3f;
        def.wheels[w].brakeShare = 0.25f;
    }
    def.maxSteerAngle = 0.6f;
    def.driveForce = 800.0f;
    def.brakeForce = 1600.0f;
    def.tireGrip = 1.5f;
    def.internalValue = M3_VEHICLE_COOKIE;
    return def;
}

int32_t m3VehicleSlot(const m3World* world, m3VehicleId vehicleId)
{
    int32_t index = vehicleId.index1 - 1;
    if (world == NULL || vehicleId.world0 != world->worldIndex0 ||
        !m3IdPoolValid(&world->vehPool, index, vehicleId.generation))
    {
        return -1;
    }
    return index;
}

int32_t m3CreateVehicleInternal(m3World* world, const m3VehicleDef* def)
{
    int32_t chassis = def->chassis.index1 - 1;
    if (chassis < 0 || chassis >= world->bodyCapacity || world->bodyPool.alive[chassis] == 0 ||
        world->bodyPool.generations[chassis] != def->chassis.generation ||
        world->types[chassis] != (uint8_t)m3_dynamicBody)
    {
        return -1;
    }
    int32_t slot = m3IdPoolAlloc(&world->vehPool);
    if (slot < 0)
    {
        return -1;
    }
    world->vehChassis[slot] = chassis;
    world->vehChassisGen[slot] = world->bodyPool.generations[chassis];
    world->vehWheelCount[slot] = def->wheelCount;
    world->vehMaxSteer[slot] = def->maxSteerAngle;
    world->vehDriveForce[slot] = def->driveForce;
    world->vehBrakeForce[slot] = def->brakeForce;
    world->vehTireGrip[slot] = def->tireGrip;
    world->vehThrottle[slot] = 0.0f;
    world->vehSteer[slot] = 0.0f;
    world->vehBrake[slot] = 0.0f;
    world->vehUserData[slot] = def->userData;
    for (int32_t w = 0; w < M3_VEHICLE_MAX_WHEELS; ++w)
    {
        int32_t k = slot * M3_VEHICLE_MAX_WHEELS + w;
        if (w < def->wheelCount)
        {
            const m3WheelDef* wd = &def->wheels[w];
            m3real len = sqrtf(m3Dot3(wd->direction, wd->direction));
            m3real inv = 1.0f / len;
            world->vehWheelAnchor[k] = wd->anchor;
            world->vehWheelDir[k] = m3MulSV3(inv, wd->direction);
            world->vehWheelRest[k] = wd->restLength;
            world->vehWheelTravel[k] = wd->travel;
            world->vehWheelHertz[k] = wd->hertz;
            world->vehWheelZeta[k] = wd->zeta;
            world->vehWheelRadius[k] = wd->radius;
            world->vehWheelFlags[k] = (uint8_t)((wd->steerable ? 1u : 0u) | (wd->driven ? 2u : 0u));
            world->vehWheelBrake[k] = wd->brakeShare;
        }
        else
        {
            world->vehWheelAnchor[k] = (m3Vec3){0.0f, 0.0f, 0.0f};
            world->vehWheelDir[k] = (m3Vec3){0.0f, -1.0f, 0.0f};
            world->vehWheelRest[k] = 0.0f;
            world->vehWheelTravel[k] = 0.0f;
            world->vehWheelHertz[k] = 0.0f;
            world->vehWheelZeta[k] = 0.0f;
            world->vehWheelRadius[k] = 0.0f;
            world->vehWheelFlags[k] = 0;
            world->vehWheelBrake[k] = 0.0f;
        }
        world->vehWheelCompression[k] = 0.0f;
        world->vehWheelContact[k] = 0;
        world->vehWheelSpin[k] = 0.0f;
    }
    return slot;
}

void m3DestroyVehicleInternal(m3World* world, int32_t slot)
{
    world->vehChassis[slot] = -1;
    world->vehChassisGen[slot] = 0;
    world->vehWheelCount[slot] = 0;
    world->vehMaxSteer[slot] = 0.0f;
    world->vehDriveForce[slot] = 0.0f;
    world->vehBrakeForce[slot] = 0.0f;
    world->vehTireGrip[slot] = 0.0f;
    world->vehThrottle[slot] = 0.0f;
    world->vehSteer[slot] = 0.0f;
    world->vehBrake[slot] = 0.0f;
    world->vehUserData[slot] = 0;
    for (int32_t w = 0; w < M3_VEHICLE_MAX_WHEELS; ++w)
    {
        int32_t k = slot * M3_VEHICLE_MAX_WHEELS + w;
        world->vehWheelAnchor[k] = (m3Vec3){0.0f, 0.0f, 0.0f};
        world->vehWheelDir[k] = (m3Vec3){0.0f, 0.0f, 0.0f};
        world->vehWheelRest[k] = 0.0f;
        world->vehWheelTravel[k] = 0.0f;
        world->vehWheelHertz[k] = 0.0f;
        world->vehWheelZeta[k] = 0.0f;
        world->vehWheelRadius[k] = 0.0f;
        world->vehWheelFlags[k] = 0;
        world->vehWheelBrake[k] = 0.0f;
        world->vehWheelCompression[k] = 0.0f;
        world->vehWheelContact[k] = 0;
        world->vehWheelSpin[k] = 0.0f;
    }
    m3IdPoolFree(&world->vehPool, slot);
}

// The suspension pass: velocity impulses on the chassis, before the
// solver prepares anything. A sleeping or vanished chassis skips
// (a parked car sleeps like any body and its wheel state freezes).
void m3VehicleApplySuspension(m3World* world, float dt)
{
    for (int32_t slot = 0; slot < world->vehPool.maxIndex; ++slot)
    {
        if (world->vehPool.alive[slot] == 0)
        {
            continue;
        }
        int32_t chassis = world->vehChassis[slot];
        if (chassis < 0 || world->bodyPool.alive[chassis] == 0 ||
            world->bodyPool.generations[chassis] != world->vehChassisGen[slot] ||
            world->types[chassis] != (uint8_t)m3_dynamicBody || world->awake[chassis] == 0)
        {
            continue;
        }
        const m3Transform* xf = &world->transforms[chassis];
        m3Vec3 rlc = m3RotateVec3(xf->q, world->localCenters[chassis]);
        m3Pos3 com = {xf->p.x + (double)rlc.x, xf->p.y + (double)rlc.y, xf->p.z + (double)rlc.z};
        m3real mass = world->invMass[chassis] > 0.0f ? 1.0f / world->invMass[chassis] : 0.0f;
        m3real wheelMass = mass / (m3real)world->vehWheelCount[slot];
        m3Mat3 invI = m3WorldInvInertia(world, chassis);

        // Two phases on purpose: every wheel reads the SAME pass
        // start velocities, then all impulses land together. A
        // sequential update lets wheel one's impulse leak into wheel
        // two's damper and the settled car holds a permanent tilt
        // (the first probe measured a centimeter of it).
        m3Vec3 v0 = world->linearVelocities[chassis];
        m3Vec3 w0 = world->angularVelocities[chassis];
        m3Vec3 impulses[M3_VEHICLE_MAX_WHEELS];
        m3Vec3 arms[M3_VEHICLE_MAX_WHEELS];
        int32_t applied = 0;

        for (int32_t w = 0; w < world->vehWheelCount[slot]; ++w)
        {
            int32_t k = slot * M3_VEHICLE_MAX_WHEELS + w;
            m3Vec3 anchorR = m3RotateVec3(xf->q, world->vehWheelAnchor[k]);
            m3Pos3 anchor = {xf->p.x + (double)anchorR.x, xf->p.y + (double)anchorR.y,
                             xf->p.z + (double)anchorR.z};
            m3Vec3 dir = m3RotateVec3(xf->q, world->vehWheelDir[k]);
            m3real reach = world->vehWheelRest[k] + world->vehWheelRadius[k];
            m3RayHit hit = m3RayClosestInternalEx(world, anchor, m3MulSV3(reach, dir), chassis);
            if (!hit.hit)
            {
                world->vehWheelCompression[k] = 0.0f;
                world->vehWheelContact[k] = 0;
                continue;
            }
            m3real suspLen = hit.fraction * reach - world->vehWheelRadius[k];
            m3real floorLen = world->vehWheelRest[k] - world->vehWheelTravel[k];
            if (suspLen < floorLen)
            {
                suspLen = floorLen; // bottomed out: travel is a hard book
            }
            m3real x = world->vehWheelRest[k] - suspLen;
            if (x < 0.0f)
            {
                x = 0.0f;
            }
            world->vehWheelCompression[k] = x;
            world->vehWheelContact[k] = 1;

            // The anchor's velocity along the suspension: positive
            // means compressing.
            m3Vec3 arm = {(m3real)(anchor.x - com.x), (m3real)(anchor.y - com.y),
                          (m3real)(anchor.z - com.z)};
            m3Vec3 vAnchor = m3Add3(v0, m3Cross3(w0, arm));
            m3real compressSpeed = m3Dot3(vAnchor, dir);

            m3real omega = 2.0f * 3.14159265358979323846f * world->vehWheelHertz[k];
            m3real stiffness = wheelMass * omega * omega;
            m3real damping = 2.0f * wheelMass * world->vehWheelZeta[k] * omega;
            m3real force = stiffness * x + damping * compressSpeed;
            if (force < 0.0f)
            {
                force = 0.0f; // suspension pushes, never pulls
            }
            m3Vec3 total = m3MulSV3(-force * dt, dir);
            // Impulses apply at the WHEEL HUB, not the ground
            // contact. Along the suspension axis the two points
            // torque identically (the offset is parallel to the
            // normal impulse), but for tangent impulses the ground
            // point adds a wheel radius of fake pitch lever: the
            // first probe showed full throttle unloading the front
            // axle to an eighth of its share, the friction circle
            // strangling the front tires, and a yaw instability
            // walking the car sideways at zero steer.
            m3Vec3 hubArm = {arm.x + dir.x * suspLen, arm.y + dir.y * suspLen,
                             arm.z + dir.z * suspLen};

            // The tire (5-2): drive, brake, and lateral impulses in
            // the contact plane, all clamped by one friction circle
            // against this wheel's suspension load. The chassis
            // local +x axis is forward by convention; a steerable
            // wheel's frame rotates about its suspension axis.
            m3Vec3 fLocal = {1.0f, 0.0f, 0.0f};
            if ((world->vehWheelFlags[k] & 1u) != 0 && world->vehSteer[slot] != 0.0f)
            {
                m3real a = world->vehSteer[slot] * world->vehMaxSteer[slot];
                m3real half = 0.5f * a;
                m3Vec3 up = m3MulSV3(-1.0f, world->vehWheelDir[k]);
                m3real sn = sinf(half);
                m3Quat qa = {up.x * sn, up.y * sn, up.z * sn, cosf(half)};
                fLocal = m3RotateVec3(qa, fLocal);
            }
            m3Vec3 fWorld = m3RotateVec3(xf->q, fLocal);
            m3Vec3 n = hit.normal;
            m3Vec3 forward = m3Sub3(fWorld, m3MulSV3(m3Dot3(fWorld, n), n));
            m3real fLen2 = m3Dot3(forward, forward);
            if (fLen2 > 1.0e-8f)
            {
                forward = m3MulSV3(1.0f / sqrtf(fLen2), forward);
                m3Vec3 side = m3Cross3(n, forward);
                // Tire velocities are RELATIVE to the surface under
                // the wheel: a car parked on a ferry must ride the
                // ferry, not fight it (an absolute kill drags every
                // moving platform to a halt under its passenger).
                int32_t hitShape = hit.shape.index1 - 1;
                int32_t hitBody = world->shapeBody[hitShape];
                m3Vec3 vSurf = {0.0f, 0.0f, 0.0f};
                if (world->types[hitBody] != (uint8_t)m3_staticBody)
                {
                    m3Vec3 rlcH =
                        m3RotateVec3(world->transforms[hitBody].q, world->localCenters[hitBody]);
                    m3Vec3 armH = {(m3real)(hit.point.x - world->transforms[hitBody].p.x) - rlcH.x,
                                   (m3real)(hit.point.y - world->transforms[hitBody].p.y) - rlcH.y,
                                   (m3real)(hit.point.z - world->transforms[hitBody].p.z) - rlcH.z};
                    vSurf = m3Add3(world->linearVelocities[hitBody],
                                   m3Cross3(world->angularVelocities[hitBody], armH));
                }
                m3Vec3 vContact = m3Sub3(m3Add3(v0, m3Cross3(w0, hubArm)), vSurf);
                m3real vLon = m3Dot3(vContact, forward);
                m3real vLat = m3Dot3(vContact, side);

                // Velocity kills use the solver's own effective
                // mass (linear plus angular response at the hub),
                // split by wheel count: four wheels each killing
                // the SHARED lateral velocity in full is a fourfold
                // overcorrection, and the hub's angular feedback
                // pushes the loop gain past one. The first probe
                // watched that pump grow a 1e-9 yaw seed into a
                // full sideways walk at 2.3x per step.
                m3Vec3 rxf = m3Cross3(hubArm, forward);
                m3real kLon = world->invMass[chassis] + m3Dot3(rxf, m3MulMV3(invI, rxf));
                m3real effLon = kLon > 0.0f ? 1.0f / kLon : 0.0f;
                m3Vec3 rxs = m3Cross3(hubArm, side);
                m3real kLat = world->invMass[chassis] + m3Dot3(rxs, m3MulMV3(invI, rxs));
                m3real effLat = kLat > 0.0f ? 1.0f / kLat : 0.0f;
                m3real share = 1.0f / (m3real)world->vehWheelCount[slot];

                m3real lon = 0.0f;
                if ((world->vehWheelFlags[k] & 2u) != 0)
                {
                    lon += world->vehThrottle[slot] * world->vehDriveForce[slot] * dt;
                }
                if (world->vehBrake[slot] > 0.0f)
                {
                    // A brake opposes rolling and never reverses it.
                    m3real budget = world->vehBrake[slot] * world->vehBrakeForce[slot] *
                                    world->vehWheelBrake[k] * dt;
                    m3real want = -vLon * effLon * share;
                    lon += want > budget ? budget : (want < -budget ? -budget : want);
                }
                // The tire kills its share of lateral slip; the
                // friction circle decides how much survives (that
                // surrender is the drift).
                m3real lat = -vLat * effLat * share;
                m3real budget2 = world->vehTireGrip[slot] * force * dt;
                m3real mag2 = lon * lon + lat * lat;
                if (mag2 > budget2 * budget2 && mag2 > 0.0f)
                {
                    m3real scale = budget2 / sqrtf(mag2);
                    lon *= scale;
                    lat *= scale;
                }
                total = m3Add3(total, m3Add3(m3MulSV3(lon, forward), m3MulSV3(lat, side)));
                world->vehWheelSpin[k] += (vLon / world->vehWheelRadius[k]) * dt;
            }

            impulses[applied] = total;
            arms[applied] = hubArm;
            applied += 1;

            // Newton's third law for dynamic ground (5-4): a wheel
            // pressing or driving on a fragment pushes the fragment
            // back, or cars would mint momentum from loose rubble.
            int32_t under = world->shapeBody[hit.shape.index1 - 1];
            if (world->types[under] == (uint8_t)m3_dynamicBody && world->invMass[under] > 0.0f)
            {
                m3Vec3 rlcU = m3RotateVec3(world->transforms[under].q, world->localCenters[under]);
                m3Vec3 armU = {(m3real)(hit.point.x - world->transforms[under].p.x) - rlcU.x,
                               (m3real)(hit.point.y - world->transforms[under].p.y) - rlcU.y,
                               (m3real)(hit.point.z - world->transforms[under].p.z) - rlcU.z};
                m3Vec3 back = m3MulSV3(-1.0f, total);
                world->linearVelocities[under] =
                    m3Add3(world->linearVelocities[under], m3MulSV3(world->invMass[under], back));
                world->angularVelocities[under] =
                    m3Add3(world->angularVelocities[under],
                           m3MulMV3(m3WorldInvInertia(world, under), m3Cross3(armU, back)));
                world->awake[under] = 1;
                world->sleepTimes[under] = 0.0f;
            }
        }
        for (int32_t a = 0; a < applied; ++a)
        {
            world->linearVelocities[chassis] = m3Add3(
                world->linearVelocities[chassis], m3MulSV3(world->invMass[chassis], impulses[a]));
            world->angularVelocities[chassis] = m3Add3(
                world->angularVelocities[chassis], m3MulMV3(invI, m3Cross3(arms[a], impulses[a])));
        }
    }
}

void m3VehicleCommandsInternal(m3World* world, int32_t slot, m3real throttle, m3real steer,
                               m3real brake)
{
    world->vehThrottle[slot] = throttle < -1.0f ? -1.0f : (throttle > 1.0f ? 1.0f : throttle);
    world->vehSteer[slot] = steer < -1.0f ? -1.0f : (steer > 1.0f ? 1.0f : steer);
    world->vehBrake[slot] = brake < 0.0f ? 0.0f : (brake > 1.0f ? 1.0f : brake);
    int32_t chassis = world->vehChassis[slot];
    if (chassis >= 0 && world->bodyPool.alive[chassis] != 0 &&
        world->bodyPool.generations[chassis] == world->vehChassisGen[slot])
    {
        world->awake[chassis] = 1; // a commanded car always wakes
        world->sleepTimes[chassis] = 0.0f;
    }
}

m3VehicleId m3CreateVehicle(m3WorldId worldId, const m3VehicleDef* def)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || def == NULL || def->internalValue != M3_VEHICLE_COOKIE ||
        def->wheelCount < 1 || def->wheelCount > M3_VEHICLE_MAX_WHEELS ||
        !m3FiniteF(def->maxSteerAngle) || def->maxSteerAngle < 0.0f ||
        !m3FiniteF(def->driveForce) || def->driveForce < 0.0f || !m3FiniteF(def->brakeForce) ||
        def->brakeForce < 0.0f || !m3FiniteF(def->tireGrip) || def->tireGrip < 0.0f)
    {
        return m3_nullVehicleId;
    }
    for (int32_t w = 0; w < def->wheelCount; ++w)
    {
        const m3WheelDef* wd = &def->wheels[w];
        m3real d2 = m3Dot3(wd->direction, wd->direction);
        if (!m3FiniteV3(wd->anchor) || !m3FiniteV3(wd->direction) || !(d2 > 0.81f) ||
            !(d2 < 1.21f) || !m3FiniteF(wd->restLength) || !(wd->restLength > 0.0f) ||
            !m3FiniteF(wd->travel) || !(wd->travel > 0.0f) || wd->travel > wd->restLength ||
            !m3FiniteF(wd->hertz) || !(wd->hertz > 0.0f) || !m3FiniteF(wd->zeta) ||
            wd->zeta < 0.0f || !m3FiniteF(wd->radius) || !(wd->radius > 0.0f) ||
            !m3FiniteF(wd->brakeShare) || wd->brakeShare < 0.0f)
        {
            return m3_nullVehicleId;
        }
    }
    int32_t slot = m3CreateVehicleInternal(world, def);
    if (slot < 0)
    {
        return m3_nullVehicleId;
    }
    m3VehicleId id = {slot + 1, world->worldIndex0, world->vehPool.generations[slot]};
    if (world->journalActive != 0)
    {
        struct
        {
            m3VehicleDef def;
            m3VehicleId expected;
        } record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.expected = id;
        m3JournalRecord(world, m3_opCreateVehicle, &record, (int32_t)sizeof(record));
    }
    return id;
}

void m3DestroyVehicle(m3VehicleId vehicleId)
{
    m3World* world = m3WorldFromIndex0(vehicleId.world0);
    int32_t slot = world != NULL ? m3VehicleSlot(world, vehicleId) : -1;
    if (slot < 0)
    {
        return; // stale: the quiet destroy contract
    }
    if (world->journalActive != 0)
    {
        m3JournalRecord(world, m3_opDestroyVehicle, &vehicleId, (int32_t)sizeof(vehicleId));
    }
    m3DestroyVehicleInternal(world, slot);
}

bool m3Vehicle_IsValid(m3VehicleId vehicleId)
{
    m3World* world = m3WorldFromIndex0(vehicleId.world0);
    return world != NULL && m3VehicleSlot(world, vehicleId) >= 0;
}

m3real m3Vehicle_GetCompression(m3VehicleId vehicleId, int32_t wheel)
{
    m3World* world = m3WorldFromIndex0(vehicleId.world0);
    int32_t slot = world != NULL ? m3VehicleSlot(world, vehicleId) : -1;
    if (slot < 0 || wheel < 0 || wheel >= world->vehWheelCount[slot])
    {
        return 0.0f;
    }
    return world->vehWheelCompression[slot * M3_VEHICLE_MAX_WHEELS + wheel];
}

void m3Vehicle_SetCommands(m3VehicleId vehicleId, m3real throttle, m3real steer, m3real brake)
{
    m3World* world = m3WorldFromIndex0(vehicleId.world0);
    int32_t slot = world != NULL ? m3VehicleSlot(world, vehicleId) : -1;
    if (slot < 0 || !m3FiniteF(throttle) || !m3FiniteF(steer) || !m3FiniteF(brake))
    {
        return; // stale id or hostile command: a documented no-op
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m3VehicleId id;
            m3real throttle;
            m3real steer;
            m3real brake;
        } record;
        memset(&record, 0, sizeof(record));
        record.id = vehicleId;
        record.throttle = throttle;
        record.steer = steer;
        record.brake = brake;
        m3JournalRecord(world, m3_opVehicleCommands, &record, (int32_t)sizeof(record));
    }
    m3VehicleCommandsInternal(world, slot, throttle, steer, brake);
}

m3real m3Vehicle_GetWheelSpin(m3VehicleId vehicleId, int32_t wheel)
{
    m3World* world = m3WorldFromIndex0(vehicleId.world0);
    int32_t slot = world != NULL ? m3VehicleSlot(world, vehicleId) : -1;
    if (slot < 0 || wheel < 0 || wheel >= world->vehWheelCount[slot])
    {
        return 0.0f;
    }
    return world->vehWheelSpin[slot * M3_VEHICLE_MAX_WHEELS + wheel];
}

bool m3Vehicle_IsWheelGrounded(m3VehicleId vehicleId, int32_t wheel)
{
    m3World* world = m3WorldFromIndex0(vehicleId.world0);
    int32_t slot = world != NULL ? m3VehicleSlot(world, vehicleId) : -1;
    if (slot < 0 || wheel < 0 || wheel >= world->vehWheelCount[slot])
    {
        return false;
    }
    return world->vehWheelContact[slot * M3_VEHICLE_MAX_WHEELS + wheel] != 0;
}
