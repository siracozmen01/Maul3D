// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The raycast vehicle (5-1): wheels are not bodies. Each wheel is a
// suspension ray cast from the chassis, a spring and damper impulse
// at the contact, and (from 5-2) tire friction impulses in the
// contact plane. The vehicle is an engine-owned object like the
// character: pooled ids, journaled ops, stepped inside m3World_Step
// in canonical slot order, every field snapshotted and hashed, so
// a replayed or rolled-back drive lands on identical bits.

#ifndef MAUL3D_VEHICLE_H
#define MAUL3D_VEHICLE_H

#include "world.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define M3_VEHICLE_MAX_WHEELS 8

    typedef struct m3VehicleId
    {
        int32_t index1;
        uint16_t world0;
        uint16_t generation;
    } m3VehicleId;

    typedef struct m3WheelDef
    {
        m3Vec3 anchor;     // suspension attach point, chassis-local
        m3Vec3 direction;  // suspension direction, chassis-local, near
                           // unit (normalized at create); usually down
        m3real restLength; // suspension length at zero load
        m3real travel;     // max compression from rest
        m3real hertz;      // spring frequency; keep well under the
                           // step rate over two pi (explicit spring)
        m3real zeta;       // damping ratio
        m3real radius;     // wheel radius (the cast reaches
                           // restLength + radius along direction)
        bool steerable;    // consumed by the drive slice (5-2)
        bool driven;       // consumed by the drive slice (5-2)
        m3real brakeShare; // consumed by the drive slice (5-2)
    } m3WheelDef;

    typedef struct m3VehicleDef
    {
        m3BodyId chassis; // a dynamic body; destroying it destroys
                          // the vehicle (the cascade rule)
        int32_t wheelCount;
        m3WheelDef wheels[M3_VEHICLE_MAX_WHEELS];
        m3real maxSteerAngle; // radians; full steer command turns
                              // every steerable wheel this far
        m3real driveForce;    // newtons per driven wheel at full
                              // throttle (the flat force model)
        m3real brakeForce;    // newtons total at full brake, split
                              // by each wheel's brakeShare
        m3real tireGrip;      // friction circle scale: tangent
                              // impulses never exceed grip times
                              // the wheel's suspension load
        uint64_t userData;
        int32_t internalValue;
    } m3VehicleDef;

    M3_API m3VehicleDef m3DefaultVehicleDef(void);

    /// Creates the vehicle bound to its chassis body. Refuses (null
    /// id) a non-dynamic or stale chassis, a wheel count outside
    /// [1, M3_VEHICLE_MAX_WHEELS], or any non-finite or non-positive
    /// geometry. Journaled with id verification.
    M3_API m3VehicleId m3CreateVehicle(m3WorldId worldId, const m3VehicleDef* def);
    M3_API void m3DestroyVehicle(m3VehicleId vehicleId);
    M3_API bool m3Vehicle_IsValid(m3VehicleId vehicleId);

    /// Last step's suspension compression of one wheel, in meters
    /// from rest length (zero when airborne, stale, or out of
    /// range). Frozen while the chassis sleeps, like the chassis.
    M3_API m3real m3Vehicle_GetCompression(m3VehicleId vehicleId, int32_t wheel);
    /// Whether the wheel's suspension cast found ground last step.
    M3_API bool m3Vehicle_IsWheelGrounded(m3VehicleId vehicleId, int32_t wheel);

    /// Drive commands (5-2): journaled STATE, not per-step
    /// parameters, so replay and rollback hold to the bit. Values
    /// clamp to their ranges (throttle and steer to [-1, 1], brake
    /// to [0, 1]); non-finite commands are hostile no-ops that
    /// never journal. Setting commands wakes the chassis. The
    /// chassis-local +x axis is the vehicle's forward by
    /// convention; steer rotates each steerable wheel's frame
    /// about its suspension axis by steer * maxSteerAngle.
    M3_API void m3Vehicle_SetCommands(m3VehicleId vehicleId, m3real throttle, m3real steer,
                                      m3real brake);

    /// Accumulated spin angle of one wheel in radians (rendering
    /// state: hosts spin their wheel meshes with it).
    M3_API m3real m3Vehicle_GetWheelSpin(m3VehicleId vehicleId, int32_t wheel);

    static const m3VehicleId m3_nullVehicleId = {0, 0, 0};

#ifdef __cplusplus
}
#endif

#endif // MAUL3D_VEHICLE_H
