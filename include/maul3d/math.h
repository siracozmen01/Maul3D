// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Deterministic 3D math. Hybrid precision: world positions are double
// (m3Pos3), everything local is float (m3real). Every operation here is
// plain IEEE arithmetic under -ffp-contract=off, so the bits agree on
// every platform; the transcendentals live in math_functions.c and are
// hand rolled (reference technique) because libm is not trusted to
// agree across systems.

#ifndef MAUL3D_MATH_H
#define MAUL3D_MATH_H

#include "maul3d/base.h"

#include <math.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef float m3real;

#define M3_PI 3.14159265358979323846f

    typedef struct m3Vec3
    {
        m3real x, y, z;
    } m3Vec3;

    /// World-space position, double precision (large-world support and
    /// the delta-position solver both lean on this split).
    typedef struct m3Pos3
    {
        double x, y, z;
    } m3Pos3;

    /// Rotation quaternion, x y z vector part, w scalar part.
    typedef struct m3Quat
    {
        m3real x, y, z, w;
    } m3Quat;

    typedef struct m3Transform
    {
        m3Pos3 p;
        m3Quat q;
    } m3Transform;

    /// Column-major 3x3 (columns cx, cy, cz).
    typedef struct m3Mat3
    {
        m3Vec3 cx, cy, cz;
    } m3Mat3;

    typedef struct m3CosSin
    {
        m3real c, s;
    } m3CosSin;

    /// Pinned minimum: exactly (a < b ? a : b), in this operand order,
    /// on every platform. MSVC x64 lowers the ternary through MINSS
    /// which matches; see Maul2D's arm64 lesson for why this is spelled
    /// out rather than assumed.
    static inline m3real m3MinF(m3real a, m3real b)
    {
        return a < b ? a : b;
    }

    /// Pinned maximum: exactly (a > b ? a : b), in this operand order.
    static inline m3real m3MaxF(m3real a, m3real b)
    {
        return a > b ? a : b;
    }

    static inline m3real m3ClampF(m3real a, m3real lo, m3real hi)
    {
        return m3MaxF(lo, m3MinF(a, hi));
    }

    static inline m3real m3AbsF(m3real a)
    {
        return a < 0.0f ? -a : a;
    }

    static inline m3Vec3 m3Add3(m3Vec3 a, m3Vec3 b)
    {
        return (m3Vec3){a.x + b.x, a.y + b.y, a.z + b.z};
    }

    static inline m3Vec3 m3Sub3(m3Vec3 a, m3Vec3 b)
    {
        return (m3Vec3){a.x - b.x, a.y - b.y, a.z - b.z};
    }

    static inline m3Vec3 m3MulSV3(m3real s, m3Vec3 v)
    {
        return (m3Vec3){s * v.x, s * v.y, s * v.z};
    }

    static inline m3Vec3 m3Neg3(m3Vec3 v)
    {
        return (m3Vec3){-v.x, -v.y, -v.z};
    }

    static inline m3real m3Dot3(m3Vec3 a, m3Vec3 b)
    {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    static inline m3Vec3 m3Cross3(m3Vec3 a, m3Vec3 b)
    {
        return (m3Vec3){a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
    }

    static inline m3real m3LengthSquared3(m3Vec3 v)
    {
        return v.x * v.x + v.y * v.y + v.z * v.z;
    }

    static inline m3real m3Length3(m3Vec3 v)
    {
        // sqrtf is IEEE correctly rounded, the one libm call the bit
        // law allows on the hot path.
        return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
    }

    /// Normalize with a fixed degenerate fallback of {0, 1, 0}: one
    /// rule, documented, so a zero vector never turns into NaN and
    /// never depends on the caller.
    static inline m3Vec3 m3Normalize3(m3Vec3 v)
    {
        m3real length = m3Length3(v);
        if (length < 1.19209290e-7f)
        {
            return (m3Vec3){0.0f, 1.0f, 0.0f};
        }
        m3real inv = 1.0f / length;
        return (m3Vec3){inv * v.x, inv * v.y, inv * v.z};
    }

    /// Convert any angle into the range [-pi, pi]. remainderf is IEEE
    /// exact (like sqrt), so this is deterministic (reference note).
    static inline m3real m3UnwindAngle(m3real radians)
    {
        return remainderf(radians, 2.0f * M3_PI);
    }

    static inline m3Quat m3MakeIdentityQuat(void)
    {
        return (m3Quat){0.0f, 0.0f, 0.0f, 1.0f};
    }

    static inline m3Quat m3MulQuat(m3Quat a, m3Quat b)
    {
        m3Quat q;
        q.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
        q.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
        q.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
        q.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
        return q;
    }

    /// Normalize with a fixed degenerate fallback of identity.
    static inline m3Quat m3NormalizeQuat(m3Quat q)
    {
        m3real mag = sqrtf(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
        if (mag < 1.19209290e-7f)
        {
            return m3MakeIdentityQuat();
        }
        m3real inv = 1.0f / mag;
        return (m3Quat){inv * q.x, inv * q.y, inv * q.z, inv * q.w};
    }

    /// Rotate a vector by a unit quaternion: v' = v + w*t + q x t with
    /// t = 2 (q x v). Plain mul and add only, no fused ops.
    static inline m3Vec3 m3RotateVec3(m3Quat q, m3Vec3 v)
    {
        m3Vec3 u = {q.x, q.y, q.z};
        m3Vec3 t = m3MulSV3(2.0f, m3Cross3(u, v));
        return m3Add3(m3Add3(v, m3MulSV3(q.w, t)), m3Cross3(u, t));
    }

    /// Rotate by the conjugate (inverse for unit quaternions).
    static inline m3Vec3 m3InvRotateVec3(m3Quat q, m3Vec3 v)
    {
        m3Quat c = {-q.x, -q.y, -q.z, q.w};
        return m3RotateVec3(c, v);
    }

    /// Integrate a rotation by an angular displacement (radians, world
    /// frame): q2 = normalize(q1 + 0.5 * (dr, 0) * q1), the reference
    /// quaternion-derivative form. Renormalization every call is the
    /// 3D numeric contract: drift never accumulates.
    static inline m3Quat m3IntegrateRotation(m3Quat q, m3Vec3 deltaRotation)
    {
        m3real ax = 0.5f * deltaRotation.x;
        m3real ay = 0.5f * deltaRotation.y;
        m3real az = 0.5f * deltaRotation.z;
        m3Quat qd;
        qd.w = -(ax * q.x + ay * q.y + az * q.z);
        qd.x = ax * q.w + ay * q.z - az * q.y;
        qd.y = -ax * q.z + ay * q.w + az * q.x;
        qd.z = ax * q.y - ay * q.x + az * q.w;
        m3Quat q2 = {q.x + qd.x, q.y + qd.y, q.z + qd.z, q.w + qd.w};
        return m3NormalizeQuat(q2);
    }

    /// Deterministic cosine and sine (Bhaskara I rational form) and
    /// atan2 (minimax polynomial): hand rolled because platform libm
    /// implementations disagree in the last bits. Reference technique.
    M3_API m3CosSin m3ComputeCosSin(m3real radians);
    M3_API m3real m3Atan2(m3real y, m3real x);

#ifdef __cplusplus
}
#endif

#endif // MAUL3D_MATH_H
