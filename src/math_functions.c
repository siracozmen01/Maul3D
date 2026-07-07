// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Deterministic transcendentals, adapted from Box3D (Copyright 2026
// Erin Catto, MIT) math_functions.c: a minimax atan2 and Bhaskara I's
// rational cosine and sine. Hand rolled because libm's sinf, cosf, and
// atan2f are not trusted to agree bit for bit across platforms; only
// sqrtf and remainderf (IEEE exact) are allowed through.

#include "maul3d/math.h"

// Minimax polynomial approximation of atan on [0, 1], mapped to the
// full circle. Matches the reference constants exactly.
m3real m3Atan2(m3real y, m3real x)
{
    // (0, 0) returns 0 to match atan2f and avoid NaN.
    if (x == 0.0f && y == 0.0f)
    {
        return 0.0f;
    }

    m3real ax = m3AbsF(x);
    m3real ay = m3AbsF(y);
    m3real mx = m3MaxF(ay, ax);
    m3real mn = m3MinF(ay, ax);
    m3real a = mn / mx;

    m3real s = a * a;
    m3real c = s * a;
    m3real q = s * s;
    m3real r = 0.024840285f * q + 0.18681418f;
    m3real t = -0.094097948f * q - 0.33213072f;
    r = r * s + t;
    r = r * c + a;

    if (ay > ax)
    {
        r = 1.57079637f - r;
    }
    if (x < 0.0f)
    {
        r = 3.14159274f - r;
    }
    if (y < 0.0f)
    {
        r = -r;
    }
    return r;
}

// Bhaskara I's rational approximation, renormalized so c*c + s*s is
// one to float rounding. The libm path stays deliberately absent: in
// the reference's testing cosf and sinf agreed across platforms, but
// neither engine trusts that result.
m3CosSin m3ComputeCosSin(m3real radians)
{
    m3real x = m3UnwindAngle(radians);
    m3real pi2 = M3_PI * M3_PI;

    // Cosine needs the angle in [-pi/2, pi/2].
    m3real c;
    if (x < -0.5f * M3_PI)
    {
        m3real y = x + M3_PI;
        m3real y2 = y * y;
        c = -(pi2 - 4.0f * y2) / (pi2 + y2);
    }
    else if (x > 0.5f * M3_PI)
    {
        m3real y = x - M3_PI;
        m3real y2 = y * y;
        c = -(pi2 - 4.0f * y2) / (pi2 + y2);
    }
    else
    {
        m3real y2 = x * x;
        c = (pi2 - 4.0f * y2) / (pi2 + y2);
    }

    // Sine needs the angle in [0, pi].
    m3real s;
    if (x < 0.0f)
    {
        m3real y = x + M3_PI;
        s = -16.0f * y * (M3_PI - y) / (5.0f * pi2 - 4.0f * y * (M3_PI - y));
    }
    else
    {
        s = 16.0f * x * (M3_PI - x) / (5.0f * pi2 - 4.0f * x * (M3_PI - x));
    }

    m3real mag = sqrtf(s * s + c * c);
    m3real invMag = mag > 0.0f ? 1.0f / mag : 0.0f;
    m3CosSin cs = {c * invMag, s * invMag};
    return cs;
}
