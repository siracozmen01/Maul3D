// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Four-lane f32 vectors under a bit law: every operation is plain IEEE
// arithmetic with IDENTICAL semantics on every backend. The canonical
// width is 4 (the reference's choice) because SSE2 is part of the
// x86-64 baseline and NEON is architectural on arm64, so the same
// logical vector exists everywhere without runtime dispatch.
//
// Rules that keep the law:
// - min/max are compare+select, NEVER the native instructions: SSE and
//   NEON disagree about NaN propagation and signed zero (the Maul2D
//   arm64 lesson).
// - There is NO fused multiply-add anywhere. This is a deliberate
//   deviation from Maul2D, argued: Maul2D's baseline (AVX2, NEON,
//   fmaf) has correctly rounded FMA on every backend, so it pinned
//   single-rounding fused ops. Maul3D's 4-wide baseline includes
//   SSE2, which has no FMA instruction, so the only law every backend
//   can honor is two-rounding mul-then-add. m3F4MulAdd is therefore
//   DEFINED as Add(Mul(a, b), c), separate instructions, on all
//   backends, matching the scalar path under -ffp-contract=off and
//   the reference's no-contraction discipline.

#ifndef MAUL3D_SIMD_H
#define MAUL3D_SIMD_H

#include <stdint.h>
#include <string.h>

#if defined(MAUL3D_SIMD_FORCE_SCALAR)
#define M3_SIMD_SCALAR 1
#elif defined(__x86_64__) || defined(_M_X64)
// SSE2 is the x86-64 baseline; no feature test needed.
#define M3_SIMD_SSE2 1
#elif defined(__aarch64__) || defined(_M_ARM64)
#define M3_SIMD_NEON 1
#else
#define M3_SIMD_SCALAR 1
#endif

#if defined(M3_SIMD_SSE2)

#include <emmintrin.h>

typedef __m128 m3f4;

static inline m3f4 m3F4Load(const float* p)
{
    return _mm_loadu_ps(p);
}
static inline void m3F4Store(float* p, m3f4 v)
{
    _mm_storeu_ps(p, v);
}
static inline m3f4 m3F4Set1(float x)
{
    return _mm_set1_ps(x);
}
static inline m3f4 m3F4Zero(void)
{
    return _mm_setzero_ps();
}
static inline m3f4 m3F4Add(m3f4 a, m3f4 b)
{
    return _mm_add_ps(a, b);
}
static inline m3f4 m3F4Sub(m3f4 a, m3f4 b)
{
    return _mm_sub_ps(a, b);
}
static inline m3f4 m3F4Mul(m3f4 a, m3f4 b)
{
    return _mm_mul_ps(a, b);
}
static inline m3f4 m3F4Neg(m3f4 a)
{
    return _mm_xor_ps(a, _mm_set1_ps(-0.0f));
}
// mask lanes are all-ones or all-zeros; mask ? a : b. SSE2 has no
// blendv, so the select is the classic and/andnot/or triple.
static inline m3f4 m3F4Select(m3f4 mask, m3f4 a, m3f4 b)
{
    return _mm_or_ps(_mm_and_ps(mask, a), _mm_andnot_ps(mask, b));
}
static inline m3f4 m3F4GT(m3f4 a, m3f4 b)
{
    return _mm_cmpgt_ps(a, b);
}
static inline m3f4 m3F4LT(m3f4 a, m3f4 b)
{
    return _mm_cmplt_ps(a, b);
}

#elif defined(M3_SIMD_NEON)

#include <arm_neon.h>

typedef float32x4_t m3f4;

static inline m3f4 m3F4Load(const float* p)
{
    return vld1q_f32(p);
}
static inline void m3F4Store(float* p, m3f4 v)
{
    vst1q_f32(p, v);
}
static inline m3f4 m3F4Set1(float x)
{
    return vdupq_n_f32(x);
}
static inline m3f4 m3F4Zero(void)
{
    return vdupq_n_f32(0.0f);
}
static inline m3f4 m3F4Add(m3f4 a, m3f4 b)
{
    return vaddq_f32(a, b);
}
static inline m3f4 m3F4Sub(m3f4 a, m3f4 b)
{
    return vsubq_f32(a, b);
}
static inline m3f4 m3F4Mul(m3f4 a, m3f4 b)
{
    // vmulq is a plain multiply; the fused vfmaq family is deliberately
    // unused (see the header law).
    return vmulq_f32(a, b);
}
static inline m3f4 m3F4Neg(m3f4 a)
{
    return vnegq_f32(a);
}
static inline m3f4 m3F4Select(m3f4 mask, m3f4 a, m3f4 b)
{
    return vbslq_f32(vreinterpretq_u32_f32(mask), a, b);
}
static inline m3f4 m3F4GT(m3f4 a, m3f4 b)
{
    return vreinterpretq_f32_u32(vcgtq_f32(a, b));
}
static inline m3f4 m3F4LT(m3f4 a, m3f4 b)
{
    return vreinterpretq_f32_u32(vcltq_f32(a, b));
}

#else // M3_SIMD_SCALAR

typedef struct m3f4
{
    float v[4];
} m3f4;

static inline m3f4 m3F4Load(const float* p)
{
    m3f4 r;
    memcpy(r.v, p, sizeof(r.v));
    return r;
}
static inline void m3F4Store(float* p, m3f4 v)
{
    memcpy(p, v.v, sizeof(v.v));
}
static inline m3f4 m3F4Set1(float x)
{
    m3f4 r = {{x, x, x, x}};
    return r;
}
static inline m3f4 m3F4Zero(void)
{
    return m3F4Set1(0.0f);
}
static inline m3f4 m3F4Add(m3f4 a, m3f4 b)
{
    m3f4 r;
    for (int32_t i = 0; i < 4; ++i)
    {
        r.v[i] = a.v[i] + b.v[i];
    }
    return r;
}
static inline m3f4 m3F4Sub(m3f4 a, m3f4 b)
{
    m3f4 r;
    for (int32_t i = 0; i < 4; ++i)
    {
        r.v[i] = a.v[i] - b.v[i];
    }
    return r;
}
static inline m3f4 m3F4Mul(m3f4 a, m3f4 b)
{
    m3f4 r;
    for (int32_t i = 0; i < 4; ++i)
    {
        r.v[i] = a.v[i] * b.v[i];
    }
    return r;
}
static inline m3f4 m3F4Neg(m3f4 a)
{
    m3f4 r;
    for (int32_t i = 0; i < 4; ++i)
    {
        r.v[i] = -a.v[i];
    }
    return r;
}
static inline m3f4 m3F4Select(m3f4 mask, m3f4 a, m3f4 b)
{
    m3f4 r;
    for (int32_t i = 0; i < 4; ++i)
    {
        uint32_t m;
        uint32_t ua;
        uint32_t ub;
        memcpy(&m, &mask.v[i], 4);
        memcpy(&ua, &a.v[i], 4);
        memcpy(&ub, &b.v[i], 4);
        uint32_t out = (ua & m) | (ub & ~m);
        memcpy(&r.v[i], &out, 4);
    }
    return r;
}
static inline m3f4 m3F4Cmp(int gt, m3f4 a, m3f4 b)
{
    m3f4 r;
    for (int32_t i = 0; i < 4; ++i)
    {
        uint32_t m = (gt ? (a.v[i] > b.v[i]) : (a.v[i] < b.v[i])) ? 0xFFFFFFFFu : 0u;
        memcpy(&r.v[i], &m, 4);
    }
    return r;
}
static inline m3f4 m3F4GT(m3f4 a, m3f4 b)
{
    return m3F4Cmp(1, a, b);
}
static inline m3f4 m3F4LT(m3f4 a, m3f4 b)
{
    return m3F4Cmp(0, a, b);
}

#endif

// Ternary-law min/max: a>b?a:b and a<b?a:b, identical on every backend
// by construction (compare + select, no native min/max).
static inline m3f4 m3F4Max(m3f4 a, m3f4 b)
{
    return m3F4Select(m3F4GT(a, b), a, b);
}
static inline m3f4 m3F4Min(m3f4 a, m3f4 b)
{
    return m3F4Select(m3F4LT(a, b), a, b);
}

// a * b + c in TWO roundings on every backend (the header law: the
// SSE2 baseline has no FMA, so nothing here may fuse).
static inline m3f4 m3F4MulAdd(m3f4 a, m3f4 b, m3f4 c)
{
    return m3F4Add(m3F4Mul(a, b), c);
}
static inline m3f4 m3F4NegMulAdd(m3f4 a, m3f4 b, m3f4 c)
{
    return m3F4Sub(c, m3F4Mul(a, b));
}

#endif // MAUL3D_SIMD_H
