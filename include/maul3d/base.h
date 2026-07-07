// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen

#ifndef MAUL3D_BASE_H
#define MAUL3D_BASE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define M3_API extern

    /// Library version, encoded as major * 10000 + minor * 100 + patch.
    /// 0.1.0 during phase 2a.
#define M3_VERSION_MAJOR 0
#define M3_VERSION_MINOR 1
#define M3_VERSION_PATCH 0

    M3_API int m3GetVersion(void);

    /// Loud failure, the constitution's way: APIs that can fail return a
    /// result code (never a silent no-op, never an abort in library
    /// code). Debug builds additionally assert.
    typedef enum m3Result
    {
        m3_success = 0,
        m3_errorInvalid = 1,  // bad argument, stale id, wrong body type
        m3_errorCapacity = 2, // a fixed pool or the step scratch ran out
        m3_errorConfig = 3,   // snapshot/journal config hash mismatch
    } m3Result;

    /// FNV-1a 64: the deterministic hash every gate is built on.
    /// Seed with M3_HASH_INIT, fold bytes in canonical order.
#define M3_HASH_INIT 0xCBF29CE484222325ull

    M3_API uint64_t m3Hash64(uint64_t h, const void* bytes, int32_t count);

#if !defined(NDEBUG)
#define M3_ASSERT(cond)                                                                            \
    do                                                                                             \
    {                                                                                              \
        if (!(cond))                                                                               \
        {                                                                                          \
            m3AssertFail(#cond, __FILE__, __LINE__);                                               \
        }                                                                                          \
    } while (0)
#else
#define M3_ASSERT(cond) ((void)0)
#endif

    M3_API void m3AssertFail(const char* condition, const char* file, int line);

#ifdef __cplusplus
}
#endif

#endif // MAUL3D_BASE_H
