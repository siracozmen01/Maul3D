// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen

#ifndef MAUL3D_BASE_H
#define MAUL3D_BASE_H

#include <stdbool.h>
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

    /// Opaque generation-tagged handles: the only identity, public and
    /// internal. index1 is 1-based (0 means null), the generation
    /// detects stale handles after slot reuse, world0 pins an id to its
    /// world. Handles encode no address, so they survive snapshot
    /// restore unchanged.
    typedef struct m3WorldId
    {
        int32_t index1;
        uint16_t generation;
    } m3WorldId;

    /// Id lifetime rule: an id is valid until its object or its
    /// WORLD is destroyed. Body, shape, and joint ids name their
    /// world by slot (not generation), so after a world is
    /// destroyed its ids must be dropped by the caller; a new
    /// world recycling the slot cannot tell foreign stale ids
    /// from its own (the reference shares this limitation). Using
    /// a stale id is a contract violation that never crashes:
    /// getters return zeros, commands and destroys no-op,
    /// creates refuse.
    typedef struct m3BodyId
    {
        int32_t index1;
        uint16_t world0;
        uint16_t generation;
    } m3BodyId;

    typedef struct m3ShapeId
    {
        int32_t index1;
        uint16_t world0;
        uint16_t generation;
    } m3ShapeId;

    /// Joint handle, same 8-byte law as bodies and shapes.
    typedef struct m3JointId
    {
        int32_t index1; // 1-based, 0 = null
        uint16_t world0;
        uint16_t generation;
    } m3JointId;

    static const m3JointId m3_nullJointId = {0, 0, 0};

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
