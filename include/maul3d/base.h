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

/// Export decoration. Static builds (the default) need none; shared
/// builds export with default visibility on ELF and Mach-O and with
/// dllexport/dllimport on Windows (maul3d_EXPORTS is defined by the
/// build of the library itself, never by consumers).
#if defined(MAUL3D_SHARED) && defined(_WIN32)
#if defined(maul3d_EXPORTS)
#define M3_API __declspec(dllexport) extern
#else
#define M3_API __declspec(dllimport) extern
#endif
#elif defined(MAUL3D_SHARED) && (defined(__GNUC__) || defined(__clang__))
#define M3_API __attribute__((visibility("default"))) extern
#else
#define M3_API extern
#endif

    /// Library version, encoded as major * 10000 + minor * 100 + patch.
#define M3_VERSION_MAJOR 1
#define M3_VERSION_MINOR 22
#define M3_VERSION_PATCH 0

    /// The linked library's version as major * 10000 + minor * 100 +
    /// patch (0.3.0 returns 300). Compare against the M3_VERSION
    /// macros to catch a header/library mismatch at startup.
    M3_API int m3GetVersion(void);

    /// Process-global allocator hook (integration audit A3), the
    /// Maul2D-parity contract: install BEFORE the first world and
    /// never change it while any world lives. The alloc function
    /// may return uninitialized memory (the engine zeroes what
    /// needs zeroing); returning NULL is a loud refusal upstream.
    /// Pass NULLs to restore the libc default. Hosts that need
    /// per-world budgets meter with m3World_MemoryUsage and
    /// enforce in their hook via the context.
    typedef void* m3AllocFn(int32_t bytes, void* context);
    typedef void m3FreeFn(void* memory, void* context);
    M3_API void m3SetAllocator(m3AllocFn* allocFn, m3FreeFn* freeFn, void* context);

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

    /// The debug assert sink (prints and aborts). Internal invariants
    /// only; never called for user input, never present in release.
    M3_API void m3AssertFail(const char* condition, const char* file, int line);

    /// Contextful host assert hook (integration audit A5): same
    /// contract as m3SetAssertHandler below, with the context the
    /// embedding host needs to route the failure to its own
    /// diagnostics without globals. When both handlers are set the
    /// contextful one wins. Return nonzero to suppress the abort.
    typedef int m3AssertCtxFn(const char* condition, const char* file, int line, void* context);
    M3_API void m3SetAssertHandlerCtx(m3AssertCtxFn* handler, void* context);

    /// Host assert hook (14-3): installed globally, called before
    /// the abort. Return nonzero to declare the failure handled and
    /// skip the abort (test harnesses, crash reporters); return
    /// zero to keep the default print-and-abort. NULL restores the
    /// default. Observer machinery: never touches simulation state.
    typedef int (*m3AssertFn)(const char* condition, const char* file, int line);
    M3_API void m3SetAssertHandler(m3AssertFn handler);

#ifdef __cplusplus
}
#endif

#endif // MAUL3D_BASE_H
