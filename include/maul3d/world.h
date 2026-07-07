// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen

#ifndef MAUL3D_WORLD_H
#define MAUL3D_WORLD_H

#include "maul3d/math.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /// World definition. Build with m3DefaultWorldDef so the cookie is
    /// valid; a zeroed or hand-rolled def is rejected loudly.
    typedef struct m3WorldDef
    {
        m3Vec3 gravity;
        int32_t bodyCapacity;
        int32_t shapeCapacity;
        int32_t workerCount; // twin worlds with different counts must hash equal
        int32_t internalValue;
    } m3WorldDef;

    M3_API m3WorldDef m3DefaultWorldDef(void);

    /// Create a world. Returns the null id on an invalid def or an
    /// exhausted world table (diagnostic in debug builds).
    M3_API m3WorldId m3CreateWorld(const m3WorldDef* def);
    M3_API void m3DestroyWorld(m3WorldId worldId);
    M3_API bool m3World_IsValid(m3WorldId worldId);

    /// Journal: every mutation of the world is a discrete recorded op,
    /// and replaying the stream through the same internal functions
    /// reproduces the world bit for bit. Begin hands the world a
    /// caller-owned buffer; End returns the bytes written (or -1 after
    /// an overflow, loudly); Replay applies a stream to this world.
    M3_API bool m3World_JournalBegin(m3WorldId worldId, void* buffer, int32_t capacity);
    M3_API int32_t m3World_JournalEnd(m3WorldId worldId);
    M3_API bool m3World_JournalReplay(m3WorldId worldId, const void* data, int32_t size);

#ifdef __cplusplus
}
#endif

#endif // MAUL3D_WORLD_H
