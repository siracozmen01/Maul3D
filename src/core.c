// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Core: version, the FNV-1a hash every determinism gate folds through,
// and the debug assert hook. No dependencies beyond libc.

#include "maul3d/base.h"

#include <stdio.h>
#include <stdlib.h>

int m3GetVersion(void)
{
    return M3_VERSION_MAJOR * 10000 + M3_VERSION_MINOR * 100 + M3_VERSION_PATCH;
}

uint64_t m3Hash64(uint64_t h, const void* bytes, int32_t count)
{
    const uint8_t* p = (const uint8_t*)bytes;
    for (int32_t i = 0; i < count; ++i)
    {
        h = (h ^ p[i]) * 0x100000001B3ull;
    }
    return h;
}

void m3AssertFail(const char* condition, const char* file, int line)
{
    // Debug-only diagnostic. Library code never aborts in release; the
    // guarded return paths carry the failure instead.
    fprintf(stderr, "maul3d assert failed: %s (%s:%d)\n", condition, file, line);
    abort();
}
