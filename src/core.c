// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Core: version, the FNV-1a hash every determinism gate folds through,
// and the debug assert hook. No dependencies beyond libc.

#if !defined(_WIN32) && !defined(_POSIX_C_SOURCE)
#define _POSIX_C_SOURCE 199309L // clock_gettime for the profile clock
#endif

#include "maul3d/base.h"

#include <stdio.h>
#include <stdlib.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <time.h>
#endif

// Monotonic milliseconds for the step profile (14-1). Observer data
// only: never a hash input, never serialized, never fed back into
// the simulation. The bench keeps its own copy on purpose (tools do
// not reach into engine internals).
double m3NowMs(void)
{
#ifdef _WIN32
    LARGE_INTEGER frequency;
    LARGE_INTEGER counter;
    QueryPerformanceFrequency(&frequency);
    QueryPerformanceCounter(&counter);
    return 1000.0 * (double)counter.QuadPart / (double)frequency.QuadPart;
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return 1000.0 * (double)ts.tv_sec + (double)ts.tv_nsec / 1000000.0;
#endif
}

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

static m3AssertFn s_assertHandler = NULL;
static m3AssertCtxFn* s_assertHandlerCtx = NULL;
static void* s_assertContext = NULL;

void m3SetAssertHandler(m3AssertFn handler)
{
    s_assertHandler = handler;
}

void m3SetAssertHandlerCtx(m3AssertCtxFn* handler, void* context)
{
    s_assertHandlerCtx = handler;
    s_assertContext = context;
}

void m3AssertFail(const char* condition, const char* file, int line)
{
    // Debug-only diagnostic. Library code never aborts in release; the
    // guarded return paths carry the failure instead. A host handler
    // returning nonzero declares the failure handled (14-3).
    if (s_assertHandlerCtx != NULL &&
        s_assertHandlerCtx(condition, file, line, s_assertContext) != 0)
    {
        return; // handled by the contextful hook (A5)
    }
    if (s_assertHandler != NULL && s_assertHandler(condition, file, line) != 0)
    {
        return;
    }
    fprintf(stderr, "maul3d assert failed: %s (%s:%d)\n", condition, file, line);
    abort();
}
