// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The three memory lifetimes, kept structurally apart because mixing
// them is the classic mistake that defeats rollback:
//   1. Persistent rollback state: per-array M3_ALLOC allocations, every
//      one registered in the world's single snapshot walker (task 6).
//   2. Per-step scratch: the m3Stack below, reset every step, never
//      snapshotted, never aborts on exhaustion (loud failure instead).
//   3. Immutable geometry and config: interned, referenced, not copied.
// The m3IdPool is the slot allocator behind every generation-tagged
// handle: FIFO recycling with a generation bump, so a stale id can
// always be detected and slots are reused in a canonical order.

#ifndef MAUL3D_ALLOCATOR_H
#define MAUL3D_ALLOCATOR_H

#include "maul3d/base.h"

// Persistent arrays: allocation with a loud null check. The snapshot
// walker, not this macro, is the source of truth for what state IS.
#define M3_ALLOC(field, count, type)                                                               \
    do                                                                                             \
    {                                                                                              \
        (field) = (type*)m3AllocZeroed((int32_t)((count) * (int32_t)sizeof(type)));                \
    } while (0)

void* m3AllocZeroed(int32_t bytes);
void m3Free(void* memory);
// Soak bookkeeping: cumulative alloc and free call counts (2d-6).
void m3DebugAllocCounts(int64_t* allocs, int64_t* frees);

/// Per-step scratch: a bump allocator with 16-byte alignment. Reset
/// once per step. Exhaustion returns NULL and latches the overflow
/// flag; the step propagates m3_errorCapacity and the world grows the
/// stack between steps. Never aborts (the Jolt lesson).
typedef struct m3Stack
{
    uint8_t* base;
    int32_t capacity;
    int32_t top;
    int32_t overflow; // latched until the next reset
} m3Stack;

m3Stack m3StackCreate(int32_t capacity);
void m3StackDestroy(m3Stack* stack);
void* m3StackAlloc(m3Stack* stack, int32_t size);
void m3StackReset(m3Stack* stack);

/// Slot allocator for generation-tagged handles. FIFO free queue (a
/// freed slot is reused last, in a canonical order) with a generation
/// bump on free, so stale handles fail validation instead of aliasing
/// a recycled slot. Generations retire a slot at 0xFFFF instead of
/// wrapping (the Maul2D rule).
typedef struct m3IdPool
{
    int32_t capacity;
    int32_t maxIndex; // high-water mark of ever-allocated slots
    uint16_t* generations;
    uint8_t* alive;
    int32_t* freeQueue; // FIFO ring
    int32_t freeHead;
    int32_t freeCount;
    int32_t retiredCount;
} m3IdPool;

m3IdPool m3IdPoolCreate(int32_t capacity);
void m3IdPoolDestroy(m3IdPool* pool);
/// Returns a slot index, or -1 when the pool is exhausted (loud
/// failure at the caller, never a hidden growth or an abort).
int32_t m3IdPoolAlloc(m3IdPool* pool);
void m3IdPoolFree(m3IdPool* pool, int32_t index);
int m3IdPoolValid(const m3IdPool* pool, int32_t index, uint16_t generation);

#endif // MAUL3D_ALLOCATOR_H
