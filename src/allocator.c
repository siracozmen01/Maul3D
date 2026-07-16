// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen

#include "allocator.h"

#include <stdlib.h>
#include <string.h>

// No-growth bookkeeping (2d-6): the soak asserts that steady-state
// stepping performs ZERO net allocation. Counters, not sizes: the
// free contract carries no size, and a flat live count plus the
// LeakSanitizer cell together are the growth proof. Main-thread
// only by the allocation map (create, restore, replay paths); task
// workers never allocate.
#if defined(_MSC_VER)
#include <intrin.h>
static volatile long long s_allocCalls = 0;
static volatile long long s_freeCalls = 0;
#define M3_COUNT_INC(c)  _InterlockedIncrement64(&(c))
#define M3_COUNT_READ(c) _InterlockedCompareExchange64(&(c), 0, 0)
#else
#include <stdatomic.h>
static _Atomic int64_t s_allocCalls = 0;
static _Atomic int64_t s_freeCalls = 0;
#define M3_COUNT_INC(c)  atomic_fetch_add_explicit(&(c), 1, memory_order_relaxed)
#define M3_COUNT_READ(c) atomic_load_explicit(&(c), memory_order_relaxed)
#endif

void m3DebugAllocCounts(int64_t* allocs, int64_t* frees)
{
    if (allocs != NULL)
    {
        *allocs = (int64_t)M3_COUNT_READ(s_allocCalls);
    }
    if (frees != NULL)
    {
        *frees = (int64_t)M3_COUNT_READ(s_freeCalls);
    }
}

// The host hook (integration audit A3): set before the first
// world, constant while worlds live. Hooked memory arrives
// uninitialized, so the zeroing happens here either way.
static m3AllocFn* s_hookAlloc = NULL;
static m3FreeFn* s_hookFree = NULL;
static void* s_hookContext = NULL;

void m3SetAllocator(m3AllocFn* allocFn, m3FreeFn* freeFn, void* context)
{
    // Both or neither: a mismatched pair would free with the wrong
    // authority.
    if ((allocFn == NULL) != (freeFn == NULL))
    {
        M3_ASSERT(false);
        return;
    }
    s_hookAlloc = allocFn;
    s_hookFree = freeFn;
    s_hookContext = context;
}

void* m3AllocZeroed(int32_t bytes)
{
    if (bytes <= 0)
    {
        M3_ASSERT(false);
        return NULL;
    }
    void* memory;
    if (s_hookAlloc != NULL)
    {
        memory = s_hookAlloc(bytes, s_hookContext);
        if (memory != NULL)
        {
            memset(memory, 0, (size_t)bytes);
        }
    }
    else
    {
        memory = calloc(1, (size_t)bytes);
    }
    M3_ASSERT(memory != NULL);
    if (memory != NULL)
    {
        M3_COUNT_INC(s_allocCalls);
    }
    return memory;
}

void m3Free(void* memory)
{
    if (memory != NULL)
    {
        M3_COUNT_INC(s_freeCalls);
    }
    if (s_hookFree != NULL)
    {
        s_hookFree(memory, s_hookContext);
    }
    else
    {
        free(memory);
    }
}

m3Stack m3StackCreate(int32_t capacity)
{
    m3Stack stack;
    memset(&stack, 0, sizeof(stack));
    if (capacity > 0)
    {
        stack.base = (uint8_t*)m3AllocZeroed(capacity);
        stack.capacity = stack.base != NULL ? capacity : 0;
    }
    return stack;
}

void m3StackDestroy(m3Stack* stack)
{
    m3Free(stack->base);
    memset(stack, 0, sizeof(*stack));
}

void* m3StackAlloc(m3Stack* stack, int32_t size)
{
    if (size <= 0)
    {
        M3_ASSERT(false);
        return NULL;
    }
    // 16-byte alignment keeps every scratch block SIMD-safe.
    int32_t aligned = (size + 15) & ~15;
    if (stack->top + aligned > stack->capacity)
    {
        // Loud failure: latch the flag, return NULL, keep running. The
        // caller propagates m3_errorCapacity and the world can grow the
        // stack between steps.
        stack->overflow = 1;
        return NULL;
    }
    void* memory = stack->base + stack->top;
    stack->top += aligned;
    return memory;
}

void m3StackReset(m3Stack* stack)
{
    // The step must read the overflow flag before resetting; reset
    // starts the next step clean.
    stack->top = 0;
    stack->overflow = 0;
}

m3IdPool m3IdPoolCreate(int32_t capacity)
{
    m3IdPool pool;
    memset(&pool, 0, sizeof(pool));
    if (capacity <= 0)
    {
        M3_ASSERT(false);
        return pool;
    }
    pool.capacity = capacity;
    M3_ALLOC(pool.generations, capacity, uint16_t);
    M3_ALLOC(pool.alive, capacity, uint8_t);
    M3_ALLOC(pool.freeQueue, capacity, int32_t);
    return pool;
}

void m3IdPoolDestroy(m3IdPool* pool)
{
    m3Free(pool->generations);
    m3Free(pool->alive);
    m3Free(pool->freeQueue);
    memset(pool, 0, sizeof(*pool));
}

int32_t m3IdPoolAlloc(m3IdPool* pool)
{
    // Recycled slots first, in FIFO order (canonical reuse), then fresh
    // slots up to capacity.
    if (pool->freeCount > 0)
    {
        int32_t index = pool->freeQueue[pool->freeHead];
        pool->freeHead = (pool->freeHead + 1) % pool->capacity;
        pool->freeCount -= 1;
        pool->alive[index] = 1;
        return index;
    }
    if (pool->maxIndex < pool->capacity)
    {
        int32_t index = pool->maxIndex;
        pool->maxIndex += 1;
        pool->alive[index] = 1;
        return index;
    }
    return -1; // exhausted: the caller fails loudly
}

void m3IdPoolFree(m3IdPool* pool, int32_t index)
{
    if (index < 0 || index >= pool->capacity || pool->alive[index] == 0)
    {
        M3_ASSERT(false);
        return;
    }
    pool->alive[index] = 0;
    if (pool->generations[index] == 0xFFFF)
    {
        // Retire instead of wrapping: a wrapped generation could make a
        // very stale handle validate again.
        pool->retiredCount += 1;
        return;
    }
    pool->generations[index] += 1;
    pool->freeQueue[(pool->freeHead + pool->freeCount) % pool->capacity] = index;
    pool->freeCount += 1;
}

int m3IdPoolValid(const m3IdPool* pool, int32_t index, uint16_t generation)
{
    return index >= 0 && index < pool->capacity && pool->alive[index] != 0 &&
           pool->generations[index] == generation;
}
