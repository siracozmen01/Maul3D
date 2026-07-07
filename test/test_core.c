// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Core gate: the step scratch stack (never aborts, loud overflow) and
// the id pool behind every generation-tagged handle (FIFO canonical
// reuse, generation bump on free, retire instead of wrap). White-box:
// includes the internal allocator header.

#include "allocator.h"
#include "simd.h"

#include "maul3d/math.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int s_failures = 0;

#define CHECK(cond, msg)                                                                           \
    do                                                                                             \
    {                                                                                              \
        if (!(cond))                                                                               \
        {                                                                                          \
            printf("FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__);                                 \
            s_failures += 1;                                                                       \
        }                                                                                          \
    } while (0)

static void TestStack(void)
{
    m3Stack stack = m3StackCreate(256);
    CHECK(stack.capacity == 256, "stack capacity");

    void* a = m3StackAlloc(&stack, 10);
    void* b = m3StackAlloc(&stack, 20);
    CHECK(a != NULL && b != NULL, "allocations succeed");
    CHECK(((uintptr_t)a & 15u) == 0 && ((uintptr_t)b & 15u) == 0, "16-byte aligned");
    CHECK((uint8_t*)b - (uint8_t*)a == 16, "10 bytes rounds up to one 16-byte block");

    // Overflow is loud but survivable: NULL plus a latched flag, and
    // the stack keeps serving what still fits.
    void* big = m3StackAlloc(&stack, 512);
    CHECK(big == NULL, "overflow returns NULL, never aborts");
    CHECK(stack.overflow == 1, "overflow is latched for the step to read");
    void* still = m3StackAlloc(&stack, 16);
    CHECK(still != NULL, "the stack keeps working after an overflow");

    // Reset starts the next step clean and reuses the same memory.
    m3StackReset(&stack);
    CHECK(stack.overflow == 0 && stack.top == 0, "reset clears top and flag");
    void* again = m3StackAlloc(&stack, 10);
    CHECK(again == a, "reset reuses the same base");

    m3StackDestroy(&stack);
    CHECK(stack.base == NULL, "destroy clears the struct");
}

static void TestIdPool(void)
{
    m3IdPool pool = m3IdPoolCreate(4);

    int32_t i0 = m3IdPoolAlloc(&pool);
    int32_t i1 = m3IdPoolAlloc(&pool);
    int32_t i2 = m3IdPoolAlloc(&pool);
    CHECK(i0 == 0 && i1 == 1 && i2 == 2, "fresh slots come in order");
    CHECK(m3IdPoolValid(&pool, i1, 0), "a live slot at generation 0 validates");

    // Free bumps the generation: the old handle must die.
    m3IdPoolFree(&pool, i1);
    CHECK(!m3IdPoolValid(&pool, i1, 0), "a stale generation is rejected");

    // FIFO reuse: free two, get them back in the order they were freed.
    m3IdPoolFree(&pool, i0);
    int32_t r0 = m3IdPoolAlloc(&pool);
    int32_t r1 = m3IdPoolAlloc(&pool);
    CHECK(r0 == i1 && r1 == i0, "recycling is FIFO, the canonical order");
    CHECK(m3IdPoolValid(&pool, r0, 1), "the recycled slot validates at its new generation");

    // Exhaustion is a -1, not a hidden growth.
    int32_t i3 = m3IdPoolAlloc(&pool);
    CHECK(i3 == 3, "last fresh slot");
    CHECK(m3IdPoolAlloc(&pool) == -1, "an exhausted pool fails loudly");

    m3IdPoolDestroy(&pool);
}

static void TestIdShapes(void)
{
    // The null id is all zero, and the structs stay padding-free sizes
    // the snapshot can rely on.
    m3BodyId null = {0, 0, 0};
    CHECK(null.index1 == 0, "null id");
    CHECK(sizeof(m3WorldId) == 8 && sizeof(m3BodyId) == 8 && sizeof(m3ShapeId) == 8,
          "ids are 8 bytes");
}

// Deterministic PRNG (SplitMix32) so the sweep reproduces bit for bit
// on every machine.
static uint32_t s_state = 0x9E3779B9u;
static uint32_t NextU32(void)
{
    s_state += 0x9E3779B9u;
    uint32_t z = s_state;
    z = (z ^ (z >> 16)) * 0x85EBCA6Bu;
    z = (z ^ (z >> 13)) * 0xC2B2AE35u;
    return z ^ (z >> 16);
}
static float NextF(void)
{
    // [-8, 8) with a 24-bit mantissa: exercises signs and magnitudes
    // without generating NaN or infinity.
    return -8.0f + 16.0f * ((float)(NextU32() >> 8) * (1.0f / 16777216.0f));
}

static void TestSimdBitLaw(void)
{
    // The vector ops must equal the pinned scalar ops bit for bit,
    // lane by lane: min/max as compare-and-select, and mul-add in TWO
    // roundings (the 4-wide law: the SSE2 baseline has no FMA, so no
    // backend may fuse).
    for (int32_t iter = 0; iter < 4096; ++iter)
    {
        float a[4];
        float b[4];
        float c[4];
        for (int32_t i = 0; i < 4; ++i)
        {
            a[i] = NextF();
            b[i] = NextF();
            c[i] = NextF();
        }
        m3f4 va = m3F4Load(a);
        m3f4 vb = m3F4Load(b);
        m3f4 vc = m3F4Load(c);

        float outMin[4];
        float outMax[4];
        float outMulAdd[4];
        float outNegMulAdd[4];
        m3F4Store(outMin, m3F4Min(va, vb));
        m3F4Store(outMax, m3F4Max(va, vb));
        m3F4Store(outMulAdd, m3F4MulAdd(va, vb, vc));
        m3F4Store(outNegMulAdd, m3F4NegMulAdd(va, vb, vc));

        for (int32_t i = 0; i < 4; ++i)
        {
            float sMin = m3MinF(a[i], b[i]);
            float sMax = m3MaxF(a[i], b[i]);
            float sMulAdd = a[i] * b[i] + c[i];
            float sNegMulAdd = c[i] - a[i] * b[i];
            CHECK(memcmp(&outMin[i], &sMin, 4) == 0, "vector min matches pinned scalar bits");
            CHECK(memcmp(&outMax[i], &sMax, 4) == 0, "vector max matches pinned scalar bits");
            CHECK(memcmp(&outMulAdd[i], &sMulAdd, 4) == 0,
                  "mul-add is two roundings, matching scalar");
            CHECK(memcmp(&outNegMulAdd[i], &sNegMulAdd, 4) == 0,
                  "neg-mul-add is two roundings, matching scalar");
        }
    }

    // Select semantics: an all-ones mask picks a, all-zeros picks b.
    float av[4] = {1.0f, 2.0f, 3.0f, 4.0f};
    float bv[4] = {9.0f, 8.0f, 7.0f, 6.0f};
    float out[4];
    m3F4Store(out, m3F4Select(m3F4GT(m3F4Set1(1.0f), m3F4Zero()), m3F4Load(av), m3F4Load(bv)));
    CHECK(out[0] == 1.0f && out[3] == 4.0f, "true mask selects a");
    m3F4Store(out, m3F4Select(m3F4GT(m3F4Zero(), m3F4Set1(1.0f)), m3F4Load(av), m3F4Load(bv)));
    CHECK(out[0] == 9.0f && out[3] == 6.0f, "false mask selects b");
}

int main(void)
{
    TestStack();
    TestIdPool();
    TestIdShapes();
    TestSimdBitLaw();
    if (s_failures == 0)
    {
        printf("test_core: all checks passed\n");
    }
    return s_failures == 0 ? 0 : 1;
}
