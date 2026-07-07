// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Core gate: the step scratch stack (never aborts, loud overflow) and
// the id pool behind every generation-tagged handle (FIFO canonical
// reuse, generation bump on free, retire instead of wrap). White-box:
// includes the internal allocator header.

#include "allocator.h"

#include <stdint.h>
#include <stdio.h>

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

int main(void)
{
    TestStack();
    TestIdPool();
    TestIdShapes();
    if (s_failures == 0)
    {
        printf("test_core: all checks passed\n");
    }
    return s_failures == 0 ? 0 : 1;
}
