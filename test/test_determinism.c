// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The determinism gate, seed edition (plan task 1). Until the world
// exists, the gate hashes a fixed float recurrence: every CI cell on
// every platform and compiler must print the identical M3_DET_HASH
// line, proving the toolchain honors the determinism flags before any
// physics lands on top. The golden-scene, replay, rollback, and twin
// gates replace this in plan task 10.

#include "maul3d/base.h"

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

int main(void)
{
    CHECK(m3GetVersion() == 100, "version 0.1.0 encodes as 100");

    // A fixed float recurrence with a multiply, an add, and a divide:
    // enough surface for fast-math or contraction to corrupt if the
    // flags ever regress.
    float x = 1.0f;
    uint64_t h = M3_HASH_INIT;
    for (int32_t i = 0; i < 1000; ++i)
    {
        x = 0.5f * x + 0.1f;
        x = x / (1.0f + 0.25f * x);
        h = m3Hash64(h, &x, (int32_t)sizeof(x));
    }
    CHECK(x > 0.0f && x < 1.0f, "the recurrence stays in its band");

    printf("M3_DET_HASH=%016llx\n", (unsigned long long)h);
    return s_failures == 0 ? 0 : 1;
}
