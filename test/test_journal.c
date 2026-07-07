// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Journal gate: a recorded session replayed into a fresh world must
// reproduce the original bit for bit, including the minted ids (id
// determinism). Until the world hash lands in task 6, equality is
// checked field by field with memcmp. Black box: public headers only.

#include "maul3d/body.h"

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

static int SamePos(m3Pos3 a, m3Pos3 b)
{
    return memcmp(&a, &b, sizeof(a)) == 0;
}

static int SameVec(m3Vec3 a, m3Vec3 b)
{
    return memcmp(&a, &b, sizeof(a)) == 0;
}

int main(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 8;

    // Record a session on world A: creates, setters, a destroy, and a
    // recreate into the recycled slot.
    m3WorldId a = m3CreateWorld(&def);
    uint8_t buffer[4096];
    CHECK(m3World_JournalBegin(a, buffer, (int32_t)sizeof(buffer)), "journal begins");

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 5.0, 0.0};
    m3BodyId a0 = m3CreateBody(a, &bd);
    bd.position = (m3Pos3){2.0, 5.0, -1.0};
    m3BodyId a1 = m3CreateBody(a, &bd);
    m3Body_SetLinearVelocity(a0, (m3Vec3){1.0f, 0.0f, 0.0f});
    m3Body_SetAngularVelocity(a1, (m3Vec3){0.0f, 3.0f, 0.0f});
    m3DestroyBody(a0);
    bd.position = (m3Pos3){-4.0, 1.0, 2.0};
    m3BodyId a2 = m3CreateBody(a, &bd); // recycles a0's slot, new generation
    CHECK(a2.index1 == a0.index1 && a2.generation != a0.generation, "recycle in the journal");

    int32_t bytes = m3World_JournalEnd(a);
    CHECK(bytes > 0, "journal ends with bytes");

    // Replay into a fresh world B: identical ids, identical state.
    m3WorldId b = m3CreateWorld(&def);
    CHECK(m3World_JournalReplay(b, buffer, bytes), "replay succeeds");

    m3BodyId b1 = {a1.index1, b.index1 - 1 == 0 ? 0 : (uint16_t)(b.index1 - 1), a1.generation};
    b1.world0 = (uint16_t)(b.index1 - 1);
    m3BodyId b2 = {a2.index1, (uint16_t)(b.index1 - 1), a2.generation};
    CHECK(m3Body_IsValid(b1) && m3Body_IsValid(b2), "replayed ids validate in world B");
    CHECK(SamePos(m3Body_GetPosition(b1), m3Body_GetPosition(a1)), "b1 position matches");
    CHECK(SamePos(m3Body_GetPosition(b2), m3Body_GetPosition(a2)), "b2 position matches");
    CHECK(SameVec(m3Body_GetAngularVelocity(b1), m3Body_GetAngularVelocity(a1)),
          "b1 angular velocity matches");
    m3BodyId b0 = {a0.index1, (uint16_t)(b.index1 - 1), a0.generation};
    CHECK(!m3Body_IsValid(b0), "the destroyed body is stale in the replay too");

    // A truncated stream is rejected loudly and never half-applies
    // silently.
    m3WorldId c = m3CreateWorld(&def);
    CHECK(!m3World_JournalReplay(c, buffer, bytes - 3), "a truncated journal is rejected");

    // Overflow: a too-small buffer latches and End reports -1.
    m3WorldId d = m3CreateWorld(&def);
    uint8_t tiny[16];
    CHECK(m3World_JournalBegin(d, tiny, (int32_t)sizeof(tiny)), "tiny journal begins");
    m3CreateBody(d, &bd);
    CHECK(m3World_JournalEnd(d) == -1, "journal overflow reports -1, never silence");

    m3DestroyWorld(a);
    m3DestroyWorld(b);
    m3DestroyWorld(c);
    m3DestroyWorld(d);

    if (s_failures == 0)
    {
        printf("test_journal: all checks passed\n");
    }
    return s_failures == 0 ? 0 : 1;
}
