// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// m3lockstep: the rollback-networking recipe as an executable
// document (9-4). Peer A plays ONLINE: it predicts missing remote
// inputs (as empty), rolls back to its confirmed frontier when
// late inputs arrive, and re-simulates to the present. Peer B is
// the offline truth: it steps a frame only when every input for
// that frame is in hand. If the engine's bit contract holds, A's
// confirmed timeline and B's timeline are the same timeline. The
// run PROVES it at every checkpoint and exits nonzero otherwise.
//
// The recipe in one breath: inputs are the only truth; snapshot
// the last all-confirmed frame; predict forward; on a late input,
// restore, re-apply, re-predict; compare hashes to catch desyncs
// the moment they are born.

#include "maul3d/shape.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define FRAMES      300
#define CHECK_EVERY 60
#define MAX_DELAY   5

typedef struct Peer
{
    m3WorldId world;
    m3BodyId crates[2]; // one crate per player
} Peer;

static void BuildScene(Peer* p)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    p->world = m3CreateWorld(&def);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(p->world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane fl = {{0.0f, 1.0f, 0.0f}, 0.0f};
    m3CreatePlaneShape(ground, &sd, &fl);
    for (int32_t k = 0; k < 2; ++k)
    {
        m3BodyDef bd = m3DefaultBodyDef();
        bd.type = m3_dynamicBody;
        bd.position = (m3Pos3){k == 0 ? -1.5 : 1.5, 0.5, 0.0};
        p->crates[k] = m3CreateBody(p->world, &bd);
        m3CreateBoxShape(p->crates[k], &sd, (m3Vec3){0.4f, 0.4f, 0.4f});
    }
}

// The input script: each player nudges their crate with a pattern
// derived only from the frame index. Deterministic on both peers.
static m3Vec3 InputFor(int32_t frame, int32_t player)
{
    uint32_t h = (uint32_t)(frame * 2654435761u) ^ (uint32_t)(player * 40503u);
    if ((h >> 4) % 3 != 0)
    {
        return (m3Vec3){0.0f, 0.0f, 0.0f}; // most frames: no input
    }
    float x = ((float)((h >> 8) % 200) - 100.0f) / 400.0f;
    float z = ((float)((h >> 16) % 200) - 100.0f) / 400.0f;
    return (m3Vec3){x, 0.0f, z};
}

// One simulated frame: apply both players' inputs, then step.
static void StepFrame(Peer* p, int32_t frame, const uint8_t* haveRemote)
{
    for (int32_t k = 0; k < 2; ++k)
    {
        // Player 1 is remote for peer A: a missing input predicts
        // as EMPTY (the classic cheap prediction; wrong guesses are
        // what the rollback exists to repair).
        if (k == 1 && haveRemote != NULL && haveRemote[frame] == 0)
        {
            continue;
        }
        m3Vec3 in = InputFor(frame, k);
        if (in.x != 0.0f || in.z != 0.0f)
        {
            m3Body_ApplyLinearImpulse(p->crates[k], in);
        }
    }
    m3World_Step(p->world, 1.0f / 60.0f, 4);
}

int main(void)
{
    // The artificial wire: remote input for frame F arrives at
    // wall frame F + delay(F), delays 1..MAX_DELAY-1, LCG-seeded.
    static int32_t arriveAt[FRAMES];
    uint32_t rng = 774411u;
    for (int32_t f = 0; f < FRAMES; ++f)
    {
        rng = rng * 1664525u + 1013904223u;
        arriveAt[f] = f + 1 + (int32_t)(rng % (MAX_DELAY - 1));
    }

    Peer online;
    Peer truth;
    BuildScene(&online);
    BuildScene(&truth);

    int32_t snapBytes = m3World_SnapshotSize(online.world);
    uint8_t* confirmedSnap = (uint8_t*)malloc((size_t)snapBytes);
    m3World_Snapshot(online.world, confirmedSnap, snapBytes);

    static uint8_t haveRemote[FRAMES + MAX_DELAY];
    memset(haveRemote, 0, sizeof(haveRemote));

    uint64_t onlineChecks[FRAMES / CHECK_EVERY];
    uint64_t truthChecks[FRAMES / CHECK_EVERY];
    memset(onlineChecks, 0, sizeof(onlineChecks));
    memset(truthChecks, 0, sizeof(truthChecks));

    int32_t confirmed = 0; // frames 0..confirmed-1 fully known
    int32_t truthFrame = 0;
    int32_t rollbacks = 0;
    int32_t deepest = 0;

    for (int32_t wall = 0; wall < FRAMES + MAX_DELAY; ++wall)
    {
        // Deliveries for this wall frame.
        for (int32_t f = 0; f < FRAMES; ++f)
        {
            if (arriveAt[f] == wall)
            {
                haveRemote[f] = 1;
            }
        }

        // ONLINE PEER: rollback to the confirmed frontier, advance
        // it over newly confirmed frames, snapshot, then re-predict
        // up to the present wall frame.
        if (wall < FRAMES || confirmed < FRAMES)
        {
            if (!m3World_Restore(online.world, confirmedSnap, snapBytes))
            {
                fprintf(stderr, "m3lockstep: restore failed\n");
                return 1;
            }
            int32_t rollbackDepth = wall < FRAMES ? wall - confirmed : FRAMES - confirmed;
            if (rollbackDepth > deepest)
            {
                deepest = rollbackDepth;
            }
            if (rollbackDepth > 0)
            {
                rollbacks += 1;
            }
            int32_t newConfirmed = confirmed;
            while (newConfirmed < FRAMES && haveRemote[newConfirmed] != 0)
            {
                StepFrame(&online, newConfirmed, NULL); // all inputs known
                newConfirmed += 1;
                if (newConfirmed % CHECK_EVERY == 0)
                {
                    onlineChecks[newConfirmed / CHECK_EVERY - 1] = m3World_Hash(online.world);
                }
            }
            if (newConfirmed > confirmed)
            {
                confirmed = newConfirmed;
                m3World_Snapshot(online.world, confirmedSnap, snapBytes);
            }
            for (int32_t f = confirmed; f < wall && f < FRAMES; ++f)
            {
                StepFrame(&online, f, haveRemote); // predicted zone
            }
        }

        // TRUTH PEER: steps only frames whose inputs are all here.
        while (truthFrame < FRAMES && haveRemote[truthFrame] != 0)
        {
            StepFrame(&truth, truthFrame, NULL);
            truthFrame += 1;
            if (truthFrame % CHECK_EVERY == 0)
            {
                truthChecks[truthFrame / CHECK_EVERY - 1] = m3World_Hash(truth.world);
            }
        }
    }

    int failures = 0;
    if (confirmed != FRAMES || truthFrame != FRAMES)
    {
        fprintf(stderr, "m3lockstep: incomplete run (%d / %d)\n", confirmed, truthFrame);
        failures += 1;
    }
    for (int32_t c = 0; c < FRAMES / CHECK_EVERY; ++c)
    {
        int ok = onlineChecks[c] == truthChecks[c];
        printf("checkpoint %3d: online %016llx truth %016llx %s\n", (c + 1) * CHECK_EVERY,
               (unsigned long long)onlineChecks[c], (unsigned long long)truthChecks[c],
               ok ? "MATCH" : "DESYNC");
        if (!ok)
        {
            failures += 1;
        }
    }
    printf("m3lockstep: %d rollbacks, deepest window %d frames, %s\n", rollbacks, deepest,
           failures == 0 ? "every checkpoint MATCHED" : "DESYNC DETECTED");
    free(confirmedSnap);
    m3DestroyWorld(online.world);
    m3DestroyWorld(truth.world);
    return failures == 0 ? 0 : 1;
}
