// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Hull infrastructure: the analytic box builder and the interned,
// refcounted, immutable hull pool (lifetime 3 in the memory model).
// The QuickHull point-cloud builder and the general mass integrator
// arrive together in a later slice (fracture fragments need them);
// boxes need neither, their mass properties are closed form.

#include "world_internal.h"

#include <string.h>

void m3BuildBoxHull(m3HullData* out, m3Vec3 halfExtents)
{
    memset(out, 0, sizeof(*out));
    m3real hx = halfExtents.x;
    m3real hy = halfExtents.y;
    m3real hz = halfExtents.z;

    // Canonical vertex order (fixed forever: the snapshot and the SAT
    // feature ids will lean on it).
    const m3Vec3 verts[8] = {
        {-hx, -hy, -hz}, {hx, -hy, -hz}, {hx, hy, -hz}, {-hx, hy, -hz},
        {-hx, -hy, hz},  {hx, -hy, hz},  {hx, hy, hz},  {-hx, hy, hz},
    };
    // Six faces, outward normals, counter-clockwise seen from outside.
    const m3Vec3 normals[6] = {
        {1.0f, 0.0f, 0.0f},  {-1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f},
        {0.0f, -1.0f, 0.0f}, {0.0f, 0.0f, 1.0f},  {0.0f, 0.0f, -1.0f},
    };
    const m3real offsets[6] = {hx, hx, hy, hy, hz, hz};
    const uint8_t loops[6][4] = {
        {1, 2, 6, 5}, // +x
        {0, 4, 7, 3}, // -x
        {2, 3, 7, 6}, // +y
        {0, 1, 5, 4}, // -y
        {4, 5, 6, 7}, // +z
        {0, 3, 2, 1}, // -z
    };

    out->vertexCount = 8;
    out->faceCount = 6;
    out->indexCount = 24;
    memcpy(out->vertices, verts, sizeof(verts));
    memcpy(out->faceNormals, normals, sizeof(normals));
    memcpy(out->faceOffsets, offsets, sizeof(offsets));
    for (int32_t f = 0; f < 6; ++f)
    {
        out->faceVertCounts[f] = 4;
        out->faceVertStart[f] = (uint8_t)(4 * f);
        for (int32_t k = 0; k < 4; ++k)
        {
            out->faceIndices[4 * f + k] = loops[f][k];
        }
    }

    // Closed-form box mass at unit density: m = 8 hx hy hz,
    // I = m/3 diag(hy^2+hz^2, hx^2+hz^2, hx^2+hy^2), centroid at the
    // origin.
    float m = 8.0f * hx * hy * hz;
    out->unitMass = m;
    out->unitCom = (m3Vec3){0.0f, 0.0f, 0.0f};
    out->unitInertiaCom = m3MakeZeroMat3();
    out->unitInertiaCom.cx.x = (m / 3.0f) * (hy * hy + hz * hz);
    out->unitInertiaCom.cy.y = (m / 3.0f) * (hx * hx + hz * hz);
    out->unitInertiaCom.cz.z = (m / 3.0f) * (hx * hx + hy * hy);
}

int32_t m3InternHull(m3World* world, const m3HullData* data)
{
    // Content dedupe in ascending slot order: identical hulls share
    // one slot, deterministically.
    int32_t maxIndex = world->hullPool.maxIndex;
    for (int32_t i = 0; i < maxIndex; ++i)
    {
        if (world->hullPool.alive[i] != 0 &&
            memcmp(&world->hullData[i], data, sizeof(m3HullData)) == 0)
        {
            world->hullRefCounts[i] += 1;
            return i;
        }
    }
    int32_t index = m3IdPoolAlloc(&world->hullPool);
    if (index < 0)
    {
        return -1; // exhausted: loud at the caller
    }
    world->hullData[index] = *data;
    world->hullRefCounts[index] = 1;
    return index;
}

void m3ReleaseHull(m3World* world, int32_t hullIndex)
{
    if (hullIndex < 0)
    {
        return;
    }
    world->hullRefCounts[hullIndex] -= 1;
    if (world->hullRefCounts[hullIndex] == 0)
    {
        memset(&world->hullData[hullIndex], 0, sizeof(m3HullData));
        m3IdPoolFree(&world->hullPool, hullIndex);
    }
}
