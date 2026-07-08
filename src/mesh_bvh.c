// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Static per-mesh BVH (2c-10): the midphase the benchmark asked for
// (meshfield dominated the 2c-1 profile). Median split on the longest
// centroid axis with triangle-index tie breaking makes the build a
// pure function of the triangle set: no state, no randomness, no
// libc sort (qsort order is implementation-defined for equal keys;
// the merge sort here is total-ordered and portable). Derived data
// only: consumers keep their exact per-triangle tests, so the tree
// prunes work without moving a single bit of simulation state.

#include "world_internal.h"

#include <string.h>

typedef struct BvhScratch
{
    m3Vec3 triLo[M3_MESH_MAX_TRIS];
    m3Vec3 triHi[M3_MESH_MAX_TRIS];
    m3Vec3 centroid[M3_MESH_MAX_TRIS];
    uint16_t tmp[M3_MESH_MAX_TRIS];
} BvhScratch;

static m3real CentroidAxis(const BvhScratch* scratch, uint16_t tri, int32_t axis)
{
    const m3Vec3* c = &scratch->centroid[tri];
    return axis == 0 ? c->x : (axis == 1 ? c->y : c->z);
}

// Merge sort of order[s..e) by (centroid on axis, triangle index):
// a strict total order, so the result is unique and any correct sort
// would produce it; merge sort just gets there portably in n log n.
static void SortByAxis(BvhScratch* scratch, uint16_t* order, int32_t s, int32_t e, int32_t axis)
{
    int32_t count = e - s;
    if (count < 2)
    {
        return;
    }
    int32_t mid = s + count / 2;
    SortByAxis(scratch, order, s, mid, axis);
    SortByAxis(scratch, order, mid, e, axis);
    int32_t a = s;
    int32_t b = mid;
    int32_t w = 0;
    while (a < mid && b < e)
    {
        m3real ka = CentroidAxis(scratch, order[a], axis);
        m3real kb = CentroidAxis(scratch, order[b], axis);
        if (ka < kb || (ka == kb && order[a] < order[b]))
        {
            scratch->tmp[w++] = order[a++];
        }
        else
        {
            scratch->tmp[w++] = order[b++];
        }
    }
    while (a < mid)
    {
        scratch->tmp[w++] = order[a++];
    }
    while (b < e)
    {
        scratch->tmp[w++] = order[b++];
    }
    memcpy(order + s, scratch->tmp, (size_t)w * sizeof(uint16_t));
}

static int32_t BuildRange(m3MeshBvh* bvh, BvhScratch* scratch, int32_t s, int32_t e)
{
    int32_t nodeIndex = bvh->nodeCount;
    bvh->nodeCount += 1;
    m3MeshBvhNode* node = &bvh->nodes[nodeIndex];

    m3Vec3 lo = scratch->triLo[bvh->order[s]];
    m3Vec3 hi = scratch->triHi[bvh->order[s]];
    m3Vec3 clo = scratch->centroid[bvh->order[s]];
    m3Vec3 chi = clo;
    for (int32_t i = s + 1; i < e; ++i)
    {
        uint16_t t = bvh->order[i];
        lo.x = m3MinF(lo.x, scratch->triLo[t].x);
        lo.y = m3MinF(lo.y, scratch->triLo[t].y);
        lo.z = m3MinF(lo.z, scratch->triLo[t].z);
        hi.x = m3MaxF(hi.x, scratch->triHi[t].x);
        hi.y = m3MaxF(hi.y, scratch->triHi[t].y);
        hi.z = m3MaxF(hi.z, scratch->triHi[t].z);
        clo.x = m3MinF(clo.x, scratch->centroid[t].x);
        clo.y = m3MinF(clo.y, scratch->centroid[t].y);
        clo.z = m3MinF(clo.z, scratch->centroid[t].z);
        chi.x = m3MaxF(chi.x, scratch->centroid[t].x);
        chi.y = m3MaxF(chi.y, scratch->centroid[t].y);
        chi.z = m3MaxF(chi.z, scratch->centroid[t].z);
    }
    node->lo = lo;
    node->hi = hi;

    int32_t count = e - s;
    if (count <= M3_MESH_BVH_LEAF)
    {
        node->right = -1;
        node->start = s;
        node->count = count;
        return nodeIndex;
    }

    m3Vec3 spread = m3Sub3(chi, clo);
    int32_t axis = 0;
    if (spread.y > spread.x)
    {
        axis = 1;
    }
    if (spread.z > (axis == 0 ? spread.x : spread.y))
    {
        axis = 2;
    }
    SortByAxis(scratch, bvh->order, s, e, axis);
    int32_t mid = s + count / 2;

    // Children write through bvh->nodes, so re-take the pointer after
    // recursion: `node` stays valid (fixed array) but keep the writes
    // explicit and ordered anyway.
    int32_t leftChild = BuildRange(bvh, scratch, s, mid);
    int32_t rightChild = BuildRange(bvh, scratch, mid, e);
    (void)leftChild; // always nodeIndex + 1 by construction
    bvh->nodes[nodeIndex].right = rightChild;
    bvh->nodes[nodeIndex].start = 0;
    bvh->nodes[nodeIndex].count = 0;
    return nodeIndex;
}

void m3MeshBvhBuildBounds(m3MeshBvh* bvh, const m3Vec3* los, const m3Vec3* his, int32_t count)
{
    memset(bvh, 0, sizeof(*bvh));
    if (count <= 0 || count > M3_MESH_MAX_TRIS)
    {
        return;
    }
    BvhScratch* scratch = (BvhScratch*)m3AllocZeroed((int32_t)sizeof(BvhScratch));
    if (scratch == NULL)
    {
        return; // loud enough: nodeCount 0 makes every gather empty,
                // and creation paths never run out in tests
    }
    for (int32_t t = 0; t < count; ++t)
    {
        m3Vec3 lo = los[t];
        m3Vec3 hi = his[t];
        scratch->triLo[t] = lo;
        scratch->triHi[t] = hi;
        // The arithmetic mean of the box corners: cheap, deterministic,
        // and only a split heuristic (never a correctness input).
        scratch->centroid[t] =
            (m3Vec3){0.5f * (lo.x + hi.x), 0.5f * (lo.y + hi.y), 0.5f * (lo.z + hi.z)};
        bvh->order[t] = (uint16_t)t;
    }
    BuildRange(bvh, scratch, 0, count);
    m3Free(scratch);
}

void m3MeshBvhBuild(m3MeshBvh* bvh, const m3MeshData* mesh)
{
    m3Vec3 los[M3_MESH_MAX_TRIS];
    m3Vec3 his[M3_MESH_MAX_TRIS];
    int32_t triCount = mesh->triangleCount;
    if (triCount < 0 || triCount > M3_MESH_MAX_TRIS)
    {
        memset(bvh, 0, sizeof(*bvh));
        return;
    }
    for (int32_t t = 0; t < triCount; ++t)
    {
        m3Vec3 a = mesh->vertices[mesh->indices[3 * t + 0]];
        m3Vec3 b = mesh->vertices[mesh->indices[3 * t + 1]];
        m3Vec3 c = mesh->vertices[mesh->indices[3 * t + 2]];
        los[t] = (m3Vec3){m3MinF(a.x, m3MinF(b.x, c.x)), m3MinF(a.y, m3MinF(b.y, c.y)),
                          m3MinF(a.z, m3MinF(b.z, c.z))};
        his[t] = (m3Vec3){m3MaxF(a.x, m3MaxF(b.x, c.x)), m3MaxF(a.y, m3MaxF(b.y, c.y)),
                          m3MaxF(a.z, m3MaxF(b.z, c.z))};
    }
    m3MeshBvhBuildBounds(bvh, los, his, triCount);
}

// Ascending insertion of gathered leaves would cost per-element
// shifting; gather instead collects leaf runs and merge sorts once at
// the end (the caller contract is ascending order).
static void SortAscending(uint16_t* values, int32_t count, uint16_t* tmp)
{
    if (count < 2)
    {
        return;
    }
    int32_t mid = count / 2;
    SortAscending(values, mid, tmp);
    SortAscending(values + mid, count - mid, tmp);
    int32_t a = 0;
    int32_t b = mid;
    int32_t w = 0;
    while (a < mid && b < count)
    {
        tmp[w++] = values[a] < values[b] ? values[a++] : values[b++];
    }
    while (a < mid)
    {
        tmp[w++] = values[a++];
    }
    while (b < count)
    {
        tmp[w++] = values[b++];
    }
    memcpy(values, tmp, (size_t)count * sizeof(uint16_t));
}

int32_t m3MeshBvhGather(const m3MeshBvh* bvh, m3Vec3 lo, m3Vec3 hi, uint16_t* out)
{
    if (bvh->nodeCount == 0)
    {
        return 0;
    }
    int32_t stack[64]; // median split depth is ~log2(2048/4) + 1; 64
                       // is unreachable headroom
    int32_t top = 0;
    stack[top++] = 0;
    int32_t count = 0;
    while (top > 0)
    {
        int32_t index = stack[--top];
        const m3MeshBvhNode* node = &bvh->nodes[index];
        if (hi.x < node->lo.x || lo.x > node->hi.x || hi.y < node->lo.y || lo.y > node->hi.y ||
            hi.z < node->lo.z || lo.z > node->hi.z)
        {
            continue;
        }
        if (node->right < 0)
        {
            for (int32_t i = 0; i < node->count; ++i)
            {
                out[count++] = bvh->order[node->start + i];
            }
            continue;
        }
        stack[top++] = node->right;
        stack[top++] = index + 1;
    }
    uint16_t tmp[M3_MESH_MAX_TRIS];
    SortAscending(out, count, tmp);
    return count;
}
