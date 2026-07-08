// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The voxel chunk (3-1): the destruction niche's foundation. State
// is a dense 16x16x16 occupancy bitset plus a uint16 payload per
// voxel (one padding-free snapshot block per chunk slot). The
// collision surface is DERIVED data under the standing law: a
// deterministic greedy sweep merges filled voxels into maximal
// boxes, a BVH goes over the boxes, and both are rebuilt as pure
// functions of the grid wherever content lands (create, journal
// replay, restore), so twin worlds agree byte for byte. Boxes are
// axis-aligned in the chunk frame BY CONSTRUCTION, which is what
// makes the sphere kernel in manifold.c an exact clamp instead of
// an iteration.

#include "world_internal.h"

#include <string.h>

bool m3VoxelGet(const m3VoxelChunkData* chunk, int32_t x, int32_t y, int32_t z)
{
    int32_t v = x + M3_VOXEL_DIM * (y + M3_VOXEL_DIM * z);
    return (chunk->occupancy[v >> 3] & (uint8_t)(1u << (v & 7))) != 0;
}

static void VoxelSet(m3VoxelChunkData* chunk, int32_t v)
{
    chunk->occupancy[v >> 3] |= (uint8_t)(1u << (v & 7));
}

// Pack a caller-friendly grid (one byte per voxel, zero = empty)
// into chunk state. Returns the filled count.
int32_t m3VoxelPack(m3VoxelChunkData* chunk, const uint8_t* voxels, const uint16_t* payload,
                    m3real cellSize)
{
    memset(chunk, 0, sizeof(*chunk));
    chunk->cellSize = cellSize;
    int32_t filled = 0;
    for (int32_t v = 0; v < M3_VOXEL_COUNT; ++v)
    {
        if (voxels[v] != 0)
        {
            VoxelSet(chunk, v);
            chunk->payload[v] = payload != NULL ? payload[v] : 0;
            filled += 1;
        }
    }
    chunk->filledCount = filled;
    return filled;
}

// The greedy merge: scan in canonical order (x fastest, then y,
// then z); from each unclaimed filled voxel grow a run along x,
// widen it along y, then deepen it along z, claiming as it goes.
// Deterministic by the fixed scan and growth order; maximal in the
// greedy sense (not globally optimal, which is NP-hard and
// unnecessary).
void m3VoxelSurfaceBuild(m3VoxelSurface* surface, const m3VoxelChunkData* chunk)
{
    memset(surface, 0, sizeof(*surface));
    uint8_t claimed[M3_VOXEL_COUNT];
    memset(claimed, 0, sizeof(claimed));

    for (int32_t z = 0; z < M3_VOXEL_DIM; ++z)
    {
        for (int32_t y = 0; y < M3_VOXEL_DIM; ++y)
        {
            for (int32_t x = 0; x < M3_VOXEL_DIM; ++x)
            {
                int32_t v = x + M3_VOXEL_DIM * (y + M3_VOXEL_DIM * z);
                if (claimed[v] != 0 || !m3VoxelGet(chunk, x, y, z))
                {
                    continue;
                }
                // Grow the run along x.
                int32_t x2 = x;
                while (x2 + 1 < M3_VOXEL_DIM && !claimed[v + (x2 - x) + 1] &&
                       m3VoxelGet(chunk, x2 + 1, y, z))
                {
                    x2 += 1;
                }
                // Widen along y: every column of the candidate row
                // must be filled and unclaimed.
                int32_t y2 = y;
                while (y2 + 1 < M3_VOXEL_DIM)
                {
                    bool rowOk = true;
                    for (int32_t xi = x; xi <= x2; ++xi)
                    {
                        int32_t vi = xi + M3_VOXEL_DIM * ((y2 + 1) + M3_VOXEL_DIM * z);
                        if (claimed[vi] != 0 || !m3VoxelGet(chunk, xi, y2 + 1, z))
                        {
                            rowOk = false;
                            break;
                        }
                    }
                    if (!rowOk)
                    {
                        break;
                    }
                    y2 += 1;
                }
                // Deepen along z: every cell of the candidate slab.
                int32_t z2 = z;
                while (z2 + 1 < M3_VOXEL_DIM)
                {
                    bool slabOk = true;
                    for (int32_t yi = y; yi <= y2 && slabOk; ++yi)
                    {
                        for (int32_t xi = x; xi <= x2; ++xi)
                        {
                            int32_t vi = xi + M3_VOXEL_DIM * (yi + M3_VOXEL_DIM * (z2 + 1));
                            if (claimed[vi] != 0 || !m3VoxelGet(chunk, xi, yi, z2 + 1))
                            {
                                slabOk = false;
                                break;
                            }
                        }
                    }
                    if (!slabOk)
                    {
                        break;
                    }
                    z2 += 1;
                }
                // Claim and emit.
                for (int32_t zi = z; zi <= z2; ++zi)
                {
                    for (int32_t yi = y; yi <= y2; ++yi)
                    {
                        for (int32_t xi = x; xi <= x2; ++xi)
                        {
                            claimed[xi + M3_VOXEL_DIM * (yi + M3_VOXEL_DIM * zi)] = 1;
                        }
                    }
                }
                int32_t b = surface->boxCount;
                M3_ASSERT(b < M3_VOXEL_MAX_BOXES); // 2048 = the exact
                                                   // isolated-voxel
                                                   // worst case
                surface->boxLo[b][0] = (uint8_t)x;
                surface->boxLo[b][1] = (uint8_t)y;
                surface->boxLo[b][2] = (uint8_t)z;
                surface->boxHi[b][0] = (uint8_t)x2;
                surface->boxHi[b][1] = (uint8_t)y2;
                surface->boxHi[b][2] = (uint8_t)z2;
                surface->boxCount = b + 1;
            }
        }
    }

    // The midphase: bounds per merged box, in chunk-frame meters.
    m3Vec3 los[M3_VOXEL_MAX_BOXES];
    m3Vec3 his[M3_VOXEL_MAX_BOXES];
    m3real cell = chunk->cellSize;
    for (int32_t b = 0; b < surface->boxCount; ++b)
    {
        los[b] = (m3Vec3){(m3real)surface->boxLo[b][0] * cell, (m3real)surface->boxLo[b][1] * cell,
                          (m3real)surface->boxLo[b][2] * cell};
        his[b] = (m3Vec3){(m3real)(surface->boxHi[b][0] + 1) * cell,
                          (m3real)(surface->boxHi[b][1] + 1) * cell,
                          (m3real)(surface->boxHi[b][2] + 1) * cell};
    }
    m3MeshBvhBuildBounds(&surface->bvh, los, his, surface->boxCount);
}

// Chunk-frame bounds of one merged box.
void m3VoxelBoxBounds(const m3VoxelSurface* surface, m3real cellSize, int32_t box, m3Vec3* lo,
                      m3Vec3* hi)
{
    *lo = (m3Vec3){(m3real)surface->boxLo[box][0] * cellSize,
                   (m3real)surface->boxLo[box][1] * cellSize,
                   (m3real)surface->boxLo[box][2] * cellSize};
    *hi = (m3Vec3){(m3real)(surface->boxHi[box][0] + 1) * cellSize,
                   (m3real)(surface->boxHi[box][1] + 1) * cellSize,
                   (m3real)(surface->boxHi[box][2] + 1) * cellSize};
}

// A merged box as hull data in the CHUNK frame (m3BuildBoxHull
// makes an origin-centered box; translate the vertices and shift
// the face plane offsets by the same center).
void m3VoxelBoxHull(const m3VoxelSurface* surface, m3real cellSize, int32_t box, m3HullData* out)
{
    m3Vec3 lo;
    m3Vec3 hi;
    m3VoxelBoxBounds(surface, cellSize, box, &lo, &hi);
    m3Vec3 half = m3MulSV3(0.5f, m3Sub3(hi, lo));
    m3Vec3 center = m3MulSV3(0.5f, m3Add3(lo, hi));
    m3BuildBoxHull(out, half);
    for (int32_t v = 0; v < out->vertexCount; ++v)
    {
        out->vertices[v] = m3Add3(out->vertices[v], center);
    }
    for (int32_t f = 0; f < out->faceCount; ++f)
    {
        out->faceOffsets[f] += m3Dot3(out->faceNormals[f], center);
    }
    out->center = m3Add3(out->center, center);
}

// ---------------------------------------------------------------
// Edits (3-2): deterministic state transitions.

// Wake every dynamic body whose fat bounds touch the edited region
// (world frame): a disturbance is a disturbance even for sleepers.
typedef struct m3VoxelWakeContext
{
    m3World* world;
} m3VoxelWakeContext;

static bool VoxelWakeCallback(int32_t shape, void* userContext)
{
    m3World* world = ((m3VoxelWakeContext*)userContext)->world;
    int32_t body = world->shapeBody[shape];
    if (world->types[body] == (uint8_t)m3_dynamicBody)
    {
        world->awake[body] = 1;
        world->sleepTimes[body] = 0.0f;
    }
    return true;
}

static void VoxelWakeRegion(m3World* world, int32_t shape, const int32_t lo[3], const int32_t hi[3])
{
    int32_t slot = world->shapeVoxelIndex[shape];
    m3real cell = world->voxelData[slot].cellSize;
    int32_t body = world->shapeBody[shape];
    const m3Transform* xf = &world->transforms[body];
    // The region's eight corners in the chunk frame, rotated out,
    // padded by the speculative margin so grazing sleepers wake too.
    double wlo[3] = {1.0e30, 1.0e30, 1.0e30};
    double whi[3] = {-1.0e30, -1.0e30, -1.0e30};
    for (int32_t c = 0; c < 8; ++c)
    {
        m3Vec3 corner = {(m3real)((c & 1) != 0 ? hi[0] + 1 : lo[0]) * cell,
                         (m3real)((c & 2) != 0 ? hi[1] + 1 : lo[1]) * cell,
                         (m3real)((c & 4) != 0 ? hi[2] + 1 : lo[2]) * cell};
        m3Vec3 r = m3RotateVec3(xf->q, corner);
        double p[3] = {xf->p.x + (double)r.x, xf->p.y + (double)r.y, xf->p.z + (double)r.z};
        for (int32_t k = 0; k < 3; ++k)
        {
            wlo[k] = p[k] < wlo[k] ? p[k] : wlo[k];
            whi[k] = p[k] > whi[k] ? p[k] : whi[k];
        }
    }
    for (int32_t k = 0; k < 3; ++k)
    {
        wlo[k] -= (double)M3_AABB_MARGIN;
        whi[k] += (double)M3_AABB_MARGIN;
    }
    m3VoxelWakeContext ctx = {world};
    m3TreeQuery(&world->tree, wlo, whi, VoxelWakeCallback, &ctx);
}

static bool VoxelCoordsValid(int32_t x, int32_t y, int32_t z)
{
    return x >= 0 && x < M3_VOXEL_DIM && y >= 0 && y < M3_VOXEL_DIM && z >= 0 && z < M3_VOXEL_DIM;
}

bool m3VoxelSetInternal(m3World* world, int32_t shape, int32_t x, int32_t y, int32_t z,
                        uint16_t payload)
{
    int32_t slot = world->shapeVoxelIndex[shape];
    m3VoxelChunkData* chunk = &world->voxelData[slot];
    int32_t v = x + M3_VOXEL_DIM * (y + M3_VOXEL_DIM * z);
    bool wasFilled = m3VoxelGet(chunk, x, y, z);
    chunk->occupancy[v >> 3] |= (uint8_t)(1u << (v & 7));
    chunk->payload[v] = payload;
    if (!wasFilled)
    {
        chunk->filledCount += 1;
        // Occupancy changed: the surface is stale. A payload-only
        // set falls through (the surface is a pure function of
        // occupancy and cell size, never of payload).
        m3VoxelSurfaceBuild(&world->voxelSurface[slot], chunk);
    }
    int32_t lo[3] = {x, y, z};
    VoxelWakeRegion(world, shape, lo, lo);
    return true;
}

bool m3VoxelClearInternal(m3World* world, int32_t shape, int32_t x, int32_t y, int32_t z)
{
    int32_t slot = world->shapeVoxelIndex[shape];
    m3VoxelChunkData* chunk = &world->voxelData[slot];
    int32_t v = x + M3_VOXEL_DIM * (y + M3_VOXEL_DIM * z);
    if (m3VoxelGet(chunk, x, y, z))
    {
        chunk->occupancy[v >> 3] &= (uint8_t)~(1u << (v & 7));
        chunk->payload[v] = 0;
        chunk->filledCount -= 1;
        m3VoxelSurfaceBuild(&world->voxelSurface[slot], chunk);
    }
    int32_t lo[3] = {x, y, z};
    VoxelWakeRegion(world, shape, lo, lo);
    return true;
}

int32_t m3VoxelClearBoxInternal(m3World* world, int32_t shape, const int32_t lo[3],
                                const int32_t hi[3])
{
    int32_t slot = world->shapeVoxelIndex[shape];
    m3VoxelChunkData* chunk = &world->voxelData[slot];
    int32_t cleared = 0;
    for (int32_t z = lo[2]; z <= hi[2]; ++z)
    {
        for (int32_t y = lo[1]; y <= hi[1]; ++y)
        {
            for (int32_t x = lo[0]; x <= hi[0]; ++x)
            {
                if (m3VoxelGet(chunk, x, y, z))
                {
                    int32_t v = x + M3_VOXEL_DIM * (y + M3_VOXEL_DIM * z);
                    chunk->occupancy[v >> 3] &= (uint8_t)~(1u << (v & 7));
                    chunk->payload[v] = 0;
                    cleared += 1;
                }
            }
        }
    }
    if (cleared > 0)
    {
        chunk->filledCount -= cleared;
        m3VoxelSurfaceBuild(&world->voxelSurface[slot], chunk);
    }
    VoxelWakeRegion(world, shape, lo, hi);
    return cleared;
}

// Public entries: validate, journal, apply (the command pattern).
static m3World* ResolveVoxelShape(m3ShapeId shapeId, int32_t* shapeOut)
{
    m3World* world = m3WorldFromIndex0(shapeId.world0);
    int32_t shape = world != NULL ? m3ShapeSlot(world, shapeId) : -1;
    if (shape < 0 || world->shapeType[shape] != (uint8_t)m3_voxelShape)
    {
        return NULL; // stale, foreign, or not a voxel chunk: contract
    }
    *shapeOut = shape;
    return world;
}

bool m3VoxelChunk_SetVoxel(m3ShapeId shapeId, int32_t x, int32_t y, int32_t z, uint16_t payload)
{
    int32_t shape;
    m3World* world = ResolveVoxelShape(shapeId, &shape);
    if (world == NULL || !VoxelCoordsValid(x, y, z))
    {
        return false;
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m3ShapeId id;
            int32_t x, y, z;
            uint16_t payload;
            uint16_t pad;
        } record;
        memset(&record, 0, sizeof(record));
        record.id = shapeId;
        record.x = x;
        record.y = y;
        record.z = z;
        record.payload = payload;
        m3JournalRecord(world, m3_opVoxelSet, &record, (int32_t)sizeof(record));
    }
    return m3VoxelSetInternal(world, shape, x, y, z, payload);
}

bool m3VoxelChunk_ClearVoxel(m3ShapeId shapeId, int32_t x, int32_t y, int32_t z)
{
    int32_t shape;
    m3World* world = ResolveVoxelShape(shapeId, &shape);
    if (world == NULL || !VoxelCoordsValid(x, y, z))
    {
        return false;
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m3ShapeId id;
            int32_t x, y, z;
        } record;
        memset(&record, 0, sizeof(record));
        record.id = shapeId;
        record.x = x;
        record.y = y;
        record.z = z;
        m3JournalRecord(world, m3_opVoxelClear, &record, (int32_t)sizeof(record));
    }
    return m3VoxelClearInternal(world, shape, x, y, z);
}

int32_t m3VoxelChunk_ClearBox(m3ShapeId shapeId, const int32_t lo[3], const int32_t hi[3])
{
    int32_t shape;
    m3World* world = ResolveVoxelShape(shapeId, &shape);
    if (world == NULL || lo == NULL || hi == NULL)
    {
        return -1;
    }
    for (int32_t k = 0; k < 3; ++k)
    {
        if (lo[k] < 0 || hi[k] >= M3_VOXEL_DIM || lo[k] > hi[k])
        {
            return -1; // bad region: contract, loud
        }
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m3ShapeId id;
            int32_t lo[3];
            int32_t hi[3];
        } record;
        memset(&record, 0, sizeof(record));
        record.id = shapeId;
        for (int32_t k = 0; k < 3; ++k)
        {
            record.lo[k] = lo[k];
            record.hi[k] = hi[k];
        }
        m3JournalRecord(world, m3_opVoxelClearBox, &record, (int32_t)sizeof(record));
    }
    return m3VoxelClearBoxInternal(world, shape, lo, hi);
}
