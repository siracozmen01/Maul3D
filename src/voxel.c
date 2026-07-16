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
            chunk->fill[v] = 255; // whole voxels at birth; SetFill wears them down
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
    // The embedded BVH owns heap arrays now (10-3): release them
    // before the wipe or the rebuild leaks the previous build.
    // CONTRACT: the surface must be zeroed or a previous build;
    // garbage pointers here are the caller's crash.
    m3MeshBvhFree(&surface->bvh);
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

    // Characters standing over the edited region lose their ground
    // THIS step if it vanished (4-5): the destruction interplay is
    // a contract, not a next-frame coincidence.
    for (int32_t c = 0; c < world->charPool.maxIndex; ++c)
    {
        if (world->charPool.alive[c] == 0 || world->charGrounded[c] == 0)
        {
            continue;
        }
        const m3Pos3* p = &world->transforms[world->charBody[c]].p;
        m3real reachDown =
            world->charHalfHeight[c] + world->charRadius[c] + world->charSnap[c] + M3_AABB_MARGIN;
        m3real reachSide = world->charRadius[c] + M3_AABB_MARGIN;
        if (p->x + (double)reachSide < wlo[0] || p->x - (double)reachSide > whi[0] ||
            p->z + (double)reachSide < wlo[2] || p->z - (double)reachSide > whi[2] ||
            p->y - (double)reachDown > whi[1] || p->y + (double)reachDown < wlo[1])
        {
            continue;
        }
        m3CharacterRefreshGrounding(world, c);
    }

    // A parked car hovers on wheel RAYS: its chassis bounds may sit
    // well above the carved region and the tree wake would leave
    // the sleeper floating on vanished floor. Any wheel ray
    // overlapping the region wakes the chassis (5-3); the next
    // suspension pass reads the new surface the same step.
    for (int32_t v = 0; v < world->vehPool.maxIndex; ++v)
    {
        if (world->vehPool.alive[v] == 0)
        {
            continue;
        }
        int32_t chassis = world->vehChassis[v];
        if (chassis < 0 || world->bodyPool.alive[chassis] == 0 ||
            world->bodyPool.generations[chassis] != world->vehChassisGen[v] ||
            world->awake[chassis] != 0)
        {
            continue;
        }
        const m3Transform* cxf = &world->transforms[chassis];
        for (int32_t w = 0; w < world->vehWheelCount[v]; ++w)
        {
            int32_t k = v * M3_VEHICLE_MAX_WHEELS + w;
            m3Vec3 anchorR = m3RotateVec3(cxf->q, world->vehWheelAnchor[k]);
            double ax = cxf->p.x + (double)anchorR.x;
            double ay = cxf->p.y + (double)anchorR.y;
            double az = cxf->p.z + (double)anchorR.z;
            m3Vec3 dir = m3RotateVec3(cxf->q, world->vehWheelDir[k]);
            m3real reach = world->vehWheelRest[k] + world->vehWheelRadius[k];
            double ex = ax + (double)(dir.x * reach);
            double ey = ay + (double)(dir.y * reach);
            double ez = az + (double)(dir.z * reach);
            double m = (double)M3_AABB_MARGIN;
            double slo[3] = {(ax < ex ? ax : ex) - m, (ay < ey ? ay : ey) - m,
                             (az < ez ? az : ez) - m};
            double shi[3] = {(ax > ex ? ax : ex) + m, (ay > ey ? ay : ey) + m,
                             (az > ez ? az : ez) + m};
            if (slo[0] <= whi[0] && shi[0] >= wlo[0] && slo[1] <= whi[1] && shi[1] >= wlo[1] &&
                slo[2] <= whi[2] && shi[2] >= wlo[2])
            {
                world->awake[chassis] = 1;
                world->sleepTimes[chassis] = 0.0f;
                break;
            }
        }
    }
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
    chunk->fill[v] = 255; // a set voxel is a whole voxel
    if (!wasFilled)
    {
        chunk->filledCount += 1;
        // Occupancy changed: the surface is stale. A payload-only
        // set falls through (the surface is a pure function of
        // occupancy and cell size, never of payload). No fracture
        // sweep here: adding a voxel can only CONNECT islands.
        m3VoxelSurfaceBuild(&world->voxelSurface[slot], chunk);
        m3VoxelCoverageRefreshAround(world, slot);
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
        chunk->fill[v] = 0;
        chunk->filledCount -= 1;
        m3VoxelSurfaceBuild(&world->voxelSurface[slot], chunk);
        m3VoxelFractureSweep(world, shape);
        m3VoxelCoverageRefreshAround(world, slot);
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
                    chunk->fill[v] = 0;
                    cleared += 1;
                }
            }
        }
    }
    if (cleared > 0)
    {
        chunk->filledCount -= cleared;
        m3VoxelSurfaceBuild(&world->voxelSurface[slot], chunk);
        m3VoxelFractureSweep(world, shape);
        m3VoxelCoverageRefreshAround(world, slot);
    }
    VoxelWakeRegion(world, shape, lo, hi);
    return cleared;
}

bool m3VoxelSetFillInternal(m3World* world, int32_t shape, int32_t x, int32_t y, int32_t z,
                            uint8_t fill)
{
    int32_t slot = world->shapeVoxelIndex[shape];
    m3VoxelChunkData* chunk = &world->voxelData[slot];
    if (!m3VoxelGet(chunk, x, y, z))
    {
        return false; // fill describes an occupied voxel, nothing else
    }
    int32_t v = x + M3_VOXEL_DIM * (y + M3_VOXEL_DIM * z);
    chunk->fill[v] = fill;
    // Mass bookkeeping only: no surface rebuild (geometry is
    // untouched), no wake (a static chunk's mass moves nothing
    // dynamic until fracture delivers it).
    return true;
}

int32_t m3VoxelCarveSphereInternal(m3World* world, int32_t shape, m3Vec3 center, m3real radius)
{
    // The explosion bite (13-3): clear every cell whose center lies
    // inside the sphere (center in the CHUNK frame), then ONE
    // surface rebuild and ONE fracture sweep for the whole bite,
    // the ClearBox economy. Cell (x,y,z) spans [x, x+1) * cellSize.
    int32_t slot = world->shapeVoxelIndex[shape];
    m3VoxelChunkData* chunk = &world->voxelData[slot];
    m3real cell = chunk->cellSize;
    int32_t lo[3];
    int32_t hi[3];
    m3real c[3] = {center.x, center.y, center.z};
    for (int32_t a = 0; a < 3; ++a)
    {
        m3real f0 = (c[a] - radius) / cell;
        m3real f1 = (c[a] + radius) / cell;
        int32_t i0 = m3CellFromF(floorf(f0), 2.0e9f);
        int32_t i1 = m3CellFromF(floorf(f1), -2.0e9f);
        lo[a] = i0 < 0 ? 0 : (i0 > M3_VOXEL_DIM - 1 ? M3_VOXEL_DIM - 1 : i0);
        hi[a] = i1 < 0 ? 0 : (i1 > M3_VOXEL_DIM - 1 ? M3_VOXEL_DIM - 1 : i1);
    }
    int32_t cleared = 0;
    m3real r2 = radius * radius;
    for (int32_t z = lo[2]; z <= hi[2]; ++z)
    {
        for (int32_t y = lo[1]; y <= hi[1]; ++y)
        {
            for (int32_t x = lo[0]; x <= hi[0]; ++x)
            {
                if (!m3VoxelGet(chunk, x, y, z))
                {
                    continue;
                }
                m3real dx = ((m3real)x + 0.5f) * cell - center.x;
                m3real dy = ((m3real)y + 0.5f) * cell - center.y;
                m3real dz = ((m3real)z + 0.5f) * cell - center.z;
                if (dx * dx + dy * dy + dz * dz > r2)
                {
                    continue;
                }
                int32_t v = x + M3_VOXEL_DIM * (y + M3_VOXEL_DIM * z);
                chunk->occupancy[v >> 3] &= (uint8_t)~(1u << (v & 7));
                chunk->payload[v] = 0;
                chunk->fill[v] = 0;
                cleared += 1;
            }
        }
    }
    if (cleared > 0)
    {
        chunk->filledCount -= cleared;
        m3VoxelSurfaceBuild(&world->voxelSurface[slot], chunk);
        m3VoxelFractureSweep(world, shape);
        m3VoxelCoverageRefreshAround(world, slot);
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

bool m3VoxelChunk_SetFill(m3ShapeId shapeId, int32_t x, int32_t y, int32_t z, uint8_t fill)
{
    int32_t shape;
    m3World* world = ResolveVoxelShape(shapeId, &shape);
    if (world == NULL || !VoxelCoordsValid(x, y, z) || fill == 0)
    {
        return false; // fill zero is a clear in disguise: refused
    }
    int32_t slot = world->shapeVoxelIndex[shape];
    if (!m3VoxelGet(&world->voxelData[slot], x, y, z))
    {
        return false;
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m3ShapeId id;
            int32_t x, y, z;
            uint8_t fill;
            uint8_t pad[3];
        } record;
        memset(&record, 0, sizeof(record));
        record.id = shapeId;
        record.x = x;
        record.y = y;
        record.z = z;
        record.fill = fill;
        m3JournalRecord(world, m3_opVoxelSetFill, &record, (int32_t)sizeof(record));
    }
    return m3VoxelSetFillInternal(world, shape, x, y, z, fill);
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

// ---------------------------------------------------------------
// Fracture (3-3): connectivity and fragment events.

// Deterministic flood fill: seeds scan in canonical linear order,
// the frontier grows with a fixed six-neighbor order, so island
// labels and event order are pure functions of the grid.
void m3VoxelFractureSweep(m3World* world, int32_t shape)
{
    int32_t slot = world->shapeVoxelIndex[shape];
    m3VoxelChunkData* chunk = &world->voxelData[slot];
    if (chunk->filledCount == 0)
    {
        return;
    }
    static const int32_t offsets[6][3] = {{-1, 0, 0}, {1, 0, 0},  {0, -1, 0},
                                          {0, 1, 0},  {0, 0, -1}, {0, 0, 1}};
    uint8_t visited[M3_VOXEL_COUNT];
    memset(visited, 0, sizeof(visited));
    uint16_t stack[M3_VOXEL_COUNT];
    uint16_t island[M3_VOXEL_COUNT];

    bool removedAny = false;
    for (int32_t seed = 0; seed < M3_VOXEL_COUNT; ++seed)
    {
        int32_t sx = seed % M3_VOXEL_DIM;
        int32_t sy = (seed / M3_VOXEL_DIM) % M3_VOXEL_DIM;
        int32_t sz = seed / (M3_VOXEL_DIM * M3_VOXEL_DIM);
        if (visited[seed] != 0 || !m3VoxelGet(chunk, sx, sy, sz))
        {
            continue;
        }
        // Collect the island.
        int32_t top = 0;
        int32_t count = 0;
        bool anchored = false;
        stack[top++] = (uint16_t)seed;
        visited[seed] = 1;
        while (top > 0)
        {
            uint16_t v = stack[--top];
            island[count++] = v;
            int32_t x = v % M3_VOXEL_DIM;
            int32_t y = (v / M3_VOXEL_DIM) % M3_VOXEL_DIM;
            int32_t z = v / (M3_VOXEL_DIM * M3_VOXEL_DIM);
            if (y == 0)
            {
                anchored = true; // the base layer is the anchor
            }
            for (int32_t n = 0; n < 6; ++n)
            {
                int32_t nx = x + offsets[n][0];
                int32_t ny = y + offsets[n][1];
                int32_t nz = z + offsets[n][2];
                if (nx < 0 || nx >= M3_VOXEL_DIM || ny < 0 || ny >= M3_VOXEL_DIM || nz < 0 ||
                    nz >= M3_VOXEL_DIM)
                {
                    continue;
                }
                int32_t nv = nx + M3_VOXEL_DIM * (ny + M3_VOXEL_DIM * nz);
                if (visited[nv] == 0 && m3VoxelGet(chunk, nx, ny, nz))
                {
                    visited[nv] = 1;
                    stack[top++] = (uint16_t)nv;
                }
            }
        }
        if (anchored)
        {
            continue; // grounded islands stand
        }

        // An unanchored island: remove it from the grid (part of the
        // edit's state transition) and emit the event.
        m3real cell = chunk->cellSize;
        m3Vec3 com = {0.0f, 0.0f, 0.0f};
        m3Vec3 comEq = {0.0f, 0.0f, 0.0f}; // unweighted twin (see below)
        int64_t fillSum = 0;
        uint8_t lo[3] = {255, 255, 255};
        uint8_t hi[3] = {0, 0, 0};
        for (int32_t k = 0; k < count; ++k)
        {
            uint16_t v = island[k];
            int32_t x = v % M3_VOXEL_DIM;
            int32_t y = (v / M3_VOXEL_DIM) % M3_VOXEL_DIM;
            int32_t z = v / (M3_VOXEL_DIM * M3_VOXEL_DIM);
            m3real w = (m3real)chunk->fill[v]; // fill-weighted (3-6)
            fillSum += (int64_t)chunk->fill[v];
            chunk->occupancy[v >> 3] &= (uint8_t)~(1u << (v & 7));
            chunk->payload[v] = 0;
            chunk->fill[v] = 0;
            m3Vec3 center = {((m3real)x + 0.5f) * cell, ((m3real)y + 0.5f) * cell,
                             ((m3real)z + 0.5f) * cell};
            com = m3Add3(com, m3MulSV3(w, center));
            comEq = m3Add3(comEq, center);
            lo[0] = x < lo[0] ? (uint8_t)x : lo[0];
            lo[1] = y < lo[1] ? (uint8_t)y : lo[1];
            lo[2] = z < lo[2] ? (uint8_t)z : lo[2];
            hi[0] = x > hi[0] ? (uint8_t)x : hi[0];
            hi[1] = y > hi[1] ? (uint8_t)y : hi[1];
            hi[2] = z > hi[2] ? (uint8_t)z : hi[2];
        }
        chunk->filledCount -= count;
        removedAny = true;
        if (fillSum > 0)
        {
            com = m3MulSV3(1.0f / (m3real)fillSum, com);
        }
        else
        {
            // A legitimate stream never gets here (occupied implies
            // fill >= 1), but a mutated snapshot can set occupancy
            // bits over zeroed fill bytes, and 1/0 would mint a NaN
            // center that infects the whole simulation. The equal
            // weight center is the honest fallback (count >= 1 in
            // this branch by construction).
            com = m3MulSV3(1.0f / (m3real)count, comEq);
        }

        if (world->fragmentEventCount >= M3_FRAGMENT_EVENT_CAP)
        {
            world->fragmentDropped += 1; // loud: the state moved, the
                                         // event did not fit
            continue;
        }
        m3FragmentEvent* ev = &world->fragmentEvents[world->fragmentEventCount];
        memset(ev, 0, sizeof(*ev));
        ev->chunkShape =
            (m3ShapeId){shape + 1, world->worldIndex0, world->shapePool.generations[shape]};
        ev->voxelCount = count;
        if (world->fragmentRecipeCount + count <= M3_FRAGMENT_RECIPE_CAP)
        {
            ev->recipeStart = world->fragmentRecipeCount;
            ev->recipeCount = count;
            for (int32_t k = 0; k < count; ++k)
            {
                world->fragmentRecipe[world->fragmentRecipeCount + k] = island[k];
            }
            world->fragmentRecipeCount += count;
        }
        else
        {
            ev->recipeStart = -1; // recipe overflow: bounds and count
                                  // still describe the island, loudly
            ev->recipeCount = 0;
        }
        ev->comChunk = com;
        int32_t body = world->shapeBody[shape];
        const m3Transform* xf = &world->transforms[body];
        m3Vec3 r = m3RotateVec3(xf->q, com);
        ev->comWorld =
            (m3Pos3){xf->p.x + (double)r.x, xf->p.y + (double)r.y, xf->p.z + (double)r.z};
        ev->mass = ((m3real)fillSum / 255.0f) * cell * cell * cell;
        for (int32_t k = 0; k < 3; ++k)
        {
            ev->boundsLo[k] = lo[k];
            ev->boundsHi[k] = hi[k];
        }
        world->fragmentEventCount += 1;
    }

    if (removedAny)
    {
        // The surface follows the grid, always (the derived law).
        m3VoxelSurfaceBuild(&world->voxelSurface[slot], chunk);
        m3VoxelCoverageRefreshAround(world, slot);
    }
}

// Interior depenetration (3-6): a body center INSIDE the solid
// (spawn mistakes, monster impulses) escapes through the nearest
// exposed face found by a deterministic BFS over the grid (fixed
// seed, fixed neighbor order). Returns false when the point is not
// inside solid. The out parameters name the exit: a chunk-frame
// axis normal and the exit plane coordinate along it.
bool m3VoxelEscape(const m3World* world, int32_t slot, m3Vec3 localPoint, m3Vec3* outNormal,
                   m3real* outPlane)
{
    const m3VoxelChunkData* chunk = &world->voxelData[slot];
    m3real cell = chunk->cellSize;
    int32_t sx = (int32_t)(localPoint.x / cell);
    int32_t sy = (int32_t)(localPoint.y / cell);
    int32_t sz = (int32_t)(localPoint.z / cell);
    if (localPoint.x < 0.0f || localPoint.y < 0.0f || localPoint.z < 0.0f || sx >= M3_VOXEL_DIM ||
        sy >= M3_VOXEL_DIM || sz >= M3_VOXEL_DIM || !m3VoxelGet(chunk, sx, sy, sz))
    {
        return false;
    }
    static const int32_t offsets[6][3] = {{-1, 0, 0}, {1, 0, 0},  {0, -1, 0},
                                          {0, 1, 0},  {0, 0, -1}, {0, 0, 1}};
    uint8_t visited[M3_VOXEL_COUNT];
    memset(visited, 0, sizeof(visited));
    uint16_t queue[M3_VOXEL_COUNT];
    int32_t head = 0;
    int32_t tail = 0;
    int32_t start = sx + M3_VOXEL_DIM * (sy + M3_VOXEL_DIM * sz);
    queue[tail++] = (uint16_t)start;
    visited[start] = 1;
    while (head < tail)
    {
        uint16_t v = queue[head++];
        int32_t x = v % M3_VOXEL_DIM;
        int32_t y = (v / M3_VOXEL_DIM) % M3_VOXEL_DIM;
        int32_t z = v / (M3_VOXEL_DIM * M3_VOXEL_DIM);
        for (int32_t n = 0; n < 6; ++n)
        {
            int32_t nx = x + offsets[n][0];
            int32_t ny = y + offsets[n][1];
            int32_t nz = z + offsets[n][2];
            bool exit = false;
            if (nx < 0 || nx >= M3_VOXEL_DIM || ny < 0 || ny >= M3_VOXEL_DIM || nz < 0 ||
                nz >= M3_VOXEL_DIM)
            {
                // The chunk boundary: an exit unless a welded
                // neighbor continues the solid there.
                int32_t link = world->voxelNeighbors[slot * 6 + n];
                if (link < 0)
                {
                    exit = true;
                }
                else
                {
                    int32_t mx = (nx + M3_VOXEL_DIM) % M3_VOXEL_DIM;
                    int32_t my = (ny + M3_VOXEL_DIM) % M3_VOXEL_DIM;
                    int32_t mz = (nz + M3_VOXEL_DIM) % M3_VOXEL_DIM;
                    exit = !m3VoxelGet(&world->voxelData[link], mx, my, mz);
                }
            }
            else if (!m3VoxelGet(chunk, nx, ny, nz))
            {
                exit = true;
            }
            if (exit)
            {
                int32_t axis = n / 2;
                m3real sign = (n & 1) != 0 ? 1.0f : -1.0f;
                *outNormal = (m3Vec3){0.0f, 0.0f, 0.0f};
                int32_t faceCell = (n & 1) != 0 ? (axis == 0 ? x + 1 : (axis == 1 ? y + 1 : z + 1))
                                                : (axis == 0 ? x : (axis == 1 ? y : z));
                if (axis == 0)
                {
                    outNormal->x = sign;
                }
                else if (axis == 1)
                {
                    outNormal->y = sign;
                }
                else
                {
                    outNormal->z = sign;
                }
                *outPlane = (m3real)faceCell * cell;
                return true;
            }
            int32_t nv = nx + M3_VOXEL_DIM * (ny + M3_VOXEL_DIM * nz);
            if (visited[nv] == 0)
            {
                visited[nv] = 1;
                queue[tail++] = (uint16_t)nv;
            }
        }
    }
    return false; // a chunk with no exposed face anywhere: welded
                  // solid on all sides; the neighbor owns the escape
}

// ---------------------------------------------------------------
// Seam welding (3-4).

void m3VoxelRebuildLinks(m3World* world)
{
    int32_t cap = world->voxelCapacity;
    for (int32_t i = 0; i < cap * 6; ++i)
    {
        world->voxelNeighbors[i] = -1;
    }
    static const m3Quat identity = {0.0f, 0.0f, 0.0f, 1.0f};
    int32_t maxSlot = world->voxelPool.maxIndex;
    for (int32_t a = 0; a < maxSlot; ++a)
    {
        if (world->voxelPool.alive[a] == 0)
        {
            continue;
        }
        int32_t bodyA = world->shapeBody[world->voxelShape[a]];
        const m3Transform* xfA = &world->transforms[bodyA];
        if (memcmp(&xfA->q, &identity, sizeof(m3Quat)) != 0)
        {
            continue; // the welding contract wants grid alignment
        }
        double extentA = (double)((m3real)M3_VOXEL_DIM * world->voxelData[a].cellSize);
        for (int32_t b = 0; b < maxSlot; ++b)
        {
            if (b == a || world->voxelPool.alive[b] == 0 ||
                world->voxelData[b].cellSize != world->voxelData[a].cellSize)
            {
                continue;
            }
            int32_t bodyB = world->shapeBody[world->voxelShape[b]];
            const m3Transform* xfB = &world->transforms[bodyB];
            if (memcmp(&xfB->q, &identity, sizeof(m3Quat)) != 0)
            {
                continue;
            }
            double dx = xfB->p.x - xfA->p.x;
            double dy = xfB->p.y - xfA->p.y;
            double dz = xfB->p.z - xfA->p.z;
            // Exact adjacency on exactly one axis (the contract).
            if (dy == 0.0 && dz == 0.0 && dx == extentA)
            {
                world->voxelNeighbors[a * 6 + 1] = b; // +x
            }
            else if (dy == 0.0 && dz == 0.0 && dx == -extentA)
            {
                world->voxelNeighbors[a * 6 + 0] = b; // -x
            }
            else if (dx == 0.0 && dz == 0.0 && dy == extentA)
            {
                world->voxelNeighbors[a * 6 + 3] = b; // +y
            }
            else if (dx == 0.0 && dz == 0.0 && dy == -extentA)
            {
                world->voxelNeighbors[a * 6 + 2] = b; // -y
            }
            else if (dx == 0.0 && dy == 0.0 && dz == extentA)
            {
                world->voxelNeighbors[a * 6 + 5] = b; // +z
            }
            else if (dx == 0.0 && dy == 0.0 && dz == -extentA)
            {
                world->voxelNeighbors[a * 6 + 4] = b; // -z
            }
        }
    }
}

// Is the axis-aligned layer just OUTSIDE face `face` of the box
// fully filled? Looks into the slot's own grid, or across the weld
// into the neighbor's border layer when the box touches the chunk
// boundary.
static bool VoxelFaceCovered(const m3World* world, int32_t slot, const uint8_t lo[3],
                             const uint8_t hi[3], int32_t face)
{
    static const int32_t axisOf[6] = {0, 0, 1, 1, 2, 2};
    static const int32_t signOf[6] = {-1, 1, -1, 1, -1, 1};
    int32_t axis = axisOf[face];
    int32_t sign = signOf[face];
    int32_t layer = sign < 0 ? (int32_t)lo[axis] - 1 : (int32_t)hi[axis] + 1;
    const m3VoxelChunkData* grid = &world->voxelData[slot];
    if (layer < 0 || layer >= M3_VOXEL_DIM)
    {
        int32_t neighbor = world->voxelNeighbors[slot * 6 + face];
        if (neighbor < 0)
        {
            return false; // no weld: the face is exposed to the world
        }
        grid = &world->voxelData[neighbor];
        layer = sign < 0 ? M3_VOXEL_DIM - 1 : 0; // the mirrored border
    }
    int32_t u = (axis + 1) % 3;
    int32_t v = (axis + 2) % 3;
    for (int32_t i = lo[u]; i <= hi[u]; ++i)
    {
        for (int32_t j = lo[v]; j <= hi[v]; ++j)
        {
            int32_t c[3];
            c[axis] = layer;
            c[u] = i;
            c[v] = j;
            if (!m3VoxelGet(grid, c[0], c[1], c[2]))
            {
                return false;
            }
        }
    }
    return true;
}

void m3VoxelCoverageBuild(m3World* world, int32_t slot)
{
    m3VoxelSurface* surface = &world->voxelSurface[slot];
    for (int32_t b = 0; b < surface->boxCount; ++b)
    {
        uint8_t covered = 0;
        for (int32_t face = 0; face < 6; ++face)
        {
            if (VoxelFaceCovered(world, slot, surface->boxLo[b], surface->boxHi[b], face))
            {
                covered |= (uint8_t)(1u << face);
            }
        }
        surface->boxCovered[b] = covered;
    }
}

void m3VoxelCoverageRefreshAround(m3World* world, int32_t slot)
{
    m3VoxelCoverageBuild(world, slot);
    for (int32_t face = 0; face < 6; ++face)
    {
        int32_t neighbor = world->voxelNeighbors[slot * 6 + face];
        if (neighbor >= 0)
        {
            // The neighbor's border coverage reads THIS grid.
            m3VoxelCoverageBuild(world, neighbor);
        }
    }
}

// A hull from raw chunk-frame bounds (the welded collision path
// extends covered faces before building, so the SAT never sees an
// interior face as a candidate).
void m3VoxelBoundsHull(m3Vec3 lo, m3Vec3 hi, m3HullData* out)
{
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
