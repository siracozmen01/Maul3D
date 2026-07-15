// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Debug draw (2c-9): a pure observer. The world describes itself as
// segments and points through host callbacks; a draw pass reads
// simulation state and never writes a bit (a test hashes the world
// across a draw to hold that promise).

#ifndef MAUL3D_DRAW_H
#define MAUL3D_DRAW_H

#include "world.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /// Colors are 0xRRGGBB. The palette is fixed so twin worlds draw
    /// identical streams: awake dynamic 0x4FA3FF, sleeping 0x777788,
    /// static 0x66BB66, kinematic 0xC9A227, sensor 0xAA66CC, contact
    /// 0xFF5544, joint 0xFFFFFF, AABB 0x333344.
    typedef struct m3DebugDraw
    {
        void (*DrawSegment)(m3Pos3 p1, m3Pos3 p2, uint32_t color, void* context);
        void (*DrawPoint)(m3Pos3 p, m3real size, uint32_t color, void* context);
        void* context;
        bool drawShapes;
        bool drawContacts; // manifold points plus normals scaled by impulse
        bool drawJoints;
        bool drawAabbs; // the broadphase FAT bounds (what the tree sees)
        bool drawSleepTint;
    } m3DebugDraw;

    /// Emit the world through the callbacks. Null callbacks are
    /// skipped; the call itself never mutates the world.
    M3_API void m3World_Draw(m3WorldId worldId, const m3DebugDraw* draw);

    /// Solid draw: the world describes itself as FILLED triangles in
    /// world space, counter-clockwise seen from outside, so a viewer
    /// can light and shadow real surfaces. A SEPARATE struct and
    /// entry point on purpose: the m3DebugDraw layout above is
    /// frozen ABI under 1.x and growing it would hand stale-compiled
    /// hosts uninitialized function pointers.
    typedef struct m3SolidDraw
    {
        void (*DrawTriangle)(m3Pos3 a, m3Pos3 b, m3Pos3 c, uint32_t color, void* context);
        void* context;
        bool drawSleepTint; // sleeping dynamics take the sleep gray
    } m3SolidDraw;

    /// Emit every live shape as triangles: spheres and capsules
    /// tessellate at fixed counts, hulls fan their face loops,
    /// meshes emit their stored triangles, voxel surfaces emit their
    /// merged-box faces. Infinite planes are SKIPPED (a viewer draws
    /// its own ground). Read-only like m3World_Draw and held by the
    /// same purity test: a draw pass never moves the world hash.
    M3_API void m3World_DrawSolid(m3WorldId worldId, const m3SolidDraw* draw);

    /// Extra layers (14-2): the analysis views on top of the base
    /// walk. A SEPARATE additive struct again; m3DebugDraw and
    /// m3SolidDraw stay frozen ABI. Read-only like both, held by
    /// the same purity gate.
    typedef struct m3ExtraDraw
    {
        void (*DrawSegment)(m3Pos3 p1, m3Pos3 p2, uint32_t color, void* context);
        void (*DrawPoint)(m3Pos3 p, m3real size, uint32_t color, void* context);
        void* context;
        /// Awake dynamic bodies marked in a color cycled by their
        /// island; sleeping bodies keep the tint of the island they
        /// fell asleep in. Labels refresh every completed step.
        bool drawIslands;
        /// Center-of-mass frames on dynamic bodies: three half-meter
        /// axes in the body rotation (x red, y green, z blue).
        bool drawMassAxes;
        /// The broadphase tree's INTERNAL node boxes (the leaves are
        /// the fat AABBs the base walk already offers), brightness
        /// stepped by node height.
        bool drawTreeBoxes;
    } m3ExtraDraw;

    M3_API void m3World_DrawExtras(m3WorldId worldId, const m3ExtraDraw* draw);

#ifdef __cplusplus
}
#endif

#endif
