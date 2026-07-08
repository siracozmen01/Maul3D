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

#ifdef __cplusplus
}
#endif

#endif
