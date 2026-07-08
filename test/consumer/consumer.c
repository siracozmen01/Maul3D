// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The installed-package smoke test: a drop, a rest, and a rollback
// through nothing but the public headers of the installed tree.

#include "maul3d/shape.h"

#include <stdio.h>
#include <stdlib.h>

int main(void)
{
    m3WorldDef wd = m3DefaultWorldDef();
    m3WorldId world = m3CreateWorld(&wd);

    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    m3CreatePlaneShape(ground, &sd, &floor);

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 3.0, 0.0};
    m3BodyId ball = m3CreateBody(world, &bd);
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.5f};
    m3CreateSphereShape(ball, &sd, &sphere);

    for (int i = 0; i < 120; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3Pos3 rest = m3Body_GetPosition(ball);
    if (rest.y < 0.45 || rest.y > 0.55)
    {
        printf("M3_CONSUMER_FAIL rest=%.4f\n", rest.y);
        return 1;
    }

    int32_t bytes = m3World_SnapshotSize(world);
    void* snap = malloc((size_t)bytes);
    if (m3World_Snapshot(world, snap, bytes) != bytes)
    {
        printf("M3_CONSUMER_FAIL snapshot\n");
        return 1;
    }
    uint64_t before = m3World_Hash(world);
    for (int i = 0; i < 60; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    if (!m3World_Restore(world, snap, bytes) || m3World_Hash(world) != before)
    {
        printf("M3_CONSUMER_FAIL rollback\n");
        return 1;
    }
    free(snap);
    m3DestroyWorld(world);
    printf("M3_CONSUMER_OK version=%d\n", m3GetVersion());
    return 0;
}
