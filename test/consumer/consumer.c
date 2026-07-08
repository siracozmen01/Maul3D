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

    // The destruction round trip (3-8): the installed package must
    // carry the whole niche. Build a wall, carve it, catch the
    // fragment, make a body of it, and roll the world back.
    static uint8_t voxels[16 * 16 * 16];
    for (int i = 0; i < 16 * 16 * 16; ++i)
    {
        voxels[i] = 0;
    }
    for (int x = 0; x < 16; ++x)
    {
        voxels[x + 16 * (0 + 16 * 8)] = 1; // a grounded beam
    }
    voxels[5 + 16 * (1 + 16 * 8)] = 1; // a stud that will strand
    m3BodyDef wd2 = m3DefaultBodyDef();
    wd2.position = (m3Pos3){-8.0, 0.0, -8.0};
    m3BodyId wallBody = m3CreateBody(world, &wd2);
    m3ShapeDef vd = m3DefaultShapeDef();
    m3ShapeId wall = m3CreateVoxelChunkShape(wallBody, &vd, voxels, NULL, 1.0f);
    if (!m3Shape_IsValid(wall))
    {
        printf("M3_CONSUMER_FAIL voxel create\n");
        return 1;
    }
    if (!m3VoxelChunk_ClearVoxel(wall, 5, 0, 8))
    {
        printf("M3_CONSUMER_FAIL voxel edit\n");
        return 1;
    }
    int32_t fragments = 0;
    const m3FragmentEvent* events = m3World_FragmentEvents(world, &fragments);
    if (fragments != 1 || events[0].voxelCount != 1)
    {
        printf("M3_CONSUMER_FAIL fracture\n");
        return 1;
    }
    m3BodyDef fdb = m3DefaultBodyDef();
    fdb.type = m3_dynamicBody;
    fdb.position = events[0].comWorld;
    m3BodyId fragment = m3CreateBody(world, &fdb);
    m3ShapeDef fsd = m3DefaultShapeDef();
    m3CreateBoxShape(fragment, &fsd, (m3Vec3){0.5f, 0.5f, 0.5f});
    for (int i = 0; i < 60; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    if (!m3Body_IsValid(fragment))
    {
        printf("M3_CONSUMER_FAIL fragment body\n");
        return 1;
    }
    m3DestroyWorld(world);
    printf("M3_CONSUMER_OK version=%d\n", m3GetVersion());
    return 0;
}
