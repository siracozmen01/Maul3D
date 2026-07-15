// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The overlap family gate (15-3): capsule, oriented box, and raw
// cloud queries answer EXACTLY, in ascending id order, through the
// same filter law as the sphere family. A tilted box finds the
// crate a sphere of its inradius misses, a corridor capsule finds
// both ends, planes and voxel walls answer per family, and reading
// never moves a bit.

#include "maul3d/body.h"
#include "maul3d/shape.h"

#include <math.h>
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

static m3WorldId Yard(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 32;
    def.shapeCapacity = 32;
    def.voxelCapacity = 2;
    m3WorldId world = m3CreateWorld(&def);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    m3CreatePlaneShape(ground, &sd, &floor);
    return world;
}

static m3BodyId CrateAt(m3WorldId world, double x, double y, double z, uint64_t category)
{
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){x, y, z};
    m3BodyId body = m3CreateBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    if (category != 0)
    {
        sd.categoryBits = category;
    }
    m3CreateBoxShape(body, &sd, (m3Vec3){0.4f, 0.4f, 0.4f});
    return body;
}

static void TestBoxBeatsTheSphereAtCorners(void)
{
    m3WorldId world = Yard();
    CrateAt(world, 3.0, 0.5, 3.0, 0);
    m3ShapeId hits[8];
    // A tilted-free box reaching exactly into the corner: the crate
    // sits 4.24 m out diagonally, inside the box's long arm but
    // outside the 3 m sphere.
    int32_t viaBox = m3World_OverlapBox(world, (m3Pos3){0.0, 0.5, 0.0}, (m3Vec3){3.5f, 1.0f, 3.5f},
                                        (m3Quat){0.0f, 0.0f, 0.0f, 1.0f}, hits, 8);
    int32_t viaSphere = m3World_OverlapSphere(world, (m3Pos3){0.0, 0.5, 0.0}, 3.0f, hits, 8);
    // The floor plane answers both queries; the crate only the box.
    CHECK(viaBox == 2, "the box finds the plane and the corner crate");
    CHECK(viaSphere == 1, "the sphere of the inradius finds only the plane");
    m3DestroyWorld(world);
}

static void TestCorridorCapsuleAndOrder(void)
{
    m3WorldId world = Yard();
    m3BodyId far = CrateAt(world, 9.0, 0.5, 0.0, 0);
    m3BodyId near = CrateAt(world, -9.0, 0.5, 0.0, 0);
    CrateAt(world, 0.0, 0.5, 6.0, 0); // off the corridor
    (void)far;
    (void)near;
    m3ShapeId hits[8];
    int32_t n = m3World_OverlapCapsule(world, (m3Pos3){-10.0, 0.5, 0.0}, (m3Pos3){10.0, 0.5, 0.0},
                                       0.6f, hits, 8);
    // Plane + the two corridor crates; the side crate stays out.
    CHECK(n == 3, "the corridor capsule finds the plane and both ends");
    for (int32_t k = 1; k < n; ++k)
    {
        CHECK(hits[k].index1 > hits[k - 1].index1, "ids arrive ascending");
    }
    m3DestroyWorld(world);
}

static void TestCloudFilterAndPurity(void)
{
    m3WorldId world = Yard();
    CrateAt(world, 2.0, 0.5, 0.0, 2ull);
    CrateAt(world, -2.0, 0.5, 0.0, 0);
    uint64_t before = m3World_Hash(world);
    m3Vec3 tetra[4] = {
        {0.0f, 0.0f, 0.0f}, {4.0f, 0.0f, 0.0f}, {-4.0f, 0.0f, 0.0f}, {0.0f, 2.0f, 0.0f}};
    m3ShapeId hits[8];
    int32_t all =
        m3World_OverlapHullPoints(world, (m3Pos3){0.0, 0.5, 0.0}, tetra, 4, 0.1f, hits, 8);
    // The cloud floats half a meter over the floor with a 0.1
    // skin: the plane is honestly OUT of reach, both crates in.
    CHECK(all == 2, "the cloud reaches exactly the two crates");
    m3QueryFilter filter = m3DefaultQueryFilter();
    filter.maskBits = ~2ull;
    int32_t masked = m3World_OverlapHullPointsEx(world, (m3Pos3){0.0, 0.5, 0.0}, tetra, 4, 0.1f,
                                                 hits, 8, filter);
    CHECK(masked == 1, "the filter hides the category-2 crate");
    // Hostile clouds refuse quietly with zero hits.
    float bad;
    uint32_t nanBits = 0x7FC00000u;
    memcpy(&bad, &nanBits, sizeof(bad));
    m3Vec3 cursed[2] = {{0.0f, 0.0f, 0.0f}, {bad, 0.0f, 0.0f}};
    CHECK(m3World_OverlapHullPoints(world, (m3Pos3){0.0, 0.5, 0.0}, cursed, 2, 0.1f, hits, 8) == 0,
          "a NaN cloud reads zero");
    CHECK(m3World_OverlapHullPoints(world, (m3Pos3){0.0, 0.5, 0.0}, tetra, 0, 0.1f, hits, 8) == 0,
          "an empty cloud reads zero");
    CHECK(m3World_Hash(world) == before, "overlap queries move no bits");
    m3DestroyWorld(world);
}

static void TestVoxelWallExactness(void)
{
    m3WorldId world = Yard();
    static uint8_t voxels[16 * 16 * 16];
    memset(voxels, 0, sizeof(voxels));
    for (int32_t y = 0; y < 8; ++y)
    {
        for (int32_t x = 0; x < 16; ++x)
        {
            voxels[x + 16 * (y + 16 * 8)] = 1;
        }
    }
    m3BodyDef gd = m3DefaultBodyDef();
    gd.position = (m3Pos3){-8.0, 0.0, -8.0};
    m3BodyId keep = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3CreateVoxelChunkShape(keep, &sd, voxels, NULL, 1.0f);
    m3ShapeId hits[8];
    // A capsule hugging the wall face touches it; the same capsule
    // pulled a meter back does not (exactness, not AABB slop).
    int32_t touching = m3World_OverlapCapsule(world, (m3Pos3){-4.0, 2.0, 0.7},
                                              (m3Pos3){4.0, 2.0, 0.7}, 0.4f, hits, 8);
    int32_t clear = m3World_OverlapCapsule(world, (m3Pos3){-4.0, 2.0, 2.0}, (m3Pos3){4.0, 2.0, 2.0},
                                           0.4f, hits, 8);
    CHECK(touching == 1, "the hugging capsule touches the wall");
    CHECK(clear == 0, "a meter of air is a miss, not an AABB hit");
    m3DestroyWorld(world);
}

int main(void)
{
    TestBoxBeatsTheSphereAtCorners();
    TestCorridorCapsuleAndOrder();
    TestCloudFilterAndPurity();
    TestVoxelWallExactness();
    if (s_failures == 0)
    {
        printf("test_overlapfamily: all passed\n");
        return 0;
    }
    return 1;
}
