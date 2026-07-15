// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The mover toolkit gate (22-1): pure capsule casts, contact plane
// collection, and the plane accumulator. Nothing here mutates a
// world: the hash fence proves it.

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

static void TestRigidWalkerRecipe(void)
{
    // 22-2: the rigid character RECIPE on the public API alone: a
    // dynamic capsule with angular locks, steered by forces toward
    // a target speed, grounded by a ray. It crosses the yard
    // upright and stops when told; the manual documents exactly
    // this loop.
    m3WorldDef wd = m3DefaultWorldDef();
    wd.bodyCapacity = 8;
    wd.shapeCapacity = 8;
    m3WorldId world = m3CreateWorld(&wd);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    m3CreatePlaneShape(ground, &sd, &floor);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 1.0, 0.0};
    m3BodyId hero = m3CreateBody(world, &bd);
    m3ShapeDef hs = m3DefaultShapeDef();
    hs.density = 985.0f;
    // Frictionless BY RECIPE: an externally-forced capsule loses a
    // tug of war against static friction (0.42 mix here holds 2290
    // N against a 2000 N drive and the walker stands still); the
    // drive does the braking too, the classic controller contract.
    hs.friction = 0.0f;
    m3Capsule cap = {{0.0f, -0.5f, 0.0f}, {0.0f, 0.5f, 0.0f}, 0.35f};
    m3CreateCapsuleShape(hero, &hs, &cap);
    m3Body_SetMotionLocks(hero, 0x38); // angular x, y, z: upright
    float mass = 556.0f;               // the capsule at density 985

    for (int32_t i = 0; i < 300; ++i)
    {
        m3Vec3 v = m3Body_GetLinearVelocity(hero);
        float want = i < 180 ? 3.0f : 0.0f; // walk, then halt
        // Grounding via the mover toolkit itself: collect the
        // contact planes around the body's own capsule and look
        // for an upward-facing one that is NOT our own shape (a
        // ray would first find our own hull; the plane list makes
        // the self-filter one comparison).
        m3MoverPlane gp[8];
        int32_t gn =
            m3World_CollideMover(world, m3Body_GetPosition(hero), 0.5f, 0.36f, 0.05f, gp, 8);
        int onGround = 0;
        for (int32_t g = 0; g < gn; ++g)
        {
            if (gp[g].normal.y > 0.7f && m3Shape_GetBody(gp[g].shape).index1 != hero.index1)
            {
                onGround = 1;
            }
        }
        if (onGround)
        {
            // The capsule weighs ~556 kg at this density: budget
            // the drive like a real actuator, not a wish.
            float fx = mass * (want - v.x) * 10.0f;
            fx = fx > 2000.0f ? 2000.0f : (fx < -2000.0f ? -2000.0f : fx);
            m3Body_ApplyForce(hero, (m3Vec3){fx, 0.0f, 0.0f});
        }
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3Pos3 p = m3Body_GetPosition(hero);
    m3Vec3 v = m3Body_GetLinearVelocity(hero);
    CHECK(p.x > 4.0, "the walker crosses the yard");
    CHECK(fabsf(v.x) < 0.3f, "the walker halts when told");
    CHECK(p.y > 0.7 && p.y < 1.1, "the walker stays upright on its locks");
    m3Quat q = m3Body_GetRotation(hero);
    CHECK(fabsf(q.x) + fabsf(q.y) + fabsf(q.z) < 0.01f, "no tumble ever leaked in");
    m3DestroyWorld(world);
}

int main(void)
{
    m3WorldDef wd = m3DefaultWorldDef();
    wd.bodyCapacity = 8;
    wd.shapeCapacity = 8;
    m3WorldId world = m3CreateWorld(&wd);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    m3CreatePlaneShape(ground, &sd, &floor);
    gd.position = (m3Pos3){2.0, 1.0, 0.0};
    m3BodyId wallBody = m3CreateBody(world, &gd);
    m3CreateBoxShape(wallBody, &sd, (m3Vec3){0.2f, 2.0f, 2.0f});
    gd.position = (m3Pos3){-2.0, 1.0, 0.0};
    m3BodyId sensorBody = m3CreateBody(world, &gd);
    m3ShapeDef sens = m3DefaultShapeDef();
    sens.isSensor = true;
    m3CreateBoxShape(sensorBody, &sens, (m3Vec3){0.5f, 2.0f, 2.0f});

    uint64_t before = m3World_Hash(world);

    // Cast: a mover dropped from above strikes the floor.
    m3RayHit hit =
        m3World_CastMover(world, (m3Pos3){0.0, 3.0, 0.0}, 0.5f, 0.4f, (m3Vec3){0.0f, -5.0f, 0.0f});
    CHECK(hit.hit, "the falling mover strikes the floor");
    CHECK(hit.normal.y > 0.9f, "the strike normal points up");

    // Collide: standing on the floor beside the wall yields both
    // planes; the sensor contributes nothing.
    m3MoverPlane planes[8];
    int32_t n = m3World_CollideMover(world, (m3Pos3){1.35, 0.9, 0.0}, 0.5f, 0.4f, 0.05f, planes, 8);
    CHECK(n == 2, "floor and wall each contribute one plane");
    int sawFloor = 0;
    int sawWall = 0;
    for (int32_t i = 0; i < n; ++i)
    {
        if (planes[i].normal.y > 0.9f)
        {
            sawFloor = 1;
        }
        if (planes[i].normal.x < -0.9f)
        {
            sawWall = 1;
        }
    }
    CHECK(sawFloor && sawWall, "the planes face out of their shapes");
    int32_t nSensor =
        m3World_CollideMover(world, (m3Pos3){-2.0, 0.92, 0.0}, 0.5f, 0.4f, 0.05f, planes, 8);
    CHECK(nSensor == 1 && planes[0].normal.y > 0.9f,
          "a sensor is invisible; only the floor speaks");

    // Solve: a downhill-and-forward wish against the floor plane
    // keeps its slide and loses its sink.
    m3MoverPlane fp;
    fp.normal = (m3Vec3){0.0f, 1.0f, 0.0f};
    fp.separation = 0.0f;
    fp.shape = (m3ShapeId){0, 0, 0};
    m3Vec3 solved = m3SolvePlanes((m3Vec3){1.0f, -0.5f, 0.0f}, &fp, 1, 8);
    CHECK(fabsf(solved.x - 1.0f) < 1.0e-5f, "the slide survives");
    CHECK(solved.y > -1.0e-5f, "the sink is absorbed");

    // The corner: two planes leave only the free axis.
    m3MoverPlane corner[2];
    corner[0] = fp;
    corner[1].normal = (m3Vec3){-1.0f, 0.0f, 0.0f};
    corner[1].separation = 0.0f;
    corner[1].shape = (m3ShapeId){0, 0, 0};
    solved = m3SolvePlanes((m3Vec3){1.0f, -0.5f, 0.7f}, corner, 2, 8);
    CHECK(solved.x < 1.0e-5f && solved.y > -1.0e-5f && fabsf(solved.z - 0.7f) < 1.0e-5f,
          "the corner leaves only the free axis");

    CHECK(m3World_Hash(world) == before, "the toolkit moved no bits");
    m3DestroyWorld(world);
    TestRigidWalkerRecipe();
    if (s_failures == 0)
    {
        printf("test_mover: all passed\n");
        return 0;
    }
    printf("test_mover: %d FAILURES\n", s_failures);
    return 1;
}
