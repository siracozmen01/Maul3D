// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Events and queries gate (2b-13): contact begin/end streams against
// hand-counted expectations, analytic ray hits on every shape family,
// closest-of-many ordering, and the determinism twins for both.

#include "maul3d/shape.h"

#include <stdio.h>
#include <stdlib.h>
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

static void TestContactEvents(void)
{
    // A ball drops onto the plane: exactly one begin event over the
    // whole approach and rest. A commanded launch upward separates
    // them: exactly one end event.
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m3WorldId world = m3CreateWorld(&def);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    m3ShapeId floorShape = m3CreatePlaneShape(ground, &sd, &floor);

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 2.0, 0.0};
    m3BodyId ball = m3CreateBody(world, &bd);
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.5f};
    m3ShapeId ballShape = m3CreateSphereShape(ball, &sd, &sphere);

    int32_t begins = 0;
    int32_t ends = 0;
    m3ContactEvent firstBegin;
    memset(&firstBegin, 0, sizeof(firstBegin));
    for (int32_t i = 0; i < 120; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
        int32_t n = 0;
        const m3ContactEvent* ev = m3World_ContactBeginEvents(world, &n);
        if (n > 0 && begins == 0)
        {
            firstBegin = ev[0];
        }
        begins += n;
        m3World_ContactEndEvents(world, &n);
        ends += n;
    }
    CHECK(begins == 1, "one begin event for the landing");
    CHECK(ends == 0, "no end event while resting");
    CHECK(firstBegin.shapeA.index1 == floorShape.index1 &&
              firstBegin.shapeB.index1 == ballShape.index1,
          "the begin event names the pair in canonical order");
    CHECK(m3Shape_IsValid(firstBegin.shapeA) && m3Shape_IsValid(firstBegin.shapeB),
          "event ids are live handles");

    m3Body_SetLinearVelocity(ball, (m3Vec3){0.0f, 8.0f, 0.0f});
    for (int32_t i = 0; i < 30; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
        int32_t n = 0;
        m3World_ContactEndEvents(world, &n);
        ends += n;
    }
    CHECK(ends == 1, "one end event for the launch");

    // Events are transient: a snapshot restore clears them.
    int32_t snapBytes = m3World_SnapshotSize(world);
    void* snap = malloc((size_t)snapBytes);
    CHECK(m3World_Snapshot(world, snap, snapBytes) == snapBytes, "snapshot writes");
    CHECK(m3World_Restore(world, snap, snapBytes), "snapshot restores");
    int32_t n = 1;
    m3World_ContactBeginEvents(world, &n);
    CHECK(n == 0, "restore clears begin events");
    m3World_ContactEndEvents(world, &n);
    CHECK(n == 0, "restore clears end events");
    free(snap);
    m3DestroyWorld(world);
}

static void TestRayHitsEveryFamily(void)
{
    // Analytic hits: one shape of every family at a known place, one
    // ray each, fraction and normal checked by hand.
    m3WorldDef def = m3DefaultWorldDef();
    def.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    m3WorldId world = m3CreateWorld(&def);
    m3BodyDef bd = m3DefaultBodyDef();
    m3ShapeDef sd = m3DefaultShapeDef();

    bd.position = (m3Pos3){10.0, 0.0, 0.0};
    m3BodyId sphereBody = m3CreateBody(world, &bd);
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 1.0f};
    m3CreateSphereShape(sphereBody, &sd, &sphere);

    bd.position = (m3Pos3){10.0, 10.0, 0.0};
    m3BodyId boxBody = m3CreateBody(world, &bd);
    m3CreateBoxShape(boxBody, &sd, (m3Vec3){1.0f, 1.0f, 1.0f});

    bd.position = (m3Pos3){10.0, 20.0, 0.0};
    m3BodyId capBody = m3CreateBody(world, &bd);
    m3Capsule capsule = {{0.0f, -1.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, 0.5f};
    m3CreateCapsuleShape(capBody, &sd, &capsule);

    bd.position = (m3Pos3){8.0, 30.0, -2.0};
    m3BodyId meshBody = m3CreateBody(world, &bd);
    m3Vec3 verts[4] = {
        {0.0f, 0.0f, 0.0f}, {4.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 4.0f}, {4.0f, 0.0f, 4.0f}};
    uint16_t tris[6] = {0, 2, 1, 1, 2, 3};
    CHECK(m3Shape_IsValid(m3CreateMeshShape(meshBody, &sd, verts, 4, tris, 2)),
          "the query mesh builds");

    // Sphere: from the origin along +x, entry at x = 9, fraction 0.45.
    m3RayHit hit =
        m3World_CastRayClosest(world, (m3Pos3){0.0, 0.0, 0.0}, (m3Vec3){20.0f, 0.0f, 0.0f});
    CHECK(hit.hit, "the sphere ray hits");
    CHECK(hit.fraction > 0.449f && hit.fraction < 0.451f, "sphere entry fraction");
    CHECK(hit.normal.x < -0.99f, "sphere entry normal faces the ray");
    CHECK(hit.point.x > 8.99 && hit.point.x < 9.01, "sphere entry point");

    // Box: entry at x = 9, fraction (9-0)/20.
    hit = m3World_CastRayClosest(world, (m3Pos3){0.0, 10.0, 0.0}, (m3Vec3){20.0f, 0.0f, 0.0f});
    CHECK(hit.hit && hit.fraction > 0.449f && hit.fraction < 0.451f, "box entry fraction");
    CHECK(hit.normal.x < -0.99f, "box entry normal");

    // Capsule side: entry at x = 9.5.
    hit = m3World_CastRayClosest(world, (m3Pos3){0.0, 20.0, 0.0}, (m3Vec3){20.0f, 0.0f, 0.0f});
    CHECK(hit.hit && hit.fraction > 0.474f && hit.fraction < 0.476f, "capsule side fraction");

    // Mesh: straight down onto the quad at (10, 30, 0).
    hit = m3World_CastRayClosest(world, (m3Pos3){10.0, 35.0, 0.0}, (m3Vec3){0.0f, -10.0f, 0.0f});
    CHECK(hit.hit && hit.fraction > 0.499f && hit.fraction < 0.501f, "mesh hit fraction");
    CHECK(hit.normal.y > 0.99f, "mesh hit normal is up");

    // A miss stays a miss.
    hit = m3World_CastRayClosest(world, (m3Pos3){0.0, 50.0, 0.0}, (m3Vec3){20.0f, 0.0f, 0.0f});
    CHECK(!hit.hit, "empty space misses");
    hit = m3World_CastRayClosest(world, (m3Pos3){0.0, 0.0, 0.0}, (m3Vec3){0.0f, 0.0f, 0.0f});
    CHECK(!hit.hit, "a zero ray misses by contract");
    m3DestroyWorld(world);
}

static void TestRayClosestOfMany(void)
{
    // Two spheres on the ray: the nearer one wins; from the other
    // side, the other one wins. Plus the plane pass.
    m3WorldDef def = m3DefaultWorldDef();
    def.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m3WorldId world = m3CreateWorld(&def);
    m3BodyDef bd = m3DefaultBodyDef();
    m3ShapeDef sd = m3DefaultShapeDef();

    bd.position = (m3Pos3){5.0, 0.0, 0.0};
    m3BodyId nearBody = m3CreateBody(world, &bd);
    m3Sphere s1 = {{0.0f, 0.0f, 0.0f}, 0.5f};
    m3ShapeId nearShape = m3CreateSphereShape(nearBody, &sd, &s1);
    bd.position = (m3Pos3){15.0, 0.0, 0.0};
    m3BodyId farBody = m3CreateBody(world, &bd);
    m3ShapeId farShape = m3CreateSphereShape(farBody, &sd, &s1);

    m3RayHit hit =
        m3World_CastRayClosest(world, (m3Pos3){0.0, 0.0, 0.0}, (m3Vec3){20.0f, 0.0f, 0.0f});
    CHECK(hit.hit && hit.shape.index1 == nearShape.index1, "the nearer sphere wins");
    hit = m3World_CastRayClosest(world, (m3Pos3){20.0, 0.0, 0.0}, (m3Vec3){-20.0f, 0.0f, 0.0f});
    CHECK(hit.hit && hit.shape.index1 == farShape.index1,
          "from the other side the other sphere wins");

    // The infinite plane through the dedicated pass.
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, -2.0f};
    m3CreatePlaneShape(ground, &sd, &floor);
    hit = m3World_CastRayClosest(world, (m3Pos3){0.0, 3.0, 0.0}, (m3Vec3){0.0f, -10.0f, 0.0f});
    CHECK(hit.hit && hit.fraction > 0.499f && hit.fraction < 0.501f, "the plane pass hits");
    m3DestroyWorld(world);
}

static void TestEventDeterminism(void)
{
    // The event stream is part of what the engine promises: the same
    // scene produces the identical stream, run after run.
    uint64_t streamHash[2] = {0, 0};
    for (int32_t run = 0; run < 2; ++run)
    {
        m3WorldDef def = m3DefaultWorldDef();
        def.bodyCapacity = 16;
        def.shapeCapacity = 16;
        m3WorldId world = m3CreateWorld(&def);
        m3BodyDef gd = m3DefaultBodyDef();
        m3BodyId ground = m3CreateBody(world, &gd);
        m3ShapeDef sd = m3DefaultShapeDef();
        m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
        m3CreatePlaneShape(ground, &sd, &floor);
        m3BodyDef bd = m3DefaultBodyDef();
        bd.type = m3_dynamicBody;
        for (int32_t k = 0; k < 5; ++k)
        {
            bd.position = (m3Pos3){0.35 * (double)k, 1.0 + 0.8 * (double)k, 0.1 * (double)k};
            m3BodyId body = m3CreateBody(world, &bd);
            m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.4f};
            m3CreateSphereShape(body, &sd, &ball);
        }
        uint64_t h = 0xcbf29ce484222325ull;
        for (int32_t i = 0; i < 240; ++i)
        {
            m3World_Step(world, 1.0f / 60.0f, 4);
            int32_t n = 0;
            const m3ContactEvent* ev = m3World_ContactBeginEvents(world, &n);
            for (int32_t k = 0; k < n; ++k)
            {
                h = (h ^ (uint64_t)ev[k].shapeA.index1) * 0x100000001B3ull;
                h = (h ^ (uint64_t)ev[k].shapeB.index1) * 0x100000001B3ull;
                h = (h ^ (uint64_t)i) * 0x100000001B3ull;
            }
            ev = m3World_ContactEndEvents(world, &n);
            for (int32_t k = 0; k < n; ++k)
            {
                h = (h ^ (uint64_t)ev[k].shapeA.index1) * 0x100000001B3ull;
                h = (h ^ (uint64_t)ev[k].shapeB.index1) * 0x100000001B3ull;
                h = (h ^ (uint64_t)(i + 7777)) * 0x100000001B3ull;
            }
        }
        streamHash[run] = h;
        m3DestroyWorld(world);
    }
    CHECK(streamHash[0] == streamHash[1], "the event stream is bit-deterministic");
    CHECK(streamHash[0] != 0xcbf29ce484222325ull, "the stream is not empty");
}

static void TestShapeCasts(void)
{
    // A sphere cast toward a wall touches when the SURFACES meet:
    // the fraction accounts for both radii, checked by hand.
    m3WorldDef def = m3DefaultWorldDef();
    def.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m3WorldId world = m3CreateWorld(&def);
    m3BodyDef bd = m3DefaultBodyDef();
    bd.position = (m3Pos3){10.0, 0.0, 0.0};
    m3BodyId wall = m3CreateBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3CreateBoxShape(wall, &sd, (m3Vec3){0.5f, 2.0f, 2.0f});

    // Cast a 0.5-radius sphere from the origin: the box face sits at
    // x = 9.5, the sphere surface leads by 0.5, so centers touch at
    // x = 9.0: fraction 9/20 = 0.45 (minus the slop skin).
    m3RayHit hit = m3World_CastSphereClosest(world, (m3Pos3){0.0, 0.0, 0.0}, 0.5f,
                                             (m3Vec3){20.0f, 0.0f, 0.0f});
    CHECK(hit.hit, "the sphere cast hits");
    CHECK(hit.fraction > 0.44f && hit.fraction < 0.4505f, "the cast fraction counts both skins");
    CHECK(hit.normal.x < -0.99f, "the cast normal faces the caster");

    // A capsule cast: the lower cap leads; the analytic touch uses
    // the cap's forward surface exactly like the sphere.
    hit = m3World_CastCapsuleClosest(world, (m3Pos3){0.0, 0.0, 0.0}, (m3Vec3){0.0f, -0.5f, 0.0f},
                                     (m3Vec3){0.0f, 0.5f, 0.0f}, 0.3f, (m3Vec3){20.0f, 0.0f, 0.0f});
    CHECK(hit.hit, "the capsule cast hits");
    CHECK(hit.fraction > 0.45f && hit.fraction < 0.4605f, "the capsule fraction is analytic");

    // Start overlapped: the documented contract is a hit at zero.
    hit =
        m3World_CastSphereClosest(world, (m3Pos3){9.6, 0.0, 0.0}, 0.5f, (m3Vec3){5.0f, 0.0f, 0.0f});
    CHECK(hit.hit && hit.fraction == 0.0f, "a start-overlapped cast hits at zero");

    // A cast that misses stays a miss.
    hit = m3World_CastSphereClosest(world, (m3Pos3){0.0, 10.0, 0.0}, 0.5f,
                                    (m3Vec3){20.0f, 0.0f, 0.0f});
    CHECK(!hit.hit, "a clear cast misses");
    m3DestroyWorld(world);
}

static void TestRayStartInsideContract(void)
{
    // The documented family-by-family contract: rays MISS shapes
    // they start inside (front faces only, everywhere).
    m3WorldDef def = m3DefaultWorldDef();
    def.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m3WorldId world = m3CreateWorld(&def);
    m3BodyDef bd = m3DefaultBodyDef();
    m3ShapeDef sd = m3DefaultShapeDef();

    bd.position = (m3Pos3){0.0, 0.0, 0.0};
    m3BodyId a = m3CreateBody(world, &bd);
    m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 1.0f};
    m3CreateSphereShape(a, &sd, &ball);
    m3RayHit hit =
        m3World_CastRayClosest(world, (m3Pos3){0.0, 0.0, 0.0}, (m3Vec3){5.0f, 0.0f, 0.0f});
    CHECK(!hit.hit, "a ray starting inside a sphere misses it");

    bd.position = (m3Pos3){10.0, 0.0, 0.0};
    m3BodyId b = m3CreateBody(world, &bd);
    m3CreateBoxShape(b, &sd, (m3Vec3){1.0f, 1.0f, 1.0f});
    hit = m3World_CastRayClosest(world, (m3Pos3){10.0, 0.0, 0.0}, (m3Vec3){5.0f, 0.0f, 0.0f});
    CHECK(!hit.hit, "a ray starting inside a hull misses it");

    bd.position = (m3Pos3){20.0, 0.0, 0.0};
    m3BodyId c = m3CreateBody(world, &bd);
    m3Capsule capsule = {{0.0f, -0.5f, 0.0f}, {0.0f, 0.5f, 0.0f}, 0.5f};
    m3CreateCapsuleShape(c, &sd, &capsule);
    hit = m3World_CastRayClosest(world, (m3Pos3){20.0, 0.0, 0.0}, (m3Vec3){5.0f, 0.0f, 0.0f});
    CHECK(!hit.hit, "a ray starting inside a capsule misses it");
    m3DestroyWorld(world);
}

static void TestMultiHitSorted(void)
{
    // Two spheres and the floor plane along one ray: three hits,
    // sorted by fraction, and the capacity drop takes the far end.
    m3WorldDef def = m3DefaultWorldDef();
    def.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m3WorldId world = m3CreateWorld(&def);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    m3CreatePlaneShape(ground, &sd, &floor);

    m3BodyDef bd = m3DefaultBodyDef();
    bd.position = (m3Pos3){4.0, 4.0, 0.0};
    m3BodyId nearBody = m3CreateBody(world, &bd);
    m3Sphere s1 = {{0.0f, 0.0f, 0.0f}, 0.5f};
    m3CreateSphereShape(nearBody, &sd, &s1);
    bd.position = (m3Pos3){8.0, 2.0, 0.0};
    m3BodyId midBody = m3CreateBody(world, &bd);
    m3CreateSphereShape(midBody, &sd, &s1);

    // A ray sloping down through both spheres into the floor.
    m3RayHit hits[8];
    int32_t n =
        m3World_CastRayAll(world, (m3Pos3){0.0, 6.0, 0.0}, (m3Vec3){16.0f, -8.0f, 0.0f}, hits, 8);
    CHECK(n == 3, "three hits along the ray");
    CHECK(hits[0].fraction < hits[1].fraction && hits[1].fraction < hits[2].fraction,
          "the hits are sorted by fraction");
    CHECK(hits[2].normal.y > 0.99f, "the last hit is the floor");

    // Capacity two: the FAR hit drops.
    m3RayHit two[2];
    int32_t n2 =
        m3World_CastRayAll(world, (m3Pos3){0.0, 6.0, 0.0}, (m3Vec3){16.0f, -8.0f, 0.0f}, two, 2);
    CHECK(n2 == 2, "capacity caps the count");
    CHECK(two[0].fraction == hits[0].fraction && two[1].fraction == hits[1].fraction,
          "the near hits survive the cap");
    m3DestroyWorld(world);
}

static void TestPointAndOverlaps(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m3WorldId world = m3CreateWorld(&def);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    m3ShapeId floorShape = m3CreatePlaneShape(ground, &sd, &floor);

    m3BodyDef bd = m3DefaultBodyDef();
    bd.position = (m3Pos3){5.0, 2.0, 0.0};
    m3BodyId boxBody = m3CreateBody(world, &bd);
    m3ShapeId boxShape = m3CreateBoxShape(boxBody, &sd, (m3Vec3){1.0f, 1.0f, 1.0f});

    // Point containment per family.
    m3ShapeId inBox = m3World_PointInside(world, (m3Pos3){5.2, 2.3, 0.4});
    CHECK(inBox.index1 == boxShape.index1, "the point inside the box finds the box");
    m3ShapeId below = m3World_PointInside(world, (m3Pos3){100.0, -0.5, 0.0});
    CHECK(below.index1 == floorShape.index1, "a point below the floor is inside the half space");
    m3ShapeId nowhere = m3World_PointInside(world, (m3Pos3){5.0, 8.0, 0.0});
    CHECK(nowhere.index1 == 0, "open air contains nothing");

    // Overlaps: the AABB box catches the box, the sphere probes reach.
    m3ShapeId found[8];
    int32_t n =
        m3World_OverlapAabb(world, (m3Pos3){4.0, 1.0, -1.0}, (m3Pos3){6.0, 3.0, 1.0}, found, 8);
    CHECK(n == 1 && found[0].index1 == boxShape.index1, "the AABB overlap finds the box");
    n = m3World_OverlapSphere(world, (m3Pos3){5.0, 4.5, 0.0}, 1.6f, found, 8);
    CHECK(n == 1 && found[0].index1 == boxShape.index1, "the sphere overlap reaches the box");
    n = m3World_OverlapSphere(world, (m3Pos3){5.0, 4.5, 0.0}, 1.4f, found, 8);
    CHECK(n == 0, "just out of reach finds nothing");
    n = m3World_OverlapSphere(world, (m3Pos3){0.0, 0.5, 0.0}, 0.6f, found, 8);
    CHECK(n == 1 && found[0].index1 == floorShape.index1, "the sphere overlap reaches the floor");
    m3DestroyWorld(world);
}

static void TestSensorPassThrough(void)
{
    // A ball falls THROUGH a static sensor box: begin fires on
    // entry, end on exit, and the fall never slows (the free-fall
    // analytic is the no-response proof). The contact streams stay
    // silent for the sensor pair.
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m3WorldId world = m3CreateWorld(&def);
    m3BodyDef gd = m3DefaultBodyDef();
    gd.position = (m3Pos3){0.0, 3.0, 0.0};
    m3BodyId zone = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.isSensor = true;
    CHECK(m3Shape_IsValid(m3CreateBoxShape(zone, &sd, (m3Vec3){1.0f, 0.5f, 1.0f})),
          "the sensor zone builds");

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 6.0, 0.0};
    m3BodyId ball = m3CreateBody(world, &bd);
    m3ShapeDef bsd = m3DefaultShapeDef();
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.3f};
    m3CreateSphereShape(ball, &bsd, &sphere);

    int32_t sensorBegins = 0;
    int32_t sensorEnds = 0;
    int32_t contactBegins = 0;
    for (int32_t i = 0; i < 90; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
        int32_t n = 0;
        m3World_SensorBeginEvents(world, &n);
        sensorBegins += n;
        m3World_SensorEndEvents(world, &n);
        sensorEnds += n;
        m3World_ContactBeginEvents(world, &n);
        contactBegins += n;
    }
    CHECK(sensorBegins == 1, "one sensor begin on entry");
    CHECK(sensorEnds == 1, "one sensor end on exit");
    CHECK(contactBegins == 0, "the contact stream stays silent for sensors");
    // Free fall for 1.5 s from rest: y = 6 - 0.5 * 10 * 1.5^2 = -5.25
    // (substep integration lands within a band).
    m3Pos3 p = m3Body_GetPosition(ball);
    CHECK(p.y < -5.0 && p.y > -5.6, "the fall never slowed: sensors do not respond");
    m3DestroyWorld(world);
}

static void TestSensorSleepAndBullets(void)
{
    // A sensor neither wakes a sleeper nor stops a bullet, and a
    // body asleep INSIDE a sensor holds its begin without an end.
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m3WorldId world = m3CreateWorld(&def);
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    m3CreatePlaneShape(ground, &sd, &floor);

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 0.4, 0.0};
    m3BodyId sleeper = m3CreateBody(world, &bd);
    m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.4f};
    m3CreateSphereShape(sleeper, &sd, &ball);

    // A sensor zone AROUND the resting spot, created before sleep.
    m3BodyDef zd = m3DefaultBodyDef();
    zd.position = (m3Pos3){0.0, 0.5, 0.0};
    m3BodyId zone = m3CreateBody(world, &zd);
    m3ShapeDef zsd = m3DefaultShapeDef();
    zsd.isSensor = true;
    m3CreateBoxShape(zone, &zsd, (m3Vec3){2.0f, 1.0f, 2.0f});

    int32_t begins = 0;
    int32_t ends = 0;
    for (int32_t i = 0; i < 120; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
        int32_t n = 0;
        m3World_SensorBeginEvents(world, &n);
        begins += n;
        m3World_SensorEndEvents(world, &n);
        ends += n;
    }
    m3Vec3 v = m3Body_GetLinearVelocity(sleeper);
    CHECK(v.x == 0.0f && v.y == 0.0f && v.z == 0.0f,
          "the body sleeps INSIDE the sensor (overlap wakes no one)");
    CHECK(begins == 1 && ends == 0, "the begin holds through sleep with no end");

    // A bullet crosses a sensor wall without slowing.
    m3BodyDef wd = m3DefaultBodyDef();
    wd.position = (m3Pos3){10.0, 5.0, 0.0};
    m3BodyId wall = m3CreateBody(world, &wd);
    m3ShapeDef wsd = m3DefaultShapeDef();
    wsd.isSensor = true;
    m3CreateBoxShape(wall, &wsd, (m3Vec3){0.05f, 2.0f, 2.0f});

    bd.position = (m3Pos3){4.0, 5.0, 0.0};
    bd.linearVelocity = (m3Vec3){200.0f, 0.0f, 0.0f};
    bd.gravityScale = 0.0f;
    bd.isBullet = true;
    m3BodyId bullet = m3CreateBody(world, &bd);
    m3Sphere small = {{0.0f, 0.0f, 0.0f}, 0.1f};
    m3CreateSphereShape(bullet, &sd, &small);

    for (int32_t i = 0; i < 10; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    m3Pos3 bp = m3Body_GetPosition(bullet);
    CHECK(bp.x > 20.0, "the bullet crosses the sensor wall unimpeded");
    m3DestroyWorld(world);
}

int main(void)
{
    TestContactEvents();
    TestRayHitsEveryFamily();
    TestRayClosestOfMany();
    TestEventDeterminism();
    TestShapeCasts();
    TestRayStartInsideContract();
    TestMultiHitSorted();
    TestPointAndOverlaps();
    TestSensorPassThrough();
    TestSensorSleepAndBullets();
    if (s_failures == 0)
    {
        printf("test_queries: all checks passed\n");
    }
    return s_failures == 0 ? 0 : 1;
}
