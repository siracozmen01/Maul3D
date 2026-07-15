// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Debug draw gate (2c-9): the observer promise, held three ways. A
// draw pass between two snapshots leaves every byte identical; a twin
// that draws every step lands bit-exact on a twin that never draws;
// and two identical worlds emit bit-identical draw streams. Plus
// coverage counts: every shape family, contacts, joints, AABBs, and
// the color story (static, sensor, sleep tint) all reach the
// callbacks.

#include "maul3d/draw.h"
#include "maul3d/joint.h"
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

// Counting sink: tallies primitives, tracks colors, and folds every
// emitted byte into an FNV-1a hash so twin streams can be compared
// bit-for-bit.
typedef struct DrawSink
{
    int32_t segments;
    int32_t points;
    int32_t colorCounts32; // distinct colors seen (capped small)
    uint32_t colors[32];
    uint64_t hash;
} DrawSink;

static void SinkFold(DrawSink* sink, const void* data, size_t bytes)
{
    const uint8_t* p = (const uint8_t*)data;
    for (size_t i = 0; i < bytes; ++i)
    {
        sink->hash = (sink->hash ^ p[i]) * 0x100000001b3ull;
    }
}

static void SinkColor(DrawSink* sink, uint32_t color)
{
    for (int32_t i = 0; i < sink->colorCounts32; ++i)
    {
        if (sink->colors[i] == color)
        {
            return;
        }
    }
    if (sink->colorCounts32 < 32)
    {
        sink->colors[sink->colorCounts32] = color;
        sink->colorCounts32 += 1;
    }
}

static bool SinkSawColor(const DrawSink* sink, uint32_t color)
{
    for (int32_t i = 0; i < sink->colorCounts32; ++i)
    {
        if (sink->colors[i] == color)
        {
            return true;
        }
    }
    return false;
}

static void SinkSegment(m3Pos3 p1, m3Pos3 p2, uint32_t color, void* context)
{
    DrawSink* sink = (DrawSink*)context;
    sink->segments += 1;
    SinkColor(sink, color);
    SinkFold(sink, &p1, sizeof(p1));
    SinkFold(sink, &p2, sizeof(p2));
    SinkFold(sink, &color, sizeof(color));
}

static void SinkPoint(m3Pos3 p, m3real size, uint32_t color, void* context)
{
    DrawSink* sink = (DrawSink*)context;
    sink->points += 1;
    SinkColor(sink, color);
    SinkFold(sink, &p, sizeof(p));
    SinkFold(sink, &size, sizeof(size));
    SinkFold(sink, &color, sizeof(color));
}

static void SinkTriangle(m3Pos3 a, m3Pos3 b, m3Pos3 c, uint32_t color, void* context)
{
    DrawSink* sink = (DrawSink*)context;
    sink->segments += 1; // triangles count as emissions like the rest
    (void)a;
    (void)b;
    (void)c;
    (void)color;
}

static DrawSink MakeSink(void)
{
    DrawSink sink;
    memset(&sink, 0, sizeof(sink));
    sink.hash = 0xcbf29ce484222325ull;
    return sink;
}

static m3DebugDraw AllOn(DrawSink* sink)
{
    m3DebugDraw draw;
    memset(&draw, 0, sizeof(draw));
    draw.DrawSegment = SinkSegment;
    draw.DrawPoint = SinkPoint;
    draw.context = sink;
    draw.drawShapes = true;
    draw.drawContacts = true;
    draw.drawJoints = true;
    draw.drawAabbs = true;
    draw.drawSleepTint = true;
    return draw;
}

// One busy world: a plane floor, a resting sphere (contact), a box on
// a spherical joint, a capsule, a small mesh ramp, a sensor box, and
// a kinematic slab. Every family and every color story in one scene.
static m3WorldId MakeZoo(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    def.jointCapacity = 4;
    def.meshCapacity = 2;
    m3WorldId world = m3CreateWorld(&def);

    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    m3CreatePlaneShape(ground, &sd, &floor);

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 0.5, 0.0};
    m3BodyId ball = m3CreateBody(world, &bd);
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.5f};
    m3CreateSphereShape(ball, &sd, &sphere);

    bd.position = (m3Pos3){3.0, 2.0, 0.0};
    m3BodyId box = m3CreateBody(world, &bd);
    m3CreateBoxShape(box, &sd, (m3Vec3){0.4f, 0.4f, 0.4f});

    bd.position = (m3Pos3){-3.0, 1.0, 0.0};
    m3BodyId pill = m3CreateBody(world, &bd);
    m3Capsule capsule = {{0.0f, -0.3f, 0.0f}, {0.0f, 0.3f, 0.0f}, 0.3f};
    m3CreateCapsuleShape(pill, &sd, &capsule);

    // A two-triangle mesh quad away from the action.
    m3BodyDef md = m3DefaultBodyDef();
    md.position = (m3Pos3){8.0, 0.0, 8.0};
    m3BodyId meshBody = m3CreateBody(world, &md);
    m3Vec3 verts[4] = {
        {0.0f, 0.0f, 0.0f}, {2.0f, 0.0f, 0.0f}, {2.0f, 0.0f, 2.0f}, {0.0f, 0.0f, 2.0f}};
    uint16_t tris[6] = {0, 2, 1, 0, 3, 2};
    m3CreateMeshShape(meshBody, &sd, verts, 4, tris, 2);

    // The sensor: a purple box in the color story.
    m3BodyDef zd = m3DefaultBodyDef();
    zd.position = (m3Pos3){0.0, 5.0, 5.0};
    m3BodyId zone = m3CreateBody(world, &zd);
    m3ShapeDef zdef = m3DefaultShapeDef();
    zdef.isSensor = true;
    m3CreateBoxShape(zone, &zdef, (m3Vec3){1.0f, 1.0f, 1.0f});

    // The kinematic slab.
    m3BodyDef kd = m3DefaultBodyDef();
    kd.type = m3_kinematicBody;
    kd.position = (m3Pos3){-8.0, 3.0, 0.0};
    m3BodyId slab = m3CreateBody(world, &kd);
    m3CreateBoxShape(slab, &sd, (m3Vec3){1.0f, 0.2f, 1.0f});

    // The joint: box hangs from a static anchor point.
    m3BodyDef ad = m3DefaultBodyDef();
    ad.position = (m3Pos3){3.0, 3.0, 0.0};
    m3BodyId anchor = m3CreateBody(world, &ad);
    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_sphericalJoint;
    jd.bodyA = anchor;
    jd.bodyB = box;
    jd.localAnchorA = (m3Vec3){0.0f, 0.0f, 0.0f};
    jd.localAnchorB = (m3Vec3){0.0f, 1.0f, 0.0f};
    m3CreateJoint(&jd);

    return world;
}

static void TestDrawCoverage(void)
{
    m3WorldId world = MakeZoo();
    for (int32_t i = 0; i < 240; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }

    DrawSink sink = MakeSink();
    m3DebugDraw draw = AllOn(&sink);
    m3World_Draw(world, &draw);

    CHECK(sink.segments > 100, "a full zoo draws a real wireframe");
    CHECK(sink.points >= 3, "contact points and joint anchors emit");
    CHECK(SinkSawColor(&sink, 0x66BB66u), "static color seen");
    CHECK(SinkSawColor(&sink, 0xC9A227u), "kinematic color seen");
    CHECK(SinkSawColor(&sink, 0xAA66CCu), "sensor color seen");
    CHECK(SinkSawColor(&sink, 0xFF5544u), "contact color seen");
    CHECK(SinkSawColor(&sink, 0xFFFFFFu), "joint color seen");
    CHECK(SinkSawColor(&sink, 0x333344u), "aabb color seen");
    // After four seconds everything dynamic is at rest and asleep:
    // the sleep tint replaces the awake blue.
    CHECK(SinkSawColor(&sink, 0x777788u), "sleep tint seen on settled bodies");

    // Flags off = silence, and null callbacks are skipped, not called.
    DrawSink quiet = MakeSink();
    m3DebugDraw off;
    memset(&off, 0, sizeof(off));
    off.DrawSegment = SinkSegment;
    off.DrawPoint = SinkPoint;
    off.context = &quiet;
    m3World_Draw(world, &off);
    CHECK(quiet.segments == 0 && quiet.points == 0, "all flags off draws nothing");

    m3DebugDraw noPoints = AllOn(&sink);
    noPoints.DrawPoint = NULL;
    m3World_Draw(world, &noPoints); // must not crash
    m3DestroyWorld(world);
}

static void TestDrawIsPureObserver(void)
{
    // The headline: snapshot, draw with everything on, snapshot again,
    // and require every byte identical. Then the interleaved twin:
    // world A draws every step, world B never draws, 240 steps, and
    // the full snapshots must match bit for bit.
    m3WorldId world = MakeZoo();
    for (int32_t i = 0; i < 90; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    int32_t bytes = m3World_SnapshotSize(world);
    uint8_t* before = (uint8_t*)malloc((size_t)bytes);
    uint8_t* after = (uint8_t*)malloc((size_t)bytes);
    CHECK(m3World_Snapshot(world, before, bytes) == bytes, "snapshot before writes");

    DrawSink sink = MakeSink();
    m3DebugDraw draw = AllOn(&sink);
    m3World_Draw(world, &draw);
    m3World_Draw(world, &draw); // twice: a second pass is as pure as the first
    m3SolidDraw solid;
    memset(&solid, 0, sizeof(solid));
    solid.DrawTriangle = SinkTriangle;
    solid.context = &sink;
    solid.drawSleepTint = true;
    m3World_DrawSolid(world, &solid); // the solid pass is held to the
                                      // same purity law
    m3World_DrawSolid(world, &solid);
    m3ExtraDraw extras;
    memset(&extras, 0, sizeof(extras));
    extras.DrawSegment = SinkSegment;
    extras.DrawPoint = SinkPoint;
    extras.context = &sink;
    extras.drawIslands = true;
    extras.drawMassAxes = true;
    extras.drawTreeBoxes = true;
    m3World_DrawExtras(world, &extras); // the extras walk obeys the
                                        // same purity law (14-2)
    m3World_DrawExtras(world, &extras);

    CHECK(m3World_Snapshot(world, after, bytes) == bytes, "snapshot after writes");
    CHECK(memcmp(before, after, (size_t)bytes) == 0, "a draw pass moves no bits");
    free(before);
    free(after);
    m3DestroyWorld(world);

    m3WorldId drawn = MakeZoo();
    m3WorldId silent = MakeZoo();
    DrawSink stepSink = MakeSink();
    m3DebugDraw stepDraw = AllOn(&stepSink);
    for (int32_t i = 0; i < 240; ++i)
    {
        m3World_Step(drawn, 1.0f / 60.0f, 4);
        m3World_Draw(drawn, &stepDraw);
        m3World_Step(silent, 1.0f / 60.0f, 4);
    }
    int32_t bytesA = m3World_SnapshotSize(drawn);
    int32_t bytesB = m3World_SnapshotSize(silent);
    CHECK(bytesA == bytesB, "twin snapshots size-match");
    uint8_t* snapA = (uint8_t*)malloc((size_t)bytesA);
    uint8_t* snapB = (uint8_t*)malloc((size_t)bytesB);
    m3World_Snapshot(drawn, snapA, bytesA);
    m3World_Snapshot(silent, snapB, bytesB);
    CHECK(memcmp(snapA, snapB, (size_t)bytesA) == 0,
          "drawing every step never perturbs the simulation");
    free(snapA);
    free(snapB);
    m3DestroyWorld(drawn);
    m3DestroyWorld(silent);
}

static void TestDrawStreamDeterminism(void)
{
    // Two identical worlds emit bit-identical streams: positions,
    // sizes, and colors all fold into the hash.
    uint64_t hashes[2];
    int32_t segments[2];
    for (int32_t run = 0; run < 2; ++run)
    {
        m3WorldId world = MakeZoo();
        for (int32_t i = 0; i < 150; ++i)
        {
            m3World_Step(world, 1.0f / 60.0f, 4);
        }
        DrawSink sink = MakeSink();
        m3DebugDraw draw = AllOn(&sink);
        m3World_Draw(world, &draw);
        hashes[run] = sink.hash;
        segments[run] = sink.segments;
        m3DestroyWorld(world);
    }
    CHECK(hashes[0] == hashes[1], "twin worlds draw bit-identical streams");
    CHECK(segments[0] == segments[1] && segments[0] > 0, "twin counts match and are live");
}

// The solid walk's winding law: on a CONVEX shape centered at the
// origin, every emitted triangle's normal must point AWAY from the
// center (counter-clockwise seen from outside). The Windows run
// found the voxel table mirrored and fort walls transparent; this
// test makes every family's winding a gate.
typedef struct WindingSink
{
    m3Pos3 center;
    int32_t total;
    int32_t inward;
} WindingSink;

static void WindingTriangle(m3Pos3 a, m3Pos3 b, m3Pos3 c, uint32_t color, void* context)
{
    (void)color;
    WindingSink* sink = (WindingSink*)context;
    double ux = b.x - a.x, uy = b.y - a.y, uz = b.z - a.z;
    double vx = c.x - a.x, vy = c.y - a.y, vz = c.z - a.z;
    double nx = uy * vz - uz * vy;
    double ny = uz * vx - ux * vz;
    double nz = ux * vy - uy * vx;
    double cx = (a.x + b.x + c.x) / 3.0 - sink->center.x;
    double cy = (a.y + b.y + c.y) / 3.0 - sink->center.y;
    double cz = (a.z + b.z + c.z) / 3.0 - sink->center.z;
    sink->total += 1;
    if (nx * cx + ny * cy + nz * cz <= 0.0)
    {
        sink->inward += 1;
    }
}

static void CheckConvexWinding(m3WorldId world, m3Pos3 center, const char* label)
{
    WindingSink sink;
    memset(&sink, 0, sizeof(sink));
    sink.center = center;
    m3SolidDraw solid;
    memset(&solid, 0, sizeof(solid));
    solid.DrawTriangle = WindingTriangle;
    solid.context = &sink;
    m3World_DrawSolid(world, &solid);
    CHECK(sink.total > 0, label);
    if (sink.inward != 0)
    {
        printf("FAIL: %s emits %d of %d triangles wound inward\n", label, sink.inward, sink.total);
        s_failures += 1;
    }
}

static void TestSolidWindings(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    def.voxelCapacity = 2;

    // One convex shape per world, centered, so the away-from-center
    // law is exact.
    {
        m3WorldId world = m3CreateWorld(&def);
        m3BodyDef bd = m3DefaultBodyDef();
        m3BodyId body = m3CreateBody(world, &bd);
        m3ShapeDef sd = m3DefaultShapeDef();
        m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.7f};
        m3CreateSphereShape(body, &sd, &ball);
        CheckConvexWinding(world, (m3Pos3){0.0, 0.0, 0.0}, "sphere solid triangles");
        m3DestroyWorld(world);
    }
    {
        m3WorldId world = m3CreateWorld(&def);
        m3BodyDef bd = m3DefaultBodyDef();
        m3BodyId body = m3CreateBody(world, &bd);
        m3ShapeDef sd = m3DefaultShapeDef();
        m3Capsule pill = {{0.0f, -0.5f, 0.0f}, {0.0f, 0.5f, 0.0f}, 0.35f};
        m3CreateCapsuleShape(body, &sd, &pill);
        CheckConvexWinding(world, (m3Pos3){0.0, 0.0, 0.0}, "capsule solid triangles");
        m3DestroyWorld(world);
    }
    {
        m3WorldId world = m3CreateWorld(&def);
        m3BodyDef bd = m3DefaultBodyDef();
        m3BodyId body = m3CreateBody(world, &bd);
        m3ShapeDef sd = m3DefaultShapeDef();
        m3CreateBoxShape(body, &sd, (m3Vec3){0.5f, 0.6f, 0.7f});
        CheckConvexWinding(world, (m3Pos3){0.0, 0.0, 0.0}, "hull solid triangles");
        m3DestroyWorld(world);
    }
    {
        // A single full voxel: one merged box centered at cell center.
        m3WorldId world = m3CreateWorld(&def);
        m3BodyDef bd = m3DefaultBodyDef();
        m3BodyId body = m3CreateBody(world, &bd);
        m3ShapeDef sd = m3DefaultShapeDef();
        static uint8_t voxels[16 * 16 * 16];
        memset(voxels, 0, sizeof(voxels));
        voxels[0] = 1;
        m3CreateVoxelChunkShape(body, &sd, voxels, NULL, 1.0f);
        CheckConvexWinding(world, (m3Pos3){0.5, 0.5, 0.5}, "voxel solid triangles");
        m3DestroyWorld(world);
    }
}

static void TestExtrasEmit(void)
{
    // The extras layers actually say something: after a settled
    // stack, islands mark every labeled dynamic body, mass axes
    // draw three segments per dynamic body, tree boxes appear once
    // the world holds more than one leaf, and twin worlds emit
    // bit-identical extra streams.
    DrawSink sinks[2];
    for (int32_t run = 0; run < 2; ++run)
    {
        m3WorldId world = MakeZoo();
        for (int32_t i = 0; i < 60; ++i)
        {
            m3World_Step(world, 1.0f / 60.0f, 4);
        }
        sinks[run] = MakeSink();
        m3ExtraDraw extras;
        memset(&extras, 0, sizeof(extras));
        extras.DrawSegment = SinkSegment;
        extras.DrawPoint = SinkPoint;
        extras.context = &sinks[run];
        extras.drawIslands = true;
        m3World_DrawExtras(world, &extras);
        if (run == 0)
        {
            CHECK(sinks[0].points > 0, "islands mark the dynamic bodies");
            int32_t islandPoints = sinks[0].points;
            extras.drawIslands = false;
            extras.drawMassAxes = true;
            m3World_DrawExtras(world, &extras);
            CHECK(sinks[0].segments == 3 * islandPoints, "three axes per labeled dynamic body");
            int32_t axisSegments = sinks[0].segments;
            extras.drawMassAxes = false;
            extras.drawTreeBoxes = true;
            m3World_DrawExtras(world, &extras);
            CHECK(sinks[0].segments > axisSegments, "internal tree boxes emit");
            extras.drawIslands = true;
            extras.drawMassAxes = true;
        }
        m3DestroyWorld(world);
    }
    CHECK(sinks[0].points == sinks[1].points, "twin extras emit the same counts");
}

int main(void)
{
    TestSolidWindings();
    TestDrawCoverage();
    TestDrawIsPureObserver();
    TestDrawStreamDeterminism();
    TestExtrasEmit();
    if (s_failures == 0)
    {
        printf("test_draw: all green\n");
        return 0;
    }
    printf("test_draw: %d failure(s)\n", s_failures);
    return 1;
}
