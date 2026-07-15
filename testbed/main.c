// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The Maul3D testbed: an interactive playground over the debug-draw
// interface. Everything here is a VIEWER; the simulation stays inside
// the engine's determinism contract and the testbed only calls the
// public API. Controls:
//   right drag  orbit        wheel   zoom
//   W A S D     move the orbit target on the ground plane
//   left click  fire a bullet along the crosshair ray
//   E           carve a charge where the crosshair ray lands (voxels)
//   B           drop a crate above the crosshair hit
//   space       pause        F       single step
//   tab         next scene   F5      restart scene
//   R (hold)    REWIND TIME through the snapshot ring
//   F1          toggle AABBs F2      toggle contacts
//   ENTER       scene browser        F3   toggle wind (cloth scene)
//   C           crouch toggle (walker scenes)
//
// The look (post-1.9): a sky gradient, a lit ground plane, and the
// engine's solid triangle stream shaded by one sun with planar
// shadows. The wireframe pass survives as an overlay toggle; the
// simulation stays untouched (the solid walk is read-only and held
// by the same purity test as the wireframe one).

#include "maul3d/character.h"
#include "maul3d/draw.h"
#include "maul3d/joint.h"
#include "maul3d/replay.h"
#include "maul3d/shape.h"
#include "maul3d/softbody.h"
#include "maul3d/vehicle.h"
#include "raylib.h"
#include "rlgl.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// ------------------------------------------------------------- camera
static float s_yaw = 0.8f;
static float s_pitch = 0.5f;
static float s_distance = 28.0f;
static Vector3 s_target = {0.0f, 3.0f, 0.0f};

static Camera3D OrbitCamera(void)
{
    Camera3D cam;
    memset(&cam, 0, sizeof(cam));
    float cp = cosf(s_pitch);
    cam.position = (Vector3){s_target.x + s_distance * cp * cosf(s_yaw),
                             s_target.y + s_distance * sinf(s_pitch),
                             s_target.z + s_distance * cp * sinf(s_yaw)};
    cam.target = s_target;
    cam.up = (Vector3){0.0f, 1.0f, 0.0f};
    cam.fovy = 55.0f;
    cam.projection = CAMERA_PERSPECTIVE;
    return cam;
}

// -------------------------------------------------------- draw bridge
static Color FromHex(uint32_t rgb)
{
    return (Color){(unsigned char)(rgb >> 16), (unsigned char)((rgb >> 8) & 0xFF),
                   (unsigned char)(rgb & 0xFF), 255};
}

static Vector3 FromPos(m3Pos3 p)
{
    return (Vector3){(float)p.x, (float)p.y, (float)p.z};
}

static void DrawSegmentCb(m3Pos3 p1, m3Pos3 p2, uint32_t color, void* context)
{
    (void)context;
    DrawLine3D(FromPos(p1), FromPos(p2), FromHex(color));
}

static void DrawPointCb(m3Pos3 p, m3real size, uint32_t color, void* context)
{
    // A cube, not a sphere: raylib's DrawSphere tessellates on the
    // CPU per call, and a resting pile emits hundreds of contact
    // points per frame (the first Windows build was a slideshow in
    // the rain scene for exactly this reason).
    (void)context;
    float s = 0.02f * size;
    DrawCubeV(FromPos(p), (Vector3){s, s, s}, FromHex(color));
}

// -------------------------------------------------- solid render pass
// The engine emits filled world-space triangles; the viewer buffers
// them, drops a planar shadow for each, then shades with one sun.
// All of it is view-side cosmetics: the stream is read-only.

#define TB_MAX_TRIS 131072

typedef struct tbTri
{
    Vector3 a;
    Vector3 b;
    Vector3 c;
    Color base;
} tbTri;

static tbTri s_tris[TB_MAX_TRIS];
static int32_t s_triCount = 0;

static void SolidTriCb(m3Pos3 a, m3Pos3 b, m3Pos3 c, uint32_t color, void* context)
{
    (void)context;
    if (s_triCount >= TB_MAX_TRIS)
    {
        return;
    }
    tbTri* t = &s_tris[s_triCount++];
    t->a = FromPos(a);
    t->b = FromPos(b);
    t->c = FromPos(c);
    t->base = FromHex(color);
}

// The sun: a fixed direction, normalized once by hand.
static const Vector3 s_sun = {-0.42f, -0.82f, -0.39f};

static Vector3 TriNormal(const tbTri* t)
{
    Vector3 u = {t->b.x - t->a.x, t->b.y - t->a.y, t->b.z - t->a.z};
    Vector3 v = {t->c.x - t->a.x, t->c.y - t->a.y, t->c.z - t->a.z};
    Vector3 n = {u.y * v.z - u.z * v.y, u.z * v.x - u.x * v.z, u.x * v.y - u.y * v.x};
    float len = sqrtf(n.x * n.x + n.y * n.y + n.z * n.z);
    if (len > 1.0e-9f)
    {
        n.x /= len;
        n.y /= len;
        n.z /= len;
    }
    return n;
}

static Vector3 ShadowProject(Vector3 v)
{
    // Project along the sun onto the ground plane, lifted a hair so
    // the shadow wins the depth fight against the ground quad.
    float t = (v.y - 0.02f) / -s_sun.y;
    return (Vector3){v.x + s_sun.x * t, 0.02f, v.z + s_sun.z * t};
}

static void FlushSolid(bool shadows)
{
    if (shadows)
    {
        // Sun-facing triangles only (half the layers, and a box
        // casts one footprint instead of piling top and bottom
        // faces into hatching), drawn with culling OFF because the
        // planar projection is free to flip a winding.
        Color ink = {20, 22, 30, 40};
        rlDisableBackfaceCulling();
        for (int32_t i = 0; i < s_triCount; ++i)
        {
            const tbTri* t = &s_tris[i];
            if (t->a.y < 0.06f && t->b.y < 0.06f && t->c.y < 0.06f)
            {
                continue; // ground-level faces cast nothing useful
            }
            Vector3 n = TriNormal(t);
            if (n.x * s_sun.x + n.y * s_sun.y + n.z * s_sun.z >= 0.0f)
            {
                continue; // back to the sun: its twin casts instead
            }
            DrawTriangle3D(ShadowProject(t->a), ShadowProject(t->b), ShadowProject(t->c), ink);
        }
        rlEnableBackfaceCulling();
    }
    for (int32_t i = 0; i < s_triCount; ++i)
    {
        const tbTri* t = &s_tris[i];
        Vector3 n = TriNormal(t);
        float lambert = -(n.x * s_sun.x + n.y * s_sun.y + n.z * s_sun.z);
        if (lambert < 0.0f)
        {
            lambert = 0.0f;
        }
        float shade = 0.44f + 0.56f * lambert;
        Color lit = {(unsigned char)((float)t->base.r * shade),
                     (unsigned char)((float)t->base.g * shade),
                     (unsigned char)((float)t->base.b * shade), 255};
        DrawTriangle3D(t->a, t->b, t->c, lit);
    }
    s_triCount = 0;
}

static void DrawSky(int width, int height)
{
    DrawRectangleGradientV(0, 0, width, (int)(height * 0.62f), (Color){86, 92, 112, 255},
                           (Color){164, 170, 184, 255});
    DrawRectangle(0, (int)(height * 0.62f), width, height, (Color){164, 170, 184, 255});
}

static void DrawGround(void)
{
    // A lit slab and a modest grid: the Box3D stage, not a void.
    DrawPlane((Vector3){0.0f, -0.015f, 0.0f}, (Vector2){240.0f, 240.0f},
              (Color){157, 157, 159, 255});
    Color minor = {143, 143, 146, 255};
    Color major = {128, 128, 132, 255};
    for (int32_t k = -60; k <= 60; ++k)
    {
        Color c = (k % 10 == 0) ? major : minor;
        DrawLine3D((Vector3){(float)k, 0.0f, -60.0f}, (Vector3){(float)k, 0.0f, 60.0f}, c);
        DrawLine3D((Vector3){-60.0f, 0.0f, (float)k}, (Vector3){60.0f, 0.0f, (float)k}, c);
    }
    DrawLine3D((Vector3){-60.0f, 0.001f, 0.0f}, (Vector3){60.0f, 0.001f, 0.0f},
               (Color){170, 90, 90, 255});
    DrawLine3D((Vector3){0.0f, 0.001f, -60.0f}, (Vector3){0.0f, 0.001f, 60.0f},
               (Color){90, 90, 170, 255});
}

// -------------------------------------------------------------- scenes
typedef struct tbScene
{
    m3WorldId world;
    m3ShapeId chunk;    // the carvable chunk, if the scene has one
    m3CharacterId hero; // the playable walker, if the scene has one
    m3VehicleId car;    // the playable car, if the scene has one
    m3SoftBodyId jelly; // the star of the jelly scene
    m3SoftBodyId cloth; // the wind scene's sheet
    m3BodyId carBody;   // its chassis (camera and wheel draw)
    m3BodyId chaseBody; // camera chase target without a vehicle
    m3BodyId ferry;     // the host-driven platform, if any
    m3JointId axles[4]; // the joint cart's wheel joints
    bool geared;        // the drivetrain HUD reads gear and RPM
    m3WorldDef def;     // the world's recipe (replay verify rebuilds
                        // a probe with the same capacities)
    const char* name;
    const char* blurb;
} tbScene;

static m3WorldDef SceneDef(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 1024;
    def.shapeCapacity = 1024;
    def.jointCapacity = 32;
    def.voxelCapacity = 2;
    def.characterCapacity = 2;
    def.vehicleCapacity = 2;
    return def;
}

static void AddFloor(m3WorldId world)
{
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    m3CreatePlaneShape(ground, &sd, &floor);
}

// Scene 1: the keep. A hollow voxel fort; carve it, shoot it, rewind
// the whole battle.
static tbScene SceneKeep(void)
{
    tbScene scene;
    memset(&scene, 0, sizeof(scene));
    scene.name = "voxfort";
    scene.blurb = "E carves at the crosshair, left click shoots, R rewinds time";
    m3WorldDef def = SceneDef();
    scene.def = def;
    scene.world = m3CreateWorld(&def);
    AddFloor(scene.world);

    static uint8_t voxels[16 * 16 * 16];
    memset(voxels, 0, sizeof(voxels));
    for (int32_t z = 0; z < 16; ++z)
    {
        for (int32_t x = 0; x < 16; ++x)
        {
            voxels[x + 16 * (0 + 16 * z)] = 1;
        }
    }
    for (int32_t y = 1; y <= 9; ++y)
    {
        for (int32_t i = 2; i <= 13; ++i)
        {
            voxels[i + 16 * (y + 16 * 2)] = 1;
            voxels[i + 16 * (y + 16 * 13)] = 1;
            voxels[2 + 16 * (y + 16 * i)] = 1;
            voxels[13 + 16 * (y + 16 * i)] = 1;
        }
    }
    m3BodyDef gd = m3DefaultBodyDef();
    gd.position = (m3Pos3){-8.0, 0.0, -8.0};
    m3BodyId keep = m3CreateBody(scene.world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.friction = 0.6f;
    scene.chunk = m3CreateVoxelChunkShape(keep, &sd, voxels, NULL, 1.0f);
    return scene;
}

// Scene 1b: the blastyard. One call does the whole demolition
// (13-2/13-3): crates and barrels ring a voxel wall, X detonates at
// the reticle, the blast shoves every body by its facing area,
// carves the wall, and the freed islands fly out as kicked
// fragments. R rewinds the explosion, which is the whole point.
static tbScene SceneBlastyard(void)
{
    tbScene scene;
    memset(&scene, 0, sizeof(scene));
    scene.name = "blastyard";
    scene.blurb = "X detonates at the crosshair; carve, shove, and rewind it all";
    m3WorldDef def = SceneDef();
    scene.def = def;
    scene.world = m3CreateWorld(&def);
    AddFloor(scene.world);
    static uint8_t voxels[16 * 16 * 16];
    memset(voxels, 0, sizeof(voxels));
    for (int32_t y = 0; y < 10; ++y)
    {
        for (int32_t x = 1; x <= 14; ++x)
        {
            voxels[x + 16 * (y + 16 * 7)] = 1;
            voxels[x + 16 * (y + 16 * 8)] = 1;
        }
    }
    m3BodyDef wd = m3DefaultBodyDef();
    wd.position = (m3Pos3){-8.0, 0.0, -8.0};
    m3BodyId wall = m3CreateBody(scene.world, &wd);
    m3ShapeDef vd = m3DefaultShapeDef();
    vd.friction = 0.6f;
    scene.chunk = m3CreateVoxelChunkShape(wall, &vd, voxels, NULL, 1.0f);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.friction = 0.5f;
    for (int32_t i = 0; i < 10; ++i)
    {
        m3BodyDef bd = m3DefaultBodyDef();
        bd.type = m3_dynamicBody;
        double a = 0.628318530718 * (double)i;
        bd.position = (m3Pos3){5.5 * cos(a), 0.55, 4.0 + 4.5 * sin(a)};
        m3BodyId crate = m3CreateBody(scene.world, &bd);
        m3CreateBoxShape(crate, &sd, (m3Vec3){0.45f, 0.45f, 0.45f});
    }
    for (int32_t i = 0; i < 4; ++i)
    {
        m3BodyDef bd = m3DefaultBodyDef();
        bd.type = m3_dynamicBody;
        bd.position = (m3Pos3){-3.0 + 2.0 * (double)i, 0.5, 2.5};
        m3BodyId barrel = m3CreateBody(scene.world, &bd);
        m3Capsule keg = {{0.0f, -0.22f, 0.0f}, {0.0f, 0.22f, 0.0f}, 0.28f};
        m3CreateCapsuleShape(barrel, &sd, &keg);
    }
    for (int32_t i = 0; i < 3; ++i)
    {
        m3BodyDef bd = m3DefaultBodyDef();
        bd.type = m3_dynamicBody;
        bd.position = (m3Pos3){-2.0 + 2.0 * (double)i, 0.4, 6.5};
        m3BodyId ball = m3CreateBody(scene.world, &bd);
        m3Sphere orb = {{0.0f, 0.0f, 0.0f}, 0.4f};
        m3CreateSphereShape(ball, &sd, &orb);
    }
    return scene;
}

// Scene 2: the rain. A sphere pyramid takes a mixed-body rain.
static tbScene SceneRain(void)
{
    tbScene scene;
    memset(&scene, 0, sizeof(scene));
    scene.name = "rain";
    scene.blurb = "mixed bodies fall on a pyramid; B drops crates";
    m3WorldDef def = SceneDef();
    scene.def = def;
    scene.world = m3CreateWorld(&def);
    AddFloor(scene.world);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.friction = 0.5f;
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    for (int32_t layer = 0; layer < 6; ++layer)
    {
        int32_t n = 6 - layer;
        for (int32_t i = 0; i < n; ++i)
        {
            for (int32_t j = 0; j < n; ++j)
            {
                bd.position = (m3Pos3){(double)i - 0.5 * (double)n, 0.5 + 1.02 * (double)layer,
                                       (double)j - 0.5 * (double)n};
                m3BodyId body = m3CreateBody(scene.world, &bd);
                m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.5f};
                m3CreateSphereShape(body, &sd, &ball);
            }
        }
    }
    for (int32_t k = 0; k < 40; ++k)
    {
        bd.position =
            (m3Pos3){-6.0 + 0.31 * (double)k, 8.0 + 0.6 * (double)k, -5.0 + 0.27 * (double)(k % 7)};
        m3BodyId body = m3CreateBody(scene.world, &bd);
        if (k % 3 == 0)
        {
            m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.3f};
            m3CreateSphereShape(body, &sd, &ball);
        }
        else if (k % 3 == 1)
        {
            m3CreateBoxShape(body, &sd, (m3Vec3){0.3f, 0.3f, 0.3f});
        }
        else
        {
            m3Capsule capsule = {{-0.3f, 0.0f, 0.0f}, {0.3f, 0.0f, 0.0f}, 0.2f};
            m3CreateCapsuleShape(body, &sd, &capsule);
        }
    }
    return scene;
}

// Scene 3: the machine room. A motor door, a swinging chain, a
// shoulder cone, a slider on its rail.
static tbScene SceneMachines(void)
{
    tbScene scene;
    memset(&scene, 0, sizeof(scene));
    scene.name = "machines";
    scene.blurb = "joints at work: motor door, chain, shoulder, slider";
    m3WorldDef def = SceneDef();
    scene.def = def;
    scene.world = m3CreateWorld(&def);
    AddFloor(scene.world);
    m3ShapeDef sd = m3DefaultShapeDef();

    // The motor door.
    m3BodyDef fd = m3DefaultBodyDef();
    fd.position = (m3Pos3){-6.0, 2.0, 0.0};
    m3BodyId frame = m3CreateBody(scene.world, &fd);
    m3BodyDef dd = m3DefaultBodyDef();
    dd.type = m3_dynamicBody;
    dd.position = (m3Pos3){-5.0, 2.0, 0.0};
    m3BodyId door = m3CreateBody(scene.world, &dd);
    m3CreateBoxShape(door, &sd, (m3Vec3){1.0f, 1.8f, 0.1f});
    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_revoluteJoint;
    jd.bodyA = frame;
    jd.bodyB = door;
    jd.localAnchorB = (m3Vec3){-1.0f, 0.0f, 0.0f};
    jd.localAxisA = (m3Vec3){0.0f, 1.0f, 0.0f};
    jd.localAxisB = (m3Vec3){0.0f, 1.0f, 0.0f};
    jd.enableMotor = true;
    jd.motorSpeed = 1.2f;
    jd.maxMotorEffort = 60.0f;
    m3CreateJoint(&jd);

    // The chain: five spherical links from a high post.
    m3BodyDef pd = m3DefaultBodyDef();
    pd.position = (m3Pos3){0.0, 9.0, 0.0};
    m3BodyId post = m3CreateBody(scene.world, &pd);
    m3BodyId previous = post;
    for (int32_t k = 0; k < 5; ++k)
    {
        m3BodyDef ld = m3DefaultBodyDef();
        ld.type = m3_dynamicBody;
        ld.position = (m3Pos3){0.6 * (double)(k + 1), 9.0, 0.0};
        m3BodyId link = m3CreateBody(scene.world, &ld);
        m3Capsule seg = {{-0.25f, 0.0f, 0.0f}, {0.25f, 0.0f, 0.0f}, 0.09f};
        m3CreateCapsuleShape(link, &sd, &seg);
        m3JointDef cd = m3DefaultJointDef();
        cd.type = m3_sphericalJoint;
        cd.bodyA = previous;
        cd.bodyB = link;
        cd.localAnchorA = previous.index1 == post.index1 ? (m3Vec3){0.0f, 0.0f, 0.0f}
                                                         : (m3Vec3){0.3f, 0.0f, 0.0f};
        cd.localAnchorB = (m3Vec3){-0.3f, 0.0f, 0.0f};
        m3CreateJoint(&cd);
        previous = link;
    }

    // The shoulder: a cone-limited arm.
    m3BodyDef ad = m3DefaultBodyDef();
    ad.position = (m3Pos3){5.0, 6.0, 0.0};
    m3BodyId anchor = m3CreateBody(scene.world, &ad);
    m3BodyDef armDef = m3DefaultBodyDef();
    armDef.type = m3_dynamicBody;
    armDef.position = (m3Pos3){5.0, 5.2, 0.0};
    m3BodyId arm = m3CreateBody(scene.world, &armDef);
    m3Capsule armSeg = {{0.0f, 0.4f, 0.0f}, {0.0f, -0.4f, 0.0f}, 0.12f};
    m3CreateCapsuleShape(arm, &sd, &armSeg);
    m3JointDef shoulder = m3DefaultJointDef();
    shoulder.type = m3_sphericalJoint;
    shoulder.bodyA = anchor;
    shoulder.bodyB = arm;
    shoulder.localAnchorB = (m3Vec3){0.0f, 0.5f, 0.0f};
    shoulder.localAxisA = (m3Vec3){1.0f, 0.0f, 0.0f};
    shoulder.localAxisB = (m3Vec3){1.0f, 0.0f, 0.0f};
    shoulder.enableCone = true;
    shoulder.coneAngle = 0.7f;
    m3CreateJoint(&shoulder);

    // The slider: a motored cart on a rail.
    m3BodyDef rd = m3DefaultBodyDef();
    rd.position = (m3Pos3){0.0, 1.0, 6.0};
    m3BodyId rail = m3CreateBody(scene.world, &rd);
    m3BodyDef cd2 = m3DefaultBodyDef();
    cd2.type = m3_dynamicBody;
    cd2.position = (m3Pos3){0.0, 1.0, 6.0};
    m3BodyId cart = m3CreateBody(scene.world, &cd2);
    m3CreateBoxShape(cart, &sd, (m3Vec3){0.35f, 0.25f, 0.25f});
    m3JointDef sj = m3DefaultJointDef();
    sj.type = m3_prismaticJoint;
    sj.bodyA = rail;
    sj.bodyB = cart;
    sj.localAxisA = (m3Vec3){1.0f, 0.0f, 0.0f};
    sj.localAxisB = (m3Vec3){1.0f, 0.0f, 0.0f};
    sj.enableLimit = true;
    sj.lowerLimit = -4.0f;
    sj.upperLimit = 4.0f;
    sj.enableMotor = true;
    sj.motorSpeed = 1.5f;
    sj.maxMotorEffort = 40.0f;
    m3CreateJoint(&sj);

    // The gear pair (16-6): the motor door's hinge work echoed
    // through a 2:1 mesh; the small wheel spins twice as fast,
    // backwards.
    m3BodyDef gearFrame = m3DefaultBodyDef();
    gearFrame.position = (m3Pos3){-5.0, 5.5, -4.0};
    m3BodyId gearPost = m3CreateBody(scene.world, &gearFrame);
    m3BodyDef gd = m3DefaultBodyDef();
    gd.type = m3_dynamicBody;
    gd.position = (m3Pos3){-5.6, 5.5, -4.0};
    m3BodyId big = m3CreateBody(scene.world, &gd);
    m3CreateBoxShape(big, &sd, (m3Vec3){0.5f, 0.5f, 0.08f});
    gd.position = (m3Pos3){-4.4, 5.5, -4.0};
    m3BodyId small = m3CreateBody(scene.world, &gd);
    m3CreateBoxShape(small, &sd, (m3Vec3){0.25f, 0.25f, 0.08f});
    m3JointDef gh = m3DefaultJointDef();
    gh.type = m3_revoluteJoint;
    gh.bodyA = gearPost;
    gh.bodyB = big;
    gh.localAnchorA = (m3Vec3){-0.6f, 0.0f, 0.0f};
    gh.localAxisA = (m3Vec3){0.0f, 0.0f, 1.0f};
    gh.localAxisB = (m3Vec3){0.0f, 0.0f, 1.0f};
    gh.enableMotor = true;
    gh.motorSpeed = 1.0f;
    gh.maxMotorEffort = 30.0f;
    m3CreateJoint(&gh);
    gh.bodyB = small;
    gh.localAnchorA = (m3Vec3){0.6f, 0.0f, 0.0f};
    gh.enableMotor = false;
    m3CreateJoint(&gh);
    m3JointDef mesh = m3DefaultJointDef();
    mesh.type = m3_gearJoint;
    mesh.bodyA = big;
    mesh.bodyB = small;
    mesh.localAxisA = (m3Vec3){0.0f, 0.0f, 1.0f};
    mesh.localAxisB = (m3Vec3){0.0f, 0.0f, 1.0f};
    mesh.ratio = 0.5f; // big drives: small turns twice as fast
    m3CreateJoint(&mesh);

    // The pulley (16-6): a heavy crate and a light one trade rope
    // over two fixed points.
    m3BodyDef crateDef = m3DefaultBodyDef();
    crateDef.type = m3_dynamicBody;
    crateDef.position = (m3Pos3){8.0, 3.0, -4.0};
    m3BodyId heavy = m3CreateBody(scene.world, &crateDef);
    m3CreateBoxShape(heavy, &sd, (m3Vec3){0.45f, 0.45f, 0.45f});
    crateDef.position = (m3Pos3){10.0, 3.0, -4.0};
    m3BodyId light = m3CreateBody(scene.world, &crateDef);
    m3CreateBoxShape(light, &sd, (m3Vec3){0.25f, 0.25f, 0.25f});
    m3JointDef rope = m3DefaultJointDef();
    rope.type = m3_pulleyJoint;
    rope.bodyA = heavy;
    rope.bodyB = light;
    rope.groundAnchorA = (m3Pos3){8.0, 6.5, -4.0};
    rope.groundAnchorB = (m3Pos3){10.0, 6.5, -4.0};
    rope.ratio = 1.0f;
    m3CreateJoint(&rope);

    // The servo weld (16-5): a plate held in the air by pure
    // budgeted drive, no rows of steel.
    m3BodyDef servoAnchor = m3DefaultBodyDef();
    servoAnchor.position = (m3Pos3){0.0, 4.0, -6.0};
    m3BodyId servoPost = m3CreateBody(scene.world, &servoAnchor);
    m3BodyDef plateDef = m3DefaultBodyDef();
    plateDef.type = m3_dynamicBody;
    plateDef.position = (m3Pos3){0.0, 3.0, -6.0};
    m3BodyId plate = m3CreateBody(scene.world, &plateDef);
    m3CreateBoxShape(plate, &sd, (m3Vec3){0.6f, 0.08f, 0.6f});
    m3JointDef servo = m3DefaultJointDef();
    servo.type = m3_motorJoint;
    servo.bodyA = servoPost;
    servo.bodyB = plate;
    m3JointId hold = m3CreateJoint(&servo);
    m3Joint_SetSpring(hold, true, 5.0f, 1.0f);
    m3Joint_SetMotorPose(hold, (m3Vec3){0.0f, -1.0f, 0.0f},
                         (m3Quat){0.0f, 0.38268343f, 0.0f, 0.92387953f});
    return scene;
}

// Scene 4: the walker. Play the character controller inside a
// carvable fort: WASD walks, SPACE jumps, E carves the floor from
// under your own feet (airborne the same step), crates get shoved
// by the dt-free push contract, and the ferry platform carries you
// across the moat. Hold R and the WHOLE story rewinds, walker
// included.
static tbScene SceneWalker(void)
{
    tbScene scene;
    memset(&scene, 0, sizeof(scene));
    scene.name = "walker";
    scene.blurb = "WASD walk, SPACE jump, E carve under feet, ride the ferry";
    m3WorldDef def = SceneDef();
    scene.def = def;
    scene.world = m3CreateWorld(&def);
    AddFloor(scene.world);

    // The fort, same bones as the keep, with a stair ramp of voxel
    // steps up the south face so the walker can climb in.
    static uint8_t voxels[16 * 16 * 16];
    memset(voxels, 0, sizeof(voxels));
    for (int32_t z = 0; z < 16; ++z)
    {
        for (int32_t x = 0; x < 16; ++x)
        {
            voxels[x + 16 * (0 + 16 * z)] = 1;
        }
    }
    for (int32_t y = 1; y <= 6; ++y)
    {
        for (int32_t i = 2; i <= 13; ++i)
        {
            voxels[i + 16 * (y + 16 * 2)] = 1;
            voxels[i + 16 * (y + 16 * 13)] = 1;
            voxels[2 + 16 * (y + 16 * i)] = 1;
            voxels[13 + 16 * (y + 16 * i)] = 1;
        }
    }
    // The doorway: knock a 2 x 3 hole in the south wall.
    for (int32_t y = 1; y <= 3; ++y)
    {
        voxels[7 + 16 * (y + 16 * 2)] = 0;
        voxels[8 + 16 * (y + 16 * 2)] = 0;
    }
    m3BodyDef gd = m3DefaultBodyDef();
    gd.position = (m3Pos3){-8.0, 0.0, -8.0};
    m3BodyId keep = m3CreateBody(scene.world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.friction = 0.6f;
    scene.chunk = m3CreateVoxelChunkShape(keep, &sd, voxels, NULL, 1.0f);

    // Crates to shove around the yard.
    m3ShapeDef crateShape = m3DefaultShapeDef();
    crateShape.friction = 0.4f;
    for (int32_t k = 0; k < 3; ++k)
    {
        m3BodyDef bd = m3DefaultBodyDef();
        bd.type = m3_dynamicBody;
        bd.position = (m3Pos3){-2.0 + (double)k * 1.6, 1.46, -3.0};
        m3BodyId crate = m3CreateBody(scene.world, &bd);
        m3CreateBoxShape(crate, &crateShape, (m3Vec3){0.45f, 0.45f, 0.45f});
    }

    // The ferry: a host-driven kinematic platform shuttling along z.
    m3BodyDef fd = m3DefaultBodyDef();
    fd.type = m3_kinematicBody;
    fd.position = (m3Pos3){9.0, 0.6, 0.0};
    scene.ferry = m3CreateBody(scene.world, &fd);
    m3ShapeDef fsd = m3DefaultShapeDef();
    fsd.friction = 0.8f;
    m3CreateBoxShape(scene.ferry, &fsd, (m3Vec3){1.6f, 0.25f, 1.6f});
    m3Body_SetLinearVelocity(scene.ferry, (m3Vec3){0.0f, 0.0f, 1.2f});

    m3CharacterDef cd = m3DefaultCharacterDef();
    // The keep's voxel floor slab tops at y = 1: the spawn stands ON
    // it (the old y = 1.0 buried the capsule to the waist, invisible
    // in wireframe, obvious the moment the renderer went solid).
    cd.position = (m3Pos3){0.0, 2.2, 4.0};
    scene.hero = m3CreateCharacter(scene.world, &cd);
    return scene;
}

// Scene 5: the circuit. Drive the raycast car: WASD, SPACE
// handbrake, a ramp to jump and a fort wall to crash through, all
// of it rewindable.
static tbScene SceneCircuit(void)
{
    tbScene scene;
    memset(&scene, 0, sizeof(scene));
    scene.name = "circuit";
    scene.blurb = "W/S throttle, A/D steer, SPACE handbrake, crash the fort";
    m3WorldDef def = SceneDef();
    scene.def = def;
    scene.world = m3CreateWorld(&def);
    AddFloor(scene.world);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.friction = 0.6f;

    // The fort to crash through (unit cells, same bones as the keep).
    static uint8_t voxels[16 * 16 * 16];
    memset(voxels, 0, sizeof(voxels));
    for (int32_t z = 0; z < 16; ++z)
    {
        for (int32_t x = 0; x < 16; ++x)
        {
            voxels[x + 16 * (0 + 16 * z)] = 1;
        }
    }
    for (int32_t y = 1; y <= 5; ++y)
    {
        for (int32_t i = 2; i <= 13; ++i)
        {
            voxels[i + 16 * (y + 16 * 2)] = 1;
            voxels[i + 16 * (y + 16 * 13)] = 1;
            voxels[2 + 16 * (y + 16 * i)] = 1;
            voxels[13 + 16 * (y + 16 * i)] = 1;
        }
    }
    m3BodyDef kd = m3DefaultBodyDef();
    kd.position = (m3Pos3){4.0, 0.0, -8.0};
    m3BodyId keep = m3CreateBody(scene.world, &kd);
    scene.chunk = m3CreateVoxelChunkShape(keep, &sd, voxels, NULL, 1.0f);

    // A quarter-cell ramp west of the fort.
    static uint8_t rampVox[16 * 16 * 16];
    memset(rampVox, 0, sizeof(rampVox));
    for (int32_t z = 0; z < 16; ++z)
    {
        for (int32_t x = 0; x < 16; ++x)
        {
            int32_t top = x / 3;
            for (int32_t y = 0; y <= top; ++y)
            {
                rampVox[x + 16 * (y + 16 * z)] = 1;
            }
        }
    }
    m3BodyDef rd = m3DefaultBodyDef();
    rd.position = (m3Pos3){-18.0, 0.0, -2.0};
    m3CreateVoxelChunkShape(m3CreateBody(scene.world, &rd), &sd, rampVox, NULL, 0.25f);

    // Crates around the yard for the bumper.
    m3ShapeDef cs = m3DefaultShapeDef();
    cs.friction = 0.4f;
    for (int32_t k = 0; k < 6; ++k)
    {
        m3BodyDef bd = m3DefaultBodyDef();
        bd.type = m3_dynamicBody;
        bd.position = (m3Pos3){-6.0 + (double)(k % 3) * 2.0, 0.45, 2.0 + (double)(k / 3) * 2.0};
        m3CreateBoxShape(m3CreateBody(scene.world, &bd), &cs, (m3Vec3){0.4f, 0.4f, 0.4f});
    }

    // The car.
    m3BodyDef cd = m3DefaultBodyDef();
    cd.type = m3_dynamicBody;
    cd.position = (m3Pos3){-10.0, 1.2, 4.0};
    scene.carBody = m3CreateBody(scene.world, &cd);
    m3ShapeDef bodyShape = m3DefaultShapeDef();
    bodyShape.density = 300.0f;
    bodyShape.friction = 0.3f;
    m3CreateBoxShape(scene.carBody, &bodyShape, (m3Vec3){1.0f, 0.25f, 0.5f});
    m3VehicleDef vd = m3DefaultVehicleDef();
    vd.chassis = scene.carBody;
    vd.wheelCount = 4;
    for (int32_t w = 0; w < 4; ++w)
    {
        vd.wheels[w].anchor =
            (m3Vec3){(w & 1) != 0 ? 0.8f : -0.8f, -0.25f, (w & 2) != 0 ? 0.45f : -0.45f};
        vd.wheels[w].driven = true;
        vd.wheels[w].steerable = (w & 1) != 0;
    }
    scene.car = m3CreateVehicle(scene.world, &vd);
    return scene;
}

// Scene 6: the jelly. Drop a wobbling lattice on the keep, carve
// the wall out from under it, and rewind the whole custard.
static tbScene SceneJelly(void)
{
    tbScene scene;
    memset(&scene, 0, sizeof(scene));
    scene.name = "jelly";
    scene.blurb = "a soft lattice on the keep: E carves, LMB shoots, R rewinds";
    m3WorldDef def = SceneDef();
    def.softBodyCapacity = 2;
    scene.def = def;
    scene.world = m3CreateWorld(&def);
    AddFloor(scene.world);

    static uint8_t voxels[16 * 16 * 16];
    memset(voxels, 0, sizeof(voxels));
    for (int32_t z = 0; z < 16; ++z)
    {
        for (int32_t x = 0; x < 16; ++x)
        {
            voxels[x + 16 * (0 + 16 * z)] = 1;
        }
    }
    for (int32_t y = 1; y <= 6; ++y)
    {
        for (int32_t i = 2; i <= 13; ++i)
        {
            voxels[i + 16 * (y + 16 * 2)] = 1;
            voxels[i + 16 * (y + 16 * 13)] = 1;
            voxels[2 + 16 * (y + 16 * i)] = 1;
            voxels[13 + 16 * (y + 16 * i)] = 1;
        }
    }
    m3BodyDef gd = m3DefaultBodyDef();
    gd.position = (m3Pos3){-8.0, 0.0, -8.0};
    m3BodyId keep = m3CreateBody(scene.world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.friction = 0.6f;
    scene.chunk = m3CreateVoxelChunkShape(keep, &sd, voxels, NULL, 1.0f);

    m3SoftBodyDef sb = m3DefaultSoftBodyDef();
    // A GENTLE drop onto the rampart: particles are not bullets, and
    // a long fall moves them further per step than their own radius
    // (the Windows run watched the custard sink into the wall).
    sb.position = (m3Pos3){-5.6, 7.5, -0.6};
    sb.countX = 7;
    sb.countY = 7;
    sb.countZ = 7;
    sb.spacing = 0.22f;
    sb.compliance = 2.0e-4f;
    sb.radius = 0.1f;
    sb.particleMass = 0.08f;
    scene.jelly = m3CreateSoftBody(scene.world, &sb);
    return scene;
}

// Scene 7: the pyramid. The classic stack benchmark, watchable.
static tbScene ScenePyramid(void)
{
    tbScene scene;
    memset(&scene, 0, sizeof(scene));
    scene.name = "pyramid";
    scene.blurb = "the classic box pyramid; LMB shoots, R rewinds the collapse";
    m3WorldDef def = SceneDef();
    scene.def = def;
    scene.world = m3CreateWorld(&def);
    AddFloor(scene.world);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.friction = 0.6f;
    const int32_t base = 9;
    for (int32_t layer = 0; layer < base; ++layer)
    {
        int32_t n = base - layer;
        for (int32_t i = 0; i < n; ++i)
        {
            for (int32_t j = 0; j < n; ++j)
            {
                m3BodyDef bd = m3DefaultBodyDef();
                bd.type = m3_dynamicBody;
                bd.position = (m3Pos3){(double)i - 0.5 * (double)n, 0.4 + 0.81 * (double)layer,
                                       (double)j - 0.5 * (double)n};
                m3BodyId body = m3CreateBody(scene.world, &bd);
                m3CreateBoxShape(body, &sd, (m3Vec3){0.4f, 0.4f, 0.4f});
            }
        }
    }
    return scene;
}

// Scene 8: the tower. One tall stack and a wrecking ball on a chain.
static tbScene SceneTower(void)
{
    tbScene scene;
    memset(&scene, 0, sizeof(scene));
    scene.name = "tower";
    scene.blurb = "a tall stack and a wrecking ball; cut the chain with a shot";
    m3WorldDef def = SceneDef();
    scene.def = def;
    scene.world = m3CreateWorld(&def);
    AddFloor(scene.world);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.friction = 0.6f;
    for (int32_t k = 0; k < 14; ++k)
    {
        m3BodyDef bd = m3DefaultBodyDef();
        bd.type = m3_dynamicBody;
        bd.position = (m3Pos3){0.0, 0.45 + 0.91 * (double)k, 0.0};
        m3CreateBoxShape(m3CreateBody(scene.world, &bd), &sd, (m3Vec3){0.45f, 0.45f, 0.45f});
    }
    // The ball hangs from a post on a breakable distance rope.
    m3BodyDef pd = m3DefaultBodyDef();
    pd.position = (m3Pos3){-6.0, 14.0, 0.0};
    m3BodyId post = m3CreateBody(scene.world, &pd);
    m3BodyDef wd = m3DefaultBodyDef();
    wd.type = m3_dynamicBody;
    // Pulled back along -x so the pendulum swings THROUGH the post
    // line and arrives exactly at the tower (the first draft pulled
    // it sideways and it swung parallel to the stack forever).
    wd.position = (m3Pos3){-12.0, 8.0, 0.0};
    m3BodyId ball = m3CreateBody(scene.world, &wd);
    m3ShapeDef bs = m3DefaultShapeDef();
    bs.density = 8.0f;
    m3Sphere heavy = {{0.0f, 0.0f, 0.0f}, 0.9f};
    m3CreateSphereShape(ball, &bs, &heavy);
    m3JointDef jd = m3DefaultJointDef();
    jd.type = m3_distanceJoint;
    jd.bodyA = post;
    jd.bodyB = ball;
    jd.enableLimit = true;
    jd.lowerLimit = 0.0f;
    jd.upperLimit = 8.4f;
    m3JointId rope = m3CreateJoint(&jd);
    m3Joint_SetBreakThresholds(rope, 2200.0f, 0.0f);
    return scene;
}

// Scene 9: the hill. The geared car against a slope that top gear
// cannot climb: the 12-1 analytic, playable.
static tbScene SceneHill(void)
{
    tbScene scene;
    memset(&scene, 0, sizeof(scene));
    scene.name = "hillclimb";
    scene.blurb = "geared car: W throttle, A/D steer, Q reverse, watch the gear HUD";
    m3WorldDef def = SceneDef();
    scene.def = def;
    scene.world = m3CreateWorld(&def);
    AddFloor(scene.world);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.friction = 0.8f;
    // The hill: a SMOOTH hull wedge, not voxel stairs (the first
    // draft used one-meter cells and built a staircase taller than
    // the car). Sixteen degrees over twelve meters: first gear
    // climbs it, fifth gear bogs, exactly the 12-1 analytic.
    m3BodyDef rd = m3DefaultBodyDef();
    rd.position = (m3Pos3){4.0, 0.0, 0.0};
    m3BodyId hill = m3CreateBody(scene.world, &rd);
    m3Vec3 wedge[6] = {{0.0f, 0.0f, -5.0f}, {0.0f, 0.0f, 5.0f},   {12.0f, 0.0f, -5.0f},
                       {12.0f, 0.0f, 5.0f}, {12.0f, 3.5f, -5.0f}, {12.0f, 3.5f, 5.0f}};
    m3CreateHullShape(hill, &sd, wedge, 6);
    // The summit plateau, so the climb has a finish line.
    m3BodyDef td = m3DefaultBodyDef();
    td.position = (m3Pos3){20.0, 1.75, 0.0};
    m3CreateBoxShape(m3CreateBody(scene.world, &td), &sd, (m3Vec3){4.0f, 1.75f, 5.0f});

    m3BodyDef cd = m3DefaultBodyDef();
    cd.type = m3_dynamicBody;
    cd.position = (m3Pos3){-8.0, 1.2, 0.0};
    scene.carBody = m3CreateBody(scene.world, &cd);
    m3ShapeDef bodyShape = m3DefaultShapeDef();
    bodyShape.density = 300.0f;
    bodyShape.friction = 0.3f;
    m3CreateBoxShape(scene.carBody, &bodyShape, (m3Vec3){1.0f, 0.25f, 0.5f});
    m3VehicleDef vd = m3DefaultVehicleDef();
    vd.chassis = scene.carBody;
    vd.wheelCount = 4;
    vd.tireGrip = 2.2f;
    for (int32_t w = 0; w < 4; ++w)
    {
        vd.wheels[w].anchor =
            (m3Vec3){(w & 1) != 0 ? 0.8f : -0.8f, -0.25f, (w & 2) != 0 ? 0.45f : -0.45f};
        vd.wheels[w].driven = true;
        vd.wheels[w].steerable = (w & 1) != 0;
    }
    scene.car = m3CreateVehicle(scene.world, &vd);
    m3DrivetrainDef dt = m3DefaultDrivetrainDef();
    m3Vehicle_SetDrivetrain(scene.car, &dt); // auto shift on
    scene.geared = true;
    return scene;
}

// Scene 10: the cart. Rigid wheels on wheel JOINTS over rubble: no
// rays anywhere, every strike a contact, axles that can snap.
static tbScene SceneJointCart(void)
{
    tbScene scene;
    memset(&scene, 0, sizeof(scene));
    scene.name = "jointcart";
    scene.blurb = "wheel-joint cart: W/S drive, A/D steer; axles snap in the rubble";
    m3WorldDef def = SceneDef();
    scene.def = def;
    scene.world = m3CreateWorld(&def);
    AddFloor(scene.world);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.friction = 0.6f;
    sd.density = 400.0f;
    for (int32_t i = 0; i < 16; ++i)
    {
        m3BodyDef bd = m3DefaultBodyDef();
        bd.type = m3_dynamicBody;
        bd.position = (m3Pos3){4.0 + (double)(i % 4) * 1.1, 0.13, -1.6 + (double)(i / 4) * 1.0};
        m3CreateBoxShape(m3CreateBody(scene.world, &bd), &sd, (m3Vec3){0.24f, 0.12f, 0.24f});
    }
    m3BodyDef cd = m3DefaultBodyDef();
    cd.type = m3_dynamicBody;
    cd.position = (m3Pos3){-4.0, 0.66, 0.0};
    scene.chaseBody = m3CreateBody(scene.world, &cd);
    m3ShapeDef bodyShape = m3DefaultShapeDef();
    bodyShape.density = 300.0f;
    m3CreateBoxShape(scene.chaseBody, &bodyShape, (m3Vec3){1.0f, 0.25f, 0.5f});
    for (int32_t w = 0; w < 4; ++w)
    {
        m3Vec3 local = {(w & 1) != 0 ? 0.8f : -0.8f, -0.35f, (w & 2) != 0 ? 0.45f : -0.45f};
        m3BodyDef wd = m3DefaultBodyDef();
        wd.type = m3_dynamicBody;
        wd.position = (m3Pos3){-4.0 + (double)local.x, 0.66 + (double)local.y, (double)local.z};
        m3BodyId wheel = m3CreateBody(scene.world, &wd);
        m3ShapeDef ws = m3DefaultShapeDef();
        ws.density = 120.0f;
        ws.friction = 0.9f;
        m3Sphere tire = {{0.0f, 0.0f, 0.0f}, 0.3f};
        m3CreateSphereShape(wheel, &ws, &tire);
        m3JointDef jd = m3DefaultJointDef();
        jd.type = m3_wheelJoint;
        jd.bodyA = scene.chaseBody;
        jd.bodyB = wheel;
        jd.localAnchorA = local;
        jd.localAxisA = (m3Vec3){0.0f, -1.0f, 0.0f};
        jd.localAxisB = (m3Vec3){0.0f, 0.0f, 1.0f};
        jd.enableLimit = true;
        jd.lowerLimit = -0.1f;
        jd.upperLimit = 0.1f;
        scene.axles[w] = m3CreateJoint(&jd);
        m3Joint_SetSpring(scene.axles[w], true, 4.0f, 0.7f);
        m3Joint_SetTargetTranslation(scene.axles[w], 0.0f);
        // The break cap sits ABOVE the drive torque: plain driving
        // never snaps an axle (the first tuning had them inverted
        // and the cart shed wheels pulling away); rubble impacts
        // spike the collinearity torque past it honestly.
        m3Joint_SetBreakThresholds(scene.axles[w], 0.0f, 170.0f);
    }
    return scene;
}

// Scene 11: the tunnel. Crouch (C) under the slab, get REFUSED the
// stand while pressed, stand past the edge: the 12-3 veto, playable.
static tbScene SceneTunnel(void)
{
    tbScene scene;
    memset(&scene, 0, sizeof(scene));
    scene.name = "tunnel";
    scene.blurb = "WASD walk, C crouch: the stand-up veto refuses under the slab";
    m3WorldDef def = SceneDef();
    scene.def = def;
    scene.world = m3CreateWorld(&def);
    AddFloor(scene.world);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3BodyDef rd = m3DefaultBodyDef();
    rd.position = (m3Pos3){6.0, 1.55, 0.0};
    m3BodyId roof = m3CreateBody(scene.world, &rd);
    m3CreateBoxShape(roof, &sd, (m3Vec3){3.0f, 0.25f, 3.0f});
    rd.position = (m3Pos3){6.0, 0.75, 3.4};
    m3CreateBoxShape(m3CreateBody(scene.world, &rd), &sd, (m3Vec3){3.0f, 0.75f, 0.4f});
    rd.position = (m3Pos3){6.0, 0.75, -3.4};
    m3CreateBoxShape(m3CreateBody(scene.world, &rd), &sd, (m3Vec3){3.0f, 0.75f, 0.4f});
    m3CharacterDef cd = m3DefaultCharacterDef();
    cd.position = (m3Pos3){0.0, 1.0, 0.0};
    scene.hero = m3CreateCharacter(scene.world, &cd);
    return scene;
}

// Scene 12: the laundry line. A pinned cloth in gusting wind over a
// conveyor belt hauling crates: the 11-3 fields, playable.
static tbScene SceneClothWind(void)
{
    tbScene scene;
    memset(&scene, 0, sizeof(scene));
    scene.name = "clothwind";
    scene.blurb = "gusting wind on a pinned sheet, a belt hauling crates; F3 wind";
    m3WorldDef def = SceneDef();
    def.softBodyCapacity = 2;
    scene.def = def;
    scene.world = m3CreateWorld(&def);
    AddFloor(scene.world);

    m3SoftBodyDef sb = m3DefaultSoftBodyDef();
    sb.position = (m3Pos3){-3.0, 5.6, -1.4};
    sb.countX = 13;
    sb.countY = 1;
    sb.countZ = 9;
    sb.spacing = 0.24f;
    sb.compliance = 4.0e-4f;
    sb.radius = 0.07f;
    sb.particleMass = 0.05f;
    scene.cloth = m3CreateSoftBody(scene.world, &sb);
    // Pin the west edge: a hanging line, free to billow east.
    for (int32_t z = 0; z < 9; ++z)
    {
        m3SoftBody_PinParticle(scene.cloth, 0 + 13 * (0 + 1 * z));
    }
    m3World_SetWind(scene.world, (m3Vec3){1.0f, 0.0f, 0.15f}, 6.0f, 0.4f, 0.6f);

    // The belt: a static slab with a surface velocity, fed crates.
    m3BodyDef beltDef = m3DefaultBodyDef();
    beltDef.position = (m3Pos3){0.0, 0.25, 4.0};
    m3BodyId belt = m3CreateBody(scene.world, &beltDef);
    m3ShapeDef beltShape = m3DefaultShapeDef();
    beltShape.friction = 1.0f;
    m3ShapeId beltTop = m3CreateBoxShape(belt, &beltShape, (m3Vec3){7.0f, 0.25f, 1.2f});
    m3Shape_SetSurfaceVelocity(beltTop, (m3Vec3){1.6f, 0.0f, 0.0f});
    m3ShapeDef crateShape = m3DefaultShapeDef();
    crateShape.friction = 0.5f;
    for (int32_t k = 0; k < 5; ++k)
    {
        m3BodyDef bd = m3DefaultBodyDef();
        bd.type = m3_dynamicBody;
        bd.position = (m3Pos3){-6.0 + (double)k * 2.2, 0.95, 4.0};
        m3CreateBoxShape(m3CreateBody(scene.world, &bd), &crateShape,
                         (m3Vec3){0.35f, 0.35f, 0.35f});
    }
    return scene;
}

// Scene 13: the meadow. A rolling heightfield catching a hull rain:
// the 10-x geometry arc on stage.
static tbScene SceneMeadow(void)
{
    tbScene scene;
    memset(&scene, 0, sizeof(scene));
    scene.name = "meadow";
    scene.blurb = "hulls rain on a rolling heightfield; B drops crates";
    m3WorldDef def = SceneDef();
    scene.def = def;
    scene.world = m3CreateWorld(&def);
    static float heights[24 * 24];
    for (int32_t z = 0; z < 24; ++z)
    {
        for (int32_t x = 0; x < 24; ++x)
        {
            heights[x + 24 * z] = 0.9f * sinf(0.55f * (float)x) * cosf(0.45f * (float)z) +
                                  0.35f * sinf(1.3f * (float)(x + z));
        }
    }
    m3BodyDef gd = m3DefaultBodyDef();
    gd.position = (m3Pos3){-11.5, 0.0, -11.5};
    m3BodyId ground = m3CreateBody(scene.world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.friction = 0.7f;
    m3CreateHeightFieldShape(ground, &sd, heights, 24, 24, 1.0f);

    m3ShapeDef hd = m3DefaultShapeDef();
    hd.friction = 0.5f;
    for (int32_t k = 0; k < 30; ++k)
    {
        m3BodyDef bd = m3DefaultBodyDef();
        bd.type = m3_dynamicBody;
        bd.position = (m3Pos3){-7.0 + 0.47 * (double)k, 7.0 + 0.8 * (double)k,
                               -6.0 + 0.83 * (double)(k % 13)};
        m3BodyId rock = m3CreateBody(scene.world, &bd);
        m3Vec3 cloud[10];
        for (int32_t v = 0; v < 10; ++v)
        {
            double a = (double)(k * 10 + v) * 2.399963;
            cloud[v] =
                (m3Vec3){0.45f * (float)cos(a) * (0.6f + 0.4f * (float)((v * 37 % 10)) * 0.1f),
                         0.35f * (float)sin(a * 1.7),
                         0.45f * (float)sin(a) * (0.6f + 0.4f * (float)((v * 53 % 10)) * 0.1f)};
        }
        if (!m3Shape_IsValid(m3CreateHullShape(rock, &hd, cloud, 10)))
        {
            m3CreateBoxShape(rock, &hd, (m3Vec3){0.35f, 0.3f, 0.35f});
        }
    }
    return scene;
}

typedef struct tbSceneEntry
{
    tbScene (*build)(void);
    const char* category;
} tbSceneEntry;

static const tbSceneEntry s_scenes[] = {
    {ScenePyramid, "Benchmark"}, {SceneTower, "Contacts"},        {SceneRain, "Contacts"},
    {SceneKeep, "Destruction"},  {SceneBlastyard, "Destruction"}, {SceneJelly, "Destruction"},
    {SceneMachines, "Joints"},   {SceneJointCart, "Vehicles"},    {SceneCircuit, "Vehicles"},
    {SceneHill, "Vehicles"},     {SceneWalker, "Characters"},     {SceneTunnel, "Characters"},
    {SceneClothWind, "Soft"},    {SceneMeadow, "Geometry"},
};
#define SCENE_COUNT ((int32_t)(sizeof(s_scenes) / sizeof(s_scenes[0])))

// ------------------------------------------------------ host recipes
// The fragment recipe from the voxfort bench, verbatim in spirit:
// small islands become hulls of their voxel corner clouds, large
// ones become bounds boxes with density matched to the event mass.
// When a detonation armed the shove, newborn fragments inherit a
// radial kick from the blast center (13-3 demo dressing).
static bool s_blastArmed = false;
static m3Pos3 s_blastAt;

static void SpawnFragments(m3WorldId world)
{
    int32_t count = 0;
    const m3FragmentEvent* events = m3World_FragmentEvents(world, &count);
    int32_t recipeTotal = 0;
    const uint16_t* recipe = m3World_FragmentRecipe(world, &recipeTotal);
    for (int32_t e = 0; e < count && e < 24; ++e)
    {
        const m3FragmentEvent* ev = &events[e];
        m3BodyDef bd = m3DefaultBodyDef();
        bd.type = m3_dynamicBody;
        bd.position = ev->comWorld;
        if (s_blastArmed)
        {
            m3real dx = (m3real)(ev->comWorld.x - s_blastAt.x);
            m3real dy = (m3real)(ev->comWorld.y - s_blastAt.y);
            m3real dz = (m3real)(ev->comWorld.z - s_blastAt.z);
            m3real d = sqrtf(dx * dx + dy * dy + dz * dz);
            m3real inv = d > 1e-4f ? 1.0f / d : 0.0f;
            m3real speed = 7.0f / (1.0f + 0.3f * d);
            bd.linearVelocity =
                (m3Vec3){dx * inv * speed, dy * inv * speed + 2.0f, dz * inv * speed};
            bd.angularVelocity = (m3Vec3){dz * inv * 2.0f, 1.0f, -dx * inv * 2.0f};
        }
        m3BodyId body = m3CreateBody(world, &bd);
        m3Vec3 half = {0.5f * (m3real)(ev->boundsHi[0] - ev->boundsLo[0] + 1),
                       0.5f * (m3real)(ev->boundsHi[1] - ev->boundsLo[1] + 1),
                       0.5f * (m3real)(ev->boundsHi[2] - ev->boundsLo[2] + 1)};
        m3ShapeDef fd = m3DefaultShapeDef();
        fd.friction = 0.6f;
        if (ev->recipeCount > 0 && ev->recipeStart >= 0 && ev->voxelCount <= 32)
        {
            // A COMPOUND of unit boxes, one per voxel (10-1 offsets):
            // the fragment keeps its true silhouette. The old recipe
            // hulled small islands (a convex hull bevels an L into a
            // chipped cube) and boxed big ones (an L became a slab);
            // the Windows run caught both in one carve.
            fd.density = ev->mass / (m3real)ev->voxelCount;
            int32_t made = 0;
            for (int32_t k = 0; k < ev->recipeCount; ++k)
            {
                uint16_t v = recipe[ev->recipeStart + k];
                m3real x = (m3real)(v % 16);
                m3real y = (m3real)((v / 16) % 16);
                m3real z = (m3real)(v / 256);
                m3ShapeDef cell = fd;
                cell.localPosition = (m3Vec3){x + 0.5f - ev->comChunk.x, y + 0.5f - ev->comChunk.y,
                                              z + 0.5f - ev->comChunk.z};
                if (m3Shape_IsValid(m3CreateBoxShape(body, &cell, (m3Vec3){0.5f, 0.5f, 0.5f})))
                {
                    made += 1;
                }
            }
            if (made > 0)
            {
                continue;
            }
        }
        fd.density = ev->mass / (8.0f * half.x * half.y * half.z);
        m3CreateBoxShape(body, &fd, half);
    }
}

// ------------------------------------------------------------------ ui
// A tiny immediate-mode panel: rows, buttons, checkboxes, steppers.
// Enough for the Box3D-style side panel without a dependency.

typedef struct tbUi
{
    int x;
    int y;
    int w;
    bool hot; // the mouse is somewhere over the panel this frame
} tbUi;

static bool UiHit(int x, int y, int w, int h)
{
    Vector2 m = GetMousePosition();
    return m.x >= (float)x && m.x <= (float)(x + w) && m.y >= (float)y && m.y <= (float)(y + h);
}

static void UiHeader(tbUi* ui, const char* label)
{
    DrawRectangle(ui->x, ui->y, ui->w, 20, (Color){40, 44, 54, 255});
    DrawText(label, ui->x + 8, ui->y + 3, 14, (Color){208, 214, 226, 255});
    ui->y += 24;
}

static void UiLabel(tbUi* ui, const char* text, Color color)
{
    DrawText(text, ui->x + 10, ui->y, 13, color);
    ui->y += 18;
}

static bool UiButton(tbUi* ui, const char* label)
{
    int h = 20;
    bool over = UiHit(ui->x + 8, ui->y, ui->w - 16, h);
    DrawRectangle(ui->x + 8, ui->y, ui->w - 16, h,
                  over ? (Color){70, 78, 96, 255} : (Color){52, 58, 72, 255});
    DrawText(label, ui->x + 16, ui->y + 3, 13, RAYWHITE);
    ui->y += h + 4;
    return over && IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
}

static bool UiCheck(tbUi* ui, const char* label, bool* value)
{
    int h = 18;
    bool over = UiHit(ui->x + 8, ui->y, ui->w - 16, h);
    bool changed = false;
    if (over && IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
    {
        *value = !*value;
        changed = true;
    }
    DrawRectangleLines(ui->x + 10, ui->y + 2, 14, 14, (Color){150, 156, 170, 255});
    if (*value)
    {
        DrawRectangle(ui->x + 13, ui->y + 5, 8, 8, (Color){120, 190, 255, 255});
    }
    DrawText(label, ui->x + 32, ui->y + 2, 13, (Color){196, 200, 212, 255});
    ui->y += h + 3;
    return changed;
}

static bool UiStepper(tbUi* ui, const char* label, int32_t* value, int32_t lo, int32_t hi)
{
    int h = 18;
    bool changed = false;
    if (UiHit(ui->x + 8, ui->y, 18, h) && IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && *value > lo)
    {
        *value -= 1;
        changed = true;
    }
    if (UiHit(ui->x + 50, ui->y, 18, h) && IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && *value < hi)
    {
        *value += 1;
        changed = true;
    }
    DrawRectangle(ui->x + 8, ui->y, 18, h, (Color){52, 58, 72, 255});
    DrawText("-", ui->x + 14, ui->y + 2, 13, RAYWHITE);
    DrawText(TextFormat("%d", *value), ui->x + 33, ui->y + 2, 13, (Color){235, 220, 160, 255});
    DrawRectangle(ui->x + 50, ui->y, 18, h, (Color){52, 58, 72, 255});
    DrawText("+", ui->x + 56, ui->y + 2, 13, RAYWHITE);
    DrawText(label, ui->x + 78, ui->y + 2, 13, (Color){196, 200, 212, 255});
    ui->y += h + 3;
    return changed;
}

// ------------------------------------------------------ the recording
// The replay studio in the testbed: journal the whole scene life,
// seal it in an M3J1 container, write it to disk, and VERIFY it by
// replaying into a fresh world and comparing final hashes. The
// moat, clickable.

#define TB_JOURNAL_BYTES (4 * 1024 * 1024)
static uint8_t s_journal[TB_JOURNAL_BYTES];
static uint8_t* s_recordSnap = NULL;
static int32_t s_recordSnapBytes = 0;
static bool s_recording = false;
static char s_recordStatus[96] = "idle";
static m3WorldDef s_recordDef; // the probe recipe for verify

// Arm on the CURRENT world: the container's snapshot covers all
// state up to this moment and the journal covers everything after,
// which is exactly the M3J1 contract.
static void RecordArm(m3WorldId world, m3WorldDef def)
{
    s_recordDef = def;
    s_recordSnapBytes = m3World_SnapshotSize(world);
    free(s_recordSnap);
    s_recordSnap = (uint8_t*)malloc((size_t)s_recordSnapBytes);
    m3World_Snapshot(world, s_recordSnap, s_recordSnapBytes);
    s_recording = m3World_JournalBegin(world, s_journal, TB_JOURNAL_BYTES);
    snprintf(s_recordStatus, sizeof(s_recordStatus), s_recording ? "recording..." : "arm failed");
}

static void RecordSave(m3WorldId world)
{
    if (!s_recording)
    {
        return;
    }
    int32_t journalBytes = m3World_JournalEnd(world);
    s_recording = false;
    if (journalBytes <= 0)
    {
        snprintf(s_recordStatus, sizeof(s_recordStatus), "journal overflow");
        return;
    }
    uint64_t final = m3World_Hash(world);
    int32_t need = m3ReplayEncodeSize(s_recordSnapBytes, journalBytes);
    uint8_t* blob = (uint8_t*)malloc((size_t)need);
    int32_t wrote =
        m3ReplayEncode(s_recordSnap, s_recordSnapBytes, s_journal, journalBytes, final, blob, need);
    if (wrote != need)
    {
        free(blob);
        snprintf(s_recordStatus, sizeof(s_recordStatus), "encode failed");
        return;
    }
    FILE* f = fopen("testbed_recording.m3j", "wb");
    if (f != NULL)
    {
        fwrite(blob, 1, (size_t)wrote, f);
        fclose(f);
        snprintf(s_recordStatus, sizeof(s_recordStatus), "saved %d KB", wrote / 1024);
    }
    else
    {
        snprintf(s_recordStatus, sizeof(s_recordStatus), "file write failed");
    }
    free(blob);
}

static void RecordVerify(void)
{
    FILE* f = fopen("testbed_recording.m3j", "rb");
    if (f == NULL)
    {
        snprintf(s_recordStatus, sizeof(s_recordStatus), "no recording on disk");
        return;
    }
    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);
    uint8_t* blob = (uint8_t*)malloc((size_t)size);
    size_t got = fread(blob, 1, (size_t)size, f);
    fclose(f);
    m3ReplayView view;
    if (got != (size_t)size || !m3ReplayDecode(blob, (int32_t)size, &view))
    {
        free(blob);
        snprintf(s_recordStatus, sizeof(s_recordStatus), "decode refused");
        return;
    }
    m3WorldId probe = m3CreateWorld(&s_recordDef);
    bool ok = m3World_Restore(probe, view.snapshot, view.snapshotBytes) &&
              m3World_JournalReplay(probe, view.journal, view.journalBytes) &&
              m3World_Hash(probe) == view.finalHash;
    m3DestroyWorld(probe);
    free(blob);
    snprintf(s_recordStatus, sizeof(s_recordStatus),
             ok ? "VERIFIED bit-exact" : "DIVERGED (wrong build?)");
}

// --------------------------------------------------------------- main
int main(void)
{
    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_VSYNC_HINT);
    InitWindow(1360, 768, "Maul3D testbed");
    SetTargetFPS(60);

    int32_t sceneIndex = 0;
    tbScene scene = s_scenes[sceneIndex].build();
    bool paused = false;
    bool showAabbs = false;
    bool showContacts = false;
    bool showJoints = true;
    bool showWire = false;
    bool showIslands = false;
    bool showMassAxes = false;
    bool showTreeBoxes = false;
    bool showStats = false;
    bool showShadows = true;
    bool sleepTint = true;
    bool browser = false;
    bool windOn = true;
    int32_t substeps = 4;
    float stepMs = 0.0f;
    int32_t vetoFlash = 0;
    bool reverseGear = false;

    int32_t snapBytes = m3World_SnapshotSize(scene.world);
    enum
    {
        RING = 64
    };
    uint8_t* ring[RING];
    for (int32_t i = 0; i < RING; ++i)
    {
        ring[i] = (uint8_t*)malloc((size_t)snapBytes);
    }
    int32_t ringHead = 0;
    int32_t ringCount = 0;
    int32_t stepCounter = 0;

    while (!WindowShouldClose())
    {
        int screenW = GetScreenWidth();
        int screenH = GetScreenHeight();
        int panelX = screenW - 272;
        bool mouseInPanel = UiHit(panelX, 0, 272, screenH) || browser;

        // ---- scene switching
        bool rebuild = false;
        if (IsKeyPressed(KEY_TAB))
        {
            sceneIndex = (sceneIndex + 1) % SCENE_COUNT;
            rebuild = true;
        }
        if (IsKeyPressed(KEY_F5))
        {
            rebuild = true;
        }
        if (IsKeyPressed(KEY_ENTER))
        {
            browser = !browser;
        }
        if (rebuild)
        {
            s_recording = false;
            m3DestroyWorld(scene.world);
            scene = s_scenes[sceneIndex].build();
            snapBytes = m3World_SnapshotSize(scene.world);
            for (int32_t i = 0; i < RING; ++i)
            {
                free(ring[i]);
                ring[i] = (uint8_t*)malloc((size_t)snapBytes);
            }
            ringHead = 0;
            ringCount = 0;
            stepCounter = 0;
            windOn = true;
            reverseGear = false;
        }

        // ---- camera
        if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT))
        {
            Vector2 d = GetMouseDelta();
            s_yaw += d.x * 0.006f;
            s_pitch += d.y * 0.004f;
            if (s_pitch < 0.05f)
            {
                s_pitch = 0.05f;
            }
            if (s_pitch > 1.45f)
            {
                s_pitch = 1.45f;
            }
        }
        if (!mouseInPanel)
        {
            s_distance -= GetMouseWheelMove() * 1.6f;
        }
        if (s_distance < 4.0f)
        {
            s_distance = 4.0f;
        }
        if (s_distance > 90.0f)
        {
            s_distance = 90.0f;
        }
        float panSpeed = 0.25f;
        Vector3 forward = {-cosf(s_yaw), 0.0f, -sinf(s_yaw)};
        Vector3 rightv = {-forward.z, 0.0f, forward.x};
        bool walkerScene = m3Character_IsValid(scene.hero);
        bool driveScene = m3Vehicle_IsValid(scene.car);
        bool cartScene = !driveScene && m3Body_IsValid(scene.chaseBody);
        if (driveScene)
        {
            m3Pos3 cp = m3Body_GetPosition(scene.carBody);
            s_target = (Vector3){(float)cp.x, (float)cp.y + 1.0f, (float)cp.z};
        }
        else if (cartScene)
        {
            m3Pos3 cp = m3Body_GetPosition(scene.chaseBody);
            s_target = (Vector3){(float)cp.x, (float)cp.y + 1.0f, (float)cp.z};
        }
        else if (!walkerScene)
        {
            if (IsKeyDown(KEY_W))
            {
                s_target.x += forward.x * panSpeed;
                s_target.z += forward.z * panSpeed;
            }
            if (IsKeyDown(KEY_S))
            {
                s_target.x -= forward.x * panSpeed;
                s_target.z -= forward.z * panSpeed;
            }
            if (IsKeyDown(KEY_A))
            {
                s_target.x -= rightv.x * panSpeed;
                s_target.z -= rightv.z * panSpeed;
            }
            if (IsKeyDown(KEY_D))
            {
                s_target.x += rightv.x * panSpeed;
                s_target.z += rightv.z * panSpeed;
            }
        }
        else
        {
            m3Pos3 hp = m3Character_GetPosition(scene.hero);
            s_target = (Vector3){(float)hp.x, (float)hp.y + 0.8f, (float)hp.z};
        }
        Camera3D camera = OrbitCamera();

        // ---- interaction along the crosshair ray
        Ray pick = GetScreenToWorldRay(GetMousePosition(), camera);
        m3Pos3 rayOrigin = {pick.position.x, pick.position.y, pick.position.z};
        m3Vec3 rayDir = {pick.direction.x * 120.0f, pick.direction.y * 120.0f,
                         pick.direction.z * 120.0f};
        m3RayHit look = m3World_CastRayClosest(scene.world, rayOrigin, rayDir);

        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && !mouseInPanel)
        {
            m3BodyDef bd = m3DefaultBodyDef();
            bd.type = m3_dynamicBody;
            bd.isBullet = true;
            bd.position = rayOrigin;
            float speed = 70.0f;
            bd.linearVelocity = (m3Vec3){pick.direction.x * speed, pick.direction.y * speed,
                                         pick.direction.z * speed};
            m3BodyId bullet = m3CreateBody(scene.world, &bd);
            m3ShapeDef sd = m3DefaultShapeDef();
            sd.density = 4.0f;
            m3Sphere slug = {{0.0f, 0.0f, 0.0f}, 0.15f};
            m3CreateSphereShape(bullet, &sd, &slug);
        }
        if (IsKeyPressed(KEY_E) && walkerScene && m3Shape_IsValid(scene.chunk))
        {
            m3Pos3 hp = m3Character_GetPosition(scene.hero);
            int32_t cx = (int32_t)floor(hp.x + 8.0);
            int32_t cy = (int32_t)floor(hp.y - 0.93);
            int32_t cz = (int32_t)floor(hp.z + 8.0);
            if (cx >= 0 && cx <= 15 && cy >= 0 && cy <= 15 && cz >= 0 && cz <= 15)
            {
                int32_t lo[3] = {cx - 1 < 0 ? 0 : cx - 1, cy, cz - 1 < 0 ? 0 : cz - 1};
                int32_t hi[3] = {cx + 1 > 15 ? 15 : cx + 1, cy, cz + 1 > 15 ? 15 : cz + 1};
                if (m3VoxelChunk_ClearBox(scene.chunk, lo, hi) > 0)
                {
                    SpawnFragments(scene.world);
                }
            }
        }
        else if (IsKeyPressed(KEY_E) && look.hit && m3Shape_IsValid(scene.chunk) &&
                 look.shape.index1 == scene.chunk.index1)
        {
            int32_t cx = (int32_t)floor(look.point.x + 8.0);
            int32_t cy = (int32_t)floor(look.point.y);
            int32_t cz = (int32_t)floor(look.point.z + 8.0);
            int32_t lo[3] = {cx - 1 < 0 ? 0 : cx - 1, cy - 1 < 0 ? 0 : cy - 1,
                             cz - 1 < 0 ? 0 : cz - 1};
            int32_t hi[3] = {cx + 1 > 15 ? 15 : cx + 1, cy + 1 > 15 ? 15 : cy + 1,
                             cz + 1 > 15 ? 15 : cz + 1};
            if (m3VoxelChunk_ClearBox(scene.chunk, lo, hi) > 0)
            {
                SpawnFragments(scene.world);
            }
        }
        if (IsKeyPressed(KEY_X) && look.hit && !mouseInPanel)
        {
            // One call does the demolition (13-2/13-3): shove every
            // convex body by facing area, carve any chunk in range,
            // then dress the freed islands with the radial kick.
            m3ExplosionDef boom = m3DefaultExplosionDef();
            boom.position = look.point;
            boom.radius = 3.5f;
            boom.falloff = 2.5f;
            boom.impulsePerArea = 8.0f;
            boom.voxelCarve = 2.5f;
            m3World_Explode(scene.world, &boom);
            s_blastArmed = true;
            s_blastAt = look.point;
            SpawnFragments(scene.world);
            s_blastArmed = false;
        }
        if (IsKeyPressed(KEY_B) && look.hit && !mouseInPanel)
        {
            m3BodyDef bd = m3DefaultBodyDef();
            bd.type = m3_dynamicBody;
            bd.position = (m3Pos3){look.point.x, look.point.y + 4.0, look.point.z};
            m3BodyId crate = m3CreateBody(scene.world, &bd);
            m3ShapeDef sd = m3DefaultShapeDef();
            sd.friction = 0.5f;
            m3CreateBoxShape(crate, &sd, (m3Vec3){0.4f, 0.4f, 0.4f});
        }

        // ---- time and toggles
        if (IsKeyPressed(KEY_SPACE) && !walkerScene && !driveScene)
        {
            paused = !paused;
        }
        if (IsKeyPressed(KEY_P))
        {
            paused = !paused;
        }
        if (IsKeyPressed(KEY_F1))
        {
            showAabbs = !showAabbs;
        }
        if (IsKeyPressed(KEY_F2))
        {
            showContacts = !showContacts;
        }
        if (IsKeyPressed(KEY_F3) && m3SoftBody_IsValid(scene.cloth))
        {
            windOn = !windOn;
            m3World_SetWind(scene.world, (m3Vec3){1.0f, 0.0f, 0.15f}, windOn ? 6.0f : 0.0f, 0.4f,
                            0.6f);
        }

        // The rain drip (scene "rain" only).
        static int32_t dripCounter = 0;
        static int32_t dripped = 0;
        if (rebuild)
        {
            dripped = 0;
        }
        if (strcmp(scene.name, "rain") == 0 && !paused)
        {
            dripCounter += 1;
            if (dripCounter >= 30 && dripped < 600)
            {
                dripCounter = 0;
                dripped += 1;
                m3BodyDef bd = m3DefaultBodyDef();
                bd.type = m3_dynamicBody;
                double a = (double)dripped * 0.61803;
                bd.position = (m3Pos3){8.0 * sin(a * 6.2831), 20.0 + 2.0 * sin(a * 2.7),
                                       8.0 * cos(a * 6.2831)};
                m3BodyId body = m3CreateBody(scene.world, &bd);
                m3ShapeDef sd = m3DefaultShapeDef();
                sd.friction = 0.5f;
                if (dripped % 3 == 0)
                {
                    m3Sphere ball = {{0.0f, 0.0f, 0.0f}, 0.35f};
                    m3CreateSphereShape(body, &sd, &ball);
                }
                else if (dripped % 3 == 1)
                {
                    m3CreateBoxShape(body, &sd, (m3Vec3){0.3f, 0.3f, 0.3f});
                }
                else
                {
                    m3Capsule capsule = {{-0.3f, 0.0f, 0.0f}, {0.3f, 0.0f, 0.0f}, 0.2f};
                    m3CreateCapsuleShape(body, &sd, &capsule);
                }
            }
        }

        // The walker: WASD, SPACE jump, C crouch with the live veto.
        static float heroVy = 0.0f;
        static bool crouched = false;
        if (rebuild)
        {
            heroVy = 0.0f;
            crouched = false;
        }
        if (walkerScene && !paused)
        {
            float mx = (IsKeyDown(KEY_D) ? 1.0f : 0.0f) - (IsKeyDown(KEY_A) ? 1.0f : 0.0f);
            float mz = (IsKeyDown(KEY_W) ? 1.0f : 0.0f) - (IsKeyDown(KEY_S) ? 1.0f : 0.0f);
            float speed = IsKeyDown(KEY_LEFT_SHIFT) ? 7.0f : 4.0f;
            if (crouched)
            {
                speed *= 0.55f;
            }
            m3Vec3 wish = {forward.x * mz + rightv.x * mx, 0.0f, forward.z * mz + rightv.z * mx};
            float wl = sqrtf(wish.x * wish.x + wish.z * wish.z);
            if (wl > 0.0f)
            {
                wish.x *= speed / wl;
                wish.z *= speed / wl;
            }
            bool grounded = m3Character_IsGrounded(scene.hero);
            if (grounded && IsKeyPressed(KEY_SPACE))
            {
                heroVy = 5.2f;
            }
            else if (grounded && heroVy < 0.0f)
            {
                heroVy = -1.0f;
            }
            else
            {
                heroVy -= 9.8f / 60.0f;
            }
            if (IsKeyPressed(KEY_C))
            {
                if (!crouched)
                {
                    if (m3Character_SetStance(scene.hero, 0.2f, 0.4f))
                    {
                        crouched = true;
                    }
                }
                else
                {
                    if (m3Character_SetStance(scene.hero, 0.5f, 0.4f))
                    {
                        crouched = false;
                    }
                    else
                    {
                        vetoFlash = 50; // the stand-up veto, live
                    }
                }
            }
            m3Character_Move(scene.hero, (m3Vec3){wish.x / 60.0f, heroVy / 60.0f, wish.z / 60.0f});
        }

        // The raycast car (flat or geared): W/S throttle, A/D steer.
        if (driveScene && !paused)
        {
            if (scene.geared && IsKeyPressed(KEY_Q))
            {
                reverseGear = !reverseGear;
                m3Vehicle_SelectGear(scene.car, reverseGear ? -1 : 1);
            }
            float throttle;
            if (scene.geared)
            {
                throttle = IsKeyDown(KEY_W) ? 1.0f : 0.0f;
            }
            else
            {
                throttle = (IsKeyDown(KEY_W) ? 1.0f : 0.0f) - (IsKeyDown(KEY_S) ? 0.7f : 0.0f);
            }
            float steer = (IsKeyDown(KEY_A) ? 1.0f : 0.0f) - (IsKeyDown(KEY_D) ? 1.0f : 0.0f);
            float brake = scene.geared ? (IsKeyDown(KEY_S) ? 1.0f : 0.0f)
                                       : (IsKeyDown(KEY_SPACE) ? 1.0f : 0.0f);
            m3Vehicle_SetCommands(scene.car, throttle, steer, brake);
        }

        // The joint cart: spin the surviving axles, steer the
        // front pair about their struts (16-3).
        if (cartScene && !paused)
        {
            float spin = (IsKeyDown(KEY_W) ? -25.0f : 0.0f) + (IsKeyDown(KEY_S) ? 14.0f : 0.0f);
            float steer = (IsKeyDown(KEY_A) ? 0.45f : 0.0f) - (IsKeyDown(KEY_D) ? 0.45f : 0.0f);
            for (int32_t w = 0; w < 4; ++w)
            {
                if (m3Joint_IsValid(scene.axles[w]))
                {
                    m3Joint_SetMotor(scene.axles[w], spin != 0.0f, spin, 70.0f);
                    if ((w & 1) != 0)
                    {
                        // The +x pair leads under W's forward spin.
                        m3Joint_SetSteer(scene.axles[w], true, steer, 8.0f, 1.0f, 0.0f);
                    }
                }
            }
        }

        // The ferry.
        if (walkerScene && m3Body_IsValid(scene.ferry) && !paused)
        {
            m3Pos3 fp = m3Body_GetPosition(scene.ferry);
            if (fp.z > 6.0)
            {
                m3Body_SetLinearVelocity(scene.ferry, (m3Vec3){0.0f, 0.0f, -1.2f});
            }
            else if (fp.z < -6.0)
            {
                m3Body_SetLinearVelocity(scene.ferry, (m3Vec3){0.0f, 0.0f, 1.2f});
            }
        }

        bool rewinding = IsKeyDown(KEY_R) && ringCount > 0;
        if (rewinding)
        {
            ringHead = (ringHead + RING - 1) % RING;
            ringCount -= 1;
            m3World_Restore(scene.world, ring[ringHead], snapBytes);
        }
        else if ((!paused && !rewinding) || IsKeyPressed(KEY_F))
        {
            double t0 = GetTime();
            m3World_Step(scene.world, 1.0f / 60.0f, substeps);
            stepMs = 0.9f * stepMs + 0.1f * (float)((GetTime() - t0) * 1000.0);
            stepCounter += 1;
            if (stepCounter % 4 == 0 && !s_recording)
            {
                // The rewind ring pauses while a recording runs: a
                // restore would fork the journaled timeline.
                if (m3World_Snapshot(scene.world, ring[ringHead], snapBytes) == snapBytes)
                {
                    ringHead = (ringHead + 1) % RING;
                    if (ringCount < RING)
                    {
                        ringCount += 1;
                    }
                }
            }
        }
        if (vetoFlash > 0)
        {
            vetoFlash -= 1;
        }

        // ---- render
        BeginDrawing();
        // ClearBackground clears the DEPTH buffer, not just color:
        // skipping it leaves last frame's depth in place and the
        // whole scene shreds as the camera moves (the first Windows
        // run proved it). The sky gradient paints OVER the clear.
        ClearBackground((Color){164, 170, 184, 255});
        DrawSky(screenW, screenH);
        BeginMode3D(camera);
        DrawGround();

        m3SolidDraw solid;
        memset(&solid, 0, sizeof(solid));
        solid.DrawTriangle = SolidTriCb;
        solid.drawSleepTint = sleepTint;
        m3World_DrawSolid(scene.world, &solid);
        FlushSolid(showShadows);

        m3DebugDraw draw;
        memset(&draw, 0, sizeof(draw));
        draw.DrawSegment = DrawSegmentCb;
        draw.DrawPoint = DrawPointCb;
        draw.drawShapes = showWire;
        draw.drawJoints = showJoints;
        draw.drawSleepTint = sleepTint;
        draw.drawContacts = showContacts;
        draw.drawAabbs = showAabbs;
        m3World_Draw(scene.world, &draw);

        if (showIslands || showMassAxes || showTreeBoxes)
        {
            m3ExtraDraw extras;
            memset(&extras, 0, sizeof(extras));
            extras.DrawSegment = DrawSegmentCb;
            extras.DrawPoint = DrawPointCb;
            extras.drawIslands = showIslands;
            extras.drawMassAxes = showMassAxes;
            extras.drawTreeBoxes = showTreeBoxes;
            m3World_DrawExtras(scene.world, &extras);
        }

        for (int32_t soft = 0; soft < 2; ++soft)
        {
            m3SoftBodyId body = soft == 0 ? scene.jelly : scene.cloth;
            if (!m3SoftBody_IsValid(body))
            {
                continue;
            }
            Color tone = soft == 0 ? (Color){120, 205, 130, 255} : (Color){225, 215, 170, 255};
            int32_t pc = m3SoftBody_GetParticleCount(body);
            for (int32_t p = 0; p < pc; ++p)
            {
                m3Pos3 q = m3SoftBody_GetParticlePosition(body, p);
                DrawCubeV(FromPos(q), (Vector3){0.13f, 0.13f, 0.13f}, tone);
            }
        }
        if (driveScene)
        {
            m3Pos3 cp = m3Body_GetPosition(scene.carBody);
            m3Quat cq = m3Body_GetRotation(scene.carBody);
            for (int32_t w = 0; w < 4; ++w)
            {
                m3Vec3 anchor = {(w & 1) != 0 ? 0.8f : -0.8f, -0.25f,
                                 (w & 2) != 0 ? 0.45f : -0.45f};
                float suspLen = 0.4f - m3Vehicle_GetCompression(scene.car, w);
                m3Vec3 local = {anchor.x, anchor.y - suspLen, anchor.z};
                m3Vec3 world3 = m3RotateVec3(cq, local);
                Vector3 hub = {(float)cp.x + world3.x, (float)cp.y + world3.y,
                               (float)cp.z + world3.z};
                bool on = m3Vehicle_IsWheelGrounded(scene.car, w);
                DrawSphere(hub, 0.3f, on ? (Color){90, 96, 104, 255} : (Color){150, 150, 160, 255});
            }
        }
        if (look.hit && !mouseInPanel)
        {
            DrawSphere(FromPos(look.point), 0.09f, (Color){255, 240, 120, 255});
        }
        EndMode3D();

        // ---- the panel
        DrawRectangle(panelX, 0, 272, screenH, (Color){30, 33, 41, 236});
        tbUi ui = {panelX, 10, 272, false};
        UiLabel(&ui, TextFormat("%s", scene.name), (Color){235, 200, 120, 255});
        UiLabel(&ui, s_scenes[sceneIndex].category, (Color){150, 156, 170, 255});
        ui.y += 4;
        UiLabel(&ui, TextFormat("%.2f ms", (double)stepMs), (Color){120, 190, 255, 255});
        UiLabel(&ui, TextFormat("step %d", stepCounter), (Color){150, 200, 150, 255});
        if (scene.geared)
        {
            int32_t gear = m3Vehicle_GetGear(scene.car);
            UiLabel(&ui,
                    TextFormat("gear %s%d   rpm %.0f", gear < 0 ? "R" : "", gear < 0 ? -gear : gear,
                               (double)m3Vehicle_GetEngineRpm(scene.car)),
                    (Color){235, 220, 160, 255});
        }
        if (cartScene)
        {
            int32_t alive = 0;
            for (int32_t w = 0; w < 4; ++w)
            {
                alive += m3Joint_IsValid(scene.axles[w]) ? 1 : 0;
            }
            UiLabel(&ui, TextFormat("axles %d/4", alive), (Color){235, 220, 160, 255});
        }
        ui.y += 6;
        UiHeader(&ui, "Solver");
        UiStepper(&ui, "Sub-steps", &substeps, 1, 8);
        if (UiButton(&ui, paused ? "Resume" : "Pause"))
        {
            paused = !paused;
        }
        if (UiButton(&ui, "Single step"))
        {
            m3World_Step(scene.world, 1.0f / 60.0f, substeps);
            stepCounter += 1;
        }
        if (UiButton(&ui, "Restart"))
        {
            m3DestroyWorld(scene.world);
            scene = s_scenes[sceneIndex].build();
            snapBytes = m3World_SnapshotSize(scene.world);
            for (int32_t i = 0; i < RING; ++i)
            {
                free(ring[i]);
                ring[i] = (uint8_t*)malloc((size_t)snapBytes);
            }
            ringHead = 0;
            ringCount = 0;
            stepCounter = 0;
            s_recording = false;
            windOn = true;
            reverseGear = false;
        }
        ui.y += 4;
        UiHeader(&ui, "Draw");
        UiCheck(&ui, "Shadows", &showShadows);
        UiCheck(&ui, "Sleep tint", &sleepTint);
        UiCheck(&ui, "Wireframe overlay", &showWire);
        UiCheck(&ui, "Joints", &showJoints);
        UiCheck(&ui, "Contacts (F2)", &showContacts);
        UiCheck(&ui, "AABBs (F1)", &showAabbs);
        UiCheck(&ui, "Islands", &showIslands);
        UiCheck(&ui, "Mass axes", &showMassAxes);
        UiCheck(&ui, "Tree boxes", &showTreeBoxes);
        UiCheck(&ui, "Stats", &showStats);
        if (showStats)
        {
            m3Counters tc = m3World_GetCounters(scene.world);
            m3Profile tp = m3World_GetProfile(scene.world);
            UiLabel(&ui,
                    TextFormat("bodies %d awake %d contacts %d", tc.bodyCount, tc.awakeCount,
                               tc.contactCount),
                    (Color){170, 190, 220, 255});
            UiLabel(&ui,
                    TextFormat("islands %d colors %d tree h%d", tc.islandCount, tc.colorCount,
                               tc.treeHeight),
                    (Color){170, 190, 220, 255});
            UiLabel(&ui,
                    TextFormat("scratch %dk/%dk snap %dk", tc.scratchPeak / 1024,
                               tc.scratchCapacity / 1024, tc.snapshotBytes / 1024),
                    (Color){170, 190, 220, 255});
            UiLabel(&ui,
                    TextFormat("pairs %.2f contact %.2f solve %.2f", (double)tp.broadphase,
                               (double)tp.narrowphase, (double)tp.solve),
                    (Color){150, 200, 170, 255});
            UiLabel(&ui,
                    TextFormat("ccd %.2f soft %.2f step %.2f", (double)tp.continuous,
                               (double)tp.softBodies, (double)tp.step),
                    (Color){150, 200, 170, 255});
        }
        ui.y += 4;
        UiHeader(&ui, "Recording");
        UiLabel(&ui, s_recordStatus,
                s_recording ? (Color){255, 120, 120, 255} : (Color){170, 176, 190, 255});
        if (UiButton(&ui, "Record (restart)"))
        {
            // A fresh scene, then arm: the snapshot covers the build
            // and the journal covers the session from second zero.
            m3DestroyWorld(scene.world);
            scene = s_scenes[sceneIndex].build();
            snapBytes = m3World_SnapshotSize(scene.world);
            for (int32_t i = 0; i < RING; ++i)
            {
                free(ring[i]);
                ring[i] = (uint8_t*)malloc((size_t)snapBytes);
            }
            ringHead = 0;
            ringCount = 0;
            stepCounter = 0;
            windOn = true;
            reverseGear = false;
            RecordArm(scene.world, scene.def);
        }
        if (UiButton(&ui, "Save .m3j"))
        {
            RecordSave(scene.world);
        }
        if (UiButton(&ui, "Verify replay"))
        {
            RecordVerify();
        }
        ui.y += 4;
        UiHeader(&ui, "Scenes (ENTER)");
        if (UiButton(&ui, "Next scene (TAB)"))
        {
            sceneIndex = (sceneIndex + 1) % SCENE_COUNT;
            m3DestroyWorld(scene.world);
            scene = s_scenes[sceneIndex].build();
            snapBytes = m3World_SnapshotSize(scene.world);
            for (int32_t i = 0; i < RING; ++i)
            {
                free(ring[i]);
                ring[i] = (uint8_t*)malloc((size_t)snapBytes);
            }
            ringHead = 0;
            ringCount = 0;
            stepCounter = 0;
            s_recording = false;
        }

        // ---- the browser overlay
        if (browser)
        {
            static const char* names[] = {"pyramid",   "tower",     "rain",     "voxfort",
                                          "blastyard", "jelly",     "machines", "jointcart",
                                          "circuit",   "hillclimb", "walker",   "tunnel",
                                          "clothwind", "meadow"};
            int bx = screenW / 2 - 220;
            int by = 80;
            DrawRectangle(bx - 14, by - 12, 470, 40 + SCENE_COUNT * 24, (Color){22, 24, 30, 242});
            DrawText("Scenes", bx, by, 16, (Color){235, 200, 120, 255});
            for (int32_t k = 0; k < SCENE_COUNT; ++k)
            {
                int ry = by + 30 + k * 24;
                bool over = UiHit(bx - 8, ry - 3, 458, 22);
                if (over)
                {
                    DrawRectangle(bx - 8, ry - 3, 458, 22, (Color){52, 58, 72, 255});
                }
                DrawText(s_scenes[k].category, bx, ry, 14, (Color){140, 146, 160, 255});
                DrawText(">", bx + 110, ry, 14, (Color){100, 104, 116, 255});
                DrawText(names[k], bx + 130, ry, 14,
                         k == sceneIndex ? (Color){235, 200, 120, 255} : RAYWHITE);
                if (over && IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
                {
                    sceneIndex = k;
                    browser = false;
                    m3DestroyWorld(scene.world);
                    scene = s_scenes[sceneIndex].build();
                    snapBytes = m3World_SnapshotSize(scene.world);
                    for (int32_t i = 0; i < RING; ++i)
                    {
                        free(ring[i]);
                        ring[i] = (uint8_t*)malloc((size_t)snapBytes);
                    }
                    ringHead = 0;
                    ringCount = 0;
                    stepCounter = 0;
                    s_recording = false;
                }
            }
        }

        // ---- HUD
        DrawText(TextFormat("Maul3D testbed  [%s]", scene.name), 12, 12, 18,
                 (Color){40, 42, 50, 255});
        DrawText(scene.blurb, 12, 36, 14, (Color){70, 74, 86, 255});
        if (rewinding)
        {
            DrawText("<< REWINDING TIME (bit-exact) <<", 12, 60, 22, (Color){40, 120, 200, 255});
        }
        else if (paused)
        {
            DrawText("PAUSED", 12, 60, 22, (Color){200, 140, 40, 255});
        }
        if (vetoFlash > 0)
        {
            DrawText("STAND-UP VETOED: something presses", 12, 88, 20, (Color){200, 60, 60, 255});
        }
        DrawText("RMB orbit | wheel zoom | LMB shoot | B crate | E carve | TAB scene | "
                 "ENTER browser | hold R REWIND",
                 12, screenH - 26, 13, (Color){80, 84, 96, 255});
        EndDrawing();
    }

    for (int32_t i = 0; i < RING; ++i)
    {
        free(ring[i]);
    }
    free(s_recordSnap);
    m3DestroyWorld(scene.world);
    CloseWindow();
    return 0;
}
