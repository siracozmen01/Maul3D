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

#include "maul3d/draw.h"
#include "maul3d/joint.h"
#include "maul3d/shape.h"
#include "raylib.h"

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

// -------------------------------------------------------------- scenes
typedef struct tbScene
{
    m3WorldId world;
    m3ShapeId chunk; // the carvable chunk, if the scene has one
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

// Scene 2: the rain. A sphere pyramid takes a mixed-body rain.
static tbScene SceneRain(void)
{
    tbScene scene;
    memset(&scene, 0, sizeof(scene));
    scene.name = "rain";
    scene.blurb = "mixed bodies fall on a pyramid; B drops crates";
    m3WorldDef def = SceneDef();
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
    return scene;
}

typedef tbScene (*SceneBuilder)(void);
static const SceneBuilder s_builders[] = {SceneKeep, SceneRain, SceneMachines};
#define SCENE_COUNT ((int32_t)(sizeof(s_builders) / sizeof(s_builders[0])))

// ------------------------------------------------------ host recipes
// The fragment recipe from the voxfort bench, verbatim in spirit:
// small islands become hulls of their voxel corner clouds, large
// ones become bounds boxes with density matched to the event mass.
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
        m3BodyId body = m3CreateBody(world, &bd);
        m3Vec3 half = {0.5f * (m3real)(ev->boundsHi[0] - ev->boundsLo[0] + 1),
                       0.5f * (m3real)(ev->boundsHi[1] - ev->boundsLo[1] + 1),
                       0.5f * (m3real)(ev->boundsHi[2] - ev->boundsLo[2] + 1)};
        m3ShapeDef fd = m3DefaultShapeDef();
        fd.friction = 0.6f;
        fd.density = ev->mass / (8.0f * half.x * half.y * half.z);
        if (ev->voxelCount <= 8 && ev->recipeStart >= 0)
        {
            m3Vec3 cloud[64];
            int32_t points = 0;
            for (int32_t k = 0; k < ev->recipeCount; ++k)
            {
                uint16_t v = recipe[ev->recipeStart + k];
                m3real x = (m3real)(v % 16);
                m3real y = (m3real)((v / 16) % 16);
                m3real z = (m3real)(v / 256);
                for (int32_t c = 0; c < 8; ++c)
                {
                    cloud[points++] = (m3Vec3){x + ((c & 1) != 0 ? 1.0f : 0.0f) - ev->comChunk.x,
                                               y + ((c & 2) != 0 ? 1.0f : 0.0f) - ev->comChunk.y,
                                               z + ((c & 4) != 0 ? 1.0f : 0.0f) - ev->comChunk.z};
                }
            }
            if (m3Shape_IsValid(m3CreateHullShape(body, &fd, cloud, points)))
            {
                continue;
            }
        }
        m3CreateBoxShape(body, &fd, half);
    }
}

// --------------------------------------------------------------- main
int main(void)
{
    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_VSYNC_HINT);
    InitWindow(1280, 720, "Maul3D testbed");
    SetTargetFPS(60);

    int32_t sceneIndex = 0;
    tbScene scene = s_builders[sceneIndex]();
    bool paused = false;
    bool showAabbs = false;
    bool showContacts = false; // F2: hundreds of markers in a pile

    // The rewind ring: a snapshot every four steps, four seconds of
    // rewindable history. Holding R walks time backward; releasing
    // resumes forward simulation from wherever you stopped.
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
        if (rebuild)
        {
            m3DestroyWorld(scene.world);
            scene = s_builders[sceneIndex]();
            snapBytes = m3World_SnapshotSize(scene.world);
            for (int32_t i = 0; i < RING; ++i)
            {
                free(ring[i]);
                ring[i] = (uint8_t*)malloc((size_t)snapBytes);
            }
            ringHead = 0;
            ringCount = 0;
            stepCounter = 0;
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
        s_distance -= GetMouseWheelMove() * 1.6f;
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
        Camera3D camera = OrbitCamera();

        // ---- interaction along the crosshair ray
        Ray pick = GetScreenToWorldRay(GetMousePosition(), camera);
        m3Pos3 rayOrigin = {pick.position.x, pick.position.y, pick.position.z};
        m3Vec3 rayDir = {pick.direction.x * 120.0f, pick.direction.y * 120.0f,
                         pick.direction.z * 120.0f};
        m3RayHit look = m3World_CastRayClosest(scene.world, rayOrigin, rayDir);

        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
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
        if (IsKeyPressed(KEY_E) && look.hit && m3Shape_IsValid(scene.chunk) &&
            look.shape.index1 == scene.chunk.index1)
        {
            // The charge: world hit -> chunk coords (the keep sits at
            // (-8, 0, -8) with unit cells and identity rotation).
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
        if (IsKeyPressed(KEY_B) && look.hit)
        {
            m3BodyDef bd = m3DefaultBodyDef();
            bd.type = m3_dynamicBody;
            bd.position = (m3Pos3){look.point.x, look.point.y + 4.0, look.point.z};
            m3BodyId crate = m3CreateBody(scene.world, &bd);
            m3ShapeDef sd = m3DefaultShapeDef();
            sd.friction = 0.5f;
            m3CreateBoxShape(crate, &sd, (m3Vec3){0.4f, 0.4f, 0.4f});
        }

        // ---- time: pause, step, rewind, run
        if (IsKeyPressed(KEY_SPACE))
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
        // The rain never stops: scene two drips a fresh body every
        // half second so there is always motion to watch (and to
        // rewind), budgeted so the pool never runs dry.
        static int32_t dripCounter = 0;
        static int32_t dripped = 0;
        if (rebuild)
        {
            dripped = 0;
        }
        if (sceneIndex == 1 && !paused)
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

        bool rewinding = IsKeyDown(KEY_R) && ringCount > 0;
        if (rewinding)
        {
            // Walk backward through the ring, one snapshot per frame.
            ringHead = (ringHead + RING - 1) % RING;
            ringCount -= 1;
            m3World_Restore(scene.world, ring[ringHead], snapBytes);
        }
        else if ((!paused && !rewinding) || IsKeyPressed(KEY_F))
        {
            m3World_Step(scene.world, 1.0f / 60.0f, 4);
            stepCounter += 1;
            if (stepCounter % 4 == 0)
            {
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

        // ---- render
        BeginDrawing();
        ClearBackground((Color){24, 26, 32, 255});
        BeginMode3D(camera);
        DrawGrid(40, 1.0f);
        m3DebugDraw draw;
        memset(&draw, 0, sizeof(draw));
        draw.DrawSegment = DrawSegmentCb;
        draw.DrawPoint = DrawPointCb;
        draw.drawShapes = true;
        draw.drawJoints = true;
        draw.drawSleepTint = true;
        draw.drawContacts = showContacts;
        draw.drawAabbs = showAabbs;
        m3World_Draw(scene.world, &draw);
        if (look.hit)
        {
            DrawSphere(FromPos(look.point), 0.09f, (Color){255, 240, 120, 255});
        }
        EndMode3D();

        DrawText(TextFormat("Maul3D testbed [%s] %s", scene.name, scene.blurb), 12, 12, 18,
                 RAYWHITE);
        DrawText("RMB orbit | wheel zoom | WASD pan | LMB shoot | E carve | B crate | TAB scene "
                 "| SPACE pause | F step | hold R REWIND",
                 12, 36, 14, (Color){170, 176, 190, 255});
        if (rewinding)
        {
            DrawText("<< REWINDING TIME (bit-exact) <<", 12, 60, 22, (Color){130, 220, 255, 255});
        }
        else if (paused)
        {
            DrawText("PAUSED", 12, 60, 22, (Color){255, 200, 90, 255});
        }
        DrawFPS(1200, 12);
        EndDrawing();
    }

    for (int32_t i = 0; i < RING; ++i)
    {
        free(ring[i]);
    }
    m3DestroyWorld(scene.world);
    CloseWindow();
    return 0;
}
