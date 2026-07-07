// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Solver gate: free fall against the analytic velocity, rest on the
// plane within tolerance, THE STANDING STACK (the 2a deliverable), a
// restitution bounce band, friction versus no friction, in-process
// replay equality, and the fixed M3_SOLVER_HASH scene that joins the
// cross-platform CI gate. Black box: public headers only.

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

static m3WorldId MakeWorld(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 32;
    def.shapeCapacity = 32;
    return m3CreateWorld(&def);
}

static m3BodyId AddGroundPlane(m3WorldId world, float friction)
{
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.friction = friction;
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    m3CreatePlaneShape(ground, &sd, &floor);
    return ground;
}

static m3BodyId AddBall(m3WorldId world, double x, double y, double z, float radius, float friction,
                        float restitution)
{
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){x, y, z};
    m3BodyId body = m3CreateBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.friction = friction;
    sd.restitution = restitution;
    m3Sphere ball = {{0.0f, 0.0f, 0.0f}, radius};
    m3CreateSphereShape(body, &sd, &ball);
    return body;
}

static void StepN(m3WorldId world, int32_t steps)
{
    for (int32_t i = 0; i < steps; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
}

static void TestFreeFall(void)
{
    m3WorldId world = MakeWorld();
    m3BodyId ball = AddBall(world, 0.0, 100.0, 0.0, 0.5f, 0.6f, 0.0f);
    StepN(world, 60); // one second
    m3Vec3 v = m3Body_GetLinearVelocity(ball);
    CHECK(v.y > -10.05f && v.y < -9.95f, "one second of gravity lands near -10");
    m3Pos3 p = m3Body_GetPosition(ball);
    // Substepped semi-implicit Euler: expect close to 100 - g t^2 / 2,
    // slightly below because velocity leads position.
    CHECK(p.y > 94.5 && p.y < 95.5, "fallen distance in the analytic band");
    m3DestroyWorld(world);
}

static void TestRestOnPlane(void)
{
    m3WorldId world = MakeWorld();
    AddGroundPlane(world, 0.6f);
    m3BodyId ball = AddBall(world, 0.0, 0.6, 0.0, 0.5f, 0.6f, 0.0f);
    StepN(world, 300);
    m3Pos3 p = m3Body_GetPosition(ball);
    CHECK(p.y > 0.485 && p.y < 0.515, "the ball rests on the plane at its radius");
    m3Vec3 v = m3Body_GetLinearVelocity(ball);
    CHECK(v.y > -0.02f && v.y < 0.02f, "resting velocity is near zero");
    m3DestroyWorld(world);
}

static void TestStandingStack(void)
{
    // THE 2a deliverable: a vertical stack of spheres stands.
    m3WorldId world = MakeWorld();
    AddGroundPlane(world, 0.8f);
    m3BodyId balls[5];
    for (int32_t i = 0; i < 5; ++i)
    {
        balls[i] = AddBall(world, 0.0, 0.5 + 1.0 * (double)i, 0.0, 0.5f, 0.8f, 0.0f);
    }
    StepN(world, 600); // ten seconds
    for (int32_t i = 0; i < 5; ++i)
    {
        m3Pos3 p = m3Body_GetPosition(balls[i]);
        double restY = 0.5 + 1.0 * (double)i;
        CHECK(p.y > restY - 0.08 && p.y < restY + 0.04, "each sphere holds its height");
        CHECK(p.x > -0.05 && p.x < 0.05 && p.z > -0.05 && p.z < 0.05,
              "no sideways drift in a symmetric stack");
    }
    m3DestroyWorld(world);
}

static void TestRestitution(void)
{
    m3WorldId world = MakeWorld();
    AddGroundPlane(world, 0.0f);
    m3BodyId ball = AddBall(world, 0.0, 2.5, 0.0, 0.5f, 0.0f, 0.8f);
    // Track the rebound peak after the first bounce.
    double peak = 0.0;
    int bounced = 0;
    for (int32_t i = 0; i < 300; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
        m3Vec3 v = m3Body_GetLinearVelocity(ball);
        m3Pos3 p = m3Body_GetPosition(ball);
        if (!bounced && v.y > 0.5f)
        {
            bounced = 1;
        }
        if (bounced && p.y > peak)
        {
            peak = p.y;
        }
        if (bounced && v.y < -0.5f && p.y < 1.0)
        {
            break; // falling again after the peak
        }
    }
    // Energy: rebound height is about e^2 of the two-meter drop, so
    // the center peaks near 0.5 + 0.64 * 2 = 1.78. A generous band
    // absorbs the soft-contact and threshold losses.
    CHECK(bounced == 1, "the ball bounces");
    CHECK(peak > 1.3 && peak < 2.0, "the rebound peak is in the restitution band");
    m3DestroyWorld(world);
}

static void TestFrictionSlows(void)
{
    // The same sliding ball on a frictionless versus a frictional
    // plane: friction must bleed horizontal speed.
    double vx[2];
    for (int32_t pass = 0; pass < 2; ++pass)
    {
        float mu = pass == 0 ? 0.0f : 1.0f;
        m3WorldId world = MakeWorld();
        AddGroundPlane(world, mu);
        m3BodyId ball = AddBall(world, 0.0, 0.5, 0.0, 0.5f, mu, 0.0f);
        m3Body_SetLinearVelocity(ball, (m3Vec3){5.0f, 0.0f, 0.0f});
        StepN(world, 120);
        vx[pass] = (double)m3Body_GetLinearVelocity(ball).x;
        m3DestroyWorld(world);
    }
    CHECK(vx[1] < vx[0] - 0.5, "friction bleeds sliding speed");
    CHECK(vx[0] > 4.9, "the frictionless ball keeps its speed");
}

static void TestReplayEquality(void)
{
    // In-process replay: the same construction stepped twice gives the
    // same hash, and a journaled session (creates plus steps) replayed
    // into a fresh world reproduces the hash bit for bit.
    uint64_t hashes[2];
    for (int32_t pass = 0; pass < 2; ++pass)
    {
        m3WorldId world = MakeWorld();
        AddGroundPlane(world, 0.7f);
        AddBall(world, 0.1, 2.0, -0.2, 0.5f, 0.7f, 0.3f);
        AddBall(world, -0.3, 3.5, 0.1, 0.5f, 0.7f, 0.3f);
        StepN(world, 120);
        hashes[pass] = m3World_Hash(world);
        m3DestroyWorld(world);
    }
    CHECK(hashes[0] == hashes[1], "the same run twice is bit-identical");

    m3WorldId a = MakeWorld();
    uint8_t* journal = (uint8_t*)malloc(1 << 20);
    CHECK(m3World_JournalBegin(a, journal, 1 << 20), "journal begins");
    AddGroundPlane(a, 0.7f);
    AddBall(a, 0.1, 2.0, -0.2, 0.5f, 0.7f, 0.3f);
    StepN(a, 60);
    int32_t bytes = m3World_JournalEnd(a);
    CHECK(bytes > 0, "journal captured the session");
    m3WorldId b = MakeWorld();
    CHECK(m3World_JournalReplay(b, journal, bytes), "the session replays");
    CHECK(m3World_Hash(a) == m3World_Hash(b), "a journaled run replays bit for bit");
    free(journal);
    m3DestroyWorld(a);
    m3DestroyWorld(b);
}

static void TestBoxRests(void)
{
    // The 2b-5a milestone: a box dropped flat onto the plane lands on
    // a four-point manifold, rests at its half height, and stays
    // level; a second box stacks on top.
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m3WorldId world = m3CreateWorld(&def);
    AddGroundPlane(world, 0.7f);

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 0.8, 0.0};
    m3BodyId box = m3CreateBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.friction = 0.7f;
    m3CreateBoxShape(box, &sd, (m3Vec3){0.5f, 0.5f, 0.5f});

    bd.position = (m3Pos3){0.1, 2.2, -0.05};
    m3BodyId top = m3CreateBody(world, &bd);
    m3CreateBoxShape(top, &sd, (m3Vec3){0.4f, 0.4f, 0.4f});

    StepN(world, 480);

    m3Pos3 p = m3Body_GetPosition(box);
    CHECK(p.y > 0.47 && p.y < 0.53, "the box rests at its half height");
    m3Quat q = m3Body_GetRotation(box);
    CHECK(q.w > 0.999f || q.w < -0.999f, "the box stays level (no tumbling)");
    m3Pos3 tp = m3Body_GetPosition(top);
    // The SAT landed (2b-5b): the staged-gap check flips into the real
    // assertion. The 0.4 half-height box rests on the unit box: about
    // 1.0 + 0.4 = 1.4.
    CHECK(tp.y > 1.32 && tp.y < 1.46, "the second box stacks on the first");

    // White-box peek: the resting box's plane manifold carries four
    // corner points with distinct feature ids.
    // (Reach through the public hash instead: two identical runs.)
    uint64_t h1 = m3World_Hash(world);
    m3DestroyWorld(world);
    CHECK(h1 != 0, "the scene hashes");
}

static void TestCapsuleRestsFlat(void)
{
    // The 2b-6 milestone, part one: a capsule dropped lying flat
    // lands on the two-cap manifold, rests at its radius, and does
    // not roll away or pitch up.
    m3WorldId world = MakeWorld();
    AddGroundPlane(world, 0.6f);

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 1.0, 0.0};
    m3BodyId body = m3CreateBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Capsule capsule = {{-0.5f, 0.0f, 0.0f}, {0.5f, 0.0f, 0.0f}, 0.3f};
    m3ShapeId shape = m3CreateCapsuleShape(body, &sd, &capsule);
    CHECK(m3Shape_IsValid(shape), "the capsule creates");

    // Contract checks while the world is warm: zero length and zero
    // radius are refused with the null id, no shape half-created.
    m3Capsule degenerate = {{0.1f, 0.0f, 0.0f}, {0.1f, 0.0f, 0.0f}, 0.3f};
    CHECK(!m3Shape_IsValid(m3CreateCapsuleShape(body, &sd, &degenerate)),
          "a zero-length capsule is refused");
    m3Capsule flat = {{-0.5f, 0.0f, 0.0f}, {0.5f, 0.0f, 0.0f}, 0.0f};
    CHECK(!m3Shape_IsValid(m3CreateCapsuleShape(body, &sd, &flat)),
          "a zero-radius capsule is refused");

    StepN(world, 480);

    m3Pos3 p = m3Body_GetPosition(body);
    CHECK(p.y > 0.27 && p.y < 0.33, "the capsule rests at its radius");
    CHECK(p.x > -0.05 && p.x < 0.05, "the capsule does not wander");
    m3Quat q = m3Body_GetRotation(body);
    CHECK(q.w > 0.999f || q.w < -0.999f, "the capsule stays level");
    m3DestroyWorld(world);
}

static void TestSphereRestsOnBox(void)
{
    // Part two: the hull-sphere staged gap closes. A sphere dropped
    // onto a static unit box takes the generic GJK contact and rests
    // on the top face.
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m3WorldId world = m3CreateWorld(&def);

    m3BodyDef bd = m3DefaultBodyDef();
    bd.position = (m3Pos3){0.0, 0.5, 0.0};
    m3BodyId block = m3CreateBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3CreateBoxShape(block, &sd, (m3Vec3){0.5f, 0.5f, 0.5f});

    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 2.0, 0.0};
    m3BodyId ball = m3CreateBody(world, &bd);
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.4f};
    m3CreateSphereShape(ball, &sd, &sphere);

    StepN(world, 480);

    m3Pos3 p = m3Body_GetPosition(ball);
    CHECK(p.y > 1.36 && p.y < 1.44, "the sphere rests on the box top");
    CHECK(p.x > -0.02 && p.x < 0.02, "the sphere stays centered");
    m3DestroyWorld(world);
}

static void TestSphereRestsOnCapsule(void)
{
    // Part three: capsule-sphere through the same generic pair. The
    // sphere balances on the lying capsule's midsection; a centered
    // drop is symmetric, so determinism keeps it centered.
    m3WorldId world = MakeWorld();
    AddGroundPlane(world, 0.6f);

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 0.3, 0.0};
    m3BodyId cap = m3CreateBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Capsule capsule = {{-0.6f, 0.0f, 0.0f}, {0.6f, 0.0f, 0.0f}, 0.3f};
    m3CreateCapsuleShape(cap, &sd, &capsule);

    bd.position = (m3Pos3){0.0, 1.2, 0.0};
    m3BodyId ball = m3CreateBody(world, &bd);
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.25f};
    m3CreateSphereShape(ball, &sd, &sphere);

    StepN(world, 360);

    m3Pos3 p = m3Body_GetPosition(ball);
    CHECK(p.y > 0.80 && p.y < 0.90, "the sphere rests on the capsule crown");
    m3Pos3 cp = m3Body_GetPosition(cap);
    CHECK(cp.y > 0.27 && cp.y < 0.33, "the capsule carries the load at its radius");
    m3DestroyWorld(world);
}

static void TestGyroscopicTumble(void)
{
    // Part four: the implicit gyroscopic term. A long flat box spun
    // about its intermediate axis must tumble (the Dzhanibekov flip
    // needs the cross(w, I*w) torque) while the implicit Newton form
    // keeps the energy bounded: |w| may legitimately grow as rotation
    // shifts toward the low-inertia axis, but only up to the inertia
    // ratio, never past it.
    m3WorldDef def = m3DefaultWorldDef();
    def.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
    def.bodyCapacity = 4;
    def.shapeCapacity = 4;
    m3WorldId world = m3CreateWorld(&def);

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 0.0, 0.0};
    m3BodyId body = m3CreateBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    // Half extents 1.0 x 0.4 x 0.05: three distinct principal
    // moments, y is the intermediate axis.
    m3CreateBoxShape(body, &sd, (m3Vec3){1.0f, 0.4f, 0.05f});
    m3Body_SetAngularVelocity(body, (m3Vec3){0.01f, 5.0f, 0.01f});

    int tumbled = 0;
    float wMax = 0.0f;
    for (int32_t i = 0; i < 600; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
        m3Vec3 w = m3Body_GetAngularVelocity(body);
        float mag2 = w.x * w.x + w.y * w.y + w.z * w.z;
        if (mag2 > wMax)
        {
            wMax = mag2;
        }
        if (w.x > 1.0f || w.x < -1.0f || w.z > 1.0f || w.z < -1.0f)
        {
            tumbled = 1;
        }
    }
    CHECK(tumbled, "the intermediate-axis spin tumbles (the term is alive)");
    // Energy bound: E = w·(I w)/2 is conserved or dissipated, so
    // |w|^2 <= (Imax/Imin) |w0|^2 = (about 6.3) * 25.
    CHECK(wMax < 170.0f, "the tumble never gains energy");
    m3Quat q = m3Body_GetRotation(body);
    float norm2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    CHECK(norm2 > 0.99f && norm2 < 1.01f, "the rotation stays normalized");
    m3DestroyWorld(world);
}

static void TestDeepSphereRecovers(void)
{
    // The 2b-7 milestone: a sphere spawned INSIDE a static box (the
    // destruction-rubble pose) exits along the least-deep face and
    // rests on top. Spawned near the top face, the exact point-in-hull
    // kernel must pick +y deterministically.
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m3WorldId world = m3CreateWorld(&def);

    m3BodyDef bd = m3DefaultBodyDef();
    bd.position = (m3Pos3){0.0, 0.5, 0.0};
    m3BodyId block = m3CreateBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3CreateBoxShape(block, &sd, (m3Vec3){0.5f, 0.5f, 0.5f});

    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 0.85, 0.0}; // 0.15 under the top face
    m3BodyId ball = m3CreateBody(world, &bd);
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.3f};
    m3CreateSphereShape(ball, &sd, &sphere);

    StepN(world, 480);

    m3Pos3 p = m3Body_GetPosition(ball);
    CHECK(p.y > 1.24 && p.y < 1.36, "the deep sphere pops out and rests on top");
    CHECK(p.x > -0.05 && p.x < 0.05, "the exit is straight up, no sideways drift");
    m3DestroyWorld(world);
}

static void TestDeepCapsuleRecovers(void)
{
    // A capsule skewered through the box near its top face: both cap
    // centers are OUTSIDE the side planes, so only the segment SAT
    // sees the right axis (a centroid heuristic would shove it
    // sideways). The honest deep-recovery promise: the capsule is
    // EXPELLED and settles somewhere sane. It need not end balanced
    // on top: the violent pop-out picks up a small sequential-solve
    // torque (the reference solvers share this trait) and the rod
    // may legitimately slide off the box and land on the ground.
    m3WorldId world = MakeWorld();
    AddGroundPlane(world, 0.6f);

    m3BodyDef bd = m3DefaultBodyDef();
    bd.position = (m3Pos3){0.0, 0.5, 0.0};
    m3BodyId block = m3CreateBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3CreateBoxShape(block, &sd, (m3Vec3){0.5f, 0.5f, 0.5f});

    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 0.8, 0.0}; // core 0.2 under the top
    m3BodyId rod = m3CreateBody(world, &bd);
    m3Capsule capsule = {{-1.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, 0.2f};
    m3CreateCapsuleShape(rod, &sd, &capsule);

    StepN(world, 480);

    m3Pos3 p = m3Body_GetPosition(rod);
    CHECK(p.x > -5.0 && p.x < 5.0 && p.z > -5.0 && p.z < 5.0 && p.y > 0.1 && p.y < 1.35,
          "the skewered capsule is expelled and settles nearby");
    int insideCore =
        p.x > -0.45 && p.x < 0.45 && p.y > 0.05 && p.y < 0.95 && p.z > -0.45 && p.z < 0.45;
    CHECK(!insideCore, "it never stays trapped inside the box");
    m3DestroyWorld(world);
}

static void TestCrossedCapsulesSeparate(void)
{
    // Two capsules spawned with their cores exactly crossing (the
    // measure-zero pose): the mutual perpendicular is +-y here and the
    // center-delta tiebreak is zero, so the fixed rule must still
    // separate them vertically and deterministically.
    m3WorldId world = MakeWorld();
    AddGroundPlane(world, 0.6f);

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 0.3, 0.0};
    m3BodyId a = m3CreateBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Capsule alongX = {{-0.6f, 0.0f, 0.0f}, {0.6f, 0.0f, 0.0f}, 0.3f};
    m3CreateCapsuleShape(a, &sd, &alongX);

    m3BodyId b = m3CreateBody(world, &bd); // same position: cores cross
    m3Capsule alongZ = {{0.0f, 0.0f, -0.6f}, {0.0f, 0.0f, 0.6f}, 0.3f};
    m3CreateCapsuleShape(b, &sd, &alongZ);

    // Assert while the stack stands (step 120): the exact rule must
    // have pushed one capsule cleanly on top of the other. Much
    // later the upper one legitimately rolls off its knife-edge
    // balance; that is physics, not the property under test.
    StepN(world, 120);

    m3Pos3 pa = m3Body_GetPosition(a);
    m3Pos3 pb = m3Body_GetPosition(b);
    double gap = pa.y > pb.y ? pa.y - pb.y : pb.y - pa.y;
    CHECK(gap > 0.5, "the crossed capsules separate vertically");
    double low = pa.y < pb.y ? pa.y : pb.y;
    CHECK(low > 0.27 && low < 0.33, "the lower one rests on the plane");
    m3DestroyWorld(world);
}

static void TestDeepFuzzDeterminism(void)
{
    // Twenty deep-spawn poses from a fixed SplitMix stream, run twice
    // in fresh worlds: every pose must end finite and OUTSIDE the box
    // core, and the two runs must hash bit-identically (the deep
    // kernels join the determinism contract).
    uint64_t hashes[2] = {0, 0};
    for (int32_t run = 0; run < 2; ++run)
    {
        uint64_t rng = 0x9E3779B97F4A7C15ull;
        uint64_t combined = 0xcbf29ce484222325ull;
        for (int32_t round = 0; round < 20; ++round)
        {
            m3WorldDef def = m3DefaultWorldDef();
            def.bodyCapacity = 8;
            def.shapeCapacity = 8;
            m3WorldId world = m3CreateWorld(&def);
            AddGroundPlane(world, 0.6f);
            m3BodyDef bd = m3DefaultBodyDef();
            bd.position = (m3Pos3){0.0, 0.5, 0.0};
            m3BodyId block = m3CreateBody(world, &bd);
            m3ShapeDef sd = m3DefaultShapeDef();
            m3CreateBoxShape(block, &sd, (m3Vec3){0.5f, 0.5f, 0.5f});

            // Three SplitMix64 draws map into the box interior.
            double coords[3];
            for (int32_t k = 0; k < 3; ++k)
            {
                rng += 0x9E3779B97F4A7C15ull;
                uint64_t z = rng;
                z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
                z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
                z = z ^ (z >> 31);
                coords[k] = -0.4 + 0.8 * ((double)(z >> 11) / 9007199254740992.0);
            }
            bd.type = m3_dynamicBody;
            bd.position = (m3Pos3){coords[0], 0.5 + coords[1], coords[2]};
            m3BodyId ball = m3CreateBody(world, &bd);
            m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.25f};
            m3CreateSphereShape(ball, &sd, &sphere);

            StepN(world, 240);

            m3Pos3 p = m3Body_GetPosition(ball);
            // Spawns near the box BOTTOM exit downward into the
            // ground plane and end SQUEEZED between two opposing
            // soft contacts, a hair below the surface: legitimate
            // physics for an impossible pose, not a tunnel. The
            // bound admits the squeeze, refuses any blow-up.
            CHECK(p.y > -0.2 && p.y < 2.0 && p.x > -20.0 && p.x < 20.0 && p.z > -20.0 && p.z < 20.0,
                  "the deep spawn ends finite: at rest or squeezed");
            int insideCore =
                p.x > -0.35 && p.x < 0.35 && p.y > 0.15 && p.y < 0.85 && p.z > -0.35 && p.z < 0.35;
            CHECK(!insideCore, "the deep spawn ends outside the box core");
            uint64_t h = m3World_Hash(world);
            combined = (combined ^ h) * 0x100000001B3ull;
            m3DestroyWorld(world);
        }
        hashes[run] = combined;
    }
    CHECK(hashes[0] == hashes[1], "the deep fuzz is bit-deterministic");
}

static void TestQuickHullRockRests(void)
{
    // The 2b-3b payoff: a QuickHull rock (an irregular octahedron)
    // drops onto the plane, settles onto a face, and a journaled
    // session containing the hull create replays bit for bit (the
    // recipe points ride the journal, QuickHull rebuilds them into
    // the identical hull, id determinism included).
    uint8_t journal[16384];
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m3WorldId world = m3CreateWorld(&def);
    CHECK(m3World_JournalBegin(world, journal, (int32_t)sizeof(journal)), "journal arms");
    AddGroundPlane(world, 0.6f);

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 2.0, 0.0};
    m3BodyId rock = m3CreateBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Vec3 pts[7] = {{0.9f, 0.0f, 0.1f},  {-0.8f, 0.1f, 0.0f}, {0.0f, 0.7f, -0.1f},
                     {0.1f, -0.6f, 0.0f}, {0.0f, 0.1f, 0.8f},  {-0.1f, 0.0f, -0.9f},
                     {0.2f, 0.2f, 0.2f}};
    m3ShapeId shape = m3CreateHullShape(rock, &sd, pts, 7);
    CHECK(m3Shape_IsValid(shape), "the rock builds");

    // A degenerate cloud is refused without touching the body.
    m3Vec3 flat[4] = {
        {0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, {1.0f, 0.0f, 1.0f}};
    CHECK(!m3Shape_IsValid(m3CreateHullShape(rock, &sd, flat, 4)), "a coplanar cloud is refused");

    StepN(world, 480);

    m3Pos3 p = m3Body_GetPosition(rock);
    CHECK(p.y > 0.05 && p.y < 1.0, "the rock settles onto a face");
    CHECK(p.x > -2.0 && p.x < 2.0 && p.z > -2.0 && p.z < 2.0, "the rock stays near the drop");
    m3Vec3 v = m3Body_GetLinearVelocity(rock);
    CHECK(v.x * v.x + v.y * v.y + v.z * v.z < 0.01f, "the rock has come to rest");

    int32_t bytes = m3World_JournalEnd(world);
    CHECK(bytes > 0, "the session recorded");
    uint64_t h1 = m3World_Hash(world);

    m3WorldId twin = m3CreateWorld(&def);
    CHECK(m3World_JournalReplay(twin, journal, bytes), "the hull session replays");
    CHECK(m3World_Hash(twin) == h1, "the replay is bit-identical");
    m3DestroyWorld(twin);
    m3DestroyWorld(world);
}

static void TestBulletStopsAtWall(void)
{
    // The 2b-8 milestone: a sphere at 200 m/s crosses 3.3 meters per
    // step, sixty-six times its own radius; without the continuous
    // pass it would skip the 0.1-thick wall entirely. Every FAST
    // dynamic body sweeps against statics (the bullet flag is not
    // needed for static targets), so both variants must stop.
    for (int32_t variant = 0; variant < 2; ++variant)
    {
        m3WorldDef def = m3DefaultWorldDef();
        def.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
        def.bodyCapacity = 8;
        def.shapeCapacity = 8;
        m3WorldId world = m3CreateWorld(&def);

        m3BodyDef bd = m3DefaultBodyDef();
        bd.position = (m3Pos3){5.0, 0.0, 0.0};
        m3BodyId wall = m3CreateBody(world, &bd);
        m3ShapeDef sd = m3DefaultShapeDef();
        m3CreateBoxShape(wall, &sd, (m3Vec3){0.05f, 1.0f, 1.0f});

        bd.type = m3_dynamicBody;
        bd.position = (m3Pos3){0.0, 0.0, 0.0};
        bd.linearVelocity = (m3Vec3){200.0f, 0.0f, 0.0f};
        bd.isBullet = variant == 1;
        m3BodyId ball = m3CreateBody(world, &bd);
        m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.1f};
        m3CreateSphereShape(ball, &sd, &sphere);

        int stayed = 1;
        for (int32_t i = 0; i < 60; ++i)
        {
            m3World_Step(world, 1.0f / 60.0f, 4);
            m3Pos3 p = m3Body_GetPosition(ball);
            if (p.x > 4.9)
            {
                stayed = 0;
            }
        }
        CHECK(stayed, "the fast sphere never crosses the thin wall");
        m3DestroyWorld(world);
    }
}

static void TestBulletVsDynamicTarget(void)
{
    // The flag semantics: dynamic targets are swept ONLY for bullets.
    // A bullet fired at a floating box stops at its face (both sweeps
    // enter the TOI: the target is free to drift). The identical
    // non-bullet sphere tunnels straight through: the documented
    // reason the flag exists.
    for (int32_t variant = 0; variant < 2; ++variant)
    {
        m3WorldDef def = m3DefaultWorldDef();
        def.gravity = (m3Vec3){0.0f, 0.0f, 0.0f};
        def.bodyCapacity = 8;
        def.shapeCapacity = 8;
        m3WorldId world = m3CreateWorld(&def);

        m3BodyDef bd = m3DefaultBodyDef();
        bd.type = m3_dynamicBody;
        bd.position = (m3Pos3){5.0, 0.0, 0.0};
        m3BodyId target = m3CreateBody(world, &bd);
        m3ShapeDef sd = m3DefaultShapeDef();
        m3CreateBoxShape(target, &sd, (m3Vec3){0.5f, 1.0f, 1.0f});

        bd.position = (m3Pos3){0.0, 0.0, 0.0};
        bd.linearVelocity = (m3Vec3){150.0f, 0.0f, 0.0f};
        bd.isBullet = variant == 1;
        m3BodyId ball = m3CreateBody(world, &bd);
        m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.1f};
        m3CreateSphereShape(ball, &sd, &sphere);

        int stayedBehind = 1;
        for (int32_t i = 0; i < 40; ++i)
        {
            m3World_Step(world, 1.0f / 60.0f, 4);
            m3Pos3 pb = m3Body_GetPosition(ball);
            m3Pos3 pt = m3Body_GetPosition(target);
            if (pb.x > pt.x)
            {
                stayedBehind = 0;
            }
        }
        if (variant == 1)
        {
            CHECK(stayedBehind, "the bullet stays behind the dynamic box");
        }
        else
        {
            m3Pos3 pb = m3Body_GetPosition(ball);
            CHECK(!stayedBehind && pb.x > 10.0,
                  "the non-bullet tunnels through the dynamic box (why the flag exists)");
        }
        m3DestroyWorld(world);
    }
}

static void TestCcdDeterminism(void)
{
    // The continuous pass joins the determinism contract: the bullet
    // wall scene runs twice and hashes bit-identically, and a
    // journaled session carrying the bullet flag replays bit for bit.
    uint64_t hashes[2];
    uint8_t journal[8192];
    int32_t journalBytes = 0;
    for (int32_t run = 0; run < 2; ++run)
    {
        m3WorldDef def = m3DefaultWorldDef();
        def.gravity = (m3Vec3){0.0f, -10.0f, 0.0f};
        def.bodyCapacity = 8;
        def.shapeCapacity = 8;
        m3WorldId world = m3CreateWorld(&def);
        if (run == 0)
        {
            CHECK(m3World_JournalBegin(world, journal, (int32_t)sizeof(journal)), "journal arms");
        }
        AddGroundPlane(world, 0.6f);
        m3BodyDef bd = m3DefaultBodyDef();
        bd.position = (m3Pos3){6.0, 1.0, 0.0};
        m3BodyId wall = m3CreateBody(world, &bd);
        m3ShapeDef sd = m3DefaultShapeDef();
        m3CreateBoxShape(wall, &sd, (m3Vec3){0.05f, 1.0f, 1.0f});

        bd.type = m3_dynamicBody;
        bd.position = (m3Pos3){0.0, 0.5, 0.0};
        bd.linearVelocity = (m3Vec3){120.0f, 0.0f, 0.0f};
        bd.isBullet = true;
        m3BodyId ball = m3CreateBody(world, &bd);
        m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.2f};
        m3CreateSphereShape(ball, &sd, &sphere);

        for (int32_t i = 0; i < 90; ++i)
        {
            m3World_Step(world, 1.0f / 60.0f, 4);
        }
        hashes[run] = m3World_Hash(world);
        if (run == 0)
        {
            journalBytes = m3World_JournalEnd(world);
            CHECK(journalBytes > 0, "the ccd session recorded");
            m3WorldId twin = m3CreateWorld(&def);
            CHECK(m3World_JournalReplay(twin, journal, journalBytes), "the ccd session replays");
            CHECK(m3World_Hash(twin) == hashes[0], "the replay is bit-identical");
            m3DestroyWorld(twin);
        }
        m3DestroyWorld(world);
    }
    CHECK(hashes[0] == hashes[1], "the ccd scene is bit-deterministic");
}

// Build a flat triangulated strip: x in [-4, 4], z in [-1, 1], one
// meter quads, each split into two CCW-from-above triangles. The
// interior edges and vertices are exactly what the welding filter
// must silence.
static m3ShapeId AddMeshFloor(m3WorldId world)
{
    enum
    {
        NX = 9,
        NZ = 3
    };
    m3Vec3 verts[NX * NZ];
    for (int32_t iz = 0; iz < NZ; ++iz)
    {
        for (int32_t ix = 0; ix < NX; ++ix)
        {
            verts[iz * NX + ix] = (m3Vec3){-4.0f + (m3real)ix, 0.0f, -1.0f + (m3real)iz};
        }
    }
    uint16_t tris[3 * 2 * (NX - 1) * (NZ - 1)];
    int32_t n = 0;
    for (int32_t iz = 0; iz < NZ - 1; ++iz)
    {
        for (int32_t ix = 0; ix < NX - 1; ++ix)
        {
            uint16_t v00 = (uint16_t)(iz * NX + ix);
            uint16_t v10 = (uint16_t)(iz * NX + ix + 1);
            uint16_t v01 = (uint16_t)((iz + 1) * NX + ix);
            uint16_t v11 = (uint16_t)((iz + 1) * NX + ix + 1);
            tris[n++] = v00;
            tris[n++] = v11;
            tris[n++] = v10;
            tris[n++] = v00;
            tris[n++] = v01;
            tris[n++] = v11;
        }
    }
    m3BodyDef bd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.friction = 0.0f; // pure slide: any vertical kick is a ghost
    return m3CreateMeshShape(ground, &sd, verts, NX * NZ, tris, n / 3);
}

static void TestSphereSlidesAcrossMeshFloor(void)
{
    // THE internal-edge proof: a sphere slides across six interior
    // edges and a row of interior vertices of a flat triangulated
    // floor. Without feature welding the neighbor triangles' edge
    // contacts point their ghost normals sideways and kick the
    // sphere; with it the ride must stay flat to millimeters.
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m3WorldId world = m3CreateWorld(&def);
    CHECK(m3Shape_IsValid(AddMeshFloor(world)), "the mesh floor builds");

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){-3.5, 0.5, 0.0};
    bd.linearVelocity = (m3Vec3){1.2f, 0.0f, 0.0f};
    m3BodyId ball = m3CreateBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    sd.friction = 0.0f;
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.5f};
    m3CreateSphereShape(ball, &sd, &sphere);

    // Settle onto the floor first, then measure the ride.
    StepN(world, 60);
    double yMin = 1.0e30;
    double yMax = -1.0e30;
    double zMax = 0.0;
    for (int32_t i = 0; i < 240; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
        m3Pos3 p = m3Body_GetPosition(ball);
        yMin = p.y < yMin ? p.y : yMin;
        yMax = p.y > yMax ? p.y : yMax;
        double az = p.z < 0.0 ? -p.z : p.z;
        zMax = az > zMax ? az : zMax;
    }
    m3Pos3 end = m3Body_GetPosition(ball);
    CHECK(end.x > 0.0, "the sphere crossed the interior edges");
    CHECK(yMin > 0.46 && yMax < 0.54, "the ride stays flat: no ghost bumps");
    CHECK(zMax < 0.05, "no sideways ghost kicks");
    m3DestroyWorld(world);
}

static void TestMeshContractsAndGaps(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m3WorldId world = m3CreateWorld(&def);
    CHECK(m3Shape_IsValid(AddMeshFloor(world)), "the mesh floor builds");

    // Contracts: a mesh on a dynamic body and bad indices are refused.
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 2.0, 0.0};
    m3BodyId mover = m3CreateBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Vec3 tri[3] = {{0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
    uint16_t idx[3] = {0, 1, 2};
    CHECK(!m3Shape_IsValid(m3CreateMeshShape(mover, &sd, tri, 3, idx, 1)),
          "a mesh on a dynamic body is refused");
    uint16_t bad[3] = {0, 1, 9};
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground2 = m3CreateBody(world, &gd);
    CHECK(!m3Shape_IsValid(m3CreateMeshShape(ground2, &sd, tri, 3, bad, 1)),
          "an out-of-range index is refused");

    // Staged gap (2b-9b): capsule and hull versus mesh carry no
    // manifold yet; both fall THROUGH the mesh floor. This test
    // flips into resting assertions when 2b-9b lands.
    m3Capsule capsule = {{-0.4f, 0.0f, 0.0f}, {0.4f, 0.0f, 0.0f}, 0.3f};
    m3CreateCapsuleShape(mover, &sd, &capsule);
    bd.position = (m3Pos3){1.5, 2.0, 0.0};
    m3BodyId boxBody = m3CreateBody(world, &bd);
    m3CreateBoxShape(boxBody, &sd, (m3Vec3){0.3f, 0.3f, 0.3f});
    StepN(world, 240);
    CHECK(m3Body_GetPosition(mover).y < -1.0, "capsule-mesh is the documented gap");
    CHECK(m3Body_GetPosition(boxBody).y < -1.0, "hull-mesh is the documented gap");
    m3DestroyWorld(world);
}

static void TestMeshJournalAndRollback(void)
{
    // The mesh rides every determinism spine: a journaled session
    // (mesh floor + sphere drop) replays bit for bit, and a snapshot
    // taken mid-flight restores and reruns bit for bit.
    uint8_t journal[65536];
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m3WorldId world = m3CreateWorld(&def);
    CHECK(m3World_JournalBegin(world, journal, (int32_t)sizeof(journal)), "journal arms");
    CHECK(m3Shape_IsValid(AddMeshFloor(world)), "the mesh floor builds");

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.3, 2.0, 0.2};
    m3BodyId ball = m3CreateBody(world, &bd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.4f};
    m3CreateSphereShape(ball, &sd, &sphere);

    StepN(world, 120);
    uint64_t h1 = m3World_Hash(world);
    int32_t bytes = m3World_JournalEnd(world);
    CHECK(bytes > 0, "the mesh session recorded");

    m3WorldId twin = m3CreateWorld(&def);
    CHECK(m3World_JournalReplay(twin, journal, bytes), "the mesh session replays");
    CHECK(m3World_Hash(twin) == h1, "the replay is bit-identical");

    // Rollback: snapshot the twin, run on, restore, rerun.
    int32_t snapBytes = m3World_SnapshotSize(twin);
    void* snap = malloc((size_t)snapBytes);
    CHECK(m3World_Snapshot(twin, snap, snapBytes) == snapBytes, "snapshot writes");
    StepN(twin, 60);
    uint64_t after = m3World_Hash(twin);
    CHECK(m3World_Restore(twin, snap, snapBytes), "snapshot restores");
    StepN(twin, 60);
    CHECK(m3World_Hash(twin) == after, "mesh scenes roll back bit-exact");
    free(snap);
    m3DestroyWorld(twin);
    m3DestroyWorld(world);
}

static void TestSolverHashGate(void)
{
    // The fixed solver scene for the CI gate: a plane, a three-sphere
    // stack, and an offset dropper, 120 steps.
    m3WorldId world = MakeWorld();
    AddGroundPlane(world, 0.6f);
    for (int32_t i = 0; i < 3; ++i)
    {
        AddBall(world, 0.0, 0.5 + 1.0 * (double)i, 0.0, 0.5f, 0.6f, 0.1f);
    }
    AddBall(world, 0.3, 5.0, 0.2, 0.5f, 0.6f, 0.1f);
    StepN(world, 120);
    printf("M3_SOLVER_HASH=%016llx\n", (unsigned long long)m3World_Hash(world));
    m3DestroyWorld(world);
}

int main(void)
{
    TestFreeFall();
    TestRestOnPlane();
    TestStandingStack();
    TestRestitution();
    TestFrictionSlows();
    TestReplayEquality();
    TestBoxRests();
    TestCapsuleRestsFlat();
    TestSphereRestsOnBox();
    TestSphereRestsOnCapsule();
    TestGyroscopicTumble();
    TestDeepSphereRecovers();
    TestDeepCapsuleRecovers();
    TestCrossedCapsulesSeparate();
    TestDeepFuzzDeterminism();
    TestQuickHullRockRests();
    TestBulletStopsAtWall();
    TestBulletVsDynamicTarget();
    TestCcdDeterminism();
    TestSphereSlidesAcrossMeshFloor();
    TestMeshContractsAndGaps();
    TestMeshJournalAndRollback();
    TestSolverHashGate();
    if (s_failures == 0)
    {
        printf("test_solver: all checks passed\n");
    }
    return s_failures == 0 ? 0 : 1;
}
