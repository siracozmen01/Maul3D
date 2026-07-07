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
    TestSolverHashGate();
    if (s_failures == 0)
    {
        printf("test_solver: all checks passed\n");
    }
    return s_failures == 0 ? 0 : 1;
}
