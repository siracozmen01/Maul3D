// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Narrowphase gate: the pure collide kernels against analytic cases,
// the speculative band, the fixed tangent basis rule, and the
// warm-start carry through a full stash-scan-rebuild cycle. White box.

#include "world_internal.h"

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

static int NearF(m3real a, m3real b, m3real tol)
{
    m3real d = a - b;
    return (d < 0.0f ? -d : d) <= tol;
}

static void TestSphereKernels(void)
{
    // Two r=0.5 spheres, centers 0.9 apart along x: penetration 0.1.
    m3Manifold m = m3CollideSpheres((m3Vec3){0.9f, 0.0f, 0.0f}, 0.5f, 0.5f);
    CHECK(m.pointCount == 1, "overlapping spheres make one point");
    CHECK(NearF(m.normal.x, 1.0f, 1.0e-6f), "normal points from A to B");
    CHECK(NearF(m.points[0].separation, -0.1f, 1.0e-6f), "penetration is analytic");
    CHECK(NearF(m.points[0].anchorA.x, 0.5f, 1.0e-6f), "anchor A on A's surface");
    CHECK(NearF(m.points[0].anchorB.x, -0.5f, 1.0e-6f), "anchor B on B's surface");

    // The speculative band: a gap inside the margin still contacts.
    m = m3CollideSpheres((m3Vec3){1.01f, 0.0f, 0.0f}, 0.5f, 0.5f);
    CHECK(m.pointCount == 1 && m.points[0].separation > 0.0f, "speculative contact in the band");
    m = m3CollideSpheres((m3Vec3){1.10f, 0.0f, 0.0f}, 0.5f, 0.5f);
    CHECK(m.pointCount == 0, "beyond the margin there is no contact");

    // Concentric centers: the fixed +y fallback, never NaN.
    m = m3CollideSpheres((m3Vec3){0.0f, 0.0f, 0.0f}, 0.5f, 0.5f);
    CHECK(m.pointCount == 1 && m.normal.y == 1.0f, "concentric fallback is the fixed rule");

    // Plane versus sphere: center 0.45 above, r 0.5: penetration 0.05.
    m = m3CollidePlaneSphere((m3Vec3){0.0f, 1.0f, 0.0f}, 0.45f, 0.5f);
    CHECK(m.pointCount == 1, "plane contact");
    CHECK(NearF(m.points[0].separation, -0.05f, 1.0e-6f), "plane penetration is analytic");
    CHECK(m.normal.y == 1.0f, "plane normal is A to B");
}

static void TestTangentBasis(void)
{
    // Orthonormal for axis and skew normals, and bit-identical on
    // repeated calls (one rule, no state).
    m3Vec3 normals[5] = {{0.0f, 1.0f, 0.0f},
                         {1.0f, 0.0f, 0.0f},
                         {0.0f, 0.0f, -1.0f},
                         {0.577350f, 0.577350f, 0.577350f},
                         {-0.267261f, 0.534522f, 0.801784f}};
    for (int32_t i = 0; i < 5; ++i)
    {
        m3Vec3 t1a;
        m3Vec3 t2a;
        m3Vec3 t1b;
        m3Vec3 t2b;
        m3MakeTangentBasis(normals[i], &t1a, &t2a);
        m3MakeTangentBasis(normals[i], &t1b, &t2b);
        CHECK(memcmp(&t1a, &t1b, sizeof(t1a)) == 0 && memcmp(&t2a, &t2b, sizeof(t2a)) == 0,
              "the basis is bit-stable");
        CHECK(NearF(m3Dot3(t1a, normals[i]), 0.0f, 1.0e-5f), "t1 orthogonal to n");
        CHECK(NearF(m3Dot3(t2a, normals[i]), 0.0f, 1.0e-5f), "t2 orthogonal to n");
        CHECK(NearF(m3Dot3(t1a, t2a), 0.0f, 1.0e-5f), "t1 orthogonal to t2");
        CHECK(NearF(m3Length3(t1a), 1.0f, 1.0e-5f), "t1 unit");
        CHECK(NearF(m3Length3(t2a), 1.0f, 1.0e-5f), "t2 unit");
    }
}

static void TestWarmStartCarry(void)
{
    // A sphere resting near a plane: build contacts, inject an
    // impulse, rebuild, and the impulse must persist by feature id;
    // separate the pair and rebuild, and it must vanish.
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 4;
    def.shapeCapacity = 4;
    m3WorldId world = m3CreateWorld(&def);
    m3World* w = m3WorldFromIndex0((uint16_t)(world.index1 - 1));

    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    m3CreatePlaneShape(ground, &sd, &floor);

    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 0.45, 0.0};
    m3BodyId ball = m3CreateBody(world, &bd);
    m3Sphere sphere = {{0.0f, 0.0f, 0.0f}, 0.5f};
    m3CreateSphereShape(ball, &sd, &sphere);

    CHECK(m3UpdatePairs(w) == m3_success, "pairs");
    CHECK(m3UpdateContacts(w, NULL, NULL, 0) == m3_success, "first contact build");
    CHECK(w->pairCount == 1 && w->manifolds[0].pointCount == 1, "one plane contact");
    CHECK((w->manifolds[0].points[0].flags & 1) == 0, "a fresh contact is not persisted");

    // Inject a solved impulse, then run the full stash-scan-rebuild
    // cycle the step will use (stash BEFORE the pair scan overwrites).
    w->manifolds[0].points[0].normalImpulse = 3.5f;
    uint64_t stashKeys[4];
    m3Manifold stashManifolds[4];
    int32_t stashCount = w->pairCount;
    memcpy(stashKeys, w->pairKeys, (size_t)stashCount * sizeof(uint64_t));
    memcpy(stashManifolds, w->manifolds, (size_t)stashCount * sizeof(m3Manifold));
    CHECK(m3UpdatePairs(w) == m3_success, "rescan");
    CHECK(m3UpdateContacts(w, stashKeys, stashManifolds, stashCount) == m3_success, "rebuild");
    CHECK(NearF(w->manifolds[0].points[0].normalImpulse, 3.5f, 1.0e-6f),
          "the warm-start impulse carries by feature id");
    CHECK((w->manifolds[0].points[0].flags & 1) == 1, "the carried point is marked persisted");

    // Separate the pair: the contact and its impulse vanish.
    m3Body_SetLinearVelocity(ball, (m3Vec3){0.0f, 0.0f, 0.0f});
    w->transforms[ball.index1 - 1].p.y = 10.0;
    stashCount = w->pairCount;
    memcpy(stashKeys, w->pairKeys, (size_t)stashCount * sizeof(uint64_t));
    memcpy(stashManifolds, w->manifolds, (size_t)stashCount * sizeof(m3Manifold));
    CHECK(m3UpdatePairs(w) == m3_success, "rescan after separation");
    CHECK(m3UpdateContacts(w, stashKeys, stashManifolds, stashCount) == m3_success,
          "rebuild after separation");
    int32_t contacts = 0;
    for (int32_t i = 0; i < w->pairCount; ++i)
    {
        contacts += w->manifolds[i].pointCount;
    }
    CHECK(contacts == 0, "a separated pair carries nothing");

    m3DestroyWorld(world);
}

int main(void)
{
    TestSphereKernels();
    TestTangentBasis();
    TestWarmStartCarry();
    if (s_failures == 0)
    {
        printf("test_contacts: all checks passed\n");
    }
    return s_failures == 0 ? 0 : 1;
}
