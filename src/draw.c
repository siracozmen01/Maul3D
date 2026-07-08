// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Debug draw (2c-9): wireframe everything through two primitives.
// Read-only by construction: every function takes the world state as
// it stands and emits segments; the twin-hash test holds the
// no-mutation promise on every commit.

#include "maul3d/draw.h"

#include "world_internal.h"

#include <stddef.h>

#define M3_DRAW_COLOR_AWAKE     0x4FA3FFu
#define M3_DRAW_COLOR_SLEEP     0x777788u
#define M3_DRAW_COLOR_STATIC    0x66BB66u
#define M3_DRAW_COLOR_KINEMATIC 0xC9A227u
#define M3_DRAW_COLOR_SENSOR    0xAA66CCu
#define M3_DRAW_COLOR_CONTACT   0xFF5544u
#define M3_DRAW_COLOR_JOINT     0xFFFFFFu
#define M3_DRAW_COLOR_AABB      0x333344u

#define M3_DRAW_CIRCLE_SEGMENTS 16

typedef struct m3DrawContext
{
    const m3DebugDraw* draw;
    const m3World* world;
} m3DrawContext;

static m3Pos3 ToWorld(const m3Transform* xf, m3Vec3 local)
{
    m3Vec3 r = m3RotateVec3(xf->q, local);
    return (m3Pos3){xf->p.x + (double)r.x, xf->p.y + (double)r.y, xf->p.z + (double)r.z};
}

static void Segment(const m3DrawContext* ctx, m3Pos3 p1, m3Pos3 p2, uint32_t color)
{
    if (ctx->draw->DrawSegment != NULL)
    {
        ctx->draw->DrawSegment(p1, p2, color, ctx->draw->context);
    }
}

static void Point(const m3DrawContext* ctx, m3Pos3 p, m3real size, uint32_t color)
{
    if (ctx->draw->DrawPoint != NULL)
    {
        ctx->draw->DrawPoint(p, size, color, ctx->draw->context);
    }
}

// A circle of fixed segment count around `center`, in the plane
// spanned by two body-frame axes, transformed out.
static void Circle(const m3DrawContext* ctx, const m3Transform* xf, m3Vec3 center, m3Vec3 axis1,
                   m3Vec3 axis2, m3real radius, uint32_t color)
{
    m3Vec3 previous = m3Add3(center, m3MulSV3(radius, axis1));
    for (int32_t k = 1; k <= M3_DRAW_CIRCLE_SEGMENTS; ++k)
    {
        m3real angle = (m3real)k * (2.0f * M3_PI / (m3real)M3_DRAW_CIRCLE_SEGMENTS);
        m3CosSin cs = m3ComputeCosSin(angle);
        m3Vec3 next =
            m3Add3(center, m3Add3(m3MulSV3(radius * cs.c, axis1), m3MulSV3(radius * cs.s, axis2)));
        Segment(ctx, ToWorld(xf, previous), ToWorld(xf, next), color);
        previous = next;
    }
}

static uint32_t ShapeColor(const m3World* world, int32_t shape, const m3DebugDraw* draw)
{
    if (world->shapeSensor[shape] != 0)
    {
        return M3_DRAW_COLOR_SENSOR;
    }
    int32_t body = world->shapeBody[shape];
    uint8_t type = world->types[body];
    if (type == (uint8_t)m3_staticBody)
    {
        return M3_DRAW_COLOR_STATIC;
    }
    if (type == (uint8_t)m3_kinematicBody)
    {
        return M3_DRAW_COLOR_KINEMATIC;
    }
    if (draw->drawSleepTint && world->awake[body] == 0)
    {
        return M3_DRAW_COLOR_SLEEP;
    }
    return M3_DRAW_COLOR_AWAKE;
}

static void DrawShape(const m3DrawContext* ctx, int32_t shape)
{
    const m3World* world = ctx->world;
    int32_t body = world->shapeBody[shape];
    const m3Transform* xf = &world->transforms[body];
    uint32_t color = ShapeColor(world, shape, ctx->draw);
    uint8_t type = world->shapeType[shape];

    if (type == (uint8_t)m3_sphereShape)
    {
        m3Vec3 c = world->shapeGeom[shape].v;
        m3real r = world->shapeGeom[shape].s;
        Circle(ctx, xf, c, (m3Vec3){1.0f, 0.0f, 0.0f}, (m3Vec3){0.0f, 1.0f, 0.0f}, r, color);
        Circle(ctx, xf, c, (m3Vec3){0.0f, 1.0f, 0.0f}, (m3Vec3){0.0f, 0.0f, 1.0f}, r, color);
        Circle(ctx, xf, c, (m3Vec3){0.0f, 0.0f, 1.0f}, (m3Vec3){1.0f, 0.0f, 0.0f}, r, color);
        return;
    }
    if (type == (uint8_t)m3_capsuleShape)
    {
        m3Vec3 p1 = world->shapeGeom[shape].v;
        m3Vec3 p2 = world->shapeGeom[shape].v2;
        m3real r = world->shapeGeom[shape].s;
        m3Vec3 axis = m3Normalize3(m3Sub3(p2, p1));
        m3Vec3 t1;
        m3Vec3 t2;
        m3MakeTangentBasis(axis, &t1, &t2);
        Circle(ctx, xf, p1, t1, t2, r, color);
        Circle(ctx, xf, p2, t1, t2, r, color);
        // Four side rails.
        for (int32_t k = 0; k < 4; ++k)
        {
            m3Vec3 dir = k == 0 ? t1 : (k == 1 ? m3Neg3(t1) : (k == 2 ? t2 : m3Neg3(t2)));
            m3Vec3 offset = m3MulSV3(r, dir);
            Segment(ctx, ToWorld(xf, m3Add3(p1, offset)), ToWorld(xf, m3Add3(p2, offset)), color);
        }
        return;
    }
    if (type == (uint8_t)m3_hullShape)
    {
        const m3HullData* hull = &world->hullData[world->shapeHullIndex[shape]];
        for (int32_t e = 0; e < hull->edgeCount; e += 2)
        {
            m3Vec3 a = hull->vertices[hull->edges[e].origin];
            m3Vec3 b = hull->vertices[hull->edges[e + 1].origin];
            Segment(ctx, ToWorld(xf, a), ToWorld(xf, b), color);
        }
        return;
    }
    if (type == (uint8_t)m3_meshShape)
    {
        const m3MeshData* mesh = &world->meshData[world->shapeMeshIndex[shape]];
        for (int32_t t = 0; t < mesh->triangleCount; ++t)
        {
            m3Vec3 a = mesh->vertices[mesh->indices[3 * t + 0]];
            m3Vec3 b = mesh->vertices[mesh->indices[3 * t + 1]];
            m3Vec3 c = mesh->vertices[mesh->indices[3 * t + 2]];
            Segment(ctx, ToWorld(xf, a), ToWorld(xf, b), color);
            Segment(ctx, ToWorld(xf, b), ToWorld(xf, c), color);
            Segment(ctx, ToWorld(xf, c), ToWorld(xf, a), color);
        }
        return;
    }
    if (type == (uint8_t)m3_voxelShape)
    {
        // Merged-box wireframe: what the collision actually sees.
        const m3VoxelSurface* surface = &world->voxelSurface[world->shapeVoxelIndex[shape]];
        m3real cell = world->voxelData[world->shapeVoxelIndex[shape]].cellSize;
        static const int32_t boxEdges[12][2] = {{0, 1}, {2, 3}, {4, 5}, {6, 7}, {0, 2}, {1, 3},
                                                {4, 6}, {5, 7}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
        for (int32_t b = 0; b < surface->boxCount; ++b)
        {
            m3Vec3 lo;
            m3Vec3 hi;
            m3VoxelBoxBounds(surface, cell, b, &lo, &hi);
            m3Vec3 c[8];
            for (int32_t k = 0; k < 8; ++k)
            {
                c[k] = (m3Vec3){(k & 1) != 0 ? hi.x : lo.x, (k & 2) != 0 ? hi.y : lo.y,
                                (k & 4) != 0 ? hi.z : lo.z};
            }
            for (int32_t k = 0; k < 12; ++k)
            {
                Segment(ctx, ToWorld(xf, c[boxEdges[k][0]]), ToWorld(xf, c[boxEdges[k][1]]), color);
            }
        }
        return;
    }
    if (type == (uint8_t)m3_planeShape)
    {
        // An infinite plane draws as a cross patch and a normal
        // whisker at the projection of the body origin.
        m3Vec3 n = world->shapeGeom[shape].v;
        m3real offset = world->shapeGeom[shape].s;
        m3Vec3 onPlane = m3MulSV3(offset, n);
        m3Vec3 t1;
        m3Vec3 t2;
        m3MakeTangentBasis(n, &t1, &t2);
        const m3real extent = 2.0f;
        Segment(ctx, ToWorld(xf, m3Add3(onPlane, m3MulSV3(-extent, t1))),
                ToWorld(xf, m3Add3(onPlane, m3MulSV3(extent, t1))), color);
        Segment(ctx, ToWorld(xf, m3Add3(onPlane, m3MulSV3(-extent, t2))),
                ToWorld(xf, m3Add3(onPlane, m3MulSV3(extent, t2))), color);
        Segment(ctx, ToWorld(xf, onPlane), ToWorld(xf, m3Add3(onPlane, m3MulSV3(0.5f, n))), color);
    }
}

void m3World_Draw(m3WorldId worldId, const m3DebugDraw* draw)
{
    m3World* world = m3WorldFromId(worldId);
    if (world == NULL || draw == NULL)
    {
        return;
    }
    m3DrawContext ctx = {draw, world};

    if (draw->drawShapes)
    {
        int32_t maxShape = world->shapePool.maxIndex;
        for (int32_t s = 0; s < maxShape; ++s)
        {
            if (world->shapePool.alive[s] != 0)
            {
                DrawShape(&ctx, s);
            }
        }
    }

    if (draw->drawAabbs)
    {
        int32_t maxShape = world->shapePool.maxIndex;
        for (int32_t s = 0; s < maxShape; ++s)
        {
            if (world->shapePool.alive[s] == 0 || world->shapeType[s] == (uint8_t)m3_planeShape)
            {
                continue; // planes have no finite bounds
            }
            double lo[3];
            double hi[3];
            m3ShapeFatAabb(world, s, lo, hi);
            // Twelve edges of the fat box.
            m3Pos3 c[8];
            for (int32_t k = 0; k < 8; ++k)
            {
                c[k] = (m3Pos3){(k & 1) != 0 ? hi[0] : lo[0], (k & 2) != 0 ? hi[1] : lo[1],
                                (k & 4) != 0 ? hi[2] : lo[2]};
            }
            const int32_t edges[12][2] = {{0, 1}, {2, 3}, {4, 5}, {6, 7}, {0, 2}, {1, 3},
                                          {4, 6}, {5, 7}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
            for (int32_t k = 0; k < 12; ++k)
            {
                Segment(&ctx, c[edges[k][0]], c[edges[k][1]], M3_DRAW_COLOR_AABB);
            }
        }
    }

    if (draw->drawContacts)
    {
        for (int32_t i = 0; i < world->pairCount; ++i)
        {
            const m3Manifold* manifold = &world->manifolds[i];
            if (manifold->pointCount == 0)
            {
                continue;
            }
            uint64_t key = world->pairKeys[i];
            int32_t bodyA = world->shapeBody[(int32_t)(key >> 32)];
            const m3Transform* xfA = &world->transforms[bodyA];
            m3Vec3 rlcA = m3RotateVec3(xfA->q, world->localCenters[bodyA]);
            for (int32_t k = 0; k < manifold->pointCount; ++k)
            {
                // anchorA is COM-relative in world orientation.
                m3Pos3 p = {xfA->p.x + (double)rlcA.x + (double)manifold->points[k].anchorA.x,
                            xfA->p.y + (double)rlcA.y + (double)manifold->points[k].anchorA.y,
                            xfA->p.z + (double)rlcA.z + (double)manifold->points[k].anchorA.z};
                Point(&ctx, p, 4.0f, M3_DRAW_COLOR_CONTACT);
                // The normal whisker scales with the warm impulse:
                // impulse over dt is force, and the host knows dt.
                m3real scale = 0.1f * manifold->points[k].normalImpulse;
                if (scale > 0.0f)
                {
                    m3Pos3 tip = {p.x + (double)(manifold->normal.x * scale),
                                  p.y + (double)(manifold->normal.y * scale),
                                  p.z + (double)(manifold->normal.z * scale)};
                    Segment(&ctx, p, tip, M3_DRAW_COLOR_CONTACT);
                }
            }
        }
    }

    if (draw->drawJoints)
    {
        int32_t maxJoint = world->jointPool.maxIndex;
        for (int32_t j = 0; j < maxJoint; ++j)
        {
            if (world->jointPool.alive[j] == 0)
            {
                continue;
            }
            const m3Transform* xfA = &world->transforms[world->jointBodyA[j]];
            const m3Transform* xfB = &world->transforms[world->jointBodyB[j]];
            m3Pos3 a = ToWorld(xfA, world->jointLocalA[j]);
            m3Pos3 b = ToWorld(xfB, world->jointLocalB[j]);
            Point(&ctx, a, 5.0f, M3_DRAW_COLOR_JOINT);
            Point(&ctx, b, 5.0f, M3_DRAW_COLOR_JOINT);
            Segment(&ctx, a, b, M3_DRAW_COLOR_JOINT);
        }
    }
}
