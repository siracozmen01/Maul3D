// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// QuickHull, adapted from the reference hull.c (Erin Catto, with
// portions contributed by Dirk Gregorius). The algorithm is kept
// whole: scaled tolerance, the four-point seed, per-face conflict
// lists with cached farthest vertices, the iterative horizon DFS,
// cone construction, then the three merge passes (flipped, concave,
// coplanar) that turn triangle soup into clean n-gon faces. What
// changed for Maul3D: one fixed-size work block instead of growing
// allocators, emission into the fixed m3HullData arrays through the
// same m3HullBuildHalfEdges law the box builder uses, and the
// unit-density mass integrator folded into the emit so every hull
// arrives with volume, centroid, and central inertia. Determinism:
// every loop runs in construction order and every comparison is a
// strict float improvement, so ties resolve to the earliest
// candidate, bit-stably.

#include "world_internal.h"

#include <float.h>
#include <string.h>

#define M3_QH_MARK_VISIBLE 0
#define M3_QH_MARK_DELETE  1

typedef struct m3QhListNode
{
    struct m3QhListNode* prev;
    struct m3QhListNode* next;
} m3QhListNode;

typedef struct m3QhFace m3QhFace;

typedef struct m3QhVertex
{
    m3QhListNode link; // must be first: nodes cast back to vertices
    m3QhFace* conflictFace;
    m3Vec3 position;
    int32_t finalIndex;
    int32_t reachable;
} m3QhVertex;

typedef struct m3QhEdge
{
    struct m3QhEdge* prev; // ring around the owning face
    struct m3QhEdge* next;
    m3QhVertex* origin;
    m3QhFace* face;
    struct m3QhEdge* twin;
} m3QhEdge;

struct m3QhFace
{
    m3QhListNode link; // must be first: nodes cast back to faces
    m3QhEdge* edge;
    int32_t mark;
    m3real area;
    m3Vec3 normal; // unit plane normal
    m3real offset; // plane offset: dot(normal, x) = offset on the face
    m3Vec3 centroid;
    m3real maxConflictDistance;
    m3QhVertex conflictListHead; // sentinel, only the link matters
    m3QhVertex* maxConflict;
    int32_t flipped;
};

// One frame of the iterative horizon DFS (the reference replaces
// recursion so stack depth is bounded by live faces).
typedef struct m3QhHorizonFrame
{
    m3QhFace* face;
    m3QhEdge* startEdge;
    m3QhEdge* edge;
    int32_t started;
} m3QhHorizonFrame;

// Work-pool capacities for N <= M3_HULL_MAX_INPUT input points and
// M = M3_HULL_MAX_VERTS output vertices, the reference bounds.
#define M3_QH_VERTEX_CAP (M3_HULL_MAX_INPUT + 4)
#define M3_QH_EDGE_CAP   (24 * M3_HULL_MAX_VERTS - 48)
#define M3_QH_FACE_CAP   (5 * M3_HULL_MAX_VERTS - 10)
#define M3_QH_RING_CAP   (3 * M3_HULL_MAX_VERTS - 6)
#define M3_QH_MERGE_CAP  (2 * M3_HULL_MAX_VERTS - 4)

typedef struct m3QhBuilder
{
    m3real tolerance;
    m3real minRadius;
    m3real minOutside;
    m3Vec3 interiorPoint;

    m3QhVertex orphanedList; // sentinels
    m3QhVertex vertexList;
    m3QhFace faceList;

    int32_t vertexCount;
    int32_t edgeCount;
    int32_t faceCount;
    m3QhEdge* edgeFreeHead;
    m3QhFace* faceFreeHead;

    int32_t horizonCount;
    int32_t coneCount;
    int32_t mergedCount;

    int32_t finalVertexCount;
    int32_t finalHalfEdgeCount;
    int32_t finalFaceCount;

    m3QhVertex vertexPool[M3_QH_VERTEX_CAP];
    m3QhEdge edgePool[M3_QH_EDGE_CAP];
    m3QhFace facePool[M3_QH_FACE_CAP];
    m3QhEdge* horizon[M3_QH_RING_CAP];
    m3QhFace* cone[M3_QH_RING_CAP];
    m3QhFace* merged[M3_QH_MERGE_CAP];
    m3QhHorizonFrame stack[M3_QH_MERGE_CAP];
    m3Vec3 shifted[M3_HULL_MAX_INPUT];
} m3QhBuilder;

static void QhListInit(m3QhListNode* head)
{
    head->prev = head;
    head->next = head;
}

static int QhListContains(const m3QhListNode* node)
{
    return node->prev != NULL;
}

static void QhListRemove(m3QhListNode* node)
{
    node->prev->next = node->next;
    node->next->prev = node->prev;
    node->prev = NULL;
    node->next = NULL;
}

static void QhListPushBack(m3QhListNode* head, m3QhListNode* node)
{
    M3_ASSERT(!QhListContains(node));
    node->prev = head->prev;
    node->next = head;
    head->prev->next = node;
    head->prev = node;
}

static m3real QhPlaneSeparation(const m3QhFace* face, m3Vec3 point)
{
    return m3Dot3(face->normal, point) - face->offset;
}

static m3QhVertex* QhNewVertex(m3QhBuilder* b, m3Vec3 position)
{
    M3_ASSERT(b->vertexCount < M3_QH_VERTEX_CAP);
    m3QhVertex* vertex = &b->vertexPool[b->vertexCount++];
    vertex->link.prev = NULL;
    vertex->link.next = NULL;
    vertex->conflictFace = NULL;
    vertex->position = position;
    vertex->finalIndex = -1;
    vertex->reachable = 0;
    return vertex;
}

static m3QhEdge* QhNewEdge(m3QhBuilder* b)
{
    m3QhEdge* edge;
    if (b->edgeFreeHead != NULL)
    {
        edge = b->edgeFreeHead;
        b->edgeFreeHead = edge->next;
    }
    else
    {
        M3_ASSERT(b->edgeCount < M3_QH_EDGE_CAP);
        edge = &b->edgePool[b->edgeCount++];
    }
    return edge;
}

static void QhRetireEdge(m3QhBuilder* b, m3QhEdge* edge)
{
    edge->next = b->edgeFreeHead;
    b->edgeFreeHead = edge;
}

static m3QhFace* QhNewFace(m3QhBuilder* b, m3QhVertex* v1, m3QhVertex* v2, m3QhVertex* v3)
{
    m3QhFace* face;
    if (b->faceFreeHead != NULL)
    {
        face = b->faceFreeHead;
        b->faceFreeHead = (m3QhFace*)face->link.next;
    }
    else
    {
        M3_ASSERT(b->faceCount < M3_QH_FACE_CAP);
        face = &b->facePool[b->faceCount++];
    }
    face->link.prev = NULL;
    face->link.next = NULL;
    face->maxConflict = NULL;
    face->maxConflictDistance = 0.0f;

    m3QhEdge* edge1 = QhNewEdge(b);
    m3QhEdge* edge2 = QhNewEdge(b);
    m3QhEdge* edge3 = QhNewEdge(b);

    m3Vec3 p1 = v1->position;
    m3Vec3 p2 = v2->position;
    m3Vec3 p3 = v3->position;
    m3Vec3 normal = m3Cross3(m3Sub3(p2, p1), m3Sub3(p3, p1));
    m3real length = sqrtf(m3Dot3(normal, normal));
    normal = length > 0.0f ? m3MulSV3(1.0f / length, normal) : (m3Vec3){0.0f, 1.0f, 0.0f};

    face->edge = edge1;
    face->mark = M3_QH_MARK_VISIBLE;
    face->area = 0.5f * length;
    face->centroid = m3MulSV3(1.0f / 3.0f, m3Add3(p1, m3Add3(p2, p3)));
    face->normal = normal;
    face->offset = m3Dot3(normal, p1);
    face->flipped = QhPlaneSeparation(face, b->interiorPoint) > 0.0f;
    QhListInit(&face->conflictListHead.link);

    edge1->prev = edge3;
    edge1->next = edge2;
    edge1->origin = v1;
    edge1->face = face;
    edge1->twin = NULL;
    edge2->prev = edge1;
    edge2->next = edge3;
    edge2->origin = v2;
    edge2->face = face;
    edge2->twin = NULL;
    edge3->prev = edge2;
    edge3->next = edge1;
    edge3->origin = v3;
    edge3->face = face;
    edge3->twin = NULL;
    return face;
}

static void QhRetireFace(m3QhBuilder* b, m3QhFace* face)
{
    if (QhListContains(&face->link))
    {
        QhListRemove(&face->link);
    }
    face->edge = NULL;
    face->link.next = (m3QhListNode*)b->faceFreeHead;
    b->faceFreeHead = face;
}

static int QhIsEdgeConvex(const m3QhEdge* edge, m3real tolerance)
{
    return QhPlaneSeparation(edge->face, edge->twin->face->centroid) < -tolerance;
}

static int QhIsEdgeConcave(const m3QhEdge* edge, m3real tolerance)
{
    return QhPlaneSeparation(edge->face, edge->twin->face->centroid) > tolerance;
}

static int32_t QhVertexCountOfFace(const m3QhFace* face)
{
    int32_t count = 0;
    const m3QhEdge* edge = face->edge;
    do
    {
        count++;
        edge = edge->next;
    } while (edge != face->edge);
    return count;
}

static void QhLinkFace(m3QhFace* face, int32_t index, m3QhEdge* twin)
{
    m3QhEdge* edge = face->edge;
    while (index-- > 0)
    {
        edge = edge->next;
    }
    edge->twin = twin;
    twin->twin = edge;
}

static void QhLinkFaces(m3QhFace* face1, int32_t index1, m3QhFace* face2, int32_t index2)
{
    m3QhEdge* edge1 = face1->edge;
    while (index1-- > 0)
    {
        edge1 = edge1->next;
    }
    m3QhEdge* edge2 = face2->edge;
    while (index2-- > 0)
    {
        edge2 = edge2->next;
    }
    edge1->twin = edge2;
    edge2->twin = edge1;
}

// Newell plane for an n-gon face: robust normal and centroid with the
// first vertex as the local origin to reduce round-off.
static void QhNewellPlane(m3QhFace* face)
{
    int32_t count = 0;
    m3Vec3 centroid = {0.0f, 0.0f, 0.0f};
    m3Vec3 normal = {0.0f, 0.0f, 0.0f};
    m3QhEdge* edge = face->edge;
    m3Vec3 origin = edge->origin->position;
    do
    {
        m3Vec3 v1 = m3Sub3(edge->origin->position, origin);
        m3Vec3 v2 = m3Sub3(edge->twin->origin->position, origin);
        count++;
        centroid = m3Add3(centroid, v1);
        normal.x += (v1.y - v2.y) * (v1.z + v2.z);
        normal.y += (v1.z - v2.z) * (v1.x + v2.x);
        normal.z += (v1.x - v2.x) * (v1.y + v2.y);
        edge = edge->next;
    } while (edge != face->edge);

    centroid = m3Add3(m3MulSV3(1.0f / (m3real)count, centroid), origin);
    m3real length = sqrtf(m3Dot3(normal, normal));
    normal = length > 0.0f ? m3MulSV3(1.0f / length, normal) : (m3Vec3){0.0f, 1.0f, 0.0f};
    face->centroid = centroid;
    face->normal = normal;
    face->offset = m3Dot3(normal, centroid);
    face->area = 0.5f * length;
}

static void QhComputeTolerance(m3QhBuilder* b, int32_t pointCount, const m3Vec3* points)
{
    m3Vec3 lo = points[0];
    m3Vec3 hi = points[0];
    for (int32_t i = 1; i < pointCount; ++i)
    {
        lo.x = m3MinF(lo.x, points[i].x);
        lo.y = m3MinF(lo.y, points[i].y);
        lo.z = m3MinF(lo.z, points[i].z);
        hi.x = m3MaxF(hi.x, points[i].x);
        hi.y = m3MaxF(hi.y, points[i].y);
        hi.z = m3MaxF(hi.z, points[i].z);
    }
    m3Vec3 maxAbs;
    maxAbs.x = m3MaxF(lo.x < 0.0f ? -lo.x : lo.x, hi.x < 0.0f ? -hi.x : hi.x);
    maxAbs.y = m3MaxF(lo.y < 0.0f ? -lo.y : lo.y, hi.y < 0.0f ? -hi.y : hi.y);
    maxAbs.z = m3MaxF(lo.z < 0.0f ? -lo.z : lo.z, hi.z < 0.0f ? -hi.z : hi.z);
    m3real maxSum = maxAbs.x + maxAbs.y + maxAbs.z;
    m3real maxCoord = m3MaxF(maxAbs.x, m3MaxF(maxAbs.y, maxAbs.z));
    m3real maxDistance = m3MinF(1.7320508f * maxCoord, maxSum);
    b->tolerance = (3.0f * maxDistance * 1.01f + maxCoord) * FLT_EPSILON;
    b->minRadius = 4.0f * b->tolerance;
    b->minOutside = 2.0f * b->minRadius;
}

// The four seed finders (reference order and tie rules: strict
// improvement keeps the earliest candidate).
static void QhFarthestAlongCardinalAxes(int32_t* i1, int32_t* i2, m3real tolerance,
                                        int32_t pointCount, const m3Vec3* points)
{
    *i1 = -1;
    *i2 = -1;
    m3Vec3 v0 = points[0];
    m3Vec3 minPt[3] = {v0, v0, v0};
    m3Vec3 maxPt[3] = {v0, v0, v0};
    int32_t minIndex[3] = {0, 0, 0};
    int32_t maxIndex[3] = {0, 0, 0};
    for (int32_t i = 1; i < pointCount; ++i)
    {
        m3Vec3 v = points[i];
        if (v.x < minPt[0].x)
        {
            minPt[0] = v;
            minIndex[0] = i;
        }
        else if (v.x > maxPt[0].x)
        {
            maxPt[0] = v;
            maxIndex[0] = i;
        }
        if (v.y < minPt[1].y)
        {
            minPt[1] = v;
            minIndex[1] = i;
        }
        else if (v.y > maxPt[1].y)
        {
            maxPt[1] = v;
            maxIndex[1] = i;
        }
        if (v.z < minPt[2].z)
        {
            minPt[2] = v;
            minIndex[2] = i;
        }
        else if (v.z > maxPt[2].z)
        {
            maxPt[2] = v;
            maxIndex[2] = i;
        }
    }
    m3real dist[3] = {maxPt[0].x - minPt[0].x, maxPt[1].y - minPt[1].y, maxPt[2].z - minPt[2].z};
    int32_t axis = 0;
    if (dist[1] > dist[axis])
    {
        axis = 1;
    }
    if (dist[2] > dist[axis])
    {
        axis = 2;
    }
    if (dist[axis] > 2.0f * tolerance)
    {
        *i1 = minIndex[axis];
        *i2 = maxIndex[axis];
    }
}

static int32_t QhFarthestFromLine(int32_t i1, int32_t i2, m3real tolerance, int32_t pointCount,
                                  const m3Vec3* points)
{
    m3Vec3 a = points[i1];
    m3Vec3 ab = m3Sub3(points[i2], a);
    m3real abLen2 = m3Dot3(ab, ab);
    if (!(abLen2 > 0.0f))
    {
        return -1;
    }
    m3real invAbLen2 = 1.0f / abLen2;
    m3real maxDist2 = 4.0f * tolerance * tolerance;
    int32_t maxIndex = -1;
    for (int32_t i = 0; i < pointCount; ++i)
    {
        if (i == i1 || i == i2)
        {
            continue;
        }
        m3Vec3 cross = m3Cross3(m3Sub3(points[i], a), ab);
        m3real dist2 = m3Dot3(cross, cross) * invAbLen2;
        if (dist2 > maxDist2)
        {
            maxDist2 = dist2;
            maxIndex = i;
        }
    }
    return maxIndex;
}

static int32_t QhFarthestFromPlane(int32_t i1, int32_t i2, int32_t i3, m3real tolerance,
                                   int32_t pointCount, const m3Vec3* points)
{
    m3Vec3 a = points[i1];
    m3Vec3 n = m3Cross3(m3Sub3(points[i2], a), m3Sub3(points[i3], a));
    m3real len = sqrtf(m3Dot3(n, n));
    if (!(len > 0.0f))
    {
        return -1;
    }
    n = m3MulSV3(1.0f / len, n);
    m3real off = m3Dot3(n, a);
    m3real maxDistance = 2.0f * tolerance;
    int32_t maxIndex = -1;
    for (int32_t i = 0; i < pointCount; ++i)
    {
        if (i == i1 || i == i2 || i == i3)
        {
            continue;
        }
        m3real d = m3Dot3(n, points[i]) - off;
        d = d < 0.0f ? -d : d;
        if (d > maxDistance)
        {
            maxDistance = d;
            maxIndex = i;
        }
    }
    return maxIndex;
}

static int QhBuildInitialHull(m3QhBuilder* b, int32_t pointCount, const m3Vec3* points)
{
    int32_t i1;
    int32_t i2;
    QhFarthestAlongCardinalAxes(&i1, &i2, b->tolerance, pointCount, points);
    if (i1 < 0 || i2 < 0)
    {
        return 0;
    }
    int32_t i3 = QhFarthestFromLine(i1, i2, b->tolerance, pointCount, points);
    if (i3 < 0)
    {
        return 0;
    }
    int32_t i4 = QhFarthestFromPlane(i1, i2, i3, b->tolerance, pointCount, points);
    if (i4 < 0)
    {
        return 0;
    }

    m3Vec3 v1 = m3Sub3(points[i1], points[i4]);
    m3Vec3 v2 = m3Sub3(points[i2], points[i4]);
    m3Vec3 v3 = m3Sub3(points[i3], points[i4]);
    if (m3Dot3(v1, m3Cross3(v2, v3)) < 0.0f)
    {
        int32_t tmp = i2;
        i2 = i3;
        i3 = tmp;
    }

    b->interiorPoint =
        m3MulSV3(0.25f, m3Add3(m3Add3(points[i1], points[i2]), m3Add3(points[i3], points[i4])));

    m3QhVertex* vertex1 = QhNewVertex(b, points[i1]);
    QhListPushBack(&b->vertexList.link, &vertex1->link);
    m3QhVertex* vertex2 = QhNewVertex(b, points[i2]);
    QhListPushBack(&b->vertexList.link, &vertex2->link);
    m3QhVertex* vertex3 = QhNewVertex(b, points[i3]);
    QhListPushBack(&b->vertexList.link, &vertex3->link);
    m3QhVertex* vertex4 = QhNewVertex(b, points[i4]);
    QhListPushBack(&b->vertexList.link, &vertex4->link);

    m3QhFace* face1 = QhNewFace(b, vertex1, vertex2, vertex3);
    QhListPushBack(&b->faceList.link, &face1->link);
    m3QhFace* face2 = QhNewFace(b, vertex4, vertex2, vertex1);
    QhListPushBack(&b->faceList.link, &face2->link);
    m3QhFace* face3 = QhNewFace(b, vertex4, vertex3, vertex2);
    QhListPushBack(&b->faceList.link, &face3->link);
    m3QhFace* face4 = QhNewFace(b, vertex4, vertex1, vertex3);
    QhListPushBack(&b->faceList.link, &face4->link);

    QhLinkFaces(face1, 0, face2, 1);
    QhLinkFaces(face1, 1, face3, 1);
    QhLinkFaces(face1, 2, face4, 1);
    QhLinkFaces(face2, 0, face3, 2);
    QhLinkFaces(face3, 0, face4, 2);
    QhLinkFaces(face4, 0, face2, 2);

    for (int32_t index = 0; index < pointCount; ++index)
    {
        if (index == i1 || index == i2 || index == i3 || index == i4)
        {
            continue;
        }
        m3Vec3 point = points[index];
        m3real maxDistance = b->minOutside;
        m3QhFace* maxFace = NULL;
        for (m3QhListNode* node = b->faceList.link.next; node != &b->faceList.link;
             node = node->next)
        {
            m3QhFace* face = (m3QhFace*)node;
            m3real distance = QhPlaneSeparation(face, point);
            if (distance > maxDistance)
            {
                maxDistance = distance;
                maxFace = face;
            }
        }
        if (maxFace != NULL)
        {
            m3QhVertex* vertex = QhNewVertex(b, point);
            vertex->conflictFace = maxFace;
            QhListPushBack(&maxFace->conflictListHead.link, &vertex->link);
            if (maxDistance > maxFace->maxConflictDistance)
            {
                maxFace->maxConflictDistance = maxDistance;
                maxFace->maxConflict = vertex;
            }
        }
    }
    return 1;
}

// Recompute the farthest-conflict cache after a face's plane moved.
static void QhRecacheConflicts(m3QhFace* face, m3real minOutside)
{
    m3QhVertex* maxVertex = NULL;
    m3real maxDistance = minOutside;
    for (m3QhListNode* node = face->conflictListHead.link.next;
         node != &face->conflictListHead.link; node = node->next)
    {
        m3QhVertex* vertex = (m3QhVertex*)node;
        m3real distance = QhPlaneSeparation(face, vertex->position);
        if (distance > maxDistance)
        {
            maxDistance = distance;
            maxVertex = vertex;
        }
    }
    face->maxConflict = maxVertex;
    face->maxConflictDistance = maxDistance;
}

static m3QhVertex* QhNextConflictVertex(const m3QhBuilder* b)
{
    m3QhVertex* maxVertex = NULL;
    m3real maxDistance = b->minOutside;
    for (const m3QhListNode* node = b->faceList.link.next; node != &b->faceList.link;
         node = node->next)
    {
        const m3QhFace* face = (const m3QhFace*)node;
        if (face->maxConflict != NULL && face->maxConflictDistance > maxDistance)
        {
            maxDistance = face->maxConflictDistance;
            maxVertex = face->maxConflict;
        }
    }
    return maxVertex;
}

static void QhDrainConflictList(m3QhBuilder* b, m3QhFace* face)
{
    m3QhListNode* node = face->conflictListHead.link.next;
    while (node != &face->conflictListHead.link)
    {
        m3QhVertex* orphan = (m3QhVertex*)node;
        node = node->next;
        orphan->conflictFace = NULL;
        QhListRemove(&orphan->link);
        QhListPushBack(&b->orphanedList.link, &orphan->link);
    }
}

static void QhEnterHorizonFace(m3QhBuilder* b, m3QhFace* face, m3QhEdge* entryEdge,
                               m3QhHorizonFrame* frame)
{
    face->mark = M3_QH_MARK_DELETE;
    QhDrainConflictList(b, face);
    frame->face = face;
    frame->started = 0;
    if (entryEdge != NULL)
    {
        frame->startEdge = entryEdge;
        frame->edge = entryEdge->next;
    }
    else
    {
        frame->startEdge = face->edge;
        frame->edge = face->edge;
    }
}

static void QhBuildHorizon(m3QhBuilder* b, m3QhVertex* apex, m3QhFace* seed)
{
    int32_t top = 0;
    QhEnterHorizonFace(b, seed, NULL, &b->stack[top++]);
    while (top > 0)
    {
        m3QhHorizonFrame* f = &b->stack[top - 1];
        if (f->started && f->edge == f->startEdge)
        {
            top--;
            continue;
        }
        f->started = 1;
        m3QhEdge* edge = f->edge;
        m3QhEdge* twin = edge->twin;
        f->edge = edge->next;
        if (twin->face->mark != M3_QH_MARK_VISIBLE)
        {
            continue;
        }
        m3real distance = QhPlaneSeparation(twin->face, apex->position);
        if (distance > b->minRadius)
        {
            M3_ASSERT(top < M3_QH_MERGE_CAP);
            QhEnterHorizonFace(b, twin->face, twin, &b->stack[top++]);
        }
        else
        {
            M3_ASSERT(b->horizonCount < M3_QH_RING_CAP);
            b->horizon[b->horizonCount++] = edge;
        }
    }
}

static void QhBuildCone(m3QhBuilder* b, m3QhVertex* apex)
{
    for (int32_t i = 0; i < b->horizonCount; ++i)
    {
        m3QhEdge* edge = b->horizon[i];
        m3QhFace* face = QhNewFace(b, apex, edge->origin, edge->twin->origin);
        M3_ASSERT(b->coneCount < M3_QH_RING_CAP);
        b->cone[b->coneCount++] = face;
        QhLinkFace(face, 1, edge->twin);
    }
    m3QhFace* face1 = b->cone[b->coneCount - 1];
    for (int32_t i = 0; i < b->coneCount; ++i)
    {
        m3QhFace* face2 = b->cone[i];
        QhLinkFaces(face1, 2, face2, 0);
        face1 = face2;
    }
}

static void QhDestroyEdges(m3QhBuilder* b, m3QhEdge* begin, m3QhEdge* end)
{
    m3QhEdge* edge = begin;
    while (edge != end)
    {
        m3QhEdge* next = edge->next;
        QhRetireEdge(b, edge);
        edge = next;
    }
}

// Splice `prev` to `next` in a face ring; when both share the same
// opposing face the pair would orphan it, so the redundant edge (and
// possibly a dead opposing triangle) is dissolved, the reference's
// two topologic repair cases kept verbatim.
static void QhConnectEdges(m3QhBuilder* b, m3QhEdge* prev, m3QhEdge* next)
{
    if (prev->twin->face == next->twin->face)
    {
        if (next->face->edge == next)
        {
            next->face->edge = prev;
        }
        m3QhEdge* twin;
        if (QhVertexCountOfFace(prev->twin->face) == 3)
        {
            m3QhEdge* deadEdge0 = prev->twin;
            m3QhEdge* deadEdge1 = next->twin;
            m3QhEdge* deadEdge2 = next->twin->prev;
            twin = deadEdge2->twin;

            m3QhFace* opposing = prev->twin->face;
            opposing->mark = M3_QH_MARK_DELETE;
            M3_ASSERT(b->mergedCount < M3_QH_MERGE_CAP);
            b->merged[b->mergedCount++] = opposing;

            prev->next = next->next;
            prev->next->prev = prev;
            prev->twin = twin;
            twin->twin = prev;
            QhListRemove(&next->origin->link);
            QhRetireEdge(b, deadEdge0);
            QhRetireEdge(b, deadEdge1);
            QhRetireEdge(b, deadEdge2);
        }
        else
        {
            twin = next->twin;
            if (twin->face->edge == prev->twin)
            {
                twin->face->edge = twin;
            }
            twin->next = prev->twin->next;
            twin->next->prev = twin;
            QhRetireEdge(b, prev->twin);
            prev->next = next->next;
            prev->next->prev = prev;
            prev->twin = twin;
            twin->twin = prev;
            QhListRemove(&next->origin->link);
        }
        QhNewellPlane(twin->face);
        QhRecacheConflicts(twin->face, b->minOutside);
    }
    else
    {
        prev->next = next;
        next->prev = prev;
    }
}

static void QhAbsorbFaces(m3QhBuilder* b, m3QhFace* face)
{
    for (int32_t i = 0; i < b->mergedCount; ++i)
    {
        m3QhListNode* head = &b->merged[i]->conflictListHead.link;
        m3QhListNode* node = head->next;
        while (node != head)
        {
            m3QhVertex* vertex = (m3QhVertex*)node;
            node = node->next;
            QhListRemove(&vertex->link);
            m3real distance = QhPlaneSeparation(face, vertex->position);
            if (distance > b->minOutside)
            {
                QhListPushBack(&face->conflictListHead.link, &vertex->link);
                vertex->conflictFace = face;
                if (distance > face->maxConflictDistance)
                {
                    face->maxConflictDistance = distance;
                    face->maxConflict = vertex;
                }
            }
            else
            {
                QhListPushBack(&b->orphanedList.link, &vertex->link);
                vertex->conflictFace = NULL;
            }
        }
        QhRetireFace(b, b->merged[i]);
    }
}

// Merge the face across `edge` into edge->face, walking shared runs
// outward, absorbing the opposing ring, then repairing with
// QhConnectEdges and a fresh Newell plane.
static void QhConnectFaces(m3QhBuilder* b, m3QhEdge* edge)
{
    m3QhFace* face = edge->face;
    m3QhEdge* twin = edge->twin;
    m3QhEdge* edgePrev = edge->prev;
    m3QhEdge* edgeNext = edge->next;
    m3QhEdge* twinPrev = twin->prev;
    m3QhEdge* twinNext = twin->next;

    while (edgePrev->twin->face == twin->face)
    {
        edgePrev = edgePrev->prev;
        twinNext = twinNext->next;
    }
    while (edgeNext->twin->face == twin->face)
    {
        edgeNext = edgeNext->next;
        twinPrev = twinPrev->prev;
    }

    face->edge = edgePrev;

    b->mergedCount = 0;
    b->merged[b->mergedCount++] = twin->face;
    twin->face->mark = M3_QH_MARK_DELETE;
    twin->face->edge = NULL;

    for (m3QhEdge* absorbed = twinNext; absorbed != twinPrev->next; absorbed = absorbed->next)
    {
        absorbed->face = face;
    }

    QhDestroyEdges(b, edgePrev->next, edgeNext);
    QhDestroyEdges(b, twinPrev->next, twinNext);
    QhConnectEdges(b, edgePrev, twinNext);
    QhConnectEdges(b, twinPrev, edgeNext);

    QhNewellPlane(face);
    QhRecacheConflicts(face, b->minOutside);
    QhAbsorbFaces(b, face);
}

static int QhMergeConcave(m3QhBuilder* b, m3QhFace* face)
{
    m3QhEdge* edge = face->edge;
    do
    {
        if (QhIsEdgeConcave(edge, b->minRadius) || QhIsEdgeConcave(edge->twin, b->minRadius))
        {
            QhConnectFaces(b, edge);
            return 1;
        }
        edge = edge->next;
    } while (edge != face->edge);
    return 0;
}

static int QhMergeCoplanar(m3QhBuilder* b, m3QhFace* face)
{
    m3QhEdge* edge = face->edge;
    do
    {
        if (!QhIsEdgeConvex(edge, b->minRadius) || !QhIsEdgeConvex(edge->twin, b->minRadius))
        {
            QhConnectFaces(b, edge);
            return 1;
        }
        edge = edge->next;
    } while (edge != face->edge);
    return 0;
}

static void QhMergeFaces(m3QhBuilder* b)
{
    // Pass one: cone faces whose plane put the interior point outside
    // (flipped) merge into their largest-area neighbor.
    for (int32_t i = 0; i < b->coneCount; ++i)
    {
        m3QhFace* face = b->cone[i];
        if (face->mark == M3_QH_MARK_VISIBLE && face->flipped)
        {
            face->flipped = 0;
            m3real bestArea = 0.0f;
            m3QhEdge* bestEdge = NULL;
            m3QhEdge* edge = face->edge;
            do
            {
                m3real area = edge->twin->face->area;
                if (area > bestArea)
                {
                    bestArea = area;
                    bestEdge = edge;
                }
                edge = edge->next;
            } while (edge != face->edge);
            QhConnectFaces(b, bestEdge);
        }
    }
    // Pass two and three: concave repair, then coplanar cleanup.
    for (int32_t i = 0; i < b->coneCount; ++i)
    {
        m3QhFace* face = b->cone[i];
        if (face->mark == M3_QH_MARK_VISIBLE)
        {
            while (QhMergeConcave(b, face))
            {
            }
        }
    }
    for (int32_t i = 0; i < b->coneCount; ++i)
    {
        m3QhFace* face = b->cone[i];
        if (face->mark == M3_QH_MARK_VISIBLE)
        {
            while (QhMergeCoplanar(b, face))
            {
            }
        }
    }
}

static void QhResolveVertices(m3QhBuilder* b)
{
    m3QhListNode* node = b->orphanedList.link.next;
    while (node != &b->orphanedList.link)
    {
        m3QhVertex* vertex = (m3QhVertex*)node;
        node = node->next;
        QhListRemove(&vertex->link);
        m3real maxDistance = b->minOutside;
        m3QhFace* maxFace = NULL;
        for (int32_t i = 0; i < b->coneCount; ++i)
        {
            if (b->cone[i]->mark == M3_QH_MARK_VISIBLE)
            {
                m3real distance = QhPlaneSeparation(b->cone[i], vertex->position);
                if (distance > maxDistance)
                {
                    maxDistance = distance;
                    maxFace = b->cone[i];
                }
            }
        }
        if (maxFace != NULL)
        {
            QhListPushBack(&maxFace->conflictListHead.link, &vertex->link);
            vertex->conflictFace = maxFace;
            if (maxDistance > maxFace->maxConflictDistance)
            {
                maxFace->maxConflictDistance = maxDistance;
                maxFace->maxConflict = vertex;
            }
        }
        // Otherwise interior: the pool slot is abandoned.
    }
}

static void QhResolveFaces(m3QhBuilder* b)
{
    m3QhListNode* node = b->faceList.link.next;
    while (node != &b->faceList.link)
    {
        m3QhFace* face = (m3QhFace*)node;
        node = node->next;
        if (face->mark == M3_QH_MARK_DELETE && QhListContains(&face->link))
        {
            QhListRemove(&face->link);
        }
    }
    for (int32_t i = 0; i < b->coneCount; ++i)
    {
        m3QhFace* face = b->cone[i];
        if (face->mark == M3_QH_MARK_DELETE)
        {
            continue;
        }
        QhListPushBack(&b->faceList.link, &face->link);
    }
}

static void QhAddVertexToHull(m3QhBuilder* b, m3QhVertex* vertex)
{
    m3QhFace* face = vertex->conflictFace;
    vertex->conflictFace = NULL;
    QhListRemove(&vertex->link);
    QhListPushBack(&b->vertexList.link, &vertex->link);

    b->horizonCount = 0;
    QhBuildHorizon(b, vertex, face);
    b->coneCount = 0;
    QhBuildCone(b, vertex);
    QhMergeFaces(b);
    QhResolveVertices(b);
    QhResolveFaces(b);
}

static void QhCleanHull(m3QhBuilder* b)
{
    int32_t faceCount = 0;
    int32_t halfEdgeCount = 0;
    for (m3QhListNode* node = b->faceList.link.next; node != &b->faceList.link; node = node->next)
    {
        m3QhFace* face = (m3QhFace*)node;
        m3QhEdge* edge = face->edge;
        do
        {
            edge->origin->reachable = 1;
            edge = edge->next;
            halfEdgeCount++;
        } while (edge != face->edge);
        faceCount++;
    }
    int32_t vertexCount = 0;
    m3QhListNode* node = b->vertexList.link.next;
    while (node != &b->vertexList.link)
    {
        m3QhVertex* vertex = (m3QhVertex*)node;
        node = node->next;
        if (!vertex->reachable)
        {
            QhListRemove(&vertex->link);
        }
        else
        {
            vertexCount++;
        }
    }
    b->finalVertexCount = vertexCount;
    b->finalHalfEdgeCount = halfEdgeCount;
    b->finalFaceCount = faceCount;
}

// Unit-density mass properties by the divergence theorem: fan each
// face into triangles against the first hull vertex, accumulate the
// signed tetra determinants (the reference integrator, byte for
// byte in structure). Returns 0 on non-positive volume.
static int QhMassProperties(m3HullData* hull)
{
    m3Vec3 origin = hull->vertices[0];
    m3real volume = 0.0f;
    m3Vec3 center = {0.0f, 0.0f, 0.0f};
    m3real xx = 0.0f;
    m3real yy = 0.0f;
    m3real zz = 0.0f;
    m3real xy = 0.0f;
    m3real xz = 0.0f;
    m3real yz = 0.0f;

    for (int32_t f = 0; f < hull->faceCount; ++f)
    {
        int32_t n = hull->faceVertCounts[f];
        int32_t start = hull->faceVertStart[f];
        m3Vec3 v1 = m3Sub3(hull->vertices[hull->faceIndices[start]], origin);
        for (int32_t k = 1; k + 1 < n; ++k)
        {
            m3Vec3 v2 = m3Sub3(hull->vertices[hull->faceIndices[start + k]], origin);
            m3Vec3 v3 = m3Sub3(hull->vertices[hull->faceIndices[start + k + 1]], origin);
            m3real det = m3Dot3(v1, m3Cross3(v2, v3));
            volume += det;
            m3Vec3 v4 = m3Add3(v1, m3Add3(v2, v3));
            center = m3Add3(center, m3MulSV3(det, v4));
            xx += det * (v1.x * v1.x + v2.x * v2.x + v3.x * v3.x + v4.x * v4.x);
            yy += det * (v1.y * v1.y + v2.y * v2.y + v3.y * v3.y + v4.y * v4.y);
            zz += det * (v1.z * v1.z + v2.z * v2.z + v3.z * v3.z + v4.z * v4.z);
            xy += det * (v1.x * v1.y + v2.x * v2.y + v3.x * v3.y + v4.x * v4.y);
            xz += det * (v1.x * v1.z + v2.x * v2.z + v3.x * v3.z + v4.x * v4.z);
            yz += det * (v1.y * v1.z + v2.y * v2.z + v3.y * v3.z + v4.y * v4.z);
        }
    }

    if (!(volume > 0.0f))
    {
        return 0;
    }
    m3Vec3 localCenter = m3MulSV3(0.25f / volume, center);
    m3real mass = volume / 6.0f;

    m3Mat3 inertia;
    inertia.cx = (m3Vec3){yy + zz, -xy, -xz};
    inertia.cy = (m3Vec3){-xy, xx + zz, -yz};
    inertia.cz = (m3Vec3){-xz, -yz, xx + yy};
    m3real scale = 1.0f / 120.0f;
    inertia.cx = m3MulSV3(scale, inertia.cx);
    inertia.cy = m3MulSV3(scale, inertia.cy);
    inertia.cz = m3MulSV3(scale, inertia.cz);

    // Steiner: shift from the fan origin to the center of mass.
    m3Vec3 c = localCenter;
    m3real c2 = m3Dot3(c, c);
    inertia.cx.x -= mass * (c2 - c.x * c.x);
    inertia.cy.y -= mass * (c2 - c.y * c.y);
    inertia.cz.z -= mass * (c2 - c.z * c.z);
    inertia.cy.x += mass * c.x * c.y;
    inertia.cx.y += mass * c.x * c.y;
    inertia.cz.x += mass * c.x * c.z;
    inertia.cx.z += mass * c.x * c.z;
    inertia.cz.y += mass * c.y * c.z;
    inertia.cy.z += mass * c.y * c.z;

    hull->unitMass = mass;
    hull->unitCom = m3Add3(localCenter, origin);
    hull->unitInertiaCom = inertia;
    return 1;
}

bool m3ComputeHull(const m3Vec3* points, int32_t count, m3HullData* out)
{
    if (points == NULL || out == NULL || count < 4 || count > M3_HULL_MAX_INPUT)
    {
        return false;
    }
    for (int32_t i = 0; i < count; ++i)
    {
        // Non-finite input is refused loudly, never built.
        if (!(points[i].x == points[i].x) || !(points[i].y == points[i].y) ||
            !(points[i].z == points[i].z) || points[i].x > 1.0e18f || points[i].x < -1.0e18f ||
            points[i].y > 1.0e18f || points[i].y < -1.0e18f || points[i].z > 1.0e18f ||
            points[i].z < -1.0e18f)
        {
            return false;
        }
    }

    // The work block is far too large for the stack; one allocation,
    // freed on every path out (the reference's single-block design).
    m3QhBuilder* b = (m3QhBuilder*)m3AllocZeroed((int32_t)sizeof(m3QhBuilder));
    if (b == NULL)
    {
        return false;
    }
    QhListInit(&b->orphanedList.link);
    QhListInit(&b->vertexList.link);
    QhListInit(&b->faceList.link);

    // Shift by the first point to reduce round-off (reference rule).
    m3Vec3 origin = points[0];
    for (int32_t i = 0; i < count; ++i)
    {
        b->shifted[i] = m3Sub3(points[i], origin);
    }

    QhComputeTolerance(b, count, b->shifted);
    if (!QhBuildInitialHull(b, count, b->shifted))
    {
        m3Free(b);
        return false;
    }

    int32_t budget = M3_HULL_MAX_VERTS - 4;
    m3QhVertex* vertex = QhNextConflictVertex(b);
    while (vertex != NULL && budget > 0)
    {
        QhAddVertexToHull(b, vertex);
        vertex = QhNextConflictVertex(b);
        budget -= 1;
    }

    QhCleanHull(b);

    // Euler identity and the fixed caps gate the emit.
    int32_t v = b->finalVertexCount;
    int32_t e = b->finalHalfEdgeCount / 2;
    int32_t f = b->finalFaceCount;
    if (v - e + f != 2 || f < 4 || v > M3_HULL_MAX_VERTS || f > M3_HULL_MAX_FACES ||
        b->finalHalfEdgeCount > M3_HULL_MAX_FACE_INDICES)
    {
        m3Free(b);
        return false;
    }

    memset(out, 0, sizeof(*out));
    int32_t vertexCount = 0;
    for (m3QhListNode* node = b->vertexList.link.next; node != &b->vertexList.link;
         node = node->next)
    {
        m3QhVertex* vtx = (m3QhVertex*)node;
        vtx->finalIndex = vertexCount;
        out->vertices[vertexCount] = m3Add3(vtx->position, origin);
        vertexCount += 1;
    }
    int32_t faceCount = 0;
    int32_t indexCount = 0;
    for (m3QhListNode* node = b->faceList.link.next; node != &b->faceList.link; node = node->next)
    {
        m3QhFace* face = (m3QhFace*)node;
        out->faceNormals[faceCount] = face->normal;
        out->faceOffsets[faceCount] = face->offset + m3Dot3(face->normal, origin);
        out->faceVertStart[faceCount] = (uint16_t)indexCount;
        int32_t n = 0;
        m3QhEdge* edge = face->edge;
        do
        {
            out->faceIndices[indexCount++] = (uint8_t)edge->origin->finalIndex;
            n += 1;
            edge = edge->next;
        } while (edge != face->edge);
        out->faceVertCounts[faceCount] = (uint8_t)n;
        faceCount += 1;
    }
    out->vertexCount = vertexCount;
    out->faceCount = faceCount;
    out->indexCount = indexCount;
    m3Free(b);

    if (!QhMassProperties(out))
    {
        memset(out, 0, sizeof(*out));
        return false;
    }
    m3HullBuildHalfEdges(out);
    return true;
}
