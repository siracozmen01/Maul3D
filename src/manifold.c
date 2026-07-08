// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Narrowphase v1: sphere-sphere and plane-sphere manifolds, the
// speculative margin, the deterministic tangent basis, and the
// warm-start carry. The collide kernels are pure functions of their
// inputs (the reference law); m3UpdateContacts walks the pairs in
// canonical order and matches impulses forward by feature id.

#include "world_internal.h"

#include <float.h>
#include <string.h>

// Contacts exist slightly before touch so the solver can catch
// approaches speculatively (the reference model).
#define M3_SPECULATIVE_DISTANCE M3_AABB_MARGIN

void m3MakeTangentBasis(m3Vec3 normal, m3Vec3* t1, m3Vec3* t2)
{
    m3real ax = m3AbsF(normal.x);
    m3real ay = m3AbsF(normal.y);
    m3real az = m3AbsF(normal.z);
    m3Vec3 axis;
    if (ax <= ay && ax <= az)
    {
        axis = (m3Vec3){1.0f, 0.0f, 0.0f};
    }
    else if (ay <= az)
    {
        axis = (m3Vec3){0.0f, 1.0f, 0.0f};
    }
    else
    {
        axis = (m3Vec3){0.0f, 0.0f, 1.0f};
    }
    *t1 = m3Normalize3(m3Cross3(normal, axis));
    *t2 = m3Cross3(normal, *t1);
}

m3Manifold m3CollideSpheres(m3Vec3 d, m3real radiusA, m3real radiusB)
{
    m3Manifold manifold;
    memset(&manifold, 0, sizeof(manifold));

    m3real distance = m3Length3(d);
    m3real separation = distance - radiusA - radiusB;
    if (separation > M3_SPECULATIVE_DISTANCE)
    {
        return manifold;
    }
    // Concentric centers take the fixed +y fallback (one rule, never
    // NaN, never caller-dependent).
    m3Vec3 normal = m3Normalize3(d);
    manifold.normal = normal;
    manifold.pointCount = 1;
    manifold.points[0].anchorA = m3MulSV3(radiusA, normal);
    manifold.points[0].anchorB = m3MulSV3(-radiusB, normal);
    manifold.points[0].separation = separation;
    manifold.points[0].id = 0;
    return manifold;
}

m3Manifold m3CollidePlaneSphere(m3Vec3 planeNormal, m3real dist, m3real radius)
{
    m3Manifold manifold;
    memset(&manifold, 0, sizeof(manifold));

    m3real separation = dist - radius;
    if (separation > M3_SPECULATIVE_DISTANCE)
    {
        return manifold;
    }
    manifold.normal = planeNormal; // A (plane) to B (sphere)
    manifold.pointCount = 1;
    // The sphere's deepest point toward the plane; anchorA is filled
    // by the contact update, which knows body A's center.
    manifold.points[0].anchorB = m3MulSV3(-radius, planeNormal);
    manifold.points[0].separation = separation;
    manifold.points[0].id = 0;
    return manifold;
}

// --- Hull-versus-hull SAT (2b-5b), adapted from the reference
// convex_manifold.c (Gauss-map edge pruning by Dirk Gregorius). All
// work happens in A's frame; B arrives via the float relative pose.

typedef struct m3FaceQuery
{
    m3real separation;
    int32_t faceIndex;
} m3FaceQuery;

// Deepest support of `hull` (in its own frame, transformed by q,p into
// the query frame) against each face plane of `ref`.
static m3FaceQuery QueryFaces(const m3HullData* ref, const m3HullData* other, m3Quat q, m3Vec3 p,
                              int refIsA)
{
    m3FaceQuery query;
    query.separation = -3.4e38f;
    query.faceIndex = 0;
    // The other hull's vertices depend on the frame, not the face:
    // transform once (2c-11, the profile's first ask). Same inputs,
    // same operations, so every dot below sees bit-identical values.
    m3Vec3 w[M3_HULL_MAX_VERTS];
    for (int32_t v = 0; v < other->vertexCount; ++v)
    {
        w[v] = refIsA ? m3Add3(m3RotateVec3(q, other->vertices[v]), p)
                      : m3InvRotateVec3(q, m3Sub3(other->vertices[v], p));
    }
    for (int32_t f = 0; f < ref->faceCount; ++f)
    {
        m3Vec3 n = ref->faceNormals[f];
        m3real off = ref->faceOffsets[f];
        m3real best = 3.4e38f;
        for (int32_t v = 0; v < other->vertexCount; ++v)
        {
            m3real d = m3Dot3(n, w[v]) - off;
            best = m3MinF(best, d);
            if (best <= query.separation)
            {
                // This face cannot win the max: the update below
                // would not fire either way, so the break is
                // bit-invisible.
                break;
            }
        }
        if (best > query.separation)
        {
            query.separation = best;
            query.faceIndex = f;
            if (best > M3_SPECULATIVE_DISTANCE)
            {
                // A separating face is a verdict, not a candidate:
                // every caller returns the empty manifold on it, so
                // the rest of the loop can only refine a number
                // nobody reads.
                return query;
            }
        }
    }
    return query;
}

typedef struct m3EdgeQuery
{
    m3real separation;
    int32_t indexA; // half-edge slots (even = one per undirected edge)
    int32_t indexB;
    m3Vec3 axis; // A to B, A's frame
} m3EdgeQuery;

static m3EdgeQuery QueryEdges(const m3HullData* hullA, const m3HullData* hullB, m3Quat q, m3Vec3 p)
{
    m3EdgeQuery query;
    query.separation = -3.4e38f;
    query.indexA = -1;
    query.indexB = -1;
    query.axis = (m3Vec3){0.0f, 1.0f, 0.0f};

    m3Vec3 centerB = m3Add3(m3RotateVec3(q, hullB->center), p);
    for (int32_t ib = 0; ib < hullB->edgeCount; ib += 2)
    {
        const m3HullHalfEdge* edgeB = &hullB->edges[ib];
        const m3HullHalfEdge* twinB = &hullB->edges[ib + 1];
        m3Vec3 qB = m3Add3(m3RotateVec3(q, hullB->vertices[twinB->origin]), p);
        m3Vec3 pB = m3Add3(m3RotateVec3(q, hullB->vertices[edgeB->origin]), p);
        m3Vec3 eB = m3Sub3(qB, pB);
        m3Vec3 uB = m3RotateVec3(q, hullB->faceNormals[edgeB->face]);
        m3Vec3 vB = m3RotateVec3(q, hullB->faceNormals[twinB->face]);

        for (int32_t ia = 0; ia < hullA->edgeCount; ia += 2)
        {
            const m3HullHalfEdge* edgeA = &hullA->edges[ia];
            const m3HullHalfEdge* twinA = &hullA->edges[ia + 1];
            m3Vec3 pA = hullA->vertices[edgeA->origin];
            m3Vec3 qA = hullA->vertices[twinA->origin];
            m3Vec3 eA = m3Sub3(qA, pA);
            m3Vec3 uA = hullA->faceNormals[edgeA->face];
            m3Vec3 vA = hullA->faceNormals[twinA->face];

            // Gauss map: the two edges build a Minkowski face only if
            // the arcs cross (the reference formulation).
            m3real cba = m3Dot3(uB, eA);
            m3real dba = m3Dot3(vB, eA);
            m3real adc = -m3Dot3(uA, eB);
            m3real bdc = -m3Dot3(vA, eB);
            if (!(cba * dba < 0.0f && adc * bdc < 0.0f && cba * bdc > 0.0f))
            {
                continue;
            }

            m3Vec3 axis = m3Cross3(eA, eB);
            m3real len = m3Length3(axis);
            if (len < 1.0e-6f)
            {
                continue; // parallel edges never make the axis
            }
            axis = m3MulSV3(1.0f / len, axis);
            // Orient away from A's center.
            if (m3Dot3(axis, m3Sub3(pA, hullA->center)) < 0.0f)
            {
                axis = m3Neg3(axis);
            }
            m3real separation = m3Dot3(axis, m3Sub3(pB, pA));
            (void)centerB;
            if (separation > query.separation)
            {
                query.separation = separation;
                query.indexA = ia;
                query.indexB = ib;
                query.axis = axis;
                if (separation > M3_SPECULATIVE_DISTANCE)
                {
                    // Same verdict rule as the face query: any
                    // separating edge axis means the caller returns
                    // empty, so stop refining.
                    return query;
                }
            }
        }
    }
    return query;
}

// Closest points between two segments (edge contact).
static void SegmentClosest(m3Vec3 p1, m3Vec3 d1, m3Vec3 p2, m3Vec3 d2, m3Vec3* c1, m3Vec3* c2)
{
    m3Vec3 r = m3Sub3(p1, p2);
    m3real a = m3Dot3(d1, d1);
    m3real e = m3Dot3(d2, d2);
    m3real f = m3Dot3(d2, r);
    m3real c = m3Dot3(d1, r);
    m3real b = m3Dot3(d1, d2);
    m3real denom = a * e - b * b;
    m3real s = denom > 1.0e-9f ? m3ClampF((b * f - c * e) / denom, 0.0f, 1.0f) : 0.0f;
    m3real t = e > 1.0e-9f ? m3ClampF((b * s + f) / e, 0.0f, 1.0f) : 0.0f;
    s = a > 1.0e-9f ? m3ClampF((b * t - c) / a, 0.0f, 1.0f) : 0.0f;
    *c1 = m3Add3(p1, m3MulSV3(s, d1));
    *c2 = m3Add3(p2, m3MulSV3(t, d2));
}

m3Manifold m3CollideHulls(const m3HullData* hullA, const m3HullData* hullB, m3Quat q, m3Vec3 p)
{
    m3Manifold manifold;
    memset(&manifold, 0, sizeof(manifold));

    m3FaceQuery faceA = QueryFaces(hullA, hullB, q, p, 1);
    if (faceA.separation > M3_SPECULATIVE_DISTANCE)
    {
        return manifold;
    }
    m3FaceQuery faceB = QueryFaces(hullB, hullA, q, p, 0);
    if (faceB.separation > M3_SPECULATIVE_DISTANCE)
    {
        return manifold;
    }
    m3EdgeQuery edge = QueryEdges(hullA, hullB, q, p);
    if (edge.indexA >= 0 && edge.separation > M3_SPECULATIVE_DISTANCE)
    {
        return manifold;
    }

    const m3real linearSlop = 0.005f;
    m3real maxFace = m3MaxF(faceA.separation, faceB.separation);
    if (edge.indexA >= 0 && edge.separation > maxFace + 0.1f * linearSlop)
    {
        // Edge contact: the crossing edges' closest points.
        const m3HullHalfEdge* eA = &hullA->edges[edge.indexA];
        const m3HullHalfEdge* tA = &hullA->edges[edge.indexA + 1];
        const m3HullHalfEdge* eB = &hullB->edges[edge.indexB];
        const m3HullHalfEdge* tB = &hullB->edges[edge.indexB + 1];
        m3Vec3 pA = hullA->vertices[eA->origin];
        m3Vec3 dA = m3Sub3(hullA->vertices[tA->origin], pA);
        m3Vec3 pB = m3Add3(m3RotateVec3(q, hullB->vertices[eB->origin]), p);
        m3Vec3 dB = m3Sub3(m3Add3(m3RotateVec3(q, hullB->vertices[tB->origin]), p), pB);
        m3Vec3 cA;
        m3Vec3 cB;
        SegmentClosest(pA, dA, pB, dB, &cA, &cB);
        manifold.normal = edge.axis;
        manifold.pointCount = 1;
        manifold.points[0].anchorA = cA; // frame A anchors; caller re-bases
        manifold.points[0].anchorB = cB;
        manifold.points[0].separation = edge.separation;
        manifold.points[0].id = (uint16_t)(0x8000u | (((uint32_t)edge.indexA >> 1) << 7) |
                                           ((uint32_t)edge.indexB >> 1));
        return manifold;
    }

    // Face contact: the reference face is the larger separation side
    // (B wins ties by the slop margin, the reference rule).
    int refIsA = faceB.separation <= faceA.separation + 0.1f * linearSlop ? 1 : 0;
    const m3HullData* ref = refIsA ? hullA : hullB;
    const m3HullData* inc = refIsA ? hullB : hullA;
    int32_t refFace = refIsA ? faceA.faceIndex : faceB.faceIndex;
    m3Vec3 refN = ref->faceNormals[refFace];
    m3real refOff = ref->faceOffsets[refFace];

    // The incident face: most anti-parallel on the incident hull, in
    // the REF frame.
    m3Vec3 refNInc = refIsA ? m3InvRotateVec3(q, refN) : m3RotateVec3(q, refN);
    int32_t incFace = 0;
    m3real minDot = 3.4e38f;
    for (int32_t f = 0; f < inc->faceCount; ++f)
    {
        m3real d = m3Dot3(refNInc, inc->faceNormals[f]);
        if (d < minDot)
        {
            minDot = d;
            incFace = f;
        }
    }

    // Incident polygon into the ref frame.
    m3Vec3 poly[M3_HULL_MAX_FACE_INDICES];
    uint16_t polyId[M3_HULL_MAX_FACE_INDICES];
    int32_t polyCount = inc->faceVertCounts[incFace];
    for (int32_t k = 0; k < polyCount; ++k)
    {
        uint8_t vi = inc->faceIndices[inc->faceVertStart[incFace] + k];
        m3Vec3 v = inc->vertices[vi];
        poly[k] = refIsA ? m3Add3(m3RotateVec3(q, v), p) : m3InvRotateVec3(q, m3Sub3(v, p));
        polyId[k] = vi;
    }

    // Sutherland-Hodgman against the reference side planes (one per
    // reference-face edge, normal = tangent x refN pointing outward).
    int32_t n = ref->faceVertCounts[refFace];
    int32_t start = ref->faceVertStart[refFace];
    for (int32_t e = 0; e < n && polyCount > 0; ++e)
    {
        m3Vec3 v1 = ref->vertices[ref->faceIndices[start + e]];
        m3Vec3 v2 = ref->vertices[ref->faceIndices[start + (e + 1) % n]];
        m3Vec3 tangent = m3Normalize3(m3Sub3(v2, v1));
        m3Vec3 sideN = m3Cross3(tangent, refN); // points outward for CCW
        m3real sideOff = m3Dot3(sideN, v1);

        m3Vec3 outPoly[M3_HULL_MAX_FACE_INDICES];
        uint16_t outId[M3_HULL_MAX_FACE_INDICES];
        int32_t outCount = 0;
        for (int32_t k = 0; k < polyCount; ++k)
        {
            m3Vec3 cur = poly[k];
            m3Vec3 nxt = poly[(k + 1) % polyCount];
            m3real dc = m3Dot3(sideN, cur) - sideOff;
            m3real dn = m3Dot3(sideN, nxt) - sideOff;
            if (dc <= 0.0f)
            {
                outPoly[outCount] = cur;
                outId[outCount] = polyId[k];
                outCount += 1;
            }
            if (dc * dn < 0.0f)
            {
                m3real t = dc / (dc - dn);
                outPoly[outCount] = m3Add3(m3MulSV3(1.0f - t, cur), m3MulSV3(t, nxt));
                // A clipped vertex takes a synthetic id from the side
                // plane and the segment, stable per configuration.
                outId[outCount] = (uint16_t)(0x4000u | ((uint32_t)e << 8) | polyId[k]);
                outCount += 1;
            }
        }
        memcpy(poly, outPoly, (size_t)outCount * sizeof(m3Vec3));
        memcpy(polyId, outId, (size_t)outCount * sizeof(uint16_t));
        polyCount = outCount;
    }

    // Keep points below the margin, deepest four, ascending id.
    int32_t candIdx[M3_HULL_MAX_FACE_INDICES];
    m3real candSep[M3_HULL_MAX_FACE_INDICES];
    int32_t candCount = 0;
    for (int32_t k = 0; k < polyCount; ++k)
    {
        m3real sep = m3Dot3(refN, poly[k]) - refOff;
        if (sep < M3_SPECULATIVE_DISTANCE)
        {
            candIdx[candCount] = k;
            candSep[candCount] = sep;
            candCount += 1;
        }
    }
    int32_t want = candCount < M3_MANIFOLD_MAX_POINTS ? candCount : M3_MANIFOLD_MAX_POINTS;
    uint8_t used[M3_HULL_MAX_FACE_INDICES];
    memset(used, 0, sizeof(used));
    int32_t kept[M3_MANIFOLD_MAX_POINTS];
    for (int32_t k = 0; k < want; ++k)
    {
        int32_t best = -1;
        for (int32_t c = 0; c < candCount; ++c)
        {
            if (used[c])
            {
                continue;
            }
            if (best < 0 || candSep[c] < candSep[best] ||
                (candSep[c] == candSep[best] && polyId[candIdx[c]] < polyId[candIdx[best]]))
            {
                best = c;
            }
        }
        used[best] = 1;
        kept[k] = best;
    }
    for (int32_t a = 0; a < want; ++a)
    {
        for (int32_t b = a + 1; b < want; ++b)
        {
            if (polyId[candIdx[kept[b]]] < polyId[candIdx[kept[a]]])
            {
                int32_t tmp = kept[a];
                kept[a] = kept[b];
                kept[b] = tmp;
            }
        }
    }

    // Emit in A's frame with the A-to-B normal.
    m3Vec3 outN = refIsA ? refN : m3Neg3(m3RotateVec3(q, refN));
    manifold.normal = outN;
    manifold.pointCount = want;
    for (int32_t k = 0; k < want; ++k)
    {
        int32_t c = kept[k];
        m3Vec3 onInc = poly[candIdx[c]]; // ref frame, incident side
        m3real sep = candSep[c];
        m3Vec3 onRef = m3Sub3(onInc, m3MulSV3(sep, refN));
        // Into A's frame.
        m3Vec3 wInc = refIsA ? onInc : m3Add3(m3RotateVec3(q, onInc), p);
        m3Vec3 wRef = refIsA ? onRef : m3Add3(m3RotateVec3(q, onRef), p);
        manifold.points[k].anchorA = refIsA ? wRef : wInc;
        manifold.points[k].anchorB = refIsA ? wInc : wRef;
        manifold.points[k].separation = sep;
        manifold.points[k].id = (uint16_t)(((uint32_t)refIsA << 15) | ((uint32_t)refFace << 8) |
                                           (polyId[candIdx[c]] & 0xFFu));
    }
    return manifold;
}

// World center of a shape's sphere (double positions, float offsets).
static void SphereWorldCenter(const m3World* world, int32_t shape, double* cx, double* cy,
                              double* cz)
{
    int32_t body = world->shapeBody[shape];
    const m3Transform* xf = &world->transforms[body];
    m3Vec3 r = m3RotateVec3(xf->q, world->shapeGeom[shape].v);
    *cx = xf->p.x + (double)r.x;
    *cy = xf->p.y + (double)r.y;
    *cz = xf->p.z + (double)r.z;
}

// Offset from a body's world center of mass to a world point (float
// is exact enough near contact). Anchors are COM-relative because
// impulses and rotation act about the COM (2b-1).
static m3Vec3 FromCom(const m3World* world, int32_t body, double px, double py, double pz)
{
    const m3Transform* xf = &world->transforms[body];
    m3Vec3 rlc = m3RotateVec3(xf->q, world->localCenters[body]);
    return (m3Vec3){(m3real)(px - xf->p.x - (double)rlc.x), (m3real)(py - xf->p.y - (double)rlc.y),
                    (m3real)(pz - xf->p.z - (double)rlc.z)};
}

// Build the GJK proxy for one shape in its own local frame. Spheres
// and capsules park their point(s) in the caller's scratch (the proxy
// only borrows the pointer); hulls point straight at the interned
// vertex array.
m3DistanceProxy m3MakeShapeProxy(const m3World* world, int32_t shape, m3Vec3 scratch[2])
{
    m3DistanceProxy proxy;
    uint8_t type = world->shapeType[shape];
    if (type == (uint8_t)m3_hullShape)
    {
        const m3HullData* hull = &world->hullData[world->shapeHullIndex[shape]];
        proxy.points = hull->vertices;
        proxy.count = hull->vertexCount;
        proxy.radius = 0.0f;
        return proxy;
    }
    if (type == (uint8_t)m3_capsuleShape)
    {
        scratch[0] = world->shapeGeom[shape].v;
        scratch[1] = world->shapeGeom[shape].v2;
        proxy.points = scratch;
        proxy.count = 2;
        proxy.radius = world->shapeGeom[shape].s;
        return proxy;
    }
    // Sphere (planes never reach the GJK path).
    scratch[0] = world->shapeGeom[shape].v;
    proxy.points = scratch;
    proxy.count = 1;
    proxy.radius = world->shapeGeom[shape].s;
    return proxy;
}

// Exact deep recovery, part one: a point core strictly inside a hull.
// The least-deep face (max signed distance, ties to the lower face
// index) IS the minimum translation: moving by -d along its normal
// reaches the supporting plane, so the point leaves the hull, and any
// smaller move keeps every face constraint strictly negative. No
// iteration, no polytope, nothing to make deterministic after the
// fact. The projected witness can land off the face polygon in
// obtuse corners; the normal and depth stay exact and the anchor
// error is bounded by one face span (the reference accepts the same).
static void DeepPointInHull(const m3HullData* hull, m3Vec3 q, m3Vec3* normalOut, m3real* coreSepOut,
                            m3Vec3* onHullOut)
{
    int32_t best = 0;
    m3real bestD = -3.4e38f;
    for (int32_t f = 0; f < hull->faceCount; ++f)
    {
        m3real d = m3Dot3(hull->faceNormals[f], q) - hull->faceOffsets[f];
        if (d > bestD)
        {
            bestD = d;
            best = f;
        }
    }
    *normalOut = hull->faceNormals[best];
    *coreSepOut = bestD;
    *onHullOut = m3Sub3(q, m3MulSV3(bestD, hull->faceNormals[best]));
}

// Capsule versus hull, one path for every depth: the segment SAT.
// Axes are the hull faces plus every hull edge crossed with the
// segment direction (the complete set for a convex against a
// segment). A winning face clips the segment's parameter interval
// against the face side planes and contacts BOTH clipped ends, which
// is what lets a lying capsule rest instead of wobbling on GJK's one
// witness; a winning edge takes the closest-point contact. In
// vertex-region approaches these axes underestimate the true
// distance, so a speculative point can appear a touch early; that
// only pre-arms the solver's speculative band and cannot snag.
// Results in the hull frame: anchorA on the hull, anchorB on the
// capsule skin, normal hull toward capsule.
static m3Manifold CollideSegmentHull(const m3HullData* hull, m3Vec3 p1, m3Vec3 p2, m3real radius)
{
    const m3real linearSlop = 0.005f;
    m3Manifold manifold;
    memset(&manifold, 0, sizeof(manifold));

    int32_t bestFace = 0;
    m3real bestFaceSep = -3.4e38f;
    for (int32_t f = 0; f < hull->faceCount; ++f)
    {
        m3real d1 = m3Dot3(hull->faceNormals[f], p1) - hull->faceOffsets[f];
        m3real d2 = m3Dot3(hull->faceNormals[f], p2) - hull->faceOffsets[f];
        m3real sep = m3MinF(d1, d2);
        if (sep > bestFaceSep)
        {
            bestFaceSep = sep;
            bestFace = f;
        }
    }

    m3Vec3 segDir = m3Sub3(p2, p1);
    m3Vec3 bestEdgeAxis = {0.0f, 1.0f, 0.0f};
    m3real bestEdgeSep = -3.4e38f;
    int32_t bestEdge = -1;
    for (int32_t e = 0; e < hull->edgeCount; e += 2)
    {
        m3Vec3 a = hull->vertices[hull->edges[e].origin];
        m3Vec3 b = hull->vertices[hull->edges[e + 1].origin];
        m3Vec3 axis = m3Cross3(m3Sub3(b, a), segDir);
        m3real len2 = m3Dot3(axis, axis);
        if (len2 < 1.0e-10f)
        {
            continue; // parallel: a face axis covers this direction
        }
        axis = m3MulSV3(1.0f / sqrtf(len2), axis);
        if (m3Dot3(axis, m3Sub3(a, hull->center)) < 0.0f)
        {
            axis = m3Neg3(axis); // outward from the hull
        }
        m3real hullMax = -3.4e38f;
        for (int32_t v = 0; v < hull->vertexCount; ++v)
        {
            m3real proj = m3Dot3(axis, hull->vertices[v]);
            if (proj > hullMax)
            {
                hullMax = proj;
            }
        }
        m3real sep = m3MinF(m3Dot3(axis, p1), m3Dot3(axis, p2)) - hullMax;
        if (sep > bestEdgeSep)
        {
            bestEdgeSep = sep;
            bestEdgeAxis = axis;
            bestEdge = e;
        }
    }

    int edgeWins = bestEdge >= 0 && bestEdgeSep > bestFaceSep + 0.1f * linearSlop;
    m3real coreSep = edgeWins ? bestEdgeSep : bestFaceSep;
    if (coreSep - radius > M3_SPECULATIVE_DISTANCE)
    {
        return manifold;
    }

    if (edgeWins)
    {
        m3Vec3 a = hull->vertices[hull->edges[bestEdge].origin];
        m3Vec3 b = hull->vertices[hull->edges[bestEdge + 1].origin];
        m3Vec3 cSeg;
        m3Vec3 cEdge;
        SegmentClosest(p1, segDir, a, m3Sub3(b, a), &cSeg, &cEdge);
        manifold.normal = bestEdgeAxis;
        manifold.pointCount = 1;
        manifold.points[0].anchorA = cEdge;
        manifold.points[0].anchorB = m3Sub3(cSeg, m3MulSV3(radius, bestEdgeAxis));
        manifold.points[0].separation = bestEdgeSep - radius;
        manifold.points[0].id = (uint16_t)(0x8000u | (uint32_t)bestEdge);
        return manifold;
    }

    m3Vec3 n = hull->faceNormals[bestFace];
    m3real off = hull->faceOffsets[bestFace];
    int32_t count = hull->faceVertCounts[bestFace];
    int32_t startIdx = hull->faceVertStart[bestFace];
    m3Vec3 centroid = {0.0f, 0.0f, 0.0f};
    for (int32_t k = 0; k < count; ++k)
    {
        centroid = m3Add3(centroid, hull->vertices[hull->faceIndices[startIdx + k]]);
    }
    centroid = m3MulSV3(1.0f / (m3real)count, centroid);

    m3real t0 = 0.0f;
    m3real t1 = 1.0f;
    int emptySlab = 0;
    for (int32_t k = 0; k < count; ++k)
    {
        m3Vec3 a = hull->vertices[hull->faceIndices[startIdx + k]];
        m3Vec3 b = hull->vertices[hull->faceIndices[startIdx + (k + 1) % count]];
        m3Vec3 sideN = m3Cross3(m3Sub3(b, a), n);
        if (m3Dot3(sideN, m3Sub3(centroid, a)) < 0.0f)
        {
            sideN = m3Neg3(sideN); // inward: keep the face side
        }
        m3real c0 = m3Dot3(sideN, m3Sub3(p1, a));
        m3real cd = m3Dot3(sideN, segDir);
        if (cd > -1.0e-9f && cd < 1.0e-9f)
        {
            if (c0 < 0.0f)
            {
                emptySlab = 1;
                break;
            }
            continue;
        }
        m3real tc = -c0 / cd;
        if (cd > 0.0f)
        {
            t0 = m3MaxF(t0, tc);
        }
        else
        {
            t1 = m3MinF(t1, tc);
        }
    }
    if (emptySlab || t0 > t1)
    {
        // Grazing a corner outside the face slab: one clamped point,
        // the deterministic middle of the crossed-over interval.
        m3real tm = 0.5f * (t0 + t1);
        tm = m3MaxF(0.0f, m3MinF(1.0f, tm));
        t0 = tm;
        t1 = tm;
    }

    int32_t emitted = 0;
    for (int32_t k = 0; k < 2; ++k)
    {
        if (k == 1 && !(t1 > t0))
        {
            break; // degenerate interval: one point only
        }
        m3real t = k == 0 ? t0 : t1;
        m3Vec3 pt = m3Add3(p1, m3MulSV3(t, segDir));
        m3real d = m3Dot3(n, pt) - off;
        m3real sepK = d - radius;
        if (sepK > M3_SPECULATIVE_DISTANCE)
        {
            continue;
        }
        manifold.points[emitted].anchorA = m3Sub3(pt, m3MulSV3(d, n));
        manifold.points[emitted].anchorB = m3Sub3(pt, m3MulSV3(radius, n));
        manifold.points[emitted].separation = sepK;
        manifold.points[emitted].id = (uint16_t)k;
        emitted += 1;
    }
    manifold.normal = n;
    manifold.pointCount = emitted;
    return manifold;
}

// ---------------------------------------------------------------
// Mesh versus convex (2b-9a sphere, 2b-9b capsule), the reference
// mesh_contact.c recipe. Feature = the closest voronoi region as a
// vertex bitmask (1|2|4; 7 = face). Face contacts are accepted
// immediately and CLAIM their triangle's edges and vertices; edge
// and vertex contacts are tentative, sorted by distance, accepted
// only while their feature is unclaimed. That filter is the
// internal-edge mitigation. Accepted manifolds then merge into ONE
// pair manifold by normal cluster around the deepest contact (the
// same-normal flat-floor case merges perfectly; the multi-normal
// valley keeps only its dominant cluster until per-manifold solver
// normals arrive with 2b-9c).
// ---------------------------------------------------------------

typedef struct m3TriPoint
{
    m3Vec3 point;
    int32_t feature; // vertex bitmask, 7 = face interior
} m3TriPoint;

// Closest point on a triangle with its voronoi feature (the
// reference math_functions.c routine, byte for byte in structure).
static m3TriPoint ClosestPointOnTriangle(m3Vec3 a, m3Vec3 b, m3Vec3 c, m3Vec3 q)
{
    m3Vec3 ab = m3Sub3(b, a);
    m3Vec3 ac = m3Sub3(c, a);
    m3Vec3 aq = m3Sub3(q, a);
    m3real d1 = m3Dot3(ab, aq);
    m3real d2 = m3Dot3(ac, aq);
    if (d1 <= 0.0f && d2 <= 0.0f)
    {
        return (m3TriPoint){a, 1};
    }
    m3Vec3 bq = m3Sub3(q, b);
    m3real d3 = m3Dot3(ab, bq);
    m3real d4 = m3Dot3(ac, bq);
    if (d3 > 0.0f && d4 <= d3)
    {
        return (m3TriPoint){b, 2};
    }
    m3real vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
    {
        m3real t = d1 / (d1 - d3);
        return (m3TriPoint){m3Add3(a, m3MulSV3(t, ab)), 1 | 2};
    }
    m3Vec3 cq = m3Sub3(q, c);
    m3real d5 = m3Dot3(ab, cq);
    m3real d6 = m3Dot3(ac, cq);
    if (d6 >= 0.0f && d5 <= d6)
    {
        return (m3TriPoint){c, 4};
    }
    m3real vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
    {
        m3real t = d2 / (d2 - d6);
        return (m3TriPoint){m3Add3(a, m3MulSV3(t, ac)), 1 | 4};
    }
    m3real va = d3 * d6 - d5 * d4;
    if (va <= 0.0f && d4 >= d3 && d5 >= d6)
    {
        m3real t = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return (m3TriPoint){m3Add3(b, m3MulSV3(t, m3Sub3(c, b))), 2 | 4};
    }
    m3real t1 = vb / (va + vb + vc);
    m3real t2 = vc / (va + vb + vc);
    m3Vec3 p = m3Add3(a, m3Add3(m3MulSV3(t1, ab), m3MulSV3(t2, ac)));
    return (m3TriPoint){p, 7};
}

// One triangle's local manifold: midway contact points (each anchor
// recovers as point -/+ half the separation along the normal).
// feature: vertex bitmask (1|2|4, 7 = triangle face) or 8 = hull
// face contact (the hull path's special acceptance rules).
#define M3_TRI_FEATURE_HULL_FACE 8

typedef struct m3TriManifold
{
    m3Vec3 normal;    // mesh frame, triangle toward shape
    m3Vec3 triNormal; // mesh frame (the hull-face acceptance reads it)
    m3real dist2;     // closest squared distance (the tentative sort key)
    int32_t pointCount;
    int32_t feature;
    m3Vec3 point[4];
    m3real separation[4];
    uint16_t localId[4];
} m3TriManifold;

static void CollideSphereTriangle(m3TriManifold* out, m3Vec3 center, m3real radius,
                                  const m3Vec3 tri[3])
{
    out->pointCount = 0;
    m3Vec3 triN = m3Cross3(m3Sub3(tri[1], tri[0]), m3Sub3(tri[2], tri[0]));
    if (m3Dot3(triN, m3Sub3(center, tri[0])) < 0.0f)
    {
        return; // back side cull (CCW winding, outward normals)
    }
    m3TriPoint closest = ClosestPointOnTriangle(tri[0], tri[1], tri[2], center);
    m3Vec3 d = m3Sub3(center, closest.point);
    m3real dist2 = m3Dot3(d, d);
    m3real reach = radius + M3_SPECULATIVE_DISTANCE;
    if (dist2 > reach * reach)
    {
        return;
    }
    m3real dist = sqrtf(dist2);
    m3Vec3 normal = dist2 > 1000.0f * FLT_MIN ? m3MulSV3(1.0f / dist, d) : m3Normalize3(triN);
    out->normal = normal;
    out->triNormal = m3Normalize3(triN);
    out->dist2 = dist2;
    out->feature = closest.feature;
    out->pointCount = 1;
    out->separation[0] = dist - radius;
    out->point[0] = m3MulSV3(0.5f, m3Add3(m3Sub3(center, m3MulSV3(radius, normal)), closest.point));
    out->localId[0] = 0;
}

// Clip the capsule segment to the triangle's side planes (reference
// b3ClipSegmentToTriangleFace).
static int ClipSegmentToTriFace(m3Vec3 segment[2], const m3Vec3 tri[3], m3Vec3 faceNormal)
{
    m3Vec3 vertex1 = tri[2];
    for (int32_t i = 0; i < 3; ++i)
    {
        m3Vec3 vertex2 = tri[i];
        m3Vec3 tangent = m3Normalize3(m3Sub3(vertex2, vertex1));
        m3Vec3 binormal = m3Cross3(tangent, faceNormal);
        m3real planeOff = m3Dot3(binormal, vertex1);

        m3Vec3 p1 = segment[0];
        m3Vec3 p2 = segment[1];
        m3real distance1 = m3Dot3(binormal, p1) - planeOff;
        m3real distance2 = m3Dot3(binormal, p2) - planeOff;
        int32_t vertexCount = 0;
        if (distance1 <= 0.0f)
        {
            segment[vertexCount++] = p1;
        }
        if (distance2 <= 0.0f)
        {
            segment[vertexCount++] = p2;
        }
        if (distance1 * distance2 < 0.0f)
        {
            m3real t = distance1 / (distance1 - distance2);
            segment[vertexCount] = m3Add3(p1, m3MulSV3(t, m3Sub3(p2, p1)));
            vertexCount++;
        }
        if (vertexCount != 2)
        {
            return 0;
        }
        vertex1 = vertex2;
    }
    return 1;
}

// Closest points between two infinite lines, with fractions
// (reference b3LineDistance).
static void LineClosest(m3Vec3 p1, m3Vec3 d1, m3Vec3 p2, m3Vec3 d2, m3real* f1, m3real* f2)
{
    m3real a11 = m3Dot3(d1, d1);
    m3real a12 = -m3Dot3(d1, d2);
    m3real a21 = m3Dot3(d2, d1);
    m3real a22 = -m3Dot3(d2, d2);
    m3Vec3 w = m3Sub3(p1, p2);
    m3real b1 = -m3Dot3(d1, w);
    m3real b2 = -m3Dot3(d2, w);
    m3real det = a11 * a22 - a12 * a21;
    if (det * det < 1000.0f * FLT_MIN)
    {
        *f1 = a11 > 0.0f ? m3Dot3(m3Sub3(p2, p1), d1) / a11 : 0.0f;
        *f2 = 0.0f;
        return;
    }
    *f1 = (a22 * b1 - a12 * b2) / det;
    *f2 = (a11 * b2 - a21 * b1) / det;
}

// Edge-edge separation with the volume-sign orientation guard
// (reference manifold.c b3EdgeEdgeSeparation).
static m3real EdgeEdgeSep(m3Vec3 p1, m3Vec3 e1, m3Vec3 c1, m3Vec3 p2, m3Vec3 e2, m3Vec3 c2)
{
    m3Vec3 u = m3Cross3(e1, e2);
    m3real length2 = m3Dot3(u, u);
    m3real scale2 = m3Dot3(e1, e1) * m3Dot3(e2, e2);
    if (length2 < 0.005f * 0.005f * scale2 || length2 < 1000.0f * FLT_MIN)
    {
        return -3.4e38f; // near parallel: a face axis covers it
    }
    m3Vec3 n = m3MulSV3(1.0f / sqrtf(length2), u);
    m3real sign1 = m3Dot3(n, m3Sub3(p1, c1));
    m3real sign2 = m3Dot3(n, m3Sub3(p2, c2));
    m3real a1 = sign1 < 0.0f ? -sign1 : sign1;
    m3real a2 = sign2 < 0.0f ? -sign2 : sign2;
    if (a1 > a2)
    {
        if (sign1 < 0.0f)
        {
            n = m3Neg3(n);
        }
    }
    else
    {
        if (sign2 > 0.0f)
        {
            n = m3Neg3(n);
        }
    }
    return m3Dot3(n, m3Sub3(p2, p1));
}

// Capsule versus one triangle (reference b3CollideCapsuleAndTriangle):
// GJK shallow path with the two-point face clip when the closest axis
// is near the face normal, single closest point otherwise; deep path
// by face and edge queries with the reference tolerance rule.
static void CollideCapsuleTriangle(m3TriManifold* out, m3Vec3 c1, m3Vec3 c2, m3real radius,
                                   const m3Vec3 tri[3])
{
    out->pointCount = 0;
    m3Vec3 triN = m3Normalize3(m3Cross3(m3Sub3(tri[1], tri[0]), m3Sub3(tri[2], tri[0])));
    m3real triOff = m3Dot3(triN, tri[0]);
    m3Vec3 mid = m3MulSV3(0.5f, m3Add3(c1, c2));
    if (m3Dot3(triN, mid) - triOff < 0.0f)
    {
        return; // back side cull
    }
    out->triNormal = triN;

    m3Vec3 segPts[2] = {c1, c2};
    m3DistanceInput input;
    memset(&input, 0, sizeof(input));
    input.proxyA.points = tri;
    input.proxyA.count = 3;
    input.proxyA.radius = 0.0f;
    input.proxyB.points = segPts;
    input.proxyB.count = 2;
    input.proxyB.radius = 0.0f;
    input.q = m3MakeIdentityQuat();
    input.p = (m3Vec3){0.0f, 0.0f, 0.0f};
    input.useRadii = false;
    m3SimplexCache cache;
    cache.count = 0;
    cache.metric = 0.0f;
    m3DistanceOutput dOut = m3ShapeDistance(&input, &cache);

    if (dOut.distance > radius + M3_SPECULATIVE_DISTANCE)
    {
        return;
    }

    if (dOut.distance > 100.0f * FLT_EPSILON)
    {
        // Shallow: prefer the two-point face clip when the closest
        // axis is not grazing the face.
        m3Vec3 delta = m3Normalize3(m3Sub3(dOut.pointB, dOut.pointA));
        m3real cosAngle = m3Dot3(triN, delta);
        cosAngle = cosAngle < 0.0f ? -cosAngle : cosAngle;
        if (cosAngle > 0.2f)
        {
            m3Vec3 segment[2] = {c1, c2};
            if (ClipSegmentToTriFace(segment, tri, triN))
            {
                out->normal = triN;
                out->dist2 = dOut.distance * dOut.distance;
                out->feature = 7;
                out->pointCount = 2;
                for (int32_t k = 0; k < 2; ++k)
                {
                    m3real d = m3Dot3(triN, segment[k]) - triOff;
                    out->separation[k] = d - radius;
                    out->point[k] = m3Sub3(segment[k], m3MulSV3(0.5f * (radius + d), triN));
                    out->localId[k] = (uint16_t)k;
                }
                return;
            }
        }
        // Single closest point; the feature comes from the simplex
        // cache exactly like the sphere path (triangle side = A).
        int32_t mask = 0;
        for (int32_t i = 0; i < (int32_t)cache.count && i < 3; ++i)
        {
            mask |= 1 << cache.indexA[i];
        }
        out->normal = delta;
        out->dist2 = dOut.distance * dOut.distance;
        out->feature = mask == 0 ? 7 : mask;
        out->pointCount = 1;
        out->separation[0] = dOut.distance - radius;
        out->point[0] =
            m3MulSV3(0.5f, m3Add3(dOut.pointA, m3Sub3(dOut.pointB, m3MulSV3(radius, delta))));
        out->localId[0] = 0;
        return;
    }

    // Deep: face query (min cap-center distance) versus edge query.
    m3real sep1 = m3Dot3(triN, c1) - triOff;
    m3real sep2 = m3Dot3(triN, c2) - triOff;
    m3real faceQuerySep = m3MinF(sep1, sep2);
    if (faceQuerySep > radius)
    {
        return;
    }
    m3Vec3 capDir = m3Sub3(c2, c1);
    m3Vec3 triCenter = m3MulSV3(1.0f / 3.0f, m3Add3(tri[0], m3Add3(tri[1], tri[2])));
    m3real bestEdgeSep = -3.4e38f;
    int32_t bestEdge = 0;
    for (int32_t k = 0; k < 3; ++k)
    {
        m3Vec3 v1 = tri[k];
        m3Vec3 e = m3Sub3(tri[(k + 1) % 3], v1);
        m3real sep = EdgeEdgeSep(v1, e, triCenter, c1, capDir, mid);
        if (sep > bestEdgeSep)
        {
            bestEdgeSep = sep;
            bestEdge = k;
        }
    }
    if (bestEdgeSep > radius)
    {
        return;
    }

    // Face contact: clip the segment, both ends against the plane.
    m3real faceSeparation = faceQuerySep - radius;
    m3Vec3 segment[2] = {c1, c2};
    if (ClipSegmentToTriFace(segment, tri, triN))
    {
        out->normal = triN;
        out->dist2 = 0.0f;
        out->feature = 7;
        out->pointCount = 2;
        m3real minSep = 3.4e38f;
        for (int32_t k = 0; k < 2; ++k)
        {
            m3real d = m3Dot3(triN, segment[k]) - triOff;
            out->separation[k] = d - radius;
            minSep = m3MinF(minSep, out->separation[k]);
            out->point[k] = m3Sub3(segment[k], m3MulSV3(0.5f * (radius + d), triN));
            out->localId[k] = (uint16_t)k;
        }
        faceSeparation = minSep;
    }

    // Edge contact only when the face clip failed or the edge axis is
    // significantly better (the reference tolerance rule).
    m3real edgeSeparation = bestEdgeSep - radius;
    if (out->pointCount == 0 || edgeSeparation > 0.5f * faceSeparation + 0.005f)
    {
        m3Vec3 v1 = tri[bestEdge];
        m3Vec3 triEdge = m3Sub3(tri[(bestEdge + 1) % 3], v1);
        m3Vec3 normal = m3Normalize3(m3Cross3(capDir, triEdge));
        if (m3Dot3(normal, m3Sub3(v1, triCenter)) < 0.0f)
        {
            normal = m3Neg3(normal);
        }
        m3real f1;
        m3real f2;
        LineClosest(v1, triEdge, c1, capDir, &f1, &f2);
        if (f1 < 0.0f || f1 > 1.0f || f2 < 0.0f || f2 > 1.0f)
        {
            return; // closest point beyond the segment ends
        }
        m3Vec3 onTriEdge = m3Add3(v1, m3MulSV3(f1, triEdge));
        m3Vec3 onCapCore = m3Add3(c1, m3MulSV3(f2, capDir));
        m3real separation = m3Dot3(normal, m3Sub3(onCapCore, onTriEdge));
        out->normal = normal;
        out->dist2 = separation > 0.0f ? separation * separation : 0.0f;
        out->feature = (1 << bestEdge) | (1 << ((bestEdge + 1) % 3));
        out->pointCount = 1;
        out->separation[0] = separation - radius;
        out->point[0] =
            m3MulSV3(0.5f, m3Add3(m3Sub3(onCapCore, m3MulSV3(radius, normal)), onTriEdge));
        out->localId[0] = 0;
    }
}

#define M3_MESH_CLIP_CAP 16

// Clip a polygon against one plane, carrying separations against the
// reference plane (a generic Sutherland-Hodgman step for the hull
// versus triangle kernel).
typedef struct m3MeshClipVertex
{
    m3Vec3 position;
    m3real separation;
} m3MeshClipVertex;

static int32_t MeshClipPolygon(m3MeshClipVertex* out, const m3MeshClipVertex* in, int32_t count,
                               m3Vec3 clipNormal, m3real clipOffset, m3Vec3 refNormal,
                               m3real refOffset)
{
    int32_t outCount = 0;
    m3MeshClipVertex prev = in[count - 1];
    m3real prevDist = m3Dot3(clipNormal, prev.position) - clipOffset;
    for (int32_t i = 0; i < count; ++i)
    {
        m3MeshClipVertex curr = in[i];
        m3real currDist = m3Dot3(clipNormal, curr.position) - clipOffset;
        if (prevDist <= 0.0f)
        {
            out[outCount++] = prev;
        }
        if (prevDist * currDist < 0.0f)
        {
            m3real t = prevDist / (prevDist - currDist);
            m3Vec3 p = m3Add3(prev.position, m3MulSV3(t, m3Sub3(curr.position, prev.position)));
            out[outCount].position = p;
            out[outCount].separation = m3Dot3(refNormal, p) - refOffset;
            outCount += 1;
        }
        prev = curr;
        prevDist = currDist;
    }
    return outCount;
}

// Keep the deepest four clip points (ties to the lower slot) in
// ascending slot order, the canonical reduction the box paths use.
static void KeepDeepestClip(m3TriManifold* out, const m3MeshClipVertex* points, int32_t count)
{
    int32_t kept[4];
    int32_t keptCount = 0;
    uint8_t used[M3_MESH_CLIP_CAP];
    memset(used, 0, sizeof(used));
    int32_t want = count < 4 ? count : 4;
    for (int32_t k = 0; k < want; ++k)
    {
        int32_t best = -1;
        for (int32_t c = 0; c < count; ++c)
        {
            if (used[c])
            {
                continue;
            }
            if (best < 0 || points[c].separation < points[best].separation)
            {
                best = c;
            }
        }
        used[best] = 1;
        kept[keptCount++] = best;
    }
    for (int32_t a = 0; a < keptCount; ++a)
    {
        for (int32_t b = a + 1; b < keptCount; ++b)
        {
            if (kept[b] < kept[a])
            {
                int32_t tmp = kept[a];
                kept[a] = kept[b];
                kept[b] = tmp;
            }
        }
    }
    out->pointCount = 0;
    for (int32_t k = 0; k < keptCount; ++k)
    {
        int32_t c = kept[k];
        if (points[c].separation > M3_SPECULATIVE_DISTANCE)
        {
            continue;
        }
        int32_t slot = out->pointCount;
        out->point[slot] =
            m3Sub3(points[c].position, m3MulSV3(0.5f * points[c].separation, out->normal));
        out->separation[slot] = points[c].separation;
        out->localId[slot] = (uint16_t)slot;
        out->pointCount += 1;
    }
}

// Hull versus one triangle (reference b3CollideHullAndTriangle, run
// cacheless: the SAT cache is a per-triangle performance memo, the
// axis selection below is identical without it and joins the BVH
// slice). Everything in the HULL's local frame; the caller converts
// results back to the mesh frame.
static void CollideHullTriangle(m3TriManifold* out, const m3HullData* hull, const m3Vec3 tri[3])
{
    out->pointCount = 0;
    const m3real linearSlop = 0.005f;

    m3Vec3 triN = m3Normalize3(m3Cross3(m3Sub3(tri[1], tri[0]), m3Sub3(tri[2], tri[0])));
    m3real triOff = m3Dot3(triN, tri[0]);
    out->triNormal = triN;
    if (m3Dot3(triN, hull->center) - triOff < -linearSlop)
    {
        return; // back side cull
    }
    m3Vec3 triCenter = m3MulSV3(1.0f / 3.0f, m3Add3(tri[0], m3Add3(tri[1], tri[2])));
    m3Vec3 triEdge[3] = {m3Sub3(tri[1], tri[0]), m3Sub3(tri[2], tri[1]), m3Sub3(tri[0], tri[2])};

    // Face query A: the triangle face against the hull support.
    m3real sepA = 3.4e38f;
    for (int32_t v = 0; v < hull->vertexCount; ++v)
    {
        m3real d = m3Dot3(triN, hull->vertices[v]) - triOff;
        sepA = m3MinF(sepA, d);
    }
    if (sepA > M3_SPECULATIVE_DISTANCE)
    {
        return;
    }

    // Face query B: every hull face against the triangle support.
    int32_t faceB = 0;
    m3real sepB = -3.4e38f;
    for (int32_t f = 0; f < hull->faceCount; ++f)
    {
        m3Vec3 n = hull->faceNormals[f];
        m3real best = 3.4e38f;
        for (int32_t k = 0; k < 3; ++k)
        {
            best = m3MinF(best, m3Dot3(n, tri[k]) - hull->faceOffsets[f]);
        }
        if (best > sepB)
        {
            sepB = best;
            faceB = f;
        }
    }
    if (sepB > M3_SPECULATIVE_DISTANCE)
    {
        return;
    }

    // Edge query, Minkowski-gated (the duality-transform test).
    m3real edgeSep = -3.4e38f;
    int32_t edgeTri = -1;
    int32_t edgeHull = -1;
    for (int32_t e = 0; e < hull->edgeCount; e += 2)
    {
        m3Vec3 hp = hull->vertices[hull->edges[e].origin];
        m3Vec3 he = m3Sub3(hull->vertices[hull->edges[e + 1].origin], hp);
        m3Vec3 hn1 = hull->faceNormals[hull->edges[e].face];
        m3Vec3 hn2 = hull->faceNormals[hull->edges[e + 1].face];
        for (int32_t j = 0; j < 3; ++j)
        {
            m3real cab = m3Dot3(hn1, triEdge[j]);
            m3real dab = m3Dot3(hn2, triEdge[j]);
            m3real bcd = m3Dot3(triN, he);
            if (cab * dab >= 0.0f || cab * bcd <= 0.0f)
            {
                continue;
            }
            m3real sep = EdgeEdgeSep(tri[j], triEdge[j], triCenter, hp, he, hull->center);
            if (sep > edgeSep)
            {
                edgeSep = sep;
                edgeTri = j;
                edgeHull = e;
            }
        }
    }
    if (edgeSep > M3_SPECULATIVE_DISTANCE)
    {
        return;
    }

    // Face contact: reference rule, the hull face wins only when it
    // is meaningfully better AND pushes against the triangle normal.
    m3real clippedSep = 3.4e38f;
    m3Vec3 hullFaceN = hull->faceNormals[faceB];
    int pushingUp = m3Dot3(hullFaceN, triN) < 0.0f;
    if (sepB > sepA + linearSlop && pushingUp)
    {
        // Reference face = the hull face; clip the TRIANGLE against
        // its side planes.
        m3MeshClipVertex buf1[M3_MESH_CLIP_CAP];
        m3MeshClipVertex buf2[M3_MESH_CLIP_CAP];
        m3real refOff = hull->faceOffsets[faceB];
        for (int32_t k = 0; k < 3; ++k)
        {
            buf1[k].position = tri[k];
            buf1[k].separation = m3Dot3(hullFaceN, tri[k]) - refOff;
        }
        int32_t count = 3;
        m3MeshClipVertex* input = buf1;
        m3MeshClipVertex* output = buf2;
        int32_t startEdge = -1;
        for (int32_t e = 0; e < hull->edgeCount && startEdge < 0; ++e)
        {
            if (hull->edges[e].face == (uint8_t)faceB)
            {
                startEdge = e;
            }
        }
        int32_t edgeIndex = startEdge;
        do
        {
            int32_t nextIndex = hull->edges[edgeIndex].next;
            m3Vec3 vertex1 = hull->vertices[hull->edges[edgeIndex].origin];
            m3Vec3 vertex2 = hull->vertices[hull->edges[nextIndex].origin];
            m3Vec3 tangent = m3Normalize3(m3Sub3(vertex2, vertex1));
            m3Vec3 binormal = m3Cross3(tangent, hullFaceN);
            count = MeshClipPolygon(output, input, count, binormal, m3Dot3(binormal, vertex1),
                                    hullFaceN, refOff);
            if (count < 3 || count > M3_MESH_CLIP_CAP - 2)
            {
                count = 0;
                break;
            }
            m3MeshClipVertex* tmp = input;
            input = output;
            output = tmp;
            edgeIndex = nextIndex;
        } while (edgeIndex != startEdge);

        if (count > 0)
        {
            out->normal = m3Neg3(hullFaceN); // triangle toward hull
            out->feature = M3_TRI_FEATURE_HULL_FACE;
            KeepDeepestClip(out, input, count);
            clippedSep = 3.4e38f;
            for (int32_t k = 0; k < out->pointCount; ++k)
            {
                clippedSep = m3MinF(clippedSep, out->separation[k]);
            }
        }
    }
    else
    {
        // Reference face = the triangle; clip the hull's most
        // anti-parallel (incident) face against the triangle sides.
        int32_t incFace = 0;
        m3real minDot = 3.4e38f;
        for (int32_t f = 0; f < hull->faceCount; ++f)
        {
            m3real d = m3Dot3(triN, hull->faceNormals[f]);
            if (d < minDot)
            {
                minDot = d;
                incFace = f;
            }
        }
        m3MeshClipVertex buf1[M3_MESH_CLIP_CAP];
        m3MeshClipVertex buf2[M3_MESH_CLIP_CAP];
        int32_t count = 0;
        int32_t startEdge = -1;
        for (int32_t e = 0; e < hull->edgeCount && startEdge < 0; ++e)
        {
            if (hull->edges[e].face == (uint8_t)incFace)
            {
                startEdge = e;
            }
        }
        int32_t edgeIndex = startEdge;
        do
        {
            m3Vec3 p = hull->vertices[hull->edges[edgeIndex].origin];
            buf1[count].position = p;
            buf1[count].separation = m3Dot3(triN, p) - triOff;
            count += 1;
            edgeIndex = hull->edges[edgeIndex].next;
        } while (edgeIndex != startEdge && count < M3_MESH_CLIP_CAP - 2);

        m3MeshClipVertex* input = buf1;
        m3MeshClipVertex* output = buf2;
        for (int32_t j = 0; j < 3 && count > 0; ++j)
        {
            m3Vec3 sideN = m3Normalize3(m3Cross3(triEdge[j], triN));
            count =
                MeshClipPolygon(output, input, count, sideN, m3Dot3(sideN, tri[j]), triN, triOff);
            if (count > M3_MESH_CLIP_CAP - 2)
            {
                count = 0;
                break;
            }
            m3MeshClipVertex* tmp = input;
            input = output;
            output = tmp;
        }
        if (count > 0)
        {
            out->normal = triN;
            out->feature = 7;
            KeepDeepestClip(out, input, count);
            clippedSep = 3.4e38f;
            for (int32_t k = 0; k < out->pointCount; ++k)
            {
                clippedSep = m3MinF(clippedSep, out->separation[k]);
            }
        }
    }

    // The edge axis overrides when it is genuinely better (the two
    // reference conditions).
    if (edgeTri >= 0)
    {
        m3real maxFaceSep = m3MaxF(sepA, sepB);
        if ((out->pointCount == 0 && edgeSep > maxFaceSep) ||
            (out->pointCount == 1 && edgeSep > clippedSep + linearSlop))
        {
            m3Vec3 hp = hull->vertices[hull->edges[edgeHull].origin];
            m3Vec3 he = m3Sub3(hull->vertices[hull->edges[edgeHull + 1].origin], hp);
            m3Vec3 normal = m3Normalize3(m3Cross3(triEdge[edgeTri], he));
            m3real outwardA = m3Dot3(normal, m3Sub3(tri[edgeTri], triCenter));
            m3real outwardB = m3Dot3(normal, m3Sub3(hull->center, hp));
            m3real aA = outwardA < 0.0f ? -outwardA : outwardA;
            m3real aB = outwardB < 0.0f ? -outwardB : outwardB;
            if (aA > aB ? outwardA < 0.0f : outwardB < 0.0f)
            {
                normal = m3Neg3(normal);
            }
            m3real f1;
            m3real f2;
            LineClosest(tri[edgeTri], triEdge[edgeTri], hp, he, &f1, &f2);
            if (f1 >= 0.0f && f1 <= 1.0f && f2 >= 0.0f && f2 <= 1.0f)
            {
                m3Vec3 onTri = m3Add3(tri[edgeTri], m3MulSV3(f1, triEdge[edgeTri]));
                m3Vec3 onHull = m3Add3(hp, m3MulSV3(f2, he));
                m3real separation = m3Dot3(normal, m3Sub3(onHull, onTri));
                out->pointCount = 1;
                out->normal = normal;
                out->feature = (1 << edgeTri) | (1 << ((edgeTri + 1) % 3));
                out->dist2 = separation > 0.0f ? separation * separation : 0.0f;
                out->point[0] = m3MulSV3(0.5f, m3Add3(onTri, onHull));
                out->separation[0] = separation;
                out->localId[0] = 0;
            }
        }
    }

    // GJK fallback: speculative SAT can strand a nearby pair with no
    // points; a single witness prevents rare tunneling (reference).
    if (out->pointCount == 0)
    {
        m3DistanceInput input;
        memset(&input, 0, sizeof(input));
        input.proxyA.points = tri;
        input.proxyA.count = 3;
        input.proxyA.radius = 0.0f;
        input.proxyB.points = hull->vertices;
        input.proxyB.count = hull->vertexCount;
        input.proxyB.radius = 0.0f;
        input.q = m3MakeIdentityQuat();
        input.p = (m3Vec3){0.0f, 0.0f, 0.0f};
        input.useRadii = false;
        m3SimplexCache cache;
        cache.count = 0;
        cache.metric = 0.0f;
        m3DistanceOutput dOut = m3ShapeDistance(&input, &cache);
        if (dOut.distance > 0.0f && dOut.distance <= M3_SPECULATIVE_DISTANCE)
        {
            int32_t mask = 0;
            for (int32_t i = 0; i < (int32_t)cache.count && i < 3; ++i)
            {
                mask |= 1 << cache.indexA[i];
            }
            out->pointCount = 1;
            out->normal = dOut.normal;
            out->feature = mask == 0 ? 7 : mask;
            out->dist2 = dOut.distance * dOut.distance;
            out->point[0] = m3MulSV3(0.5f, m3Add3(dOut.pointA, dOut.pointB));
            out->separation[0] = dOut.distance;
            out->localId[0] = 0;
        }
        return;
    }
    out->dist2 = clippedSep > 0.0f ? clippedSep * clippedSep : 0.0f;
}

#define M3_MESH_CANDIDATE_CAP 64

typedef struct m3FeatureSet
{
    int32_t edges[3 * M3_MESH_CANDIDATE_CAP]; // packed lo*65536+hi
    int32_t edgeCount;
    int32_t verts[3 * M3_MESH_CANDIDATE_CAP];
    int32_t vertCount;
} m3FeatureSet;

// Returns 1 when the edge or vertex was NEW (unclaimed until now).
static int ClaimEdge(m3FeatureSet* set, int32_t v1, int32_t v2)
{
    int32_t lo = v1 < v2 ? v1 : v2;
    int32_t hi = v1 < v2 ? v2 : v1;
    int32_t key = lo * 65536 + hi;
    for (int32_t i = 0; i < set->edgeCount; ++i)
    {
        if (set->edges[i] == key)
        {
            return 0;
        }
    }
    if (set->edgeCount < 3 * M3_MESH_CANDIDATE_CAP)
    {
        set->edges[set->edgeCount++] = key;
    }
    return 1;
}

static int ClaimVertex(m3FeatureSet* set, int32_t v)
{
    for (int32_t i = 0; i < set->vertCount; ++i)
    {
        if (set->verts[i] == v)
        {
            return 0;
        }
    }
    if (set->vertCount < 3 * M3_MESH_CANDIDATE_CAP)
    {
        set->verts[set->vertCount++] = v;
    }
    return 1;
}

typedef struct m3MeshCandidate
{
    m3TriManifold local;
    int32_t triIndex;
} m3MeshCandidate;

static void CollideMeshConvex(m3World* world, m3Manifold* fresh, int32_t meshShape,
                              int32_t otherShape, int meshIsA)
{
    const m3MeshData* mesh = &world->meshData[world->shapeMeshIndex[meshShape]];
    int32_t meshBody = world->shapeBody[meshShape];
    int32_t otherBody = world->shapeBody[otherShape];
    const m3Transform* xfM = &world->transforms[meshBody];
    const m3Transform* xfO = &world->transforms[otherBody];
    uint8_t otherType = world->shapeType[otherShape];

    // Localize the convex shape into the mesh frame (doubles here).
    m3Quat conjM = {-xfM->q.x, -xfM->q.y, -xfM->q.z, xfM->q.w};
    m3Quat qRel = m3MulQuat(conjM, xfO->q);
    m3Vec3 dp = {(m3real)(xfO->p.x - xfM->p.x), (m3real)(xfO->p.y - xfM->p.y),
                 (m3real)(xfO->p.z - xfM->p.z)};
    m3Vec3 pRel = m3InvRotateVec3(xfM->q, dp);

    m3real radius = world->shapeGeom[otherShape].s;
    m3Vec3 s1 = {0.0f, 0.0f, 0.0f};
    m3Vec3 s2 = {0.0f, 0.0f, 0.0f};
    m3Vec3 boundLo;
    m3Vec3 boundHi;
    const m3HullData* hull = NULL;
    m3Quat qHull = m3MakeIdentityQuat(); // mesh frame -> hull frame
    m3Vec3 pHull = {0.0f, 0.0f, 0.0f};
    if (otherType == (uint8_t)m3_hullShape)
    {
        // The hull kernel runs in the HULL frame: triangles transform
        // in, results transform back out with qRel/pRel.
        hull = &world->hullData[world->shapeHullIndex[otherShape]];
        qHull = (m3Quat){-qRel.x, -qRel.y, -qRel.z, qRel.w};
        pHull = m3Neg3(m3InvRotateVec3(qRel, pRel));
        boundLo = (m3Vec3){3.4e38f, 3.4e38f, 3.4e38f};
        boundHi = (m3Vec3){-3.4e38f, -3.4e38f, -3.4e38f};
        for (int32_t v = 0; v < hull->vertexCount; ++v)
        {
            m3Vec3 p = m3Add3(m3RotateVec3(qRel, hull->vertices[v]), pRel);
            boundLo.x = m3MinF(boundLo.x, p.x);
            boundLo.y = m3MinF(boundLo.y, p.y);
            boundLo.z = m3MinF(boundLo.z, p.z);
            boundHi.x = m3MaxF(boundHi.x, p.x);
            boundHi.y = m3MaxF(boundHi.y, p.y);
            boundHi.z = m3MaxF(boundHi.z, p.z);
        }
        radius = 0.0f;
    }
    else if (otherType == (uint8_t)m3_sphereShape)
    {
        s1 = m3Add3(m3RotateVec3(qRel, world->shapeGeom[otherShape].v), pRel);
        boundLo = s1;
        boundHi = s1;
    }
    else
    {
        s1 = m3Add3(m3RotateVec3(qRel, world->shapeGeom[otherShape].v), pRel);
        s2 = m3Add3(m3RotateVec3(qRel, world->shapeGeom[otherShape].v2), pRel);
        boundLo.x = m3MinF(s1.x, s2.x);
        boundLo.y = m3MinF(s1.y, s2.y);
        boundLo.z = m3MinF(s1.z, s2.z);
        boundHi.x = m3MaxF(s1.x, s2.x);
        boundHi.y = m3MaxF(s1.y, s2.y);
        boundHi.z = m3MaxF(s1.z, s2.z);
    }
    m3real reach = radius + M3_SPECULATIVE_DISTANCE;

    // Midphase (2c-10): the static BVH prunes, then the exact
    // per-triangle reject below runs unchanged, so the accepted
    // sequence is bit-identical to the full scan this replaced
    // (gather returns ascending order; the cap break fires at the
    // same processing point).
    uint16_t gather[M3_MESH_MAX_TRIS];
    int32_t gatherCount =
        m3MeshBvhGather(&world->meshBvh[world->shapeMeshIndex[meshShape]],
                        (m3Vec3){boundLo.x - reach, boundLo.y - reach, boundLo.z - reach},
                        (m3Vec3){boundHi.x + reach, boundHi.y + reach, boundHi.z + reach}, gather);

    m3MeshCandidate faceAccepted[M3_MESH_CANDIDATE_CAP];
    int32_t faceCount = 0;
    m3MeshCandidate tentative[M3_MESH_CANDIDATE_CAP];
    int32_t tentativeCount = 0;

    for (int32_t g = 0; g < gatherCount; ++g)
    {
        int32_t t = gather[g];
        if (faceCount >= M3_MESH_CANDIDATE_CAP || tentativeCount >= M3_MESH_CANDIDATE_CAP)
        {
            break;
        }
        m3Vec3 tri[3] = {mesh->vertices[mesh->indices[3 * t + 0]],
                         mesh->vertices[mesh->indices[3 * t + 1]],
                         mesh->vertices[mesh->indices[3 * t + 2]]};
        m3real lox = m3MinF(tri[0].x, m3MinF(tri[1].x, tri[2].x)) - reach;
        m3real hix = m3MaxF(tri[0].x, m3MaxF(tri[1].x, tri[2].x)) + reach;
        m3real loy = m3MinF(tri[0].y, m3MinF(tri[1].y, tri[2].y)) - reach;
        m3real hiy = m3MaxF(tri[0].y, m3MaxF(tri[1].y, tri[2].y)) + reach;
        m3real loz = m3MinF(tri[0].z, m3MinF(tri[1].z, tri[2].z)) - reach;
        m3real hiz = m3MaxF(tri[0].z, m3MaxF(tri[1].z, tri[2].z)) + reach;
        if (boundHi.x < lox || boundLo.x > hix || boundHi.y < loy || boundLo.y > hiy ||
            boundHi.z < loz || boundLo.z > hiz)
        {
            continue;
        }

        m3TriManifold local;
        memset(&local, 0, sizeof(local)); // the whole struct is copied
                                          // below; keep O3 flow checks
                                          // and MSVC C4701 quiet
        if (otherType == (uint8_t)m3_sphereShape)
        {
            CollideSphereTriangle(&local, s1, radius, tri);
        }
        else if (otherType == (uint8_t)m3_capsuleShape)
        {
            CollideCapsuleTriangle(&local, s1, s2, radius, tri);
        }
        else
        {
            // The hull kernel wants the triangle in the hull frame;
            // its results come back into the mesh frame here.
            m3Vec3 triH[3];
            for (int32_t k = 0; k < 3; ++k)
            {
                triH[k] = m3Add3(m3RotateVec3(qHull, tri[k]), pHull);
            }
            CollideHullTriangle(&local, hull, triH);
            if (local.pointCount > 0)
            {
                local.normal = m3RotateVec3(qRel, local.normal);
                local.triNormal = m3RotateVec3(qRel, local.triNormal);
                for (int32_t k = 0; k < local.pointCount; ++k)
                {
                    local.point[k] = m3Add3(m3RotateVec3(qRel, local.point[k]), pRel);
                }
            }
        }
        if (local.pointCount == 0)
        {
            continue;
        }
        m3MeshCandidate cand;
        cand.local = local;
        cand.triIndex = t;
        if (local.feature == 7)
        {
            faceAccepted[faceCount++] = cand;
        }
        else if (local.feature == M3_TRI_FEATURE_HULL_FACE)
        {
            // The reference acceptance for hull-face contacts: accept
            // when the contact normal agrees with the triangle normal
            // or the overlap is deep; otherwise tentative.
            m3real cosAngle = m3Dot3(local.triNormal, local.normal);
            m3real minSep = 3.4e38f;
            for (int32_t k = 0; k < local.pointCount; ++k)
            {
                minSep = m3MinF(minSep, local.separation[k]);
            }
            if (cosAngle > 0.5f || minSep < -2.0f * 0.005f)
            {
                faceAccepted[faceCount++] = cand;
            }
            else
            {
                tentative[tentativeCount++] = cand;
            }
        }
        else
        {
            tentative[tentativeCount++] = cand;
        }
    }

    // Face contacts claim their features first (triangle order).
    m3FeatureSet set;
    set.edgeCount = 0;
    set.vertCount = 0;
    for (int32_t k = 0; k < faceCount; ++k)
    {
        int32_t t = faceAccepted[k].triIndex;
        int32_t i1 = mesh->indices[3 * t + 0];
        int32_t i2 = mesh->indices[3 * t + 1];
        int32_t i3 = mesh->indices[3 * t + 2];
        (void)ClaimEdge(&set, i1, i2);
        (void)ClaimEdge(&set, i2, i3);
        (void)ClaimEdge(&set, i3, i1);
        (void)ClaimVertex(&set, i1);
        (void)ClaimVertex(&set, i2);
        (void)ClaimVertex(&set, i3);
    }

    // Tentatives in ascending distance (ties to the lower triangle).
    for (int32_t a = 0; a < tentativeCount; ++a)
    {
        int32_t best = a;
        for (int32_t b = a + 1; b < tentativeCount; ++b)
        {
            if (tentative[b].local.dist2 < tentative[best].local.dist2 ||
                (tentative[b].local.dist2 == tentative[best].local.dist2 &&
                 tentative[b].triIndex < tentative[best].triIndex))
            {
                best = b;
            }
        }
        m3MeshCandidate tmp = tentative[a];
        tentative[a] = tentative[best];
        tentative[best] = tmp;
    }

    // Accept the surviving tentatives into the face list.
    for (int32_t k = 0; k < tentativeCount && faceCount < M3_MESH_CANDIDATE_CAP; ++k)
    {
        int32_t t = tentative[k].triIndex;
        int32_t i1 = mesh->indices[3 * t + 0];
        int32_t i2 = mesh->indices[3 * t + 1];
        int32_t i3 = mesh->indices[3 * t + 2];
        int newEdge1 = ClaimEdge(&set, i1, i2);
        int newEdge2 = ClaimEdge(&set, i2, i3);
        int newEdge3 = ClaimEdge(&set, i3, i1);
        int newVert1 = ClaimVertex(&set, i1);
        int newVert2 = ClaimVertex(&set, i2);
        int newVert3 = ClaimVertex(&set, i3);
        // Baked convexity (2b-9d): a genuinely convex ridge or a
        // boundary is a REAL feature and overrides the claim filter;
        // only flat and concave edges can be ghosts.
        uint8_t convex = mesh->edgeFlags[t];
        int shouldCollide = 0;
        switch (tentative[k].local.feature)
        {
        case 1 | 2:
            shouldCollide = (convex & 1) != 0 || newEdge1;
            break;
        case 2 | 4:
            shouldCollide = (convex & 2) != 0 || newEdge2;
            break;
        case 1 | 4:
            shouldCollide = (convex & 4) != 0 || newEdge3;
            break;
        case 1:
            shouldCollide = newVert1;
            break;
        case 2:
            shouldCollide = newVert2;
            break;
        case 4:
            shouldCollide = newVert3;
            break;
        case M3_TRI_FEATURE_HULL_FACE:
            // A tilted hull-face contact on a triangle with a real
            // convex edge is legitimate (a box teetering on a roof
            // ridge); on all-flat triangles it survives only when
            // the whole triangle is unclaimed (the reference's
            // only-ignore-flat-edges rule).
            shouldCollide = convex != 0 ||
                            (newEdge1 && newEdge2 && newEdge3 && newVert1 && newVert2 && newVert3);
            break;
        default:
            break;
        }
        if (shouldCollide)
        {
            faceAccepted[faceCount++] = tentative[k];
        }
    }

    if (faceCount == 0)
    {
        return;
    }

    // Cluster around the deepest accepted contact: manifolds whose
    // normals agree merge their points; the rest wait for 2b-9c.
    int32_t repIndex = 0;
    m3real repSep = 3.4e38f;
    for (int32_t k = 0; k < faceCount; ++k)
    {
        for (int32_t p = 0; p < faceAccepted[k].local.pointCount; ++p)
        {
            m3real sep = faceAccepted[k].local.separation[p];
            if (sep < repSep ||
                (sep == repSep && faceAccepted[k].triIndex < faceAccepted[repIndex].triIndex))
            {
                repSep = sep;
                repIndex = k;
            }
        }
    }
    m3Vec3 repNormal = faceAccepted[repIndex].local.normal;

    // Gather cluster points, keep the deepest four (ties by triangle
    // then local id: the canonical rule).
    enum
    {
        GATHER_CAP = 2 * M3_MESH_CANDIDATE_CAP
    };
    m3Vec3 gPoint[GATHER_CAP];
    m3real gSep[GATHER_CAP];
    uint16_t gId[GATHER_CAP];
    int32_t gCount = 0;
    for (int32_t k = 0; k < faceCount && gCount < GATHER_CAP; ++k)
    {
        if (m3Dot3(faceAccepted[k].local.normal, repNormal) < 0.99f)
        {
            continue;
        }
        for (int32_t p = 0; p < faceAccepted[k].local.pointCount && gCount < GATHER_CAP; ++p)
        {
            gPoint[gCount] = faceAccepted[k].local.point[p];
            gSep[gCount] = faceAccepted[k].local.separation[p];
            gId[gCount] =
                (uint16_t)((faceAccepted[k].triIndex << 2) | faceAccepted[k].local.localId[p]);
            gCount += 1;
        }
    }
    int32_t kept[M3_MANIFOLD_MAX_POINTS];
    int32_t keptCount = 0;
    uint8_t used[GATHER_CAP];
    memset(used, 0, sizeof(used));
    int32_t want = gCount < M3_MANIFOLD_MAX_POINTS ? gCount : M3_MANIFOLD_MAX_POINTS;
    for (int32_t k = 0; k < want; ++k)
    {
        int32_t best = -1;
        for (int32_t c = 0; c < gCount; ++c)
        {
            if (used[c])
            {
                continue;
            }
            if (best < 0 || gSep[c] < gSep[best] || (gSep[c] == gSep[best] && gId[c] < gId[best]))
            {
                best = c;
            }
        }
        used[best] = 1;
        kept[keptCount++] = best;
    }
    // Ascending id among the kept (canonical point order).
    for (int32_t a = 0; a < keptCount; ++a)
    {
        for (int32_t b = a + 1; b < keptCount; ++b)
        {
            if (gId[kept[b]] < gId[kept[a]])
            {
                int32_t tmp = kept[a];
                kept[a] = kept[b];
                kept[b] = tmp;
            }
        }
    }

    // Emit: midway points split into both anchors along the normal.
    m3Vec3 nWorld = m3RotateVec3(xfM->q, repNormal); // mesh toward shape
    fresh->normal = meshIsA ? nWorld : m3Neg3(nWorld);
    fresh->pointCount = keptCount;
    for (int32_t k = 0; k < keptCount; ++k)
    {
        int32_t c = kept[k];
        m3Vec3 pw = m3RotateVec3(xfM->q, gPoint[c]);
        double px = xfM->p.x + (double)pw.x;
        double py = xfM->p.y + (double)pw.y;
        double pz = xfM->p.z + (double)pw.z;
        m3real half = 0.5f * gSep[c];
        m3Vec3 anchorMesh = FromCom(world, meshBody, px - (double)(nWorld.x * half),
                                    py - (double)(nWorld.y * half), pz - (double)(nWorld.z * half));
        m3Vec3 anchorOther =
            FromCom(world, otherBody, px + (double)(nWorld.x * half),
                    py + (double)(nWorld.y * half), pz + (double)(nWorld.z * half));
        fresh->points[k].anchorA = meshIsA ? anchorMesh : anchorOther;
        fresh->points[k].anchorB = meshIsA ? anchorOther : anchorMesh;
        fresh->points[k].separation = gSep[c];
        fresh->points[k].id = gId[c];
    }
}

void m3UpdateContactsRange(m3World* world, int32_t start, int32_t end, const uint64_t* oldKeys,
                           const m3Manifold* oldManifolds, int32_t oldCount)
{
    // Rebuild manifolds in pair order, carrying impulses forward by
    // feature id from the caller's stash. The old keys are sorted
    // (canonical order), so the lookup is a binary search. Each pair
    // writes ONLY manifolds[i]: the range is safe under any split.
    for (int32_t i = start; i < end; ++i)
    {
        uint64_t key = world->pairKeys[i];
        int32_t shapeA = (int32_t)(key >> 32);
        int32_t shapeB = (int32_t)(key & 0xFFFFFFFFu);
        uint8_t typeA = world->shapeType[shapeA];
        uint8_t typeB = world->shapeType[shapeB];

        m3Manifold fresh;
        int meshPair = typeA == (uint8_t)m3_meshShape || typeB == (uint8_t)m3_meshShape;
        int planePair = typeA == (uint8_t)m3_planeShape || typeB == (uint8_t)m3_planeShape;
        int hullPair = typeA == (uint8_t)m3_hullShape || typeB == (uint8_t)m3_hullShape;
        int capsulePair = typeA == (uint8_t)m3_capsuleShape || typeB == (uint8_t)m3_capsuleShape;
        if (meshPair)
        {
            memset(&fresh, 0, sizeof(fresh));
            int32_t meshShape = typeA == (uint8_t)m3_meshShape ? shapeA : shapeB;
            int32_t otherShape = meshShape == shapeA ? shapeB : shapeA;
            if (world->shapeType[otherShape] != (uint8_t)m3_planeShape)
            {
                // Sphere, capsule, and hull all ride the welded
                // per-triangle pipeline (2b-9a through 2b-9c).
                CollideMeshConvex(world, &fresh, meshShape, otherShape, meshShape == shapeA);
            }
        }
        else if (planePair && hullPair)
        {
            // Plane versus hull (2b-5a): every hull vertex below the
            // margin becomes a candidate; the four deepest survive
            // (ties break on the lower vertex index) and emit in
            // ascending vertex order, the canonical point order.
            // Feature id = vertex index, so the carry follows each
            // corner across rebuilds.
            memset(&fresh, 0, sizeof(fresh));
            int32_t planeShape = typeA == (uint8_t)m3_planeShape ? shapeA : shapeB;
            int32_t hullShape = planeShape == shapeA ? shapeB : shapeA;
            const m3HullData* hull = &world->hullData[world->shapeHullIndex[hullShape]];
            int32_t planeBody = world->shapeBody[planeShape];
            int32_t hullBody = world->shapeBody[hullShape];
            const m3Transform* xf = &world->transforms[hullBody];
            m3Vec3 n = world->shapeGeom[planeShape].v;
            m3real offset = world->shapeGeom[planeShape].s;

            int32_t candIndex[M3_HULL_MAX_VERTS];
            m3real candSep[M3_HULL_MAX_VERTS];
            double candW[M3_HULL_MAX_VERTS][3];
            int32_t candCount = 0;
            for (int32_t v = 0; v < hull->vertexCount; ++v)
            {
                m3Vec3 r = m3RotateVec3(xf->q, hull->vertices[v]);
                double wx = xf->p.x + (double)r.x;
                double wy = xf->p.y + (double)r.y;
                double wz = xf->p.z + (double)r.z;
                double sep =
                    (double)n.x * wx + (double)n.y * wy + (double)n.z * wz - (double)offset;
                if ((m3real)sep < M3_SPECULATIVE_DISTANCE)
                {
                    candIndex[candCount] = v;
                    candSep[candCount] = (m3real)sep;
                    candW[candCount][0] = wx;
                    candW[candCount][1] = wy;
                    candW[candCount][2] = wz;
                    candCount += 1;
                }
            }
            // Keep the four deepest (selection by min separation, ties
            // to the lower vertex index), then emit in ascending
            // vertex-index order.
            int32_t kept[M3_MANIFOLD_MAX_POINTS];
            int32_t keptCount = 0;
            uint8_t used[M3_HULL_MAX_VERTS];
            memset(used, 0, sizeof(used));
            int32_t want = candCount < M3_MANIFOLD_MAX_POINTS ? candCount : M3_MANIFOLD_MAX_POINTS;
            for (int32_t k = 0; k < want; ++k)
            {
                int32_t best = -1;
                for (int32_t c = 0; c < candCount; ++c)
                {
                    if (used[c])
                    {
                        continue;
                    }
                    if (best < 0 || candSep[c] < candSep[best] ||
                        (candSep[c] == candSep[best] && candIndex[c] < candIndex[best]))
                    {
                        best = c;
                    }
                }
                used[best] = 1;
                kept[keptCount++] = best;
            }
            // Ascending vertex index among the kept (canonical).
            for (int32_t a = 0; a < keptCount; ++a)
            {
                for (int32_t b = a + 1; b < keptCount; ++b)
                {
                    if (candIndex[kept[b]] < candIndex[kept[a]])
                    {
                        int32_t tmp = kept[a];
                        kept[a] = kept[b];
                        kept[b] = tmp;
                    }
                }
            }
            fresh.normal = n; // plane (A) toward hull (B)
            fresh.pointCount = keptCount;
            for (int32_t k = 0; k < keptCount; ++k)
            {
                int32_t c = kept[k];
                double sepD = (double)candSep[c];
                double px = candW[c][0] - (double)n.x * sepD;
                double py = candW[c][1] - (double)n.y * sepD;
                double pz = candW[c][2] - (double)n.z * sepD;
                fresh.points[k].anchorA = FromCom(world, planeBody, px, py, pz);
                fresh.points[k].anchorB =
                    FromCom(world, hullBody, candW[c][0], candW[c][1], candW[c][2]);
                fresh.points[k].separation = candSep[c];
                fresh.points[k].id = (uint16_t)candIndex[c];
            }
            if (planeShape != shapeA)
            {
                fresh.normal = m3Neg3(fresh.normal);
                for (int32_t k = 0; k < keptCount; ++k)
                {
                    m3Vec3 tmp = fresh.points[k].anchorA;
                    fresh.points[k].anchorA = fresh.points[k].anchorB;
                    fresh.points[k].anchorB = tmp;
                }
            }
        }
        else if (typeA == (uint8_t)m3_hullShape && typeB == (uint8_t)m3_hullShape)
        {
            // Hull versus hull (2b-5b): the SAT runs in A's frame on a
            // float relative pose (doubles localized here), then the
            // manifold rotates out to world with COM-relative anchors.
            int32_t bodyA = world->shapeBody[shapeA];
            int32_t bodyB = world->shapeBody[shapeB];
            const m3Transform* xfA = &world->transforms[bodyA];
            const m3Transform* xfB = &world->transforms[bodyB];
            m3Quat conjA = {-xfA->q.x, -xfA->q.y, -xfA->q.z, xfA->q.w};
            m3Quat qRel = m3MulQuat(conjA, xfB->q);
            m3Vec3 dp = {(m3real)(xfB->p.x - xfA->p.x), (m3real)(xfB->p.y - xfA->p.y),
                         (m3real)(xfB->p.z - xfA->p.z)};
            m3Vec3 pRel = m3InvRotateVec3(xfA->q, dp);
            fresh = m3CollideHulls(&world->hullData[world->shapeHullIndex[shapeA]],
                                   &world->hullData[world->shapeHullIndex[shapeB]], qRel, pRel);
            if (fresh.pointCount > 0)
            {
                fresh.normal = m3RotateVec3(xfA->q, fresh.normal);
                for (int32_t k = 0; k < fresh.pointCount; ++k)
                {
                    // Anchors arrive as positions in A's frame; lift to
                    // world, then re-base to each COM.
                    m3Vec3 lA = fresh.points[k].anchorA;
                    m3Vec3 lB = fresh.points[k].anchorB;
                    m3Vec3 rA = m3RotateVec3(xfA->q, lA);
                    m3Vec3 rB = m3RotateVec3(xfA->q, lB);
                    fresh.points[k].anchorA =
                        FromCom(world, bodyA, xfA->p.x + (double)rA.x, xfA->p.y + (double)rA.y,
                                xfA->p.z + (double)rA.z);
                    fresh.points[k].anchorB =
                        FromCom(world, bodyB, xfA->p.x + (double)rB.x, xfA->p.y + (double)rB.y,
                                xfA->p.z + (double)rB.z);
                }
            }
        }
        else if (planePair && capsulePair)
        {
            // Plane versus capsule: the two cap centers are the only
            // candidates. Both inside the margin means the capsule
            // lies flat and gets the two-point manifold that keeps it
            // from rocking. Feature id = cap index (0 or 1), emitted
            // in cap order (canonical).
            memset(&fresh, 0, sizeof(fresh));
            int32_t planeShape = typeA == (uint8_t)m3_planeShape ? shapeA : shapeB;
            int32_t capShape = planeShape == shapeA ? shapeB : shapeA;
            int32_t planeBody = world->shapeBody[planeShape];
            int32_t capBody = world->shapeBody[capShape];
            const m3Transform* xf = &world->transforms[capBody];
            m3Vec3 n = world->shapeGeom[planeShape].v;
            m3real offset = world->shapeGeom[planeShape].s;
            m3real radius = world->shapeGeom[capShape].s;
            m3Vec3 caps[2] = {world->shapeGeom[capShape].v, world->shapeGeom[capShape].v2};
            int32_t count = 0;
            for (int32_t k = 0; k < 2; ++k)
            {
                m3Vec3 r = m3RotateVec3(xf->q, caps[k]);
                double wx = xf->p.x + (double)r.x;
                double wy = xf->p.y + (double)r.y;
                double wz = xf->p.z + (double)r.z;
                double centerDist =
                    (double)n.x * wx + (double)n.y * wy + (double)n.z * wz - (double)offset;
                m3real sep = (m3real)centerDist - radius;
                if (sep > M3_SPECULATIVE_DISTANCE)
                {
                    continue;
                }
                // anchorA: the cap center projected onto the plane.
                // anchorB: the deepest point of that cap sphere.
                double px = wx - (double)n.x * centerDist;
                double py = wy - (double)n.y * centerDist;
                double pz = wz - (double)n.z * centerDist;
                fresh.points[count].anchorA = FromCom(world, planeBody, px, py, pz);
                fresh.points[count].anchorB =
                    FromCom(world, capBody, wx - (double)(n.x * radius),
                            wy - (double)(n.y * radius), wz - (double)(n.z * radius));
                fresh.points[count].separation = sep;
                fresh.points[count].id = (uint16_t)k;
                count += 1;
            }
            fresh.normal = n;
            fresh.pointCount = count;
            if (count > 0 && planeShape != shapeA)
            {
                fresh.normal = m3Neg3(fresh.normal);
                for (int32_t k = 0; k < count; ++k)
                {
                    m3Vec3 tmp = fresh.points[k].anchorA;
                    fresh.points[k].anchorA = fresh.points[k].anchorB;
                    fresh.points[k].anchorB = tmp;
                }
            }
        }
        else if (hullPair && capsulePair)
        {
            // Capsule versus hull: the segment SAT at every depth,
            // never GJK. GJK's one witness cannot hold a lying
            // capsule (it wobbles off the single point), and a deep
            // skewer needs the same SAT axes anyway. Doubles localize
            // into the hull frame, the kernel answers there, and the
            // manifold lifts out with COM anchors like every other
            // pair.
            memset(&fresh, 0, sizeof(fresh));
            int32_t hullShape = typeA == (uint8_t)m3_hullShape ? shapeA : shapeB;
            int32_t capShape = hullShape == shapeA ? shapeB : shapeA;
            int32_t hullBody = world->shapeBody[hullShape];
            int32_t capBody = world->shapeBody[capShape];
            const m3Transform* xfH = &world->transforms[hullBody];
            const m3Transform* xfC = &world->transforms[capBody];
            m3Quat conjH = {-xfH->q.x, -xfH->q.y, -xfH->q.z, xfH->q.w};
            m3Quat qRel = m3MulQuat(conjH, xfC->q);
            m3Vec3 dp = {(m3real)(xfC->p.x - xfH->p.x), (m3real)(xfC->p.y - xfH->p.y),
                         (m3real)(xfC->p.z - xfH->p.z)};
            m3Vec3 pRel = m3InvRotateVec3(xfH->q, dp);
            m3Vec3 s1 = m3Add3(m3RotateVec3(qRel, world->shapeGeom[capShape].v), pRel);
            m3Vec3 s2 = m3Add3(m3RotateVec3(qRel, world->shapeGeom[capShape].v2), pRel);
            const m3HullData* hull = &world->hullData[world->shapeHullIndex[hullShape]];
            m3Manifold local = CollideSegmentHull(hull, s1, s2, world->shapeGeom[capShape].s);
            if (local.pointCount > 0)
            {
                m3Vec3 nWorld = m3RotateVec3(xfH->q, local.normal); // hull toward capsule
                fresh.normal = hullShape == shapeA ? nWorld : m3Neg3(nWorld);
                fresh.pointCount = local.pointCount;
                for (int32_t k = 0; k < local.pointCount; ++k)
                {
                    m3Vec3 rH = m3RotateVec3(xfH->q, local.points[k].anchorA);
                    m3Vec3 rC = m3RotateVec3(xfH->q, local.points[k].anchorB);
                    m3Vec3 aHull = FromCom(world, hullBody, xfH->p.x + (double)rH.x,
                                           xfH->p.y + (double)rH.y, xfH->p.z + (double)rH.z);
                    m3Vec3 aCap = FromCom(world, capBody, xfH->p.x + (double)rC.x,
                                          xfH->p.y + (double)rC.y, xfH->p.z + (double)rC.z);
                    fresh.points[k].anchorA = hullShape == shapeA ? aHull : aCap;
                    fresh.points[k].anchorB = hullShape == shapeA ? aCap : aHull;
                    fresh.points[k].separation = local.points[k].separation;
                    fresh.points[k].id = local.points[k].id;
                }
            }
        }
        else if (hullPair || capsulePair)
        {
            // The remaining convex pairs (hull-sphere, capsule-sphere,
            // capsule-capsule): GJK on the cores in A's frame, radii
            // applied analytically, one contact point, feature id 0.
            // Deep core overlap has an exact answer for each family;
            // EPA never became necessary (hull-hull and capsule-hull
            // deep pairs go through their SATs).
            memset(&fresh, 0, sizeof(fresh));
            int32_t bodyA = world->shapeBody[shapeA];
            int32_t bodyB = world->shapeBody[shapeB];
            const m3Transform* xfA = &world->transforms[bodyA];
            const m3Transform* xfB = &world->transforms[bodyB];
            m3Quat conjA = {-xfA->q.x, -xfA->q.y, -xfA->q.z, xfA->q.w};
            m3DistanceInput input;
            memset(&input, 0, sizeof(input));
            input.q = m3MulQuat(conjA, xfB->q);
            m3Vec3 dp = {(m3real)(xfB->p.x - xfA->p.x), (m3real)(xfB->p.y - xfA->p.y),
                         (m3real)(xfB->p.z - xfA->p.z)};
            input.p = m3InvRotateVec3(xfA->q, dp);
            m3Vec3 pointsA[2];
            m3Vec3 pointsB[2];
            input.proxyA = m3MakeShapeProxy(world, shapeA, pointsA);
            input.proxyB = m3MakeShapeProxy(world, shapeB, pointsB);
            input.useRadii = false;
            m3SimplexCache cache;
            cache.count = 0;
            cache.metric = 0.0f;
            m3DistanceOutput out = m3ShapeDistance(&input, &cache);
            m3real rA = input.proxyA.radius;
            m3real rB = input.proxyB.radius;
            m3real sep = out.distance - rA - rB;
            if (sep <= M3_SPECULATIVE_DISTANCE)
            {
                m3Vec3 nLocal;
                m3Vec3 pALocal;
                m3Vec3 pBLocal;
                if (out.distance > 0.0f)
                {
                    nLocal = out.normal;
                    pALocal = m3Add3(out.pointA, m3MulSV3(rA, nLocal));
                    pBLocal = m3Sub3(out.pointB, m3MulSV3(rB, nLocal));
                }
                else if (hullPair)
                {
                    // A sphere center inside a hull: the least-deep
                    // face is the exact minimum translation (2b-7).
                    int hullIsA = typeA == (uint8_t)m3_hullShape;
                    const m3HullData* hull =
                        &world->hullData[world->shapeHullIndex[hullIsA ? shapeA : shapeB]];
                    const m3DistanceProxy* round = hullIsA ? &input.proxyB : &input.proxyA;
                    m3Vec3 core = hullIsA
                                      ? m3Add3(m3RotateVec3(input.q, round->points[0]), input.p)
                                      : m3InvRotateVec3(input.q, m3Sub3(round->points[0], input.p));
                    m3Vec3 nHull; // hull toward sphere, hull frame
                    m3real coreSep;
                    m3Vec3 onHull;
                    DeepPointInHull(hull, core, &nHull, &coreSep, &onHull);
                    m3real rRound = round->radius;
                    sep = coreSep - rRound;
                    if (hullIsA)
                    {
                        nLocal = nHull;
                        pALocal = onHull;
                        pBLocal = m3Sub3(core, m3MulSV3(rRound, nHull));
                    }
                    else
                    {
                        m3Vec3 nA = m3RotateVec3(input.q, nHull);
                        nLocal = m3Neg3(nA);
                        pALocal = m3Sub3(m3Add3(m3RotateVec3(input.q, core), input.p),
                                         m3MulSV3(rRound, nA));
                        pBLocal = m3Add3(m3RotateVec3(input.q, onHull), input.p);
                    }
                }
                else
                {
                    // Sphere and capsule cores meeting exactly (a
                    // center on a segment, two segments crossing):
                    // measure-zero poses. The mutual perpendicular is
                    // the exact axis and the skins overlap by exactly
                    // rA + rB along it.
                    m3Vec3 axis;
                    m3Vec3 cA;
                    m3Vec3 cB;
                    if (input.proxyA.count == 2 && input.proxyB.count == 2)
                    {
                        m3Vec3 dirA = m3Sub3(pointsA[1], pointsA[0]);
                        m3Vec3 dirB = m3RotateVec3(input.q, m3Sub3(pointsB[1], pointsB[0]));
                        axis = m3Cross3(dirA, dirB);
                        cA = m3MulSV3(0.5f, m3Add3(pointsA[0], pointsA[1]));
                        cB = m3Add3(
                            m3RotateVec3(input.q, m3MulSV3(0.5f, m3Add3(pointsB[0], pointsB[1]))),
                            input.p);
                        if (m3Dot3(axis, axis) < 1.0e-10f)
                        {
                            axis = m3Sub3(cB, cA); // parallel: center delta
                        }
                    }
                    else
                    {
                        // Capsule versus sphere: any segment
                        // perpendicular works; the tangent basis rule
                        // makes the pick bit-stable.
                        m3Vec3 dir = input.proxyA.count == 2
                                         ? m3Sub3(pointsA[1], pointsA[0])
                                         : m3RotateVec3(input.q, m3Sub3(pointsB[1], pointsB[0]));
                        m3Vec3 t1;
                        m3Vec3 t2;
                        m3MakeTangentBasis(m3Normalize3(dir), &t1, &t2);
                        axis = t1;
                        cA = input.proxyA.count == 1
                                 ? pointsA[0]
                                 : m3MulSV3(0.5f, m3Add3(pointsA[0], pointsA[1]));
                        cB = input.proxyB.count == 1
                                 ? m3Add3(m3RotateVec3(input.q, pointsB[0]), input.p)
                                 : m3Add3(m3RotateVec3(input.q, m3MulSV3(0.5f, m3Add3(pointsB[0],
                                                                                      pointsB[1]))),
                                          input.p);
                    }
                    nLocal = m3Normalize3(axis); // zero falls back to +y
                    if (m3Dot3(nLocal, m3Sub3(cB, cA)) < 0.0f)
                    {
                        nLocal = m3Neg3(nLocal);
                    }
                    m3Vec3 mid = m3MulSV3(0.5f, m3Add3(cA, cB));
                    pALocal = mid;
                    pBLocal = mid;
                    sep = -(rA + rB);
                }
                fresh.normal = m3RotateVec3(xfA->q, nLocal);
                m3Vec3 wA = m3RotateVec3(xfA->q, pALocal);
                m3Vec3 wB = m3RotateVec3(xfA->q, pBLocal);
                fresh.points[0].anchorA = FromCom(world, bodyA, xfA->p.x + (double)wA.x,
                                                  xfA->p.y + (double)wA.y, xfA->p.z + (double)wA.z);
                fresh.points[0].anchorB = FromCom(world, bodyB, xfA->p.x + (double)wB.x,
                                                  xfA->p.y + (double)wB.y, xfA->p.z + (double)wB.z);
                fresh.points[0].separation = sep;
                fresh.points[0].id = 0;
                fresh.pointCount = 1;
            }
        }
        else if (typeA == (uint8_t)m3_planeShape || typeB == (uint8_t)m3_planeShape)
        {
            // Canonical orientation: the plane plays A. If the sphere
            // has the lower index the manifold flips on the way out.
            int32_t planeShape = typeA == (uint8_t)m3_planeShape ? shapeA : shapeB;
            int32_t sphereShape = planeShape == shapeA ? shapeB : shapeA;
            m3Vec3 n = world->shapeGeom[planeShape].v;
            m3real offset = world->shapeGeom[planeShape].s;
            double cx;
            double cy;
            double cz;
            SphereWorldCenter(world, sphereShape, &cx, &cy, &cz);
            double distD = (double)n.x * cx + (double)n.y * cy + (double)n.z * cz - (double)offset;
            fresh = m3CollidePlaneSphere(n, (m3real)distD, world->shapeGeom[sphereShape].s);
            if (fresh.pointCount > 0)
            {
                // anchorA: from the plane BODY's origin to the contact
                // point (the sphere's deepest point projected).
                int32_t planeBody = world->shapeBody[planeShape];
                int32_t ballBody = world->shapeBody[sphereShape];
                double px = cx - (double)(fresh.normal.x * (m3real)distD);
                double py = cy - (double)(fresh.normal.y * (m3real)distD);
                double pz = cz - (double)(fresh.normal.z * (m3real)distD);
                fresh.points[0].anchorA = FromCom(world, planeBody, px, py, pz);
                fresh.points[0].anchorB =
                    m3Add3(FromCom(world, ballBody, cx, cy, cz), fresh.points[0].anchorB);
                if (planeShape != shapeA)
                {
                    // Flip to keep the manifold in key order (A = the
                    // lower shape index, always).
                    m3Vec3 tmp = fresh.points[0].anchorA;
                    fresh.points[0].anchorA = fresh.points[0].anchorB;
                    fresh.points[0].anchorB = tmp;
                    fresh.normal = m3Neg3(fresh.normal);
                }
            }
        }
        else
        {
            double ax;
            double ay;
            double az;
            double bx;
            double by;
            double bz;
            SphereWorldCenter(world, shapeA, &ax, &ay, &az);
            SphereWorldCenter(world, shapeB, &bx, &by, &bz);
            m3Vec3 d = {(m3real)(bx - ax), (m3real)(by - ay), (m3real)(bz - az)};
            fresh = m3CollideSpheres(d, world->shapeGeom[shapeA].s, world->shapeGeom[shapeB].s);
            if (fresh.pointCount > 0)
            {
                // Kernel anchors are from the sphere CENTERS; re-base
                // them to each body's COM.
                fresh.points[0].anchorA = m3Add3(
                    FromCom(world, world->shapeBody[shapeA], ax, ay, az), fresh.points[0].anchorA);
                fresh.points[0].anchorB = m3Add3(
                    FromCom(world, world->shapeBody[shapeB], bx, by, bz), fresh.points[0].anchorB);
            }
        }

        // Warm-start carry: find the old manifold for this key and
        // match points by feature id.
        if (fresh.pointCount > 0 && oldCount > 0)
        {
            int32_t lo = 0;
            int32_t hi = oldCount - 1;
            while (lo <= hi)
            {
                int32_t mid = (lo + hi) / 2;
                if (oldKeys[mid] == key)
                {
                    const m3Manifold* previous = &oldManifolds[mid];
                    for (int32_t k = 0; k < fresh.pointCount; ++k)
                    {
                        for (int32_t o = 0; o < previous->pointCount; ++o)
                        {
                            if (previous->points[o].id == fresh.points[k].id)
                            {
                                fresh.points[k].normalImpulse = previous->points[o].normalImpulse;
                                fresh.points[k].tangentImpulse1 =
                                    previous->points[o].tangentImpulse1;
                                fresh.points[k].tangentImpulse2 =
                                    previous->points[o].tangentImpulse2;
                                fresh.points[k].flags |= 1; // persisted
                                break;
                            }
                        }
                    }
                    break;
                }
                if (oldKeys[mid] < key)
                {
                    lo = mid + 1;
                }
                else
                {
                    hi = mid - 1;
                }
            }
        }
        world->manifolds[i] = fresh;
    }
}

// The task-function shim: the host calls back into the range worker.
typedef struct m3ContactTaskContext
{
    m3World* world;
    const uint64_t* oldKeys;
    const m3Manifold* oldManifolds;
    int32_t oldCount;
} m3ContactTaskContext;

static void ContactTask(int32_t startIndex, int32_t endIndex, void* taskContext)
{
    m3ContactTaskContext* ctx = (m3ContactTaskContext*)taskContext;
    m3UpdateContactsRange(ctx->world, startIndex, endIndex, ctx->oldKeys, ctx->oldManifolds,
                          ctx->oldCount);
}

m3Result m3UpdateContacts(m3World* world, const uint64_t* oldKeys, const m3Manifold* oldManifolds,
                          int32_t oldCount)
{
    if (world->enqueueTask != NULL && world->pairCount > 1)
    {
        m3ContactTaskContext ctx = {world, oldKeys, oldManifolds, oldCount};
        void* task =
            world->enqueueTask(ContactTask, world->pairCount, 16, &ctx, world->userTaskContext);
        world->finishTask(task, world->userTaskContext);
    }
    else
    {
        m3UpdateContactsRange(world, 0, world->pairCount, oldKeys, oldManifolds, oldCount);
    }
    return m3_success;
}
