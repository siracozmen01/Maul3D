// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Shapes: spheres and static half-spaces, their mass, and the body
// shape lists. Same law as bodies: public functions validate and
// journal, internal functions mutate, replay drives the internals.

#include "world_internal.h"

#include <string.h>

int32_t m3ShapeSlot(const m3World* world, m3ShapeId shapeId)
{
    int32_t index = shapeId.index1 - 1;
    if (world == NULL || shapeId.world0 != world->worldIndex0 ||
        !m3IdPoolValid(&world->shapePool, index, shapeId.generation))
    {
        return -1;
    }
    return index;
}

m3QueryFilter m3DefaultQueryFilter(void)
{
    m3QueryFilter f;
    f.categoryBits = ~0ull;
    f.maskBits = ~0ull;
    return f;
}

m3ShapeDef m3DefaultShapeDef(void)
{
    m3ShapeDef def;
    memset(&def, 0, sizeof(def));
    def.density = 1.0f;
    def.friction = 0.6f;
    def.restitution = 0.0f;
    def.categoryBits = 1ull;
    def.maskBits = ~0ull;
    def.groupIndex = 0;
    def.localRotation = (m3Quat){0.0f, 0.0f, 0.0f, 1.0f};
    def.internalValue = M3_SHAPE_COOKIE;
    return def;
}

// Invert a symmetric positive-definite 3x3 via the adjugate. A
// singular or non-positive matrix returns zero (an unrotatable body),
// never NaN.
static m3Mat3 InvertSymmetric(m3Mat3 m)
{
    m3real a = m.cx.x;
    m3real b = m.cy.x; // = m.cx.y by symmetry
    m3real c = m.cz.x;
    m3real d = m.cy.y;
    m3real e = m.cz.y;
    m3real f = m.cz.z;
    m3real co00 = d * f - e * e;
    m3real co01 = c * e - b * f;
    m3real co02 = b * e - c * d;
    m3real det = a * co00 + b * co01 + c * co02;
    if (!(det > 0.0f))
    {
        return m3MakeZeroMat3();
    }
    m3real inv = 1.0f / det;
    m3Mat3 r;
    r.cx = (m3Vec3){co00 * inv, co01 * inv, co02 * inv};
    r.cy = (m3Vec3){co01 * inv, (a * f - c * c) * inv, (b * c - a * e) * inv};
    r.cz = (m3Vec3){co02 * inv, (b * c - a * e) * inv, (a * d - b * b) * inv};
    return r;
}

// Density-scaled mass, centroid, and centroid inertia of one shape.
// Returns 0 for shapes that carry no mass (planes).
// Compose a shape's mass properties through its compound offset
// (10-1): the raw props live in the SHAPE frame; the body wants
// them in ITS frame. c' = p + R c, I' = R I R^T. Identity offsets
// short-circuit: default scenes never enter the rotation.
static void ComposeMassProps(const m3World* world, int32_t s, m3Vec3* com, m3Mat3* inertia)
{
    if (world->shapeHasOffset[s] == 0)
    {
        return;
    }
    m3Quat q = world->shapeLocalRot[s];
    *com = m3Add3(world->shapeLocalPos[s], m3RotateVec3(q, *com));
    m3Mat3 r;
    r.cx = m3RotateVec3(q, (m3Vec3){1.0f, 0.0f, 0.0f});
    r.cy = m3RotateVec3(q, (m3Vec3){0.0f, 1.0f, 0.0f});
    r.cz = m3RotateVec3(q, (m3Vec3){0.0f, 0.0f, 1.0f});
    // I' = R I R^T, built column by column: first T = I R^T, then
    // I' = R T. Columns of R^T are rows of R.
    m3Mat3 t;
    t.cx = (m3Vec3){inertia->cx.x * r.cx.x + inertia->cy.x * r.cx.y + inertia->cz.x * r.cx.z,
                    inertia->cx.y * r.cx.x + inertia->cy.y * r.cx.y + inertia->cz.y * r.cx.z,
                    inertia->cx.z * r.cx.x + inertia->cy.z * r.cx.y + inertia->cz.z * r.cx.z};
    t.cy = (m3Vec3){inertia->cx.x * r.cy.x + inertia->cy.x * r.cy.y + inertia->cz.x * r.cy.z,
                    inertia->cx.y * r.cy.x + inertia->cy.y * r.cy.y + inertia->cz.y * r.cy.z,
                    inertia->cx.z * r.cy.x + inertia->cy.z * r.cy.y + inertia->cz.z * r.cy.z};
    t.cz = (m3Vec3){inertia->cx.x * r.cz.x + inertia->cy.x * r.cz.y + inertia->cz.x * r.cz.z,
                    inertia->cx.y * r.cz.x + inertia->cy.y * r.cz.y + inertia->cz.y * r.cz.z,
                    inertia->cx.z * r.cz.x + inertia->cy.z * r.cz.y + inertia->cz.z * r.cz.z};
    inertia->cx = (m3Vec3){r.cx.x * t.cx.x + r.cy.x * t.cx.y + r.cz.x * t.cx.z,
                           r.cx.y * t.cx.x + r.cy.y * t.cx.y + r.cz.y * t.cx.z,
                           r.cx.z * t.cx.x + r.cy.z * t.cx.y + r.cz.z * t.cx.z};
    inertia->cy = (m3Vec3){r.cx.x * t.cy.x + r.cy.x * t.cy.y + r.cz.x * t.cy.z,
                           r.cx.y * t.cy.x + r.cy.y * t.cy.y + r.cz.y * t.cy.z,
                           r.cx.z * t.cy.x + r.cy.z * t.cy.y + r.cz.z * t.cy.z};
    inertia->cz = (m3Vec3){r.cx.x * t.cz.x + r.cy.x * t.cz.y + r.cz.x * t.cz.z,
                           r.cx.y * t.cz.x + r.cy.y * t.cz.y + r.cz.y * t.cz.z,
                           r.cx.z * t.cz.x + r.cy.z * t.cz.y + r.cz.z * t.cz.z};
}

static int ShapeMassProps(const m3World* world, int32_t s, float* massOut, m3Vec3* comOut,
                          m3Mat3* inertiaOut)
{
    uint8_t type = world->shapeType[s];
    if (type == (uint8_t)m3_sphereShape)
    {
        float r = world->shapeGeom[s].s;
        float m = world->shapeDensity[s] * (4.0f / 3.0f) * M3_PI * r * r * r;
        float ic = 0.4f * m * r * r;
        *massOut = m;
        *comOut = world->shapeGeom[s].v;
        *inertiaOut = m3MakeZeroMat3();
        inertiaOut->cx.x = ic;
        inertiaOut->cy.y = ic;
        inertiaOut->cz.z = ic;
        return 1;
    }
    if (type == (uint8_t)m3_capsuleShape)
    {
        // Closed form: a cylinder of length L plus two hemispheres.
        // About the COM (the segment midpoint), with u the unit axis:
        //   I = Iperp * Identity + (Iaxial - Iperp) * (u outer u)
        // because t1(x)t1 + t2(x)t2 = Identity - u(x)u for any
        // orthonormal basis {t1, u, t2}. No basis matrix needed and
        // the result is exactly symmetric.
        m3Vec3 p1 = world->shapeGeom[s].v;
        m3Vec3 p2 = world->shapeGeom[s].v2;
        float r = world->shapeGeom[s].s;
        m3Vec3 axis = m3Sub3(p2, p1);
        float length = sqrtf(m3Dot3(axis, axis));
        m3Vec3 u = m3MulSV3(1.0f / length, axis); // length > 0 by contract
        float density = world->shapeDensity[s];
        float mCyl = density * M3_PI * r * r * length;
        float mSph = density * (4.0f / 3.0f) * M3_PI * r * r * r;
        float axial = 0.5f * mCyl * r * r + 0.4f * mSph * r * r;
        float perp = mCyl * (length * length / 12.0f + 0.25f * r * r) +
                     mSph * (0.4f * r * r + 0.25f * length * length + 0.375f * length * r);
        *massOut = mCyl + mSph;
        *comOut = m3MulSV3(0.5f, m3Add3(p1, p2));
        m3Mat3 ic2 = m3MakeZeroMat3();
        float d = axial - perp;
        ic2.cx = (m3Vec3){perp + d * u.x * u.x, d * u.x * u.y, d * u.x * u.z};
        ic2.cy = (m3Vec3){d * u.x * u.y, perp + d * u.y * u.y, d * u.y * u.z};
        ic2.cz = (m3Vec3){d * u.x * u.z, d * u.y * u.z, perp + d * u.z * u.z};
        *inertiaOut = ic2;
        return 1;
    }
    if (type == (uint8_t)m3_hullShape)
    {
        const m3HullData* hull = &world->hullData[world->shapeHullIndex[s]];
        float density = world->shapeDensity[s];
        *massOut = density * hull->unitMass;
        *comOut = hull->unitCom;
        m3Mat3 ic = hull->unitInertiaCom;
        ic.cx = m3MulSV3(density, ic.cx);
        ic.cy = m3MulSV3(density, ic.cy);
        ic.cz = m3MulSV3(density, ic.cz);
        *inertiaOut = ic;
        return 1;
    }
    return 0;
}

void m3RecomputeMass(m3World* world, int32_t bodyIndex)
{
    if (world->types[bodyIndex] != (uint8_t)m3_dynamicBody)
    {
        world->invMass[bodyIndex] = 0.0f;
        world->invInertiaLocal[bodyIndex] = m3MakeZeroMat3();
        world->inertiaLocal[bodyIndex] = m3MakeZeroMat3();
        world->localCenters[bodyIndex] = (m3Vec3){0.0f, 0.0f, 0.0f};
        world->minExtents[bodyIndex] = 1.0e30f;
        world->maxExtents[bodyIndex] = 0.0f;
        return;
    }
    // Two passes, the Maul2D lesson: first total mass and the mass
    // weighted center, THEN inertia about that center via the parallel
    // axis theorem. Every term is non-negative and small; no
    // big-minus-big cancellation can occur.
    float mass = 0.0f;
    m3Vec3 center = {0.0f, 0.0f, 0.0f};
    for (int32_t s = world->bodyShapeHead[bodyIndex]; s != -1; s = world->shapeNext[s])
    {
        float m;
        m3Vec3 c;
        m3Mat3 ic;
        if (!ShapeMassProps(world, s, &m, &c, &ic))
        {
            continue;
        }
        ComposeMassProps(world, s, &c, &ic);
        mass += m;
        center = m3Add3(center, m3MulSV3(m, c));
    }
    if (!(mass > 0.0f))
    {
        // Shapeless dynamic body: unit mass, zero inertia (the
        // reference convention).
        world->invMass[bodyIndex] = 1.0f;
        world->invInertiaLocal[bodyIndex] = m3MakeZeroMat3();
        world->inertiaLocal[bodyIndex] = m3MakeZeroMat3();
        world->localCenters[bodyIndex] = (m3Vec3){0.0f, 0.0f, 0.0f};
        world->minExtents[bodyIndex] = 1.0e30f;
        world->maxExtents[bodyIndex] = 0.0f;
        return;
    }
    center = m3MulSV3(1.0f / mass, center);

    m3Mat3 inertia = m3MakeZeroMat3();
    for (int32_t s = world->bodyShapeHead[bodyIndex]; s != -1; s = world->shapeNext[s])
    {
        float m;
        m3Vec3 c;
        m3Mat3 ic;
        if (!ShapeMassProps(world, s, &m, &c, &ic))
        {
            continue;
        }
        ComposeMassProps(world, s, &c, &ic);
        m3Vec3 d = m3Sub3(c, center);
        float d2 = m3Dot3(d, d);
        // I += Ic + m * (|d|^2 Identity - d outer d), the full 3D
        // parallel axis theorem, term by term non-negative diagonals.
        inertia.cx.x += ic.cx.x + m * (d2 - d.x * d.x);
        inertia.cy.y += ic.cy.y + m * (d2 - d.y * d.y);
        inertia.cz.z += ic.cz.z + m * (d2 - d.z * d.z);
        inertia.cy.x += ic.cy.x - m * d.x * d.y;
        inertia.cx.y += ic.cx.y - m * d.x * d.y;
        inertia.cz.x += ic.cz.x - m * d.x * d.z;
        inertia.cx.z += ic.cx.z - m * d.x * d.z;
        inertia.cz.y += ic.cz.y - m * d.y * d.z;
        inertia.cy.z += ic.cy.z - m * d.y * d.z;
    }
    world->invMass[bodyIndex] = 1.0f / mass;
    world->inertiaLocal[bodyIndex] = inertia; // the gyroscopic solve reads it
    world->invInertiaLocal[bodyIndex] = InvertSymmetric(inertia);

    // Extents drive continuous collision (2b-8): minExtent is the
    // thinnest measure any shape brings (motion past half of it in
    // one step marks the body fast), maxExtent bounds the rotation
    // arc in the sweep advance.
    float minExtent = 1.0e30f;
    float maxExtent = 0.0f;
    for (int32_t s2 = world->bodyShapeHead[bodyIndex]; s2 != -1; s2 = world->shapeNext[s2])
    {
        uint8_t type = world->shapeType[s2];
        if (type == (uint8_t)m3_sphereShape)
        {
            float r = world->shapeGeom[s2].s;
            m3Vec3 d = m3Sub3(world->shapeGeom[s2].v, center);
            minExtent = m3MinF(minExtent, r);
            maxExtent = m3MaxF(maxExtent, sqrtf(m3Dot3(d, d)) + r);
        }
        else if (type == (uint8_t)m3_capsuleShape)
        {
            float r = world->shapeGeom[s2].s;
            m3Vec3 d1 = m3Sub3(world->shapeGeom[s2].v, center);
            m3Vec3 d2 = m3Sub3(world->shapeGeom[s2].v2, center);
            minExtent = m3MinF(minExtent, r);
            maxExtent = m3MaxF(maxExtent, sqrtf(m3MaxF(m3Dot3(d1, d1), m3Dot3(d2, d2))) + r);
        }
        else if (type == (uint8_t)m3_hullShape)
        {
            const m3HullData* hull = &world->hullData[world->shapeHullIndex[s2]];
            for (int32_t f = 0; f < hull->faceCount; ++f)
            {
                float dist = hull->faceOffsets[f] - m3Dot3(hull->faceNormals[f], center);
                minExtent = m3MinF(minExtent, dist);
            }
            for (int32_t v = 0; v < hull->vertexCount; ++v)
            {
                m3Vec3 d = m3Sub3(hull->vertices[v], center);
                maxExtent = m3MaxF(maxExtent, sqrtf(m3Dot3(d, d)));
            }
        }
    }
    world->minExtents[bodyIndex] = minExtent;
    world->maxExtents[bodyIndex] = maxExtent;
    world->localCenters[bodyIndex] = center;
}

// Edge convexity for the welding filter: for every triangle edge,
// find the neighbor sharing the undirected vertex pair. No neighbor
// (a boundary) or a neighbor bending away (convex ridge) marks a
// REAL feature; flat and concave edges stay ghost candidates.
void m3BakeMeshEdgeFlags(m3MeshData* mesh)
{
    const m3real tol = 0.005f;
    int32_t triCount = mesh->triangleCount;
    for (int32_t t = 0; t < triCount; ++t)
    {
        mesh->edgeFlags[t] = 0;
        m3Vec3 a = mesh->vertices[mesh->indices[3 * t + 0]];
        m3Vec3 b = mesh->vertices[mesh->indices[3 * t + 1]];
        m3Vec3 c = mesh->vertices[mesh->indices[3 * t + 2]];
        m3Vec3 n = m3Normalize3(m3Cross3(m3Sub3(b, a), m3Sub3(c, a)));
        m3real off = m3Dot3(n, a);
        for (int32_t k = 0; k < 3; ++k)
        {
            int32_t v1 = mesh->indices[3 * t + k];
            int32_t v2 = mesh->indices[3 * t + (k + 1) % 3];
            int32_t neighborOpp = -1;
            for (int32_t u = 0; u < triCount && neighborOpp < 0; ++u)
            {
                if (u == t)
                {
                    continue;
                }
                for (int32_t j = 0; j < 3; ++j)
                {
                    int32_t w1 = mesh->indices[3 * u + j];
                    int32_t w2 = mesh->indices[3 * u + (j + 1) % 3];
                    if ((w1 == v2 && w2 == v1) || (w1 == v1 && w2 == v2))
                    {
                        neighborOpp = mesh->indices[3 * u + (j + 2) % 3];
                        break;
                    }
                }
            }
            if (neighborOpp < 0)
            {
                mesh->edgeFlags[t] |= (uint8_t)(1 << k); // boundary: real
                continue;
            }
            m3real d = m3Dot3(n, mesh->vertices[neighborOpp]) - off;
            if (d < -tol)
            {
                mesh->edgeFlags[t] |= (uint8_t)(1 << k); // convex ridge: real
            }
            // Flat or concave: stays zero, a ghost candidate.
        }
    }
}

int32_t m3CreateShapeInternal(m3World* world, int32_t bodyIndex, uint8_t type,
                              const m3ShapeGeom* geom, const m3ShapeDef* def,
                              const m3HullData* prebuilt, const m3MeshData* meshPrebuilt,
                              const m3VoxelChunkData* voxelPrebuilt)
{
    int32_t index = m3IdPoolAlloc(&world->shapePool);
    if (index < 0)
    {
        return -1;
    }
    world->shapeBody[index] = bodyIndex;
    world->shapeType[index] = type;
    world->shapeGeom[index] = *geom;
    world->shapeDensity[index] = def->density;
    world->shapeFriction[index] = def->friction;
    world->shapeRestitution[index] = def->restitution;
    world->shapeRollingResistance[index] = def->rollingResistance;
    world->shapeCategory[index] = def->categoryBits;
    world->shapeMask[index] = def->maskBits;
    world->shapeGroup[index] = def->groupIndex;
    world->shapeUserData[index] = def->userData;
    world->shapeSensor[index] = def->isSensor ? 1 : 0;
    world->shapeHitEvents[index] = def->enableHitEvents ? 1 : 0;
    world->shapePreSolve[index] = def->enablePreSolveEvents ? 1 : 0;
    world->shapeSurfaceVel[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
    world->shapeLocalPos[index] = def->localPosition;
    world->shapeLocalRot[index] = def->localRotation;
    world->shapeHasOffset[index] = (def->localPosition.x != 0.0f || def->localPosition.y != 0.0f ||
                                    def->localPosition.z != 0.0f || def->localRotation.x != 0.0f ||
                                    def->localRotation.y != 0.0f || def->localRotation.z != 0.0f ||
                                    def->localRotation.w != 1.0f)
                                       ? 1
                                       : 0;
    // Push onto the body's list head (canonical: creation order is
    // recoverable because replay recreates in the same order).
    world->shapeNext[index] = world->bodyShapeHead[bodyIndex];
    world->bodyShapeHead[bodyIndex] = index;
    world->shapeHullIndex[index] = -1;
    if (type == (uint8_t)m3_hullShape)
    {
        // Boxes rebuild from geom.v (the journaled half extents);
        // general hulls arrive prebuilt from QuickHull (their recipe
        // rides the dedicated journal op instead).
        m3HullData data;
        if (prebuilt == NULL)
        {
            m3BuildBoxHull(&data, geom->v);
        }
        world->shapeHullIndex[index] = m3InternHull(world, prebuilt != NULL ? prebuilt : &data);
        if (world->shapeHullIndex[index] < 0)
        {
            world->bodyShapeHead[bodyIndex] = world->shapeNext[index];
            world->shapeNext[index] = -1;
            world->shapeBody[index] = -1;
            m3IdPoolFree(&world->shapePool, index);
            return -1;
        }
    }
    world->shapeMeshIndex[index] = -1;
    if (type == (uint8_t)m3_meshShape)
    {
        // No content dedupe: meshes are big and user-authored; each
        // create claims a fresh slot.
        int32_t meshIndex = m3IdPoolAlloc(&world->meshPool);
        if (meshIndex < 0)
        {
            world->bodyShapeHead[bodyIndex] = world->shapeNext[index];
            world->shapeNext[index] = -1;
            world->shapeBody[index] = -1;
            m3IdPoolFree(&world->shapePool, index);
            return -1; // mesh slots exhausted: loud at the caller
        }
        world->meshData[meshIndex] = *meshPrebuilt;
        m3BakeMeshEdgeFlags(&world->meshData[meshIndex]);
        m3MeshBvhBuild(&world->meshBvh[meshIndex], &world->meshData[meshIndex]);
        world->meshRefCounts[meshIndex] = 1;
        world->shapeMeshIndex[index] = meshIndex;
    }
    world->shapeVoxelIndex[index] = -1;
    if (type == (uint8_t)m3_voxelShape)
    {
        // The mesh-slot pattern: fresh slot, state block copied in,
        // the DERIVED surface built from it (the BVH law).
        int32_t voxelIndex = m3IdPoolAlloc(&world->voxelPool);
        if (voxelIndex < 0)
        {
            world->bodyShapeHead[bodyIndex] = world->shapeNext[index];
            world->shapeNext[index] = -1;
            world->shapeBody[index] = -1;
            m3IdPoolFree(&world->shapePool, index);
            return -1; // voxel slots exhausted: loud at the caller
        }
        world->voxelData[voxelIndex] = *voxelPrebuilt;
        m3VoxelSurfaceBuild(&world->voxelSurface[voxelIndex], &world->voxelData[voxelIndex]);
        world->voxelRefCounts[voxelIndex] = 1;
        world->shapeVoxelIndex[index] = voxelIndex;
        world->voxelShape[voxelIndex] = index;
        // A new chunk can weld to existing ones and change THEIR
        // border coverage too: rebuild links, refresh the seam.
        m3VoxelRebuildLinks(world);
        m3VoxelCoverageRefreshAround(world, voxelIndex);
    }
    // Spheres and hulls enter the broadphase tree; infinite planes
    // stay out and take the dedicated pair pass.
    if (type != (uint8_t)m3_planeShape)
    {
        double lo[3];
        double hi[3];
        m3ShapeFatAabb(world, index, lo, hi);
        world->proxyIds[index] = m3TreeInsert(&world->tree, lo, hi, index);
        if (world->proxyIds[index] == M3_TREE_NULL)
        {
            // Tree pool exhausted: undo loudly, never a half-created
            // shape.
            m3ReleaseHull(world, world->shapeHullIndex[index]);
            world->shapeHullIndex[index] = -1;
            world->bodyShapeHead[bodyIndex] = world->shapeNext[index];
            world->shapeNext[index] = -1;
            world->shapeBody[index] = -1;
            m3IdPoolFree(&world->shapePool, index);
            return -1;
        }
    }
    else
    {
        world->proxyIds[index] = M3_TREE_NULL;
    }
    m3RecomputeMass(world, bodyIndex);
    return index;
}

void m3DestroyShapeInternal(m3World* world, int32_t index)
{
    int32_t bodyIndex = world->shapeBody[index];
    // Unlink from the body's list.
    int32_t* cursor = &world->bodyShapeHead[bodyIndex];
    while (*cursor != -1)
    {
        if (*cursor == index)
        {
            *cursor = world->shapeNext[index];
            break;
        }
        cursor = &world->shapeNext[*cursor];
    }
    world->shapeBody[index] = -1;
    world->shapeType[index] = 0;
    world->shapeGeom[index] = (m3ShapeGeom){{0.0f, 0.0f, 0.0f}, 0.0f, {0.0f, 0.0f, 0.0f}, 0.0f};
    world->shapeDensity[index] = 0.0f;
    world->shapeFriction[index] = 0.0f;
    world->shapeRestitution[index] = 0.0f;
    world->shapeRollingResistance[index] = 0.0f;
    world->shapeCategory[index] = 0;
    world->shapeMask[index] = 0;
    world->shapeGroup[index] = 0;
    world->shapeUserData[index] = 0;
    world->shapeSensor[index] = 0;
    world->shapeHitEvents[index] = 0;
    world->shapePreSolve[index] = 0;
    world->shapeSurfaceVel[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
    world->shapeLocalPos[index] = (m3Vec3){0.0f, 0.0f, 0.0f};
    world->shapeLocalRot[index] = (m3Quat){0.0f, 0.0f, 0.0f, 1.0f};
    world->shapeHasOffset[index] = 0;
    world->shapeNext[index] = -1;
    if (world->proxyIds[index] != M3_TREE_NULL)
    {
        m3TreeRemove(&world->tree, world->proxyIds[index]);
        world->proxyIds[index] = M3_TREE_NULL;
    }
    m3ReleaseHull(world, world->shapeHullIndex[index]);
    world->shapeHullIndex[index] = -1;
    if (world->shapeMeshIndex[index] >= 0)
    {
        int32_t meshIndex = world->shapeMeshIndex[index];
        world->meshRefCounts[meshIndex] -= 1;
        if (world->meshRefCounts[meshIndex] == 0)
        {
            m3MeshDataFree(&world->meshData[meshIndex]);
            m3MeshBvhFree(&world->meshBvh[meshIndex]);
            m3IdPoolFree(&world->meshPool, meshIndex);
        }
        world->shapeMeshIndex[index] = -1;
    }
    if (world->shapeVoxelIndex[index] >= 0)
    {
        int32_t voxelIndex = world->shapeVoxelIndex[index];
        world->voxelRefCounts[voxelIndex] -= 1;
        if (world->voxelRefCounts[voxelIndex] == 0)
        {
            memset(&world->voxelData[voxelIndex], 0, sizeof(m3VoxelChunkData));
            m3MeshBvhFree(&world->voxelSurface[voxelIndex].bvh);
            memset(&world->voxelSurface[voxelIndex], 0, sizeof(m3VoxelSurface));
            world->voxelShape[voxelIndex] = -1;
            m3IdPoolFree(&world->voxelPool, voxelIndex);
            // A vanished chunk un-welds its neighbors: their border
            // faces just became exposed.
            m3VoxelRebuildLinks(world);
            for (int32_t v = 0; v < world->voxelPool.maxIndex; ++v)
            {
                if (world->voxelPool.alive[v] != 0)
                {
                    m3VoxelCoverageBuild(world, v);
                }
            }
        }
        world->shapeVoxelIndex[index] = -1;
    }
    m3IdPoolFree(&world->shapePool, index);
    m3RecomputeMass(world, bodyIndex);
}

static m3ShapeId CreateShapeCommon(m3BodyId bodyId, const m3ShapeDef* def, uint8_t type,
                                   const m3ShapeGeom* geom)
{
    m3World* world = m3WorldFromIndex0(bodyId.world0);
    int32_t bodyIndex = world != NULL ? m3BodySlot(world, bodyId) : -1;
    if (bodyIndex < 0 || def == NULL || def->internalValue != M3_SHAPE_COOKIE)
    {
        // Contract, not invariant: bad input returns the null id.
        return m3_nullShapeId;
    }
    float rotLen2 =
        def->localRotation.x * def->localRotation.x + def->localRotation.y * def->localRotation.y +
        def->localRotation.z * def->localRotation.z + def->localRotation.w * def->localRotation.w;
    if (!m3FiniteV3(def->localPosition) || !m3FiniteQuat(def->localRotation) || rotLen2 < 0.99f ||
        rotLen2 > 1.01f)
    {
        // The compound offset demands a near-unit rotation (10-1).
        return m3_nullShapeId;
    }
    if (!m3FiniteF(def->density) || !(def->density > 0.0f) || !m3FiniteF(def->friction) ||
        def->friction < 0.0f || !m3FiniteF(def->restitution) || def->restitution < 0.0f ||
        !m3FiniteF(def->rollingResistance) || def->rollingResistance < 0.0f)
    {
        return m3_nullShapeId; // hostile material: refused loudly
    }
    int32_t index = m3CreateShapeInternal(world, bodyIndex, type, geom, def, NULL, NULL, NULL);
    if (index < 0)
    {
        return m3_nullShapeId;
    }
    m3ShapeId id = {index + 1, world->worldIndex0, world->shapePool.generations[index]};
    if (world->journalActive != 0)
    {
        m3CreateShapeOp record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.geom = *geom;
        record.body = bodyId;
        record.expected = id;
        record.type = type;
        m3JournalRecord(world, m3_opCreateShape, &record, (int32_t)sizeof(record));
    }
    return id;
}

m3ShapeId m3CreateSphereShape(m3BodyId bodyId, const m3ShapeDef* def, const m3Sphere* sphere)
{
    if (sphere == NULL || !(sphere->radius > 0.0f) || !m3FiniteF(sphere->radius) ||
        !m3FiniteV3(sphere->center))
    {
        return m3_nullShapeId;
    }
    m3World* world = m3WorldFromIndex0(bodyId.world0);
    int32_t bodyIndex = world != NULL ? m3BodySlot(world, bodyId) : -1;
    if (bodyIndex < 0)
    {
        return m3_nullShapeId;
    }
    // The 2a off-origin refusal is gone: the full inertia tensor and
    // center-of-mass bookkeeping (2b-1) make offset spheres exact.
    m3ShapeGeom geom = {sphere->center, sphere->radius, {0.0f, 0.0f, 0.0f}, 0.0f};
    return CreateShapeCommon(bodyId, def, (uint8_t)m3_sphereShape, &geom);
}

m3ShapeId m3CreatePlaneShape(m3BodyId bodyId, const m3ShapeDef* def, const m3Plane* plane)
{
    if (plane == NULL || !m3FiniteV3(plane->normal) || !m3FiniteF(plane->offset) ||
        !(m3Dot3(plane->normal, plane->normal) > 1.0e-12f))
    {
        return m3_nullShapeId; // a zero or poisoned normal never
                               // reaches the normalize below
    }
    m3World* world = m3WorldFromIndex0(bodyId.world0);
    int32_t bodyIndex = world != NULL ? m3BodySlot(world, bodyId) : -1;
    if (bodyIndex < 0 || world->types[bodyIndex] != (uint8_t)m3_staticBody)
    {
        // A plane on a dynamic body is refused loudly: an infinite
        // shape has no mass.
        return m3_nullShapeId;
    }
    m3ShapeGeom geom = {m3Normalize3(plane->normal), plane->offset, {0.0f, 0.0f, 0.0f}, 0.0f};
    return CreateShapeCommon(bodyId, def, (uint8_t)m3_planeShape, &geom);
}

m3ShapeId m3CreateCapsuleShape(m3BodyId bodyId, const m3ShapeDef* def, const m3Capsule* capsule)
{
    if (capsule == NULL || !(capsule->radius > 0.0f) || !m3FiniteF(capsule->radius) ||
        !m3FiniteV3(capsule->point1) || !m3FiniteV3(capsule->point2))
    {
        return m3_nullShapeId;
    }
    m3Vec3 axis = m3Sub3(capsule->point2, capsule->point1);
    if (!(m3Dot3(axis, axis) > 0.0f))
    {
        // A zero-length capsule is a sphere; asking for one is a
        // contract violation, refused loudly (use m3CreateSphereShape).
        return m3_nullShapeId;
    }
    m3ShapeGeom geom;
    geom.v = capsule->point1;
    geom.s = capsule->radius;
    geom.v2 = capsule->point2;
    geom.s2 = 0.0f;
    return CreateShapeCommon(bodyId, def, (uint8_t)m3_capsuleShape, &geom);
}

m3ShapeId m3CreateHullShape(m3BodyId bodyId, const m3ShapeDef* def, const m3Vec3* points,
                            int32_t count)
{
    m3World* world = m3WorldFromIndex0(bodyId.world0);
    int32_t bodyIndex = world != NULL ? m3BodySlot(world, bodyId) : -1;
    if (bodyIndex < 0 || def == NULL || def->internalValue != M3_SHAPE_COOKIE)
    {
        return m3_nullShapeId;
    }
    if (points == NULL || count <= 0)
    {
        return m3_nullShapeId;
    }
    for (int32_t i = 0; i < count; ++i)
    {
        if (!m3FiniteV3(points[i]))
        {
            return m3_nullShapeId; // a poisoned cloud never reaches
                                   // QuickHull's arithmetic
        }
    }
    m3HullData data;
    if (!m3ComputeHull(points, count, &data))
    {
        // Degenerate cloud or over the caps: contract, null id.
        return m3_nullShapeId;
    }
    m3ShapeGeom geom;
    memset(&geom, 0, sizeof(geom));
    int32_t index = m3CreateShapeInternal(world, bodyIndex, (uint8_t)m3_hullShape, &geom, def,
                                          &data, NULL, NULL);
    if (index < 0)
    {
        return m3_nullShapeId;
    }
    m3ShapeId id = {index + 1, world->worldIndex0, world->shapePool.generations[index]};
    if (world->journalActive != 0)
    {
        m3CreateHullShapeOp record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.body = bodyId;
        record.expected = id;
        record.count = count;
        memcpy(record.points, points, (size_t)count * sizeof(m3Vec3));
        m3JournalRecord(world, m3_opCreateHullShape, &record, (int32_t)sizeof(record));
    }
    return id;
}

m3ShapeId m3CreateMeshShape(m3BodyId bodyId, const m3ShapeDef* def, const m3Vec3* vertices,
                            int32_t vertexCount, const uint16_t* indices, int32_t triangleCount)
{
    m3World* world = m3WorldFromIndex0(bodyId.world0);
    int32_t bodyIndex = world != NULL ? m3BodySlot(world, bodyId) : -1;
    if (bodyIndex < 0 || def == NULL || def->internalValue != M3_SHAPE_COOKIE ||
        world->types[bodyIndex] != (uint8_t)m3_staticBody)
    {
        // Meshes are static world geometry: a dynamic mesh body is
        // refused loudly (no mass model for triangle soup).
        return m3_nullShapeId;
    }
    if (vertices == NULL || indices == NULL || vertexCount < 3 || vertexCount > M3_MESH_MAX_VERTS ||
        triangleCount < 1 || triangleCount > M3_MESH_MAX_TRIS)
    {
        return m3_nullShapeId;
    }
    for (int32_t i = 0; i < 3 * triangleCount; ++i)
    {
        if (indices[i] >= (uint16_t)vertexCount)
        {
            return m3_nullShapeId; // out-of-range index: contract
        }
    }
    for (int32_t i = 0; i < vertexCount; ++i)
    {
        if (!m3FiniteV3(vertices[i]))
        {
            return m3_nullShapeId; // poisoned vertex: contract
        }
    }
    m3MeshData mesh;
    memset(&mesh, 0, sizeof(mesh));
    mesh.vertexCount = vertexCount;
    mesh.triangleCount = triangleCount;
    if (!m3MeshDataAlloc(&mesh))
    {
        return m3_nullShapeId;
    }
    memcpy(mesh.vertices, vertices, (size_t)vertexCount * sizeof(m3Vec3));
    memcpy(mesh.indices, indices, (size_t)(3 * triangleCount) * sizeof(uint16_t));
    m3ShapeGeom geom;
    memset(&geom, 0, sizeof(geom));
    // The slot takes OWNERSHIP of the arrays on success (the struct
    // copy carries the pointers); failure frees them here.
    int32_t index = m3CreateShapeInternal(world, bodyIndex, (uint8_t)m3_meshShape, &geom, def, NULL,
                                          &mesh, NULL);
    if (index < 0)
    {
        m3MeshDataFree(&mesh);
        return m3_nullShapeId;
    }
    m3ShapeId id = {index + 1, world->worldIndex0, world->shapePool.generations[index]};
    if (world->journalActive != 0)
    {
        // Exact-size payload: header, then the raw vertex and index
        // arrays (the recipe; replay rebuilds and verifies the id).
        int32_t vertexBytes = vertexCount * (int32_t)sizeof(m3Vec3);
        int32_t indexBytes = 3 * triangleCount * (int32_t)sizeof(uint16_t);
        int32_t payloadBytes = (int32_t)sizeof(m3CreateMeshShapeOp) + vertexBytes + indexBytes;
        uint8_t* payload = (uint8_t*)m3AllocZeroed(payloadBytes);
        if (payload != NULL)
        {
            m3CreateMeshShapeOp record;
            memset(&record, 0, sizeof(record));
            record.def = *def;
            record.body = bodyId;
            record.expected = id;
            record.vertexCount = vertexCount;
            record.triangleCount = triangleCount;
            memcpy(payload, &record, sizeof(record));
            memcpy(payload + sizeof(record), vertices, (size_t)vertexBytes);
            memcpy(payload + sizeof(record) + vertexBytes, indices, (size_t)indexBytes);
            m3JournalRecord(world, m3_opCreateMeshShape, payload, payloadBytes);
            m3Free(payload);
        }
    }
    return id;
}

m3ShapeId m3CreateHeightFieldShape(m3BodyId bodyId, const m3ShapeDef* def, const float* heights,
                                   int32_t nx, int32_t nz, m3real cellSize)
{
    if (heights == NULL || nx < 2 || nx > 32 || nz < 2 || nz > 32 || !(cellSize > 0.0f) ||
        !m3FiniteF(cellSize))
    {
        return m3_nullShapeId; // grid contract: chunks tile larger terrain
    }
    for (int32_t i = 0; i < nx * nz; ++i)
    {
        if (!m3FiniteF(heights[i]))
        {
            return m3_nullShapeId; // poisoned sample: contract
        }
    }
    // Triangulate the grid (CCW seen from +y) and reuse the mesh
    // path whole: welding, journaling, snapshotting all come free.
    // Heap staging (10-3): 65k-scale grids no longer fit a stack.
    m3Vec3* verts = (m3Vec3*)m3AllocZeroed(nx * nz * (int32_t)sizeof(m3Vec3));
    uint16_t* tris = (uint16_t*)m3AllocZeroed(6 * (nx - 1) * (nz - 1) * (int32_t)sizeof(uint16_t));
    if (verts == NULL || tris == NULL)
    {
        m3Free(verts);
        m3Free(tris);
        return m3_nullShapeId;
    }
    for (int32_t iz = 0; iz < nz; ++iz)
    {
        for (int32_t ix = 0; ix < nx; ++ix)
        {
            verts[iz * nx + ix] =
                (m3Vec3){cellSize * (m3real)ix, heights[iz * nx + ix], cellSize * (m3real)iz};
        }
    }
    int32_t n = 0;
    for (int32_t iz = 0; iz < nz - 1; ++iz)
    {
        for (int32_t ix = 0; ix < nx - 1; ++ix)
        {
            uint16_t v00 = (uint16_t)(iz * nx + ix);
            uint16_t v10 = (uint16_t)(iz * nx + ix + 1);
            uint16_t v01 = (uint16_t)((iz + 1) * nx + ix);
            uint16_t v11 = (uint16_t)((iz + 1) * nx + ix + 1);
            tris[n++] = v00;
            tris[n++] = v11;
            tris[n++] = v10;
            tris[n++] = v00;
            tris[n++] = v01;
            tris[n++] = v11;
        }
    }
    m3ShapeId id = m3CreateMeshShape(bodyId, def, verts, nx * nz, tris, n / 3);
    m3Free(verts);
    m3Free(tris);
    return id;
}

m3ShapeId m3CreateVoxelChunkShape(m3BodyId bodyId, const m3ShapeDef* def, const uint8_t* voxels,
                                  const uint16_t* payload, m3real cellSize)
{
    m3World* world = m3WorldFromIndex0(bodyId.world0);
    int32_t bodyIndex = world != NULL ? m3BodySlot(world, bodyId) : -1;
    if (bodyIndex < 0 || def == NULL || def->internalValue != M3_SHAPE_COOKIE || voxels == NULL ||
        !(cellSize > 0.0f) || !m3FiniteF(cellSize))
    {
        return m3_nullShapeId;
    }
    if (world->types[bodyIndex] != (uint8_t)m3_staticBody || def->isSensor)
    {
        // Static level geometry only in 3-1 (dynamic voxel bodies
        // have no consumer yet), and sensors are convex volumes by
        // contract: both refusals are loud.
        return m3_nullShapeId;
    }
    m3VoxelChunkData* chunk = (m3VoxelChunkData*)m3AllocZeroed((int32_t)sizeof(m3VoxelChunkData));
    if (chunk == NULL)
    {
        return m3_nullShapeId;
    }
    int32_t filled = m3VoxelPack(chunk, voxels, payload, cellSize);
    if (filled == 0)
    {
        m3Free(chunk);
        return m3_nullShapeId; // an empty chunk is a request for nothing
    }
    m3ShapeGeom geom;
    memset(&geom, 0, sizeof(geom));
    geom.s = cellSize;
    int32_t index = m3CreateShapeInternal(world, bodyIndex, (uint8_t)m3_voxelShape, &geom, def,
                                          NULL, NULL, chunk);
    m3Free(chunk);
    if (index < 0)
    {
        return m3_nullShapeId;
    }
    m3ShapeId id = {index + 1, world->worldIndex0, world->shapePool.generations[index]};
    if (world->journalActive != 0)
    {
        // Header + the packed grid (bitset and payload): the exact
        // recipe, so replay rebuilds the identical chunk and surface.
        struct
        {
            m3ShapeDef def;
            m3BodyId body;
            m3ShapeId expected;
            m3real cellSize;
        } record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.body = bodyId;
        record.expected = id;
        record.cellSize = cellSize;
        uint8_t payloadBuf[sizeof(record) + sizeof(((m3VoxelChunkData*)0)->occupancy) +
                           sizeof(((m3VoxelChunkData*)0)->payload) +
                           sizeof(((m3VoxelChunkData*)0)->fill)];
        memcpy(payloadBuf, &record, sizeof(record));
        const m3VoxelChunkData* stored = &world->voxelData[world->shapeVoxelIndex[index]];
        memcpy(payloadBuf + sizeof(record), stored->occupancy, sizeof(stored->occupancy));
        memcpy(payloadBuf + sizeof(record) + sizeof(stored->occupancy), stored->payload,
               sizeof(stored->payload));
        memcpy(payloadBuf + sizeof(record) + sizeof(stored->occupancy) + sizeof(stored->payload),
               stored->fill, sizeof(stored->fill));
        m3JournalRecord(world, m3_opCreateVoxelChunkShape, payloadBuf, (int32_t)sizeof(payloadBuf));
    }
    return id;
}

m3ShapeId m3CreateBoxShape(m3BodyId bodyId, const m3ShapeDef* def, m3Vec3 halfExtents)
{
    if (!(halfExtents.x > 0.0f) || !(halfExtents.y > 0.0f) || !(halfExtents.z > 0.0f) ||
        !m3FiniteV3(halfExtents))
    {
        return m3_nullShapeId; // contract: bad extents return null
    }
    m3ShapeGeom geom = {halfExtents, 0.0f, {0.0f, 0.0f, 0.0f}, 0.0f};
    return CreateShapeCommon(bodyId, def, (uint8_t)m3_hullShape, &geom);
}

bool m3Shape_IsValid(m3ShapeId shapeId)
{
    m3World* world = m3WorldFromIndex0(shapeId.world0);
    return world != NULL && m3ShapeSlot(world, shapeId) >= 0;
}

void m3DestroyShape(m3ShapeId shapeId)
{
    m3World* world = m3WorldFromIndex0(shapeId.world0);
    int32_t index = world != NULL ? m3ShapeSlot(world, shapeId) : -1;
    if (index < 0)
    {
        return;
    }
    if (world->journalActive != 0)
    {
        m3JournalRecord(world, m3_opDestroyShape, &shapeId, (int32_t)sizeof(shapeId));
    }
    int32_t bodyIndex = world->shapeBody[index];
    m3DestroyShapeInternal(world, index);
    m3RecomputeMass(world, bodyIndex);
    if (world->types[bodyIndex] == (uint8_t)m3_dynamicBody)
    {
        m3SetAwakeInternal(world, bodyIndex, 1);
    }
}

// --- Runtime materials (8-4) ------------------------------------------------

void m3SetShapeFrictionInternal(m3World* world, int32_t slot, float value)
{
    world->shapeFriction[slot] = value;
}

void m3SetShapeRestitutionInternal(m3World* world, int32_t slot, float value)
{
    world->shapeRestitution[slot] = value;
}

void m3SetShapeRollingInternal(m3World* world, int32_t slot, float value)
{
    world->shapeRollingResistance[slot] = value;
}

void m3SetShapeDensityInternal(m3World* world, int32_t slot, float value, int32_t updateMass)
{
    world->shapeDensity[slot] = value;
    if (updateMass != 0)
    {
        m3RecomputeMass(world, world->shapeBody[slot]);
    }
}

// One resolve + one journal + one internal, the body.c pattern.
static m3World* ResolveShape(m3ShapeId shapeId, int32_t* outSlot)
{
    m3World* world = m3WorldFromIndex0(shapeId.world0);
    if (world == NULL)
    {
        return NULL;
    }
    int32_t slot = m3ShapeSlot(world, shapeId);
    if (slot < 0)
    {
        return NULL;
    }
    *outSlot = slot;
    return world;
}

static void ShapeScalarOp(m3ShapeId shapeId, int32_t op, float value)
{
    int32_t slot;
    m3World* world = ResolveShape(shapeId, &slot);
    if (world == NULL)
    {
        return;
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m3ShapeId id;
            float value;
        } record;
        memset(&record, 0, sizeof(record));
        record.id = shapeId;
        record.value = value;
        m3JournalRecord(world, op, &record, (int32_t)sizeof(record));
    }
    if (op == m3_opSetShapeFriction)
    {
        m3SetShapeFrictionInternal(world, slot, value);
    }
    else if (op == m3_opSetShapeRestitution)
    {
        m3SetShapeRestitutionInternal(world, slot, value);
    }
    else
    {
        m3SetShapeRollingInternal(world, slot, value);
    }
}

void m3Shape_SetFriction(m3ShapeId shapeId, float friction)
{
    if (!m3FiniteF(friction) || friction < 0.0f)
    {
        return;
    }
    ShapeScalarOp(shapeId, m3_opSetShapeFriction, friction);
}

float m3Shape_GetFriction(m3ShapeId shapeId)
{
    int32_t slot;
    m3World* world = ResolveShape(shapeId, &slot);
    return world != NULL ? world->shapeFriction[slot] : 0.0f;
}

void m3Shape_SetRestitution(m3ShapeId shapeId, float restitution)
{
    if (!m3FiniteF(restitution) || restitution < 0.0f)
    {
        return;
    }
    ShapeScalarOp(shapeId, m3_opSetShapeRestitution, restitution);
}

float m3Shape_GetRestitution(m3ShapeId shapeId)
{
    int32_t slot;
    m3World* world = ResolveShape(shapeId, &slot);
    return world != NULL ? world->shapeRestitution[slot] : 0.0f;
}

void m3Shape_SetRollingResistance(m3ShapeId shapeId, float value)
{
    if (!m3FiniteF(value) || value < 0.0f)
    {
        return;
    }
    ShapeScalarOp(shapeId, m3_opSetShapeRolling, value);
}

float m3Shape_GetRollingResistance(m3ShapeId shapeId)
{
    int32_t slot;
    m3World* world = ResolveShape(shapeId, &slot);
    return world != NULL ? world->shapeRollingResistance[slot] : 0.0f;
}

void m3Shape_SetDensity(m3ShapeId shapeId, float density, bool updateBodyMass)
{
    int32_t slot;
    m3World* world = ResolveShape(shapeId, &slot);
    if (world == NULL || !m3FiniteF(density) || density <= 0.0f)
    {
        return;
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m3ShapeId id;
            float value;
            int32_t updateMass;
        } record;
        memset(&record, 0, sizeof(record));
        record.id = shapeId;
        record.value = density;
        record.updateMass = updateBodyMass ? 1 : 0;
        m3JournalRecord(world, m3_opSetShapeDensity, &record, (int32_t)sizeof(record));
    }
    m3SetShapeDensityInternal(world, slot, density, updateBodyMass ? 1 : 0);
}

float m3Shape_GetDensity(m3ShapeId shapeId)
{
    int32_t slot;
    m3World* world = ResolveShape(shapeId, &slot);
    return world != NULL ? world->shapeDensity[slot] : 0.0f;
}

void m3EnableShapeHitEventsInternal(m3World* world, int32_t slot, int32_t on)
{
    world->shapeHitEvents[slot] = on != 0 ? 1 : 0;
}

void m3EnableShapePreSolveInternal(m3World* world, int32_t slot, int32_t on)
{
    world->shapePreSolve[slot] = on != 0 ? 1 : 0;
}

static void ShapeFlagOp(m3ShapeId shapeId, int32_t op, bool flag)
{
    int32_t slot;
    m3World* world = ResolveShape(shapeId, &slot);
    if (world == NULL)
    {
        return;
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m3ShapeId id;
            int32_t on;
        } record;
        memset(&record, 0, sizeof(record));
        record.id = shapeId;
        record.on = flag ? 1 : 0;
        m3JournalRecord(world, op, &record, (int32_t)sizeof(record));
    }
    if (op == m3_opEnableShapeHitEvents)
    {
        m3EnableShapeHitEventsInternal(world, slot, flag ? 1 : 0);
    }
    else
    {
        m3EnableShapePreSolveInternal(world, slot, flag ? 1 : 0);
    }
}

void m3Shape_EnableHitEvents(m3ShapeId shapeId, bool flag)
{
    ShapeFlagOp(shapeId, m3_opEnableShapeHitEvents, flag);
}

bool m3Shape_AreHitEventsEnabled(m3ShapeId shapeId)
{
    int32_t slot;
    m3World* world = ResolveShape(shapeId, &slot);
    return world != NULL && world->shapeHitEvents[slot] != 0;
}

void m3Shape_EnablePreSolve(m3ShapeId shapeId, bool flag)
{
    ShapeFlagOp(shapeId, m3_opEnableShapePreSolve, flag);
}

bool m3Shape_IsPreSolveEnabled(m3ShapeId shapeId)
{
    int32_t slot;
    m3World* world = ResolveShape(shapeId, &slot);
    return world != NULL && world->shapePreSolve[slot] != 0;
}

void m3SetSurfaceVelocityInternal(m3World* world, int32_t slot, m3Vec3 v)
{
    world->shapeSurfaceVel[slot] = v;
    int32_t body = world->shapeBody[slot];
    if (world->types[body] == (uint8_t)m3_dynamicBody)
    {
        m3SetAwakeInternal(world, body, 1);
    }
}

void m3Shape_SetSurfaceVelocity(m3ShapeId shapeId, m3Vec3 velocity)
{
    int32_t slot;
    m3World* world = ResolveShape(shapeId, &slot);
    if (world == NULL || !m3FiniteV3(velocity))
    {
        return;
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m3ShapeId id;
            m3Vec3 v;
        } record;
        memset(&record, 0, sizeof(record));
        record.id = shapeId;
        record.v = velocity;
        m3JournalRecord(world, m3_opSetSurfaceVelocity, &record, (int32_t)sizeof(record));
    }
    m3SetSurfaceVelocityInternal(world, slot, velocity);
}

// --- Mesh content ownership (10-3) ------------------------------------------

bool m3MeshDataAlloc(m3MeshData* mesh)
{
    m3Free(mesh->vertices);
    m3Free(mesh->indices);
    m3Free(mesh->edgeFlags);
    mesh->vertices = NULL;
    mesh->indices = NULL;
    mesh->edgeFlags = NULL;
    if (mesh->vertexCount <= 0 || mesh->triangleCount <= 0)
    {
        return mesh->vertexCount == 0 && mesh->triangleCount == 0; // an empty slot is legal
    }
    mesh->vertices = (m3Vec3*)m3AllocZeroed(mesh->vertexCount * (int32_t)sizeof(m3Vec3));
    mesh->indices = (uint16_t*)m3AllocZeroed(3 * mesh->triangleCount * (int32_t)sizeof(uint16_t));
    mesh->edgeFlags = (uint8_t*)m3AllocZeroed(mesh->triangleCount);
    if (mesh->vertices == NULL || mesh->indices == NULL || mesh->edgeFlags == NULL)
    {
        m3MeshDataFree(mesh);
        return false;
    }
    return true;
}

void m3MeshDataFree(m3MeshData* mesh)
{
    m3Free(mesh->vertices);
    m3Free(mesh->indices);
    m3Free(mesh->edgeFlags);
    memset(mesh, 0, sizeof(*mesh));
}
