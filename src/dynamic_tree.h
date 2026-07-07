// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The fat-AABB dynamic tree (the classic reference structure: surface
// area heuristic insertion, AVL balancing), in DOUBLE bounds because
// world positions are double and a float tree degrades far from the
// origin (the documented large-world hazard). The tree is persistent
// snapshot state: its shape is a deterministic function of the op
// history, and restoring it bit-exactly keeps rollback total.

#ifndef MAUL3D_DYNAMIC_TREE_H
#define MAUL3D_DYNAMIC_TREE_H

#include "maul3d/base.h"

#define M3_TREE_NULL (-1)

typedef struct m3TreeNode
{
    double lo[3];
    double hi[3];
    int32_t parent; // the free-list next when the node is free
    int32_t child1;
    int32_t child2;
    int32_t userData; // the shape index on leaves
    int32_t height;   // -1 = free, 0 = leaf
    int32_t pad;      // keeps the node padding-free at 8 alignment
} m3TreeNode;

_Static_assert(sizeof(m3TreeNode) == 72, "tree node must be padding-free");

typedef struct m3Tree
{
    m3TreeNode* nodes; // fixed capacity, allocated by the world
    int32_t capacity;
    int32_t root;     // M3_TREE_NULL when empty
    int32_t freeList; // head of the free chain
} m3Tree;

m3Tree m3TreeCreate(int32_t capacity);
void m3TreeDestroy(m3Tree* tree);

/// Insert a leaf with fat bounds. Returns the node id, or M3_TREE_NULL
/// when the pool is exhausted (loud at the caller).
int32_t m3TreeInsert(m3Tree* tree, const double lo[3], const double hi[3], int32_t userData);
void m3TreeRemove(m3Tree* tree, int32_t nodeId);

/// True when the leaf's fat bounds still contain the given tight
/// bounds (no move needed).
bool m3TreeContains(const m3Tree* tree, int32_t nodeId, const double lo[3], const double hi[3]);

/// Query every leaf overlapping the bounds, in deterministic stack
/// order; the callback returns false to stop early.
typedef bool (*m3TreeQueryFn)(int32_t userData, void* context);
void m3TreeQuery(const m3Tree* tree, const double lo[3], const double hi[3], m3TreeQueryFn fn,
                 void* context);

#endif // MAUL3D_DYNAMIC_TREE_H
