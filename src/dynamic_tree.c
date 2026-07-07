// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The classic dynamic tree, adapted from the reference family (Box2D
// v3 / Box3D dynamic_tree.c, MIT, Erin Catto): surface-area-heuristic
// descent for the best sibling, AVL rotations for balance. All bounds
// math is double and every branch is a strict comparison, so the tree
// shape is a pure function of the insertion history.

#include "dynamic_tree.h"

#include "allocator.h"

#include <string.h>

static double SurfaceArea(const double lo[3], const double hi[3])
{
    double dx = hi[0] - lo[0];
    double dy = hi[1] - lo[1];
    double dz = hi[2] - lo[2];
    return 2.0 * (dx * dy + dy * dz + dz * dx);
}

static void Union(const double aLo[3], const double aHi[3], const double bLo[3],
                  const double bHi[3], double outLo[3], double outHi[3])
{
    for (int32_t k = 0; k < 3; ++k)
    {
        outLo[k] = aLo[k] < bLo[k] ? aLo[k] : bLo[k];
        outHi[k] = aHi[k] > bHi[k] ? aHi[k] : bHi[k];
    }
}

static bool Overlap(const double aLo[3], const double aHi[3], const double bLo[3],
                    const double bHi[3])
{
    return aLo[0] <= bHi[0] && bLo[0] <= aHi[0] && aLo[1] <= bHi[1] && bLo[1] <= aHi[1] &&
           aLo[2] <= bHi[2] && bLo[2] <= aHi[2];
}

m3Tree m3TreeCreate(int32_t capacity)
{
    m3Tree tree;
    memset(&tree, 0, sizeof(tree));
    tree.capacity = capacity;
    tree.root = M3_TREE_NULL;
    M3_ALLOC(tree.nodes, capacity, m3TreeNode);
    for (int32_t i = 0; i < capacity; ++i)
    {
        tree.nodes[i].parent = i + 1 < capacity ? i + 1 : M3_TREE_NULL;
        tree.nodes[i].height = -1;
    }
    tree.freeList = capacity > 0 ? 0 : M3_TREE_NULL;
    return tree;
}

void m3TreeDestroy(m3Tree* tree)
{
    m3Free(tree->nodes);
    memset(tree, 0, sizeof(*tree));
    tree->root = M3_TREE_NULL;
    tree->freeList = M3_TREE_NULL;
}

static int32_t AllocateNode(m3Tree* tree)
{
    if (tree->freeList == M3_TREE_NULL)
    {
        return M3_TREE_NULL; // exhausted: loud at the caller
    }
    int32_t id = tree->freeList;
    m3TreeNode* node = tree->nodes + id;
    tree->freeList = node->parent;
    node->parent = M3_TREE_NULL;
    node->child1 = M3_TREE_NULL;
    node->child2 = M3_TREE_NULL;
    node->height = 0;
    node->userData = -1;
    node->pad = 0;
    return id;
}

static void FreeNode(m3Tree* tree, int32_t id)
{
    m3TreeNode* node = tree->nodes + id;
    memset(node, 0, sizeof(*node));
    node->parent = tree->freeList;
    node->height = -1;
    tree->freeList = id;
}

// The reference AVL balance: rotate the subtree at iA if it is two
// levels out of balance, returning the new subtree root.
static int32_t Balance(m3Tree* tree, int32_t iA)
{
    m3TreeNode* nodes = tree->nodes;
    m3TreeNode* A = nodes + iA;
    if (A->height < 2 || A->child1 == M3_TREE_NULL)
    {
        return iA;
    }
    int32_t iB = A->child1;
    int32_t iC = A->child2;
    m3TreeNode* B = nodes + iB;
    m3TreeNode* C = nodes + iC;
    int32_t balance = C->height - B->height;

    if (balance > 1)
    {
        // Rotate C up.
        int32_t iF = C->child1;
        int32_t iG = C->child2;
        m3TreeNode* F = nodes + iF;
        m3TreeNode* G = nodes + iG;
        C->child1 = iA;
        C->parent = A->parent;
        A->parent = iC;
        if (C->parent != M3_TREE_NULL)
        {
            if (nodes[C->parent].child1 == iA)
            {
                nodes[C->parent].child1 = iC;
            }
            else
            {
                nodes[C->parent].child2 = iC;
            }
        }
        else
        {
            tree->root = iC;
        }
        if (F->height > G->height)
        {
            C->child2 = iF;
            A->child2 = iG;
            G->parent = iA;
            Union(B->lo, B->hi, G->lo, G->hi, A->lo, A->hi);
            Union(A->lo, A->hi, F->lo, F->hi, C->lo, C->hi);
            A->height = 1 + (B->height > G->height ? B->height : G->height);
            C->height = 1 + (A->height > F->height ? A->height : F->height);
        }
        else
        {
            C->child2 = iG;
            A->child2 = iF;
            F->parent = iA;
            Union(B->lo, B->hi, F->lo, F->hi, A->lo, A->hi);
            Union(A->lo, A->hi, G->lo, G->hi, C->lo, C->hi);
            A->height = 1 + (B->height > F->height ? B->height : F->height);
            C->height = 1 + (A->height > G->height ? A->height : G->height);
        }
        return iC;
    }
    if (balance < -1)
    {
        // Rotate B up (mirror case).
        int32_t iD = B->child1;
        int32_t iE = B->child2;
        m3TreeNode* D = nodes + iD;
        m3TreeNode* E = nodes + iE;
        B->child1 = iA;
        B->parent = A->parent;
        A->parent = iB;
        if (B->parent != M3_TREE_NULL)
        {
            if (nodes[B->parent].child1 == iA)
            {
                nodes[B->parent].child1 = iB;
            }
            else
            {
                nodes[B->parent].child2 = iB;
            }
        }
        else
        {
            tree->root = iB;
        }
        if (D->height > E->height)
        {
            B->child2 = iD;
            A->child1 = iE;
            E->parent = iA;
            Union(C->lo, C->hi, E->lo, E->hi, A->lo, A->hi);
            Union(A->lo, A->hi, D->lo, D->hi, B->lo, B->hi);
            A->height = 1 + (C->height > E->height ? C->height : E->height);
            B->height = 1 + (A->height > D->height ? A->height : D->height);
        }
        else
        {
            B->child2 = iE;
            A->child1 = iD;
            D->parent = iA;
            Union(C->lo, C->hi, D->lo, D->hi, A->lo, A->hi);
            Union(A->lo, A->hi, E->lo, E->hi, B->lo, B->hi);
            A->height = 1 + (C->height > D->height ? C->height : D->height);
            B->height = 1 + (A->height > E->height ? A->height : E->height);
        }
        return iB;
    }
    return iA;
}

static void FixUpward(m3Tree* tree, int32_t index)
{
    m3TreeNode* nodes = tree->nodes;
    while (index != M3_TREE_NULL)
    {
        index = Balance(tree, index);
        m3TreeNode* node = nodes + index;
        int32_t c1 = node->child1;
        int32_t c2 = node->child2;
        node->height =
            1 + (nodes[c1].height > nodes[c2].height ? nodes[c1].height : nodes[c2].height);
        Union(nodes[c1].lo, nodes[c1].hi, nodes[c2].lo, nodes[c2].hi, node->lo, node->hi);
        index = node->parent;
    }
}

int32_t m3TreeInsert(m3Tree* tree, const double lo[3], const double hi[3], int32_t userData)
{
    int32_t leaf = AllocateNode(tree);
    if (leaf == M3_TREE_NULL)
    {
        return M3_TREE_NULL;
    }
    m3TreeNode* nodes = tree->nodes;
    memcpy(nodes[leaf].lo, lo, sizeof(double) * 3);
    memcpy(nodes[leaf].hi, hi, sizeof(double) * 3);
    nodes[leaf].userData = userData;
    nodes[leaf].height = 0;

    if (tree->root == M3_TREE_NULL)
    {
        tree->root = leaf;
        return leaf;
    }

    // Find the best sibling by the surface area heuristic (the
    // reference descent: strict comparisons, deterministic path).
    int32_t index = tree->root;
    while (nodes[index].height > 0)
    {
        int32_t child1 = nodes[index].child1;
        int32_t child2 = nodes[index].child2;

        double combinedLo[3];
        double combinedHi[3];
        Union(nodes[index].lo, nodes[index].hi, lo, hi, combinedLo, combinedHi);
        double combinedArea = SurfaceArea(combinedLo, combinedHi);
        double cost = 2.0 * combinedArea;
        double inheritance = 2.0 * (combinedArea - SurfaceArea(nodes[index].lo, nodes[index].hi));

        double cost1;
        Union(nodes[child1].lo, nodes[child1].hi, lo, hi, combinedLo, combinedHi);
        if (nodes[child1].height == 0)
        {
            cost1 = SurfaceArea(combinedLo, combinedHi) + inheritance;
        }
        else
        {
            cost1 = SurfaceArea(combinedLo, combinedHi) -
                    SurfaceArea(nodes[child1].lo, nodes[child1].hi) + inheritance;
        }

        double cost2;
        Union(nodes[child2].lo, nodes[child2].hi, lo, hi, combinedLo, combinedHi);
        if (nodes[child2].height == 0)
        {
            cost2 = SurfaceArea(combinedLo, combinedHi) + inheritance;
        }
        else
        {
            cost2 = SurfaceArea(combinedLo, combinedHi) -
                    SurfaceArea(nodes[child2].lo, nodes[child2].hi) + inheritance;
        }

        if (cost < cost1 && cost < cost2)
        {
            break;
        }
        index = cost1 < cost2 ? child1 : child2;
    }

    // Splice a new parent above the sibling.
    int32_t sibling = index;
    int32_t oldParent = nodes[sibling].parent;
    int32_t newParent = AllocateNode(tree);
    if (newParent == M3_TREE_NULL)
    {
        FreeNode(tree, leaf);
        return M3_TREE_NULL;
    }
    nodes[newParent].parent = oldParent;
    nodes[newParent].userData = -1;
    Union(lo, hi, nodes[sibling].lo, nodes[sibling].hi, nodes[newParent].lo, nodes[newParent].hi);
    nodes[newParent].height = nodes[sibling].height + 1;

    if (oldParent != M3_TREE_NULL)
    {
        if (nodes[oldParent].child1 == sibling)
        {
            nodes[oldParent].child1 = newParent;
        }
        else
        {
            nodes[oldParent].child2 = newParent;
        }
    }
    else
    {
        tree->root = newParent;
    }
    nodes[newParent].child1 = sibling;
    nodes[newParent].child2 = leaf;
    nodes[sibling].parent = newParent;
    nodes[leaf].parent = newParent;

    FixUpward(tree, newParent);
    return leaf;
}

void m3TreeRemove(m3Tree* tree, int32_t nodeId)
{
    m3TreeNode* nodes = tree->nodes;
    if (nodeId == tree->root)
    {
        tree->root = M3_TREE_NULL;
        FreeNode(tree, nodeId);
        return;
    }
    int32_t parent = nodes[nodeId].parent;
    int32_t grandParent = nodes[parent].parent;
    int32_t sibling = nodes[parent].child1 == nodeId ? nodes[parent].child2 : nodes[parent].child1;

    if (grandParent != M3_TREE_NULL)
    {
        if (nodes[grandParent].child1 == parent)
        {
            nodes[grandParent].child1 = sibling;
        }
        else
        {
            nodes[grandParent].child2 = sibling;
        }
        nodes[sibling].parent = grandParent;
        FreeNode(tree, parent);
        FixUpward(tree, grandParent);
    }
    else
    {
        tree->root = sibling;
        nodes[sibling].parent = M3_TREE_NULL;
        FreeNode(tree, parent);
    }
    FreeNode(tree, nodeId);
}

bool m3TreeContains(const m3Tree* tree, int32_t nodeId, const double lo[3], const double hi[3])
{
    const m3TreeNode* node = tree->nodes + nodeId;
    return node->lo[0] <= lo[0] && node->lo[1] <= lo[1] && node->lo[2] <= lo[2] &&
           hi[0] <= node->hi[0] && hi[1] <= node->hi[1] && hi[2] <= node->hi[2];
}

void m3TreeQuery(const m3Tree* tree, const double lo[3], const double hi[3], m3TreeQueryFn fn,
                 void* context)
{
    // An explicit stack keeps traversal order a pure function of the
    // tree shape (deterministic, no recursion depth hazards).
    int32_t stack[64];
    int32_t top = 0;
    if (tree->root != M3_TREE_NULL)
    {
        stack[top++] = tree->root;
    }
    while (top > 0)
    {
        int32_t id = stack[--top];
        const m3TreeNode* node = tree->nodes + id;
        if (!Overlap(node->lo, node->hi, lo, hi))
        {
            continue;
        }
        if (node->height == 0)
        {
            if (!fn(node->userData, context))
            {
                return;
            }
        }
        else
        {
            // Children pushed child2 first so child1 pops first: one
            // fixed order.
            if (top + 2 <= 64)
            {
                stack[top++] = node->child2;
                stack[top++] = node->child1;
            }
        }
    }
}
