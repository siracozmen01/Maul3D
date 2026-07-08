# Maul3D

A deterministic 3D rigid body physics engine for sandbox and
destruction games. Written in C17 with a plain C API. MIT licensed.
Zero dependencies.

Sibling of [Maul2D](https://github.com/siracozmen01/Maul2D), which
validated the solver core and the determinism discipline this engine
is built on.

## Status: v0.9, the 1.0 release candidate

Phases 2b, 2c, and the 2d hardening pass are complete: red-team
rounds, 93 percent measured coverage, an installable package with a
consumer gate, a manual, and a twenty-thousand-step soak under
sanitizers. The API is a 1.0 candidate; the 1.0 tag waits for the
destruction phase's first consumer. The engine simulates:

- **Shapes**: spheres, boxes, capsules, convex hulls from point
  clouds (built-in QuickHull with coplanar face merging and exact
  mass integration), static triangle meshes with internal-edge ghost
  filtering and baked edge convexity, heightfield chunks, and native
  infinite planes.
- **Bodies**: static, kinematic (commanded velocity, immovable by
  contact), and dynamic, with full inertia tensors, offset centers of
  mass, and the implicit gyroscopic term (long skinny bodies tumble
  correctly and never gain energy).
- **Solver**: the Soft Step scheme (soft constraints, speculative
  contacts, relax pass, restitution pass, Coulomb disc friction) on
  up to four-point manifolds, solved in graph-color order so hosts
  can parallelize inside a color without moving a single bit.
- **Collision**: SAT hull pairs with Gauss-map edge pruning, GJK
  distance with warm-started simplex caching, exact deep-overlap
  recovery (no EPA needed by construction), and real continuous
  collision: fast bodies sweep against statics, bullets sweep against
  dynamics and kinematics with both sweeps in the time-of-impact
  kernel, meshes included.
- **Joints**: spherical (with an optional shoulder cone and twist
  limits), revolute (limits and a motor), and prismatic (limits and
  a motor), all warm-started through the snapshot, journaled with
  id-verified replay, and solved before contacts inside every
  substep. The two hand-derived Jacobians are pinned by central
  finite-difference gates.
- **Queries**: closest and all-hits ray casts (sorted, capacity
  bounded), sphere and capsule shape casts through the shared
  time-of-impact kernel, point-inside tests, AABB and sphere
  overlaps. Start-overlapped casts report fraction zero; rays are
  front-face-only by contract.
- **Sensors**: shapes that detect and never respond. Their begin and
  end events ride the same canonical stream walk as contacts; they
  wake nobody, stop nothing, and never sense each other.
- **Debug draw**: a pure observer interface (segments and points
  through host callbacks, fixed palette). A draw pass between two
  snapshots leaves every byte identical, and the test suite holds
  that promise on every commit.
- **World machinery**: fat-AABB dynamic tree broadphase with a
  brute-force referee, a per-mesh static BVH midphase (derived data,
  rebuilt on restore, referee kept), islands with per-island sleep
  (sleeping worlds are bit-frozen), contact begin/end events, and a
  host-owned task interface (the library never spawns threads;
  worker count never changes results).

## The determinism promise

Same inputs, same bits, everywhere. Every commit must hold four gates
in CI, across three OSes, two ISAs, and four compilers, and the
printed state hashes must match across every cell:

1. **The golden scene.** A fixed pyramid scene plus droppers; every
   cell must produce the identical state hash on every platform,
   compiler, and SIMD backend.
2. **Replay.** The same run twice is bit-identical, and a journaled
   session (creates plus commands plus steps) replays bit for bit
   into a fresh world, minted ids included.
3. **Rollback.** Snapshot mid-flight, run on, restore, rerun: the
   resimulated timeline is bit-identical, and a changed continuation
   diverges (restore is total, resume is real simulation). The
   snapshot format is portable and versioned, never a raw memory
   image; SIMD width and worker count are deliberately not part of
   the format.
4. **Worker twins.** One worker or four, a real thread pool or none,
   the hash never moves.

Every capacity refusal is loud (a null id or a false return, never a
crash, never a silent no-op), and the failure paths are themselves
deterministic. Fast-math is refused at configure time.

## Quick start

```c
#include "maul3d/shape.h"

m3WorldDef wd = m3DefaultWorldDef();
m3WorldId world = m3CreateWorld(&wd);

m3BodyDef ground = m3DefaultBodyDef();
m3BodyId floor = m3CreateBody(world, &ground);
m3ShapeDef sd = m3DefaultShapeDef();
m3Plane plane = {{0.0f, 1.0f, 0.0f}, 0.0f};
m3CreatePlaneShape(floor, &sd, &plane);

m3BodyDef bd = m3DefaultBodyDef();
bd.type = m3_dynamicBody;
bd.position = (m3Pos3){0.0, 5.0, 0.0};
m3BodyId body = m3CreateBody(world, &bd);
m3CreateBoxShape(body, &sd, (m3Vec3){0.5f, 0.5f, 0.5f});

for (int i = 0; i < 300; ++i)
{
    m3World_Step(world, 1.0f / 60.0f, 4);
}

m3Pos3 p = m3Body_GetPosition(body); // resting at y = 0.5
m3DestroyWorld(world);
```

Snapshot and rollback:

```c
int32_t size = m3World_SnapshotSize(world);
void* buffer = malloc(size);
m3World_Snapshot(world, buffer, size); // save the exact world
// ... run steps, mispredict, whatever ...
m3World_Restore(world, buffer, size);  // bit-exact rewind
```

## Build

```
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build
```

No dependencies. `-DMAUL3D_SIMD=scalar` forces the scalar backend,
which must match the vector backends bit for bit; `-DMAUL3D_SANITIZE=ON`
adds ASan and UBSan.

## Performance

The benchmark harness ships in `bench/` and is never a test gate
(wall time is not deterministic; the printed state hashes are). On
one mid-range x86-64 core, Release, 300 steps:

| scene     | bodies                      | ms/step |
| --------- | --------------------------- | ------- |
| pyramid   | 91 spheres on a plane       | 0.12    |
| hulljam   | 80 hulls jammed in a pit    | 0.53    |
| meshfield | rain on a 1922-triangle field | 0.38  |

## Roadmap

- Phase 3: destruction and voxels, the reason this engine exists.
  Two-tier voxel storage, deterministic fracture as voxel-state
  transitions inside the rollback delta, fragment hulls from the
  built-in QuickHull.

## License

MIT. See LICENSE.
