# Maul3D

A 3D rigid body physics engine for sandbox games. Written in C17 with a
plain C API. MIT licensed. Zero dependencies.

Sibling of [Maul2D](https://github.com/siracozmen01/Maul2D), which
validated the solver core and the determinism discipline this engine is
built on.

## Status

Phase 2a, "the standing spine," is complete: spheres and planes
simulate under the Soft Step solver, a five-sphere stack stands, and
the four determinism gates below hold on every commit. Phase 2b (the
full 3D narrowphase: hulls, capsules, boxes, meshes, real CCD) builds
on this proven spine.

## The determinism promise

Same inputs, same bits, everywhere. Every commit must hold four gates
in CI, across three OSes, two ISAs, and four compilers:

1. **The golden scene.** A 30-sphere pyramid plus droppers, 300 steps;
   every cell must produce the identical state hash.
2. **Replay.** The same run twice is bit-identical, and a journaled
   session (creates plus steps) replays bit for bit into a fresh world.
3. **Rollback.** Snapshot mid-flight, run on, restore, rerun: the
   resimulated timeline is bit-identical, and a changed continuation
   diverges (restore is total, resume is real simulation). The snapshot
   format is portable and versioned, never a raw memory image.
4. **Worker twins.** Different worker counts produce identical bits.

Fast-math is refused at configure time.

## Build

```
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build
```

`-DMAUL3D_SIMD=scalar` forces the scalar backend, which must match the
vector backends bit for bit.
