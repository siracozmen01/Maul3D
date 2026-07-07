# Maul3D

A 3D rigid body physics engine for sandbox games. Written in C17 with a
plain C API. MIT licensed. Zero dependencies.

Sibling of [Maul2D](https://github.com/siracozmen01/Maul2D), which
validated the solver core and the determinism discipline this engine is
built on.

## Status

Phase 2a, "the standing spine": the deterministic core, the snapshot and
rollback machinery, and the simplest collision path are being built
first, gate by gate. The full 3D narrowphase lands on top of a proven
spine, not the other way around.

## The determinism promise

Same inputs, same bits, everywhere. The simulation is bit-exact across
platforms, compilers, and worker counts, and the world can be
snapshotted, rolled back, and resumed bit-exactly. Every commit must
hold the determinism gates in CI: hash lines compared across every cell,
and from task 10 of the current plan, record and replay equality, a
snapshot round-trip that resimulates to the identical hash, and
worker-count twins. Fast-math is refused at configure time.

## Build

```
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build
```

`-DMAUL3D_SIMD=scalar` forces the scalar backend, which must match the
vector backends bit for bit.
