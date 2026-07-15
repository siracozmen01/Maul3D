# Maul3D

[![ci](https://github.com/siracozmen01/Maul3D/actions/workflows/ci.yml/badge.svg)](https://github.com/siracozmen01/Maul3D/actions/workflows/ci.yml)

A deterministic 3D physics engine for games. Written in C17 with a
pure C API, zero dependencies, MIT licensed. The 3D sibling of
[Maul2D](https://github.com/siracozmen01/Maul2D), built on the same
constitution: the same inputs produce the same bits on every
supported platform, and the engine is designed around that promise
end to end.

Maul3D's difference is a contract, not a feature flag. Snapshots
restore worlds bit-exactly and resume identically (the rollback
most engines explicitly do not promise), a command journal records
sessions and replays them byte for byte with id verification and
atomic backout, and CI fails on a single differing bit across
seven platform cells spanning x64, arm64, MSVC, sanitizers and
portable scalar. Rollback netcode, deterministic lockstep,
kill-cam replays and server-verified simulation stop being
research projects and become four lines of code:

```c
int32_t size = m3World_SnapshotSize(world);
m3World_Snapshot(world, buffer, size);
/* ... mispredicted steps ... */
m3World_Restore(world, buffer, size); // bit-exact resimulation from here
```

## What is in the box

- **Rigid bodies**: spheres, capsules, cylinders (honest faceted
  prisms through the hull path), convex hulls (64 vertices),
  triangle meshes (65k triangles with a BVH), heightfields, native
  infinite planes, and compound children with local transforms;
  static, kinematic, dynamic; forces, impulses, motion locks,
  kinematic targets, runtime type switching, per-shape materials
  with conveyor surface velocities, gusting wind fields.
- **Voxel destruction inside the rollback contract**: chunked
  voxel shapes with merged-box collision, journaled carving,
  fracture events with island recipes for host-spawned fragments,
  voxel CCD, and seam welding; carve the floor and the character,
  the car, and the rope all react the same step.
- **Explosions in one call**: m3World_Explode pushes every body
  by the area it shows to the blast (falloff band, wake,
  implosions), carves voxel chunks, and shoves soft particles,
  all journaled and bit-exact under rollback.
- **A soft-step solver** adapted from the Box2D v3 lineage:
  speculative contacts, warm starting, graph coloring, sub-steps;
  hybrid f64 positions keep worlds exact far from the origin.
- **Seven joint types**: spherical (with cone and twist limits),
  revolute, prismatic, fixed, distance, a 6-DOF generic, and a
  composed wheel joint (suspension slide plus free spin); limits,
  motors, springs with position targets, constraint force
  readback, and built-in BREAKING as a deterministic in-step
  state transition with events.
- **Vehicles two ways**: an arcade raycast car with suspension,
  tire friction circles, and a full drivetrain (pinned torque
  curve, gearbox with deterministic auto shift, clutch), or rigid
  wheel-joint carts for trucks over rubble.
- **A character controller**: collide-and-slide capsule with step
  climbing, slope limits, ground snapping, moving-platform
  carrying, dt-free pushing, and crouch/resize guarded by a
  stand-up overlap veto.
- **Soft bodies**: XPBD particle lattices (boxes, ropes, cloth
  sheets) colliding with the rigid world and with EACH OTHER,
  anchored to bodies or across lattices, driven by wind; all
  inside the same snapshot, journal and hash contract.
- **The replay studio**: the M3J1 container seals a snapshot plus
  journal plus final hash into one artifact; the `m3replay` CLI
  records, verifies, seeks, plays and DIFFS two runs to the exact
  divergence frame; `m3lockstep` demonstrates two processes
  staying bit-identical over exchanged inputs.
- **Events and queries**: contact begin/end, sensors, hit events
  with speed thresholds, body move events, joint break events,
  pre-solve vetoes; rays, sphere/capsule/hull/box casts, overlap
  queries, all filterable by category/mask/group and canonically
  ordered.
- **Continuous collision** for bullets, island-based sleeping,
  world tuning knobs, debug draw (wireframe and solid triangle
  streams, both held by a draw-purity test).

## Quick start

```
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build
```

No dependencies. A C17 compiler is required. The default build
uses SIMD kernels with a portable scalar fallback
(`-DMAUL3D_SIMD=scalar`) that produces bit-identical results, and
CI enforces that equality on every commit.

The interactive testbed (raylib, viewer only, outside the engine's
dependency surface) builds with `-DMAUL3D_BUILD_TESTBED=ON`:
fourteen scenes across benchmarks, voxel destruction (including a
blastyard where one keypress detonates, carves the wall, and kicks
the fragments), three kinds of vehicle, a playable character with
the crouch veto, cloth in gusting wind over a conveyor, and a
rolling heightfield; solid
shading with shadows, a solver panel, and a Recording panel that
seals the live session into an M3J1 replay and verifies it
bit-exact on a button. Hold R and time runs backward through a
snapshot ring, bit for bit.

## Determinism and performance

Determinism here means pinned IEEE arithmetic (no fast math, no FP
contraction), fixed tessellations and canonical ordering on every
path, journaled defs treated as untrusted bytes on replay, and a
golden world hash that has moved exactly twice in the engine's
life, both times argued on the record. Every commit runs 51 test
suites in three build flavors plus a shared-library cell, ASAN and
UBSAN, cross-platform hash equality across all cells, and seven
pinned benchmark hashes that must not move: a 5000-body city block
with destruction, a 10k-body smoke test with twin-run matching,
mesh and hull rain fields, and the classic pyramid.

## Status and stability

Current release: 1.17. The 1.x API surface is frozen: functions and
defs may be added in minor releases, but existing signatures,
semantics and id layouts do not change until a 2.0. Defs are
cookie-guarded, so a stale compiled caller fails loudly instead of
subtly. Snapshots and journal tapes are versioned artifacts of a
single library version and refuse loudly across versions.

## Learn more

- [The manual](docs/manual.md): the engine-host contract, chapter
  by chapter: rollback, the journal, destruction, vehicles, the
  character, soft bodies, the replay studio, lockstep networking,
  and the integration checklist.
- [The changelog](docs/changelog.md): every release with its
  determinism ledger, including the convictions the road collected
  and what they cost.
- [CONTRIBUTING.md](CONTRIBUTING.md) for contributions.
- [THIRD_PARTY.md](THIRD_PARTY.md) for adapted-code licenses.
- [Maul2D](https://github.com/siracozmen01/Maul2D): the 2D sibling,
  same constitution, with particle fluids and a browser-playable
  testbed.

## Acknowledgments

Maul3D stands on the shoulders of [Box2D and Box3D](https://github.com/erincatto)
by Erin Catto (MIT): the soft-step solver structure, joint
formulations, and several collision kernels are adapted from that
lineage, with adaptations noted in the sources and licensed in
[THIRD_PARTY.md](THIRD_PARTY.md). The determinism architecture
(hybrid f64 positions, snapshot rollback, the command journal,
config-hash refusal, cross-platform hash gating) and the
voxel-destruction-inside-rollback design are Maul's own, shared
with [Maul2D](https://github.com/siracozmen01/Maul2D).
