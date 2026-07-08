# Changelog

All notable changes to Maul3D. Versions are tags on green CI SHAs;
every entry's determinism gates were verified across all CI cells at
tag time.

## v0.9 (2026-07-08) release candidate

The hardening phase (2d). No new simulation features; everything
that moves is proven, and the API surface is a 1.0 candidate. The
1.0 tag itself waits for the first destruction-phase consumer to
prove the API in anger.

- Hostile-input wall: every def field and every geometry family
  refuses NaN, infinities, and degenerate forms at the public
  boundary, before journaling. Hostile commands are documented
  no-ops. Twin-hash equality on all refusal paths.
- Hostile sequences: journal replay is ATOMIC (a refused replay
  leaves the target world byte-identical to its pre-call state).
  User-input asserts removed from the API border (contract, not
  invariant). Id lifetime rule documented, including world-slot
  recycling. Generation retirement at 0xFFFF proven by churn.
- Coverage measured: 93 percent of src lines under the suite; the
  campaign flushed out a real bug (plane shape casts stopped one
  radius short; fixed, pinned by analytic fractions).
- Portable package: CMake install and export (maul3d::maul3d),
  pkg-config, SONAME discipline, a working shared build with a real
  export macro, and a consumer project that builds against the
  INSTALLED package as a CI gate on every cell.
- MANUAL.md: the full engine-host contract, engineer to engineer.
- The soak: 20100 steps with per-segment journal replay and rolling
  rollback under ASan and UBSan; zero net allocation in steady
  state, enforced by the engine's own allocator counters.
- Seventeen test suites; seven CI cells; the four determinism gates
  and the cross-cell hash equality unchanged since phase 2a.

## v0.3 (2026-07-08)

Joints, queries, events, and the profiled midphase (phase 2c).

- Joints: spherical (optional shoulder cone and twist limits),
  revolute (limits, motor), prismatic (limits, motor); warm starts
  ride the snapshot; journaled with id-verified replay; both
  hand-derived Jacobians pinned by central finite-difference gates.
- Queries: closest and sorted all-hits rays, sphere and capsule
  shape casts, point-inside, AABB and sphere overlaps, with
  documented contracts (front-face rays, start-inside miss,
  fraction-zero casts, open meshes, solid half-spaces).
- Sensors: detect and never respond; own event streams; no waking,
  no stopping power, no sensor-sensor.
- Debug draw: a pure observer, held by a bytewise no-mutation test.
- Per-mesh static BVH midphase (meshfield 2.7x) and SAT fast paths
  (hulljam 16 percent), both behind unchanged golden and bench
  hashes: speed from pruning, never from changing answers.
- The benchmark harness with pinned scene hashes.

## v0.2 (2026-07-07)

The full narrowphase (phase 2b).

- Shapes: spheres, boxes, capsules, QuickHull convex hulls, static
  triangle meshes with ghost-edge welding, heightfield chunks,
  native infinite planes.
- Collision: SAT hull pairs with Gauss-map edge pruning, GJK with
  warm simplex caching, exact deep-overlap recovery, continuous
  collision for fast bodies and bullets, mesh TOI included.
- Islands with per-island sleep (sleeping worlds are bit-frozen),
  contact begin and end events, closest-hit rays, a host-owned task
  interface with graph-colored solver batches.

## v0.1 (2026-07-07, untagged)

The standing spine (phase 2a): deterministic core math, the three
arenas, snapshot and rollback and journal machinery, sphere and
plane narrowphase, the Soft Step solver in 3D, and the four
determinism CI gates that have run on every commit since.
