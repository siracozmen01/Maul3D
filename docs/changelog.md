# Changelog

All notable changes to Maul3D. Versions are tags on green CI SHAs;
every entry's determinism gates were verified across all CI cells at
tag time.

## Unreleased (v1.14 in tree)

Memory engineering and painted floors.

- Count-derived hull snapshots (v46): the 5808-byte fixed hull
  slabs left the snapshot's fixed prefix; an empty slot now costs
  16 bytes and a box hull its used prefix. The 5000-body rollback
  benchmark's snapshot shrank 65.9 MB to 19.7 MB and the snapshot
  call 84 ms to 20 ms. The m3replay scrubber
  learned per-keyframe snapshot sizes (count-derived snapshots
  grow with the world).
- Painted mesh materials (op 72, snapshot v47):
  m3Shape_SetMeshMaterials paints up to 8 surface materials and a
  group byte per triangle; the struck triangle's entry replaces
  the mesh side of the contact mix and runs a per-triangle
  conveyor through the tangential target. Group bits ride the
  manifold point flags, so unpainted worlds hash exactly as
  before.
- Quantized mesh BVH: nodes shrank 36 to 20 bytes (uint16 grid
  bounds against the root box, doubly-outward rounding, float
  early-out for far queries). Bit-identical results, proven by
  the pinned bench hashes; meshfield stepped 17 percent faster.
- m3World_RebuildBroadphase (op 73): a deterministic, journaled,
  balanced top-down rebuild of the broadphase after bulk level
  construction. Changes no answer (hash-equality proven in the
  suite); queries just walk a balanced tree.
- The moving-platform carry check rode along: a crate on a
  gliding kinematic platform rides by friction (test-first, no
  code needed).
- Suites 49 (world grew the hull-shrink gate, rebuild, platform
  carry, painted floors) and the phase 17 mutation storm (300
  journal-aimed bit flips over a paint + rebuild dense session).

## v1.13 (2026-07-15)

Machines: drives that budget honestly, wheels that steer, diffs
that lock, and five new joint types.

- Filter joint (type 7): the connected-pair collision filter with
  zero solver rows, BY LAW. Parallel joint (type 8): two angular
  rows keep two body axes parallel; translations and the shared
  twist stay free.
- The four-state drive law: a joint's velocity motor and position
  spring now share ONE budget (the motor's maxMotorEffort), so a
  starved drive sags honestly instead of borrowing force.
- Wheel steering (op 70): m3Joint_SetSteer yaws a wheel joint
  about its strut toward a target angle, soft and budgeted;
  m3Joint_GetSteerAngle reads it back. Steerable carts on real
  contacts.
- Drivetrain differentials: diffMode open/limited/locked plus
  diffCouple (host-tuned to mass scale) squeeze the torque split
  toward the driven mean through one step of wheel-speed lag,
  inside the friction circle. Snapshot v44.
- Motor joint (type 9, op 71): the servo weld. Soft 3-DOF
  rotation + translation drives toward a commanded pose
  (m3Joint_SetMotorPose), stiffness from SetSpring, independent
  force/torque budgets from SetLimits, idles free without a
  spring, aims at its create pose by default.
- Gear joint (type 10): spinA + ratio * spinB holds its create
  value across two hinged bodies (positive ratio = external
  mesh), drift-corrected softly, under a documented common-frame
  mounting contract. Pulley joint (type 11): length1 + ratio *
  length2 holds across two world anchors, rigid both ways. Def
  tail grew ratio + groundAnchorA/B; snapshot v45.
- The 16-7 wall sweep: every def-carrying create internal (body,
  shape, character, joint, and the earlier soft body and vehicle)
  now validates raw replay bytes itself, a flipped joint-type
  byte refuses instead of minting an unowned joint, and every
  scalar/vector setter decode refuses non-finite payloads. Plus
  the phase 16 mutation storm (300 journal-aimed bit flips over
  a steering, servo, gear, and pulley dense session).
- Suites 47 (newjoints grew the servo), gearpulley, and the
  storm; testbed: A/D steering on the joint cart, a 2:1 gear
  mesh, a pulley trade, and a servo-held plate in the machine
  room.

## v1.12 (2026-07-15)

The modeling gaps: cylinders, live reshaping, and richer queries.

- The honest cylinder: m3CreateCylinderShape mints a 2N-vertex
  prism through the interned hull path (segments clamp 3..32).
  Exact hull math everywhere downstream; the N-gon side is the
  documented trade, and the analytic round cylinder stays on the
  ledger until a consumer needs it.
- Runtime geometry replacement (op 69): m3Shape_SetSphere and
  m3Shape_SetCapsule, conversions included; interned families
  refuse. Mass follows the volume, sleepers around both the old
  and new bounds wake, refused swaps journal nothing.
- The overlap family: m3World_OverlapCapsule, oriented
  m3World_OverlapBox, m3World_OverlapHullPoints (base + up to 64
  relative points + radius). Exact per family (GJK, min plane
  distance, BVH-gathered voxel boxes and mesh triangles),
  ascending ids, filtered, pure.
- Suites 44 (cylinder), 45 (reshape), 46 (overlapfamily) and the
  phase 15 mutation storm (300 journal-aimed bit flips over an
  op-69-dense session with a rolling prism).

## v1.11 (2026-07-15)

The adoption gate: the engine explains itself.

- Introspection (14-1): m3Counters (live census, tree height,
  scratch high water, snapshot size, allocation calls) and
  m3Profile (wall-clock per step phase), both pure observers
  with a twin-polling purity gate. The step now carries an
  island census and a solver color count.
- Extra draw layers (14-2): m3World_DrawExtras with island tint
  points, center-of-mass axis frames, and internal broadphase
  tree boxes; a separate additive struct, the frozen draw
  structs untouched, the purity law extended over the new walk.
- Debug names, contact readback, and the assert hook (14-3):
  m3Body_SetName (op 68, journaled, snapshot-carried, never
  hashed), m3Body_GetContactData / m3Shape_GetContactData in
  canonical pair order, m3SetAssertHandler ahead of the debug
  abort. No log hook on purpose: there is no log stream.
- Suite 42 (introspection) and the extras gates in the draw
  suite; the testbed grew a Stats panel and toggles for the new
  layers. Snapshot v43 additionally carries body names.

## v1.10 (2026-07-15)

The explosion release, opening Roadmap 4: one call demolishes,
and the spin clamp the reference always paired with an escape
hatch finally ships with one.

- Explosions (op 67): m3World_Explode with m3ExplosionDef.
  Impulse per facing area (the reference projected-area model
  over spheres, capsules, and hull face fans), recentered GJK
  closest points, lever-arm spin, linear falloff band, wake on
  reach, query filter, negative impulse implodes. Hostile defs
  bounce at the cookie and at the replay wall.
- Voxel charge coupling: voxelCarve bites a sphere out of every
  chunk in range through one surface rebuild and one fracture
  sweep per chunk; freed islands arrive as fragment events. The
  testbed blastyard scene detonates at the crosshair and kicks
  newborn fragments radially.
- Soft push: softPush lands the same falloff push on soft
  particles as pending kicks integrated exactly once by the next
  step; the blast-to-step window is real snapshot state, hashed
  only while it exists (snapshot v43 with the angular cap).
- Angular speed cap (ops 65/66): the reference spin clamp with
  m3Body_SetAllowFastRotation as the per-body escape hatch
  (bodyLocks bit 6, free of the snapshot). The default 800 rad/s
  is a catastrophe guard like the linear 400 m/s: pinned scenes
  keep their bits, the knob folds into the hash only
  off-default, in its own block.
- Suites 40 and 41 (angularcap, explosion) plus the phase 13
  mutation storm in the container gate: 300 journal-aimed
  mutations over a session dense in ops 65, 66, and 67.
- A red-team scar with a cure: the phase 13 storm flipped one
  bit in a journaled step op and bought a BILLION substeps (the
  replay wall never bounded the count; a two-second replay
  became hours of honest math). Substeps now cap at 256 at the
  public wall and the replay wall alike; hostile tapes refuse
  loudly instead of running forever.
- The storm's second bite, caught by the UBSAN cell after tag
  content froze: replay handed raw soft defs to an internal that
  trusted its caller, and a flipped bit could mint NaN particle
  positions (a NaN voxel cell index is undefined behavior). The
  soft def wall moved into the internal where BOTH doors pass,
  and the 8-4 speed cap now covers particles in the integrator.
- Portability cure: MSVC C4310 in the fast-rotation flag clear
  (a constant truncation); mask the live byte, cast the
  expression, same bits everywhere.
- The testbed grew up: thirteen categorized scenes (benchmarks,
  destruction, geared and wheel-joint vehicles, the crouch veto,
  cloth in gusting wind over a conveyor, a rolling heightfield),
  solid shading under one sun with planar shadows on a
  sky-and-ground stage, a side panel (solver, draw toggles), a
  scene browser, and a Recording panel that seals the session
  into an M3J1 container and verifies it bit-exact on demand.
- New additive API: m3World_DrawSolid emits every shape as filled
  world-space triangles for lighting viewers (a separate struct
  and entry point; the original m3DebugDraw layout stays frozen
  ABI). Held by the same draw-purity test as the wireframe walk.

## v1.9 (2026-07-09)

Drivetrain and stance: the last phase of Roadmap 3. The vehicle
grows a real engine, wheels earn an optional joint body, and the
character learns to crouch.

- Drivetrain (op 62/63): pinned torque curve with deterministic
  linear interpolation, 1..6 gears plus reverse, final drive,
  pinned shift RPMs, clutch in steps; auto shift never mid
  clutch; gear, countdown, and tachometer are snapshot state
  hashed only when attached. Snapshot v42.
- The wheel joint (m3_wheelJoint): one frame composing the
  revolute's axle half (collinearity pair, spin motor, angle
  read) with the prismatic's suspension half (point-to-line
  pair, travel limits, translation spring). The def grew
  nothing; the axle snaps perpendicular at create or refuses;
  8-6 breakage applies to axles unchanged.
- Stance (op 64): m3Character_SetStance, feet anchored, shrink
  always, grow through the stand-up veto cast through the move
  core itself (fraction-zero initial overlap PROBED before being
  built on); vetoed changes journal nothing.
- The red team: fuzz aimed point-blank at ops 62/63/64, a shift
  thrash storm against a rollback loop, an axle cascade that
  degenerates a car into a sled on identical bits, and a crouch
  spammer under a moving press whose grant pattern twins
  exactly.
- MANUAL: the drivetrain, the wheel joint, and stance chapters.
- One process scar on the ledger: the 12-1 push skipped the
  local format pass and the format cell caught it (every physics
  cell was green). The local gate ritual now includes
  clang-format before every push.

## v1.8 (2026-07-09)

The admitted gaps, closed: the three capabilities the
competitors' own documentation concedes, all under the bit
contract.

- Soft-vs-soft collision: canonical particle pairs between
  lattices, XPBD split projection, zero new snapshot state.
- Soft-to-soft anchors: particle-to-particle pins across
  lattices, lower-slot canonical ownership, silent two-way
  liveness release; a rope bridges two cloths and carries a ball.
- Wind: gusts from an ACCUMULATED phase in the snapshot, so
  rollbacks resume the same wave; no host randomness anywhere.
- Conveyors: surface velocities feed the reference
  tangentVelocity slots; belts drive riders, dynamic decks
  recoil by Newton.
- The road paid twice, both on the ledger: V-LAYOUT (the scratch
  pre-flight's one-step pair-count lag made silent stall ticks
  sizeof-dependent; fixed, cityblock and smoke10k re-pinned as a
  correctness fix) and the minus-zero relabeling from zero belt
  targets (benign, deterministic, pad-probe proven; one argued
  re-pin).
- MANUAL: soft interactions and fields, including the honest
  porosity bound of particle sheets.
- Snapshot v40 and v41; ops 59 through 61; suites 33 and 34.

## v1.7 (2026-07-09)

Geometry scale: the ceilings the July deep dive flagged, raised
without moving a single default bit.

- Compound shapes: every shape takes an optional local transform
  (position + rotation) behind ONE composed-read gate with an
  identity short-circuit; mass composes per shape (c' = p + Rc,
  I' = R I R^T); queries, rays, contacts, soft bodies, and debug
  draw all see true placement. Offsets ride the existing create
  ops and hash off-identity only.
- Hull vertices to 64 (faces 124, half edges 372 by Euler; input
  clouds to 256); index types widened where a byte ran out.
- Mesh chunks to 65,535 triangles: content went count-derived
  (exact heap arrays behind ownership gates), the snapshot gained
  its first variable-size section with atomic pre-validation, and
  a 59,858-triangle terrain is proven under traffic, rays, and
  bit-identical rollback. The meshgrand bench prices the ceiling.
- m3DestroyShape: journaled single-shape destruction with mass
  rebuild (the red team found bodies could shed shapes only by
  dying).
- Red team: compound storms under mid-flight snapshots (and the
  rollback lesson applied to the test itself: host handles are
  game state), hull boundary clouds at 63/64/65 with degenerate
  stews, LeakSanitizer-clean ownership across every embedded
  derived tree.
- Golden and every bench pin stood through the whole phase;
  snapshot formats v37 through v39.

## v1.6 (2026-07-08)

The replay studio: the moat made visible. Nobody else can ship a
bit-exact scrubber, because nobody else has the bit contract.

- The M3J1 container (include/maul3d/replay.h): a pure-memory
  codec for [initial snapshot + journal] sessions carrying step
  and op counts and the recorder's final hash. The library never
  touches files; refusals are loud and touch nothing.
- tools/m3replay: record, info, verify (MATCH or DIVERGED with a
  nonzero exit), seek and play through a 30-step keyframe
  scrubber that PROVES every landing against a straight-run
  prefix hash, and diff: a prefix-hash binary search that names
  the first divergent step and the worst bodies at it, backed by
  m3World_DiffReport (a pure debug read, worst-first).
- tools/m3lockstep: rollback networking as executable
  documentation; an online peer absorbs hundreds of rollbacks
  against an offline truth peer and every checkpoint hash
  matches. MANUAL gained the networking chapter.
- CI now runs the scrubber selftest, the divergence difftest, and
  the lockstep sample in every cell on every commit; the fuzz law
  batters the container with 200 bit-mutations under sanitizers.
- Zero engine-step changes in the whole phase: golden and every
  rev-21 bench pin stood from v1.5 to v1.6.

## v1.5 (2026-07-08)

The table-stakes release: everything an engine evaluator checks
first, all of it journaled, snapshotted, and bit-stable.

- Collision filters: category/mask/group on every shape, one
  chokepoint for pairs, sensors, and the continuous phase;
  filtered variants of every ray, cast, and overlap query.
- Forces and impulses: the six-op family with one-step force
  accumulators (snapshot state) and analytic books.
- Runtime body control: teleport with two-neighborhood wake, the
  kinematic servo, type flips with mass rebuild, enable/disable
  ghosting everywhere, per-axis motion locks, per-body sleep
  controls.
- Central friction (solver rev 21, the second argued golden
  move): the reference 3D layout, one coupled tangent row per
  manifold at the mean anchors plus a twist row, pass-local
  Coulomb budgets, rolling resistance in the manifold warm start.
  Convicted by the slide-distance law, proven against Maul2D and
  the reference; the law test now pins v*v/(2*mu*g) for good.
- Materials and world tuning: runtime friction/restitution/
  rolling/density setters (density with mass rebuild), gravity,
  contact tuning, restitution threshold, a hard linear speed cap,
  sleep and continuous toggles. Knobs are state, not config.
- Events: opt-in hit events at the analytic impact speed, body
  move events with the fell-asleep edge, the joint break stream,
  and the pre-solve veto under a loud purity contract.
- Joint runtime control: limits, motors, collide-connected, break
  thresholds (a deliberate addition over the reference: breakage
  is a deterministic in-step transition, not a host poll),
  reaction readback, and position drives: the three reference
  spring rows toward runtime targets.
- Snapshot v29 through v36; journal ops 26 through 57; 30 suites;
  the shared cell runs 20. Golden moved once, argued, with rev 21.

## v1.4 (2026-07-08)

The soft body release, and the close of the ratified roadmap.

- Deterministic XPBD particle lattices (`m3CreateSoftBody`): rope,
  cloth, and jelly from one box-lattice factory; structural and
  face-diagonal rods with compliance; small-steps schedule; fixed
  index order everywhere; snapshot v27/v28; journal ops 22 to 25
  with id verification.
- Collision against the full rigid shape set with friction from
  the touched shape; two-way contact impulses (jelly has weight);
  the carved floor drops the rope the same step.
- Pins and body-frame anchors (capture position, lattice pull
  lands on the body, silent release on body death).
- The scale suite carries a lattice through its worker-twin,
  replay, and rollback gates; the red team storms a jelly on a
  collapsing deck to bit-identical twins.
- MANUAL chapter: soft bodies. Testbed: the jelly scene.
- With this release the ratified competitive-gap roadmap is
  complete: character controller, joints, and casts (v1.1),
  vehicles (v1.2), the 5k scale proof (v1.3), deterministic soft
  body (v1.4). GPU stays declined on the record: cross-vendor GPU
  floats cannot hold the bit contract that IS this engine.

## v1.3 (2026-07-08)

The scale release: the 5k-body proof, the profile-convicted
fixes, and the honest ledger corrections they forced.

- The cityblock bench (5000 mixed bodies, welded tower stacks, a
  fracture wave, per-phase timing, in-process twin check) and the
  scale gauntlet (5k rollback cost measured and bit-exact, 10k
  smoke with twin-stable hash).
- The scale suite (22nd): worker-count twins at a thousand bodies
  against a real four-thread pool, full-session journal replay,
  mid-storm rollback, capacity sweeps to 8192.
- Solver rev 20: the eternal-skater defect closed (the plane CCD
  arm accepted fraction-zero hits and froze sliding bodies at the
  slop gap frictionless forever; the friction cap now reads the
  reference's step-long normal-impulse sum; friction no longer
  solves during the bias pass). The golden hash moved, argued, to
  243acb1ac705c303.
- Rolling resistance on the shape def (reference recipe): sphere
  piles can finally stop rolling and sleep. Folds into the hash
  only where nonzero, so pre-existing scenes keep their bits.
- Broadphase fat reinsertion (the refresh had been reinserting
  tight boxes: a remove, insert, and rebalance per moving shape
  per step, about forty percent of the 5k profile).
- Pre-flight scratch sizing from pinned per-item byte budgets:
  the documented scratch growth finally implemented, the
  cross-compiler stall hazard closed, and the bench ledger's
  stalled-step fictions (pyramid froze mid-pileup since 2c-1)
  cured and re-pinned.
- The mover list: substep loops iterate awake dynamics and
  kinematics instead of walking the whole body array nine times a
  step.
- MANUAL chapter: scale.

## v1.2 (2026-07-08)

The vehicle release: deterministic raycast vehicles on the full
terrain stack, all additive under the 1.x freeze.

- The raycast vehicle (`m3CreateVehicle`): wheels are suspension
  ray casts, not bodies; spring and damper from hertz and zeta
  with per-wheel mass shares; two-phase impulse application from
  pass-start velocities; snapshot v24/v25; journal ops 19..21
  with id verification; the chassis cascade rule.
- Drive, steer, brake (`m3Vehicle_SetCommands`): journaled command
  STATE with range clamps and hostile no-ops; flat drive force,
  brake that never reverses rolling, lateral kill by the solver's
  effective mass split across wheels; ONE friction circle per
  wheel against the suspension load (the surrender is the drift).
  Tangent impulses at the wheel hub. Wheel spin as render state.
- Terrain: welded voxel seams, quarter-cell ramps, walls, fragment
  storms, and heightfields all driven and pinned; VoxelWakeRegion
  wakes sleeping chassis whose wheel rays overlap a carved region
  (a parked car drops the step its deck vanishes).
- Surface-relative tires (5-4): a car parked on a ferry rides it,
  brakes hold against the deck under it, and Newton's third law
  applies wheel impulses back onto dynamic ground (driving on a
  slab kicks the slab).
- MANUAL chapter: vehicles.
- Testbed: the circuit scene (WASD drive, chase camera, SPACE
  handbrake, a fort to crash through).

## v1.1 (2026-07-08)

The game-ready release: generic shape casts, the full joint set,
and a character controller, all additive under the 1.x freeze.

- Box and hull casts (`m3World_CastBoxClosest`,
  `m3World_CastHullClosest`): oriented boxes and 24-point clouds
  through the existing TOI kernel; skinless casts legal for real
  clouds.
- `m3_fixedJoint` (weld: create pose is the weld pose) and
  `m3_distanceJoint` (rope, rod, soft spring; bias-pass-only
  spring so relax cannot deaden it).
- `m3_genericJoint`: per-axis lock/free/limited plus one motor on
  any movable axis; the v1 angular contract (one limited angular
  axis, neighbors both locked or both free).
- The character controller: kinematic capsule, journaled
  `m3Character_Move`, collide and slide, walkable slope law, snap,
  step-up with the forward-probe floor classifier, dt-free push
  contract (impulse = mass * blocked displacement, mass-ratio
  cap), riders carried by their floor's rigid motion through the
  slide casts, same-step airborne on carved floors, ground body
  API.
- The caster's float budget: any cast or move translation
  component beyond 1e18 misses (or no-ops) by contract instead of
  minting NaN.
- Solver rev 19: the prismatic rotation-lock bias sign (a latent
  positive-feedback amplifier no prismatic scene could express;
  the weld found it).
- Snapshot format v23 (character pool, stepHeight, mass, push
  ratio, ground-body reference, generic joint state).
- MANUAL chapters: shape casts, joints, the character controller.
- Testbed: a playable character scene (WASD walk, carve the fort
  from under your own feet).

## v1.0 (2026-07-08)

The destruction phase (3), and the freeze. Voxel chunk shapes
(16x16x16 occupancy, payload, and fill fraction as journaled,
hashed, rollback-covered state), a derived greedy-merged collision
surface with automatic seam welding, journaled edits that wake what
they disturb, deterministic fracture (base-layer anchoring, flood
fill, fragment-spawn events with recipes; the engine never spawns
bodies), voxel continuous collision (one-voxel walls stop bullets;
cleared holes are real), fill-weighted fragment mass, bounded
interior depenetration, and a red-team suite from negative
coordinates to 448-island fracture storms. The voxfort bench scene
and the installed-package consumer drive all of it through public
API only: the constitution's freeze-when-proven condition is met,
and the 1.x API freeze is in effect.

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
