# Maul3D Manual

This is the contract between the engine and the host, engineer to
engineer. The README says what Maul3D is; this document says exactly
what it promises, what it deliberately does not promise, and the
rules a host must follow to keep the promises alive. Everything here
is enforced by a test that runs on every commit; where a rule has a
number, the number is the one in the code.

## The object model

A **world** owns everything: bodies, shapes, joints, contacts,
events. Worlds are isolated; nothing is shared between two worlds,
and ids from one world never operate on another.

A **body** is a frame with mass and velocity (static, kinematic, or
dynamic). A **shape** attaches geometry to a body: sphere, box,
capsule, convex hull (built by the internal QuickHull), static
triangle mesh, heightfield chunk, or infinite plane (static bodies
only). A **joint** constrains two bodies: spherical (optional swing
cone and twist limits), revolute (limits, motor), prismatic (limits,
motor).

Everything is created from a **def** struct obtained from its
`m3Default*Def()` function. Defs carry a private cookie; a def that
was not built from the default function is refused. Every create
returns an **id**; every failure returns the null id or false. The
library never aborts on user input: bad defs, stale ids, exhausted
pools, and hostile floats are refused through return values
(assertions guard internal invariants only, states that cannot be
reached from outside).

### Id lifecycle

Ids are generation-tagged: `{index1, world0, generation}`. A slot's
generation bumps when the object is destroyed, so a stale id can
never alias a recycled slot. At generation 0xFFFF the slot retires
instead of wrapping; a pool whose slots have all retired refuses
forever, loudly. The rule for staleness is uniform: getters return
zeros, commands and destroys no-op, creates refuse.

One id is valid until its object or its WORLD is destroyed. Body,
shape, and joint ids name their world by slot, not generation, so
after `m3DestroyWorld` the caller must drop every id minted from
that world; a new world recycling the slot cannot tell foreign
stale ids from its own. Nothing crashes either way.

Destroying a body cascades: its shapes and its joints go with it.
Destroy of an already-stale id is a quiet no-op (the destroy
contract).

## The determinism law

**Promised:** the same build of Maul3D, given the same sequence of
API calls with the same arguments, produces bit-identical simulation
state on every supported platform, compiler, architecture, SIMD
backend, and worker count. This is CI-enforced across six cells
(Linux gcc and clang, Debug and Release, forced-scalar with
sanitizers, macOS arm64, Windows MSVC) whose printed state hashes
must be equal on every commit.

**Deliberately not promised:**

- Hash stability across library versions. A solver revision or a
  snapshot format bump moves the bits on purpose and says so.
- Determinism across DIFFERENT builds of the same version (another
  compiler's build is another timeline; each build is internally
  deterministic and all builds we ship cells for agree, but that
  agreement is a tested property of our flag discipline, not
  something to assume for arbitrary flags).
- Wall-clock time of anything.
- Id values under different creation orders. Ids are deterministic
  for a deterministic call sequence, nothing more.

**Host obligations:** feed the same calls in the same order with the
same values. Never derive a simulation input from wall time, pointer
values, hash-map iteration order, or thread scheduling. Use the
fixed-timestep loop you would use for rollback netcode anyway.

Fast-math is refused at configure time; the build owns its float
semantics (`-ffp-contract=off`, no unsafe optimizations, `/fp:precise
/fp:contract-` on MSVC).

## The step

`m3World_Step(world, dt, substeps)` advances the world by dt seconds
in `substeps` solver substeps (4 is the tested default). Commands
(create, destroy, set velocity) may be issued between steps in any
deterministic order; issuing them from inside task callbacks is not
supported.

The solver is the Soft Step scheme: soft constraints, speculative
contacts, warm starting carried in the persistent manifolds, a relax
pass, then restitution. Joints solve before contacts inside every
substep. Sleeping is per island: a dynamic body whose motion stays
under the sleep velocity (0.05) for half a second sleeps; a sleeping
island is bit-frozen (its state does not drift by even one bit) until
contact, joint change, command, or a waking neighbor returns it.

## The snapshot law

`m3World_SnapshotSize / m3World_Snapshot / m3World_Restore` save and
load the COMPLETE simulation state, bit for bit, in a portable,
versioned, field-by-field format. Never a raw memory image.

In the snapshot: pool identities (including free lists, generations,
and retirements), body state, shape state, interned hull and mesh
content, joint state including warm-start impulses, persistent
contact manifolds and their impulses, broadphase tree, sleep state,
step counter.

Deliberately NOT in the snapshot:

- **SIMD width and worker count.** A snapshot taken on a 4-wide AVX
  build restores on a scalar build and continues bit-identically;
  parallelism is not state.
- **Derived data.** Acceleration structures that are pure functions
  of content (the per-mesh BVH) are rebuilt on restore and proven
  byte-identical by test.
- **Event streams.** Events are transient observations; restore
  clears every stream (contact, sensor, and fragment alike).

A snapshot restores only into a world with the same capacities and
the same build semantics (a config hash guards both). Truncated
buffers, corrupted headers, and wrong-shaped worlds are refused with
the target world untouched.

The header carries a format version. There is no migration; a
snapshot is a save-state for rollback and replay, not an archival
format.

## The journal law

The journal records every MUTATION as a discrete op: creates (with
their full recipes: hull input points, mesh vertex and index
payloads), destroys, velocity commands, and steps (dt and substep
count). Getters and queries are not ops.

`m3World_JournalBegin(world, buffer, capacity)` arms recording into a
caller-owned buffer. `m3World_JournalEnd` returns the bytes written,
or -1 if the buffer overflowed (an overflowed journal records
nothing more and can never be mistaken for a good one).
`m3World_JournalReplay(world, data, size)` applies a recorded session
to a world and verifies that every minted id matches the recorded
one; a world that starts from the same state WILL mint the same ids,
so a mismatch means the session does not belong to this starting
state.

**Replay is atomic.** On any refusal (truncation, corruption, an op
that cannot re-mint its recorded id) the target world is restored to
its pre-call state. A half-applied session is impossible.

**The composition rule:** a journal must close BEFORE a rollback
block. A restore is not an op; a journal spanning one would replay a
longer history than the world lived. Record, End, then snapshot and
restore freely; arm a new journal after.

## The task interface

The world def takes `enqueueTask`, `finishTask`, and
`userTaskContext`. Both hooks or neither. The library NEVER spawns
threads; it only offers work to the host.

The contract: `enqueueTask(task, count, context)` is called with a
task function to run over a range; the host may run it on any
thread, split it, or run it inline. `finishTask` must not return
until that work is complete. Calls are paired and never nested. The
work the library hands out is data-parallel by construction (
narrowphase pairs, same-color contact batches of at least 16), so
the worker count and the split points change wall time and nothing
else. The four-gate CI proves one worker and four workers produce
identical bits.

Reentering the API from inside a task callback is not supported.

## Sensors

A shape created with `isSensor` detects and never responds:

- No contact response: bodies pass through, with no impulse and no
  effect on trajectories.
- Its own event streams: `m3World_SensorBeginEvents` and
  `m3World_SensorEndEvents`, derived by the same canonical walk as
  contact events. Contact streams stay silent for sensor pairs.
- No waking: a sleeping body inside a sensor keeps sleeping; the
  begin event stays open (narrowphase keeps watching because only
  the solver sleeps). No island coupling.
- No stopping power: continuous collision ignores sensor targets,
  and a sensor shape on a fast body sweeps nothing.
- Sensors do not sense other sensors.

## Events

Contact begin/end and sensor begin/end streams are rebuilt every
step and valid until the next `m3World_Step`, `m3World_Restore`, or
world destruction. Restore clears all streams (events are
observations, not state). Event order within a step is canonical and
deterministic. The count pointer of the accessor may be null if the
caller only wants the array (it is then unbounded and useless: pass
the pointer).

## Queries

- Rays (`m3World_CastRayClosest`, `m3World_CastRayAll`) hit front
  faces only. A ray starting inside or exactly on a surface reports
  a miss for that shape; use `m3World_PointInside` to ask about
  containment. All-hits results are sorted by fraction, ties by
  shape index; when the output array fills, the farthest hit drops.
- Shape casts (`m3World_CastSphereClosest`,
  `m3World_CastCapsuleClosest`, `m3World_CastBoxClosest`,
  `m3World_CastHullClosest`) report the first time of impact along
  a translation. A cast that starts overlapped reports fraction
  zero with a zero normal. Box casts take a center, half extents,
  and an orientation; hull casts take up to 24 base-relative cloud
  points. A one-point skinless cloud misses by contract (that is a
  ray). Any translation component beyond 1e18 misses by contract:
  past that, the kernels' float arithmetic would overflow (the
  caster's float budget).
- `m3World_PointInside` treats planes as solid half-spaces and
  meshes as open surfaces (a point is never "inside" a mesh).
- Overlap queries (`m3World_OverlapAabb`, `m3World_OverlapSphere`)
  return exact shape-level results, planes included.

Queries read the world and never write it; they are safe to call at
any point between steps and have no effect on determinism.

## Continuous collision

Every fast dynamic body sweeps against static geometry, so nothing
tunnels through a floor. A body flagged `isBullet` additionally
sweeps against dynamic and kinematic targets, with both bodies'
sweeps in the time-of-impact kernel. Bullet versus bullet is not
resolved (the reference's limitation, shared and documented).
Sensors neither stop nor are stopped.

## Debug draw

`m3World_Draw` emits the world as segments and points through host
callbacks (wireframes for every family, contact points with
impulse-scaled normal whiskers, joint anchors, broadphase fat AABBs,
a fixed color palette with sensor, sleeping, kinematic, and static
tints). It is a pure observer: a draw pass between two snapshots
leaves every byte identical, and the suite holds that promise on
every commit.

## The derived-data law

Anything the engine can recompute from content is NEVER state: not
snapshotted, not hashed, rebuilt wherever content lands (create,
journal replay, restore), and the rebuild must be a pure function so
twin worlds agree bit for bit. The per-mesh BVH is the standing
example; future acceleration caches follow the same law.

## Voxel chunks

A voxel chunk is a shape on a STATIC body: a dense 16x16x16 grid of
cells anchored at the body origin, each cell carrying an occupancy
bit, a uint16 payload (material, health, type: yours to define),
and a fill fraction (255 a whole voxel, 1 a sliver). All of it is
state: hashed, snapshotted, journaled, inside the rollback delta.

The collision surface is derived: a deterministic greedy merge
produces maximal boxes and a BVH over them; you never see or manage
it. Chunks placed with bit-exact identity rotations, equal cell
sizes, and world positions exactly one chunk extent apart WELD:
their shared border becomes interior geometry, and bodies roll
across it without seam impulses. Rotated or misaligned chunks
still collide; they just do not weld.

Edits are commands: set a voxel (with payload), clear a voxel,
clear a box region, set a fill fraction. Every edit is journaled
and replayed like any other mutation; the surface follows the grid
as a pure function; dynamic bodies touching the edited region wake.
Fill is mass, never geometry: a worn voxel collides at full size
but its fragment weighs fill over 255. A chunk edited to empty
stays a valid shape that collides with nothing.

Everything sees chunks: contacts, rays (front faces, born-inside
misses), shape casts, continuous collision (a one-voxel wall stops
a bullet; a hole cleared last step is a real hole), point-inside
(solid voxels are CLOSED volumes), and overlaps. A body whose
center ends up inside the solid walks out through the nearest
exposed face at a bounded pace instead of exploding.

## Fracture

The anchor convention: an island of voxels is anchored if and only
if it has a path (six-connectivity) to the chunk's y = 0 base
layer. Build structures at their chunk's base; a floating platform
is a chunk whose BODY sits in the air. After every
occupancy-clearing edit, a deterministic flood fill removes every
unanchored island from the grid AS PART OF THE SAME STATE
TRANSITION (rollback and replay re-derive identical grids) and
emits one fragment event per island, in canonical order.

The event carries the chunk id, voxel count, mass (fill-weighted,
at density one), the center of mass in chunk and world frames, the
island bounds, and a recipe: chunk-local voxel indices
(v = x + 16 * (y + 16 * z)) in a transient buffer. The engine
NEVER spawns bodies; what a fragment becomes is yours. The
reference recipe feeds small islands' voxel corners to the
built-in QuickHull and gives large ones their bounds box, with
density matched to the event mass (see the voxfort scene in
bench/).

Fragment events and recipes are transient like every stream: the
next step clears them, restore clears them. When one edit strands
more islands than the event capacity (256), every island is STILL
removed from the grid; only the reporting has a ceiling, and the
dropped count says so loudly.

## Joints

Six types, every one built from the same proven row blocks (point,
axial, rotation-vector) so a new joint is a recipe, not new math:

- `m3_sphericalJoint`: a shared point, plus an optional cone limit
  (`coneAngle`) and twist limit pair.
- `m3_revoluteJoint`: hinge; angle limits and a motor
  (`motorSpeed`, `maxMotorTorque`).
- `m3_prismaticJoint`: slider; translation limits and a motor.
- `m3_fixedJoint`: weld. The relative pose at create time IS the
  weld pose (axes and limits are ignored by contract).
- `m3_distanceJoint`: rope or rod along the anchor axis. Equal
  bounds make a rod; `hertz`/`zeta` in the motor fields make a
  soft spring whose rest length rides `coneAngle` and must lie
  inside the bounds.
- `m3_genericJoint`: per-axis `m3AxisMode` (locked, free, limited)
  over three linear and three angular axes, plus one motor on any
  movable axis. The v1 angular contract: at most one limited
  angular axis, and its neighbors both locked or both free.

Anchors are body-local; the joint frame's axis is local z. Joint
impulses persist in the snapshot (warm starts are simulation
state). Creation validates everything and returns the null id on
any violation; destroying a body cascades through its joints.

## Vehicles

A vehicle (`m3CreateVehicle`) is the classic raycast car: the
chassis is a normal dynamic body you create and shape; wheels are
NOT bodies. Each wheel is a suspension ray cast from a
chassis-local anchor (the chassis never blocks its own rays), a
spring and damper impulse scaled by hertz, zeta, and a per-wheel
share of the chassis mass, and a tire working in the contact
plane. All wheel impulses land at the top of the step, before the
solver, from the same pass-start velocities.

- Commands (`m3Vehicle_SetCommands`: throttle, steer, brake) are
  journaled STATE, clamped to range; non-finite values are hostile
  no-ops that never journal; a commanded car wakes. Replay and
  rollback reproduce a drive to the bit.
- The chassis-local +x axis is forward. Steerable wheels rotate
  their frame about the suspension axis by steer * maxSteerAngle.
- The tire: driven wheels push with a flat driveForce, brakes
  oppose rolling and never reverse it, and lateral slip dies at
  the solver's effective mass split across wheels. One friction
  circle per wheel clamps the tangent sum against tireGrip times
  the suspension load: what surrenders under a hard corner is the
  drift.
- Tires work in SURFACE-RELATIVE velocity: a car parked (brake on)
  on a moving platform rides it, and wheel impulses push back on
  dynamic ground (driving across a loose slab kicks the slab).
- Free-rolling wheels have no longitudinal grip by design; parking
  on a slope or a ferry takes the brake, like a real car.
- Destruction interplay: carving the deck from under a parked,
  even sleeping, car wakes it the same step (wheel rays count as
  presence for the voxel wake).
- Suspension compression, wheel contact, and spin angle read back
  per wheel for rendering; destroying the chassis destroys the
  vehicle.

## The character controller

A character (`m3CreateCharacter`) is a kinematic capsule the
engine owns, driven only by `m3Character_Move`: a journaled,
deterministic, rollback-covered command. The body has zero
velocity by contract and never moves during `m3World_Step` except
to ride its floor (below). Gravity is the host's job: add it to
the move each tick, netcode style.

- Collide and slide: the move casts the capsule, advances to the
  first hit keeping `skin`, slides the remainder, up to four
  times. Steep faces (steeper than `maxSlopeAngle`) are walls and
  never mint upward motion out of horizontal intent.
- Stairs: a grounded walker blocked by a steep face lifts up to
  `stepHeight`, advances, and lands. The landing is accepted only
  when a ray probing half a radius AHEAD of the landing axis finds
  walkable floor and the net rise fits one step height (the ray,
  not the capsule normal, is the floor classifier: capsule casts
  read stair corners as steep, and an axis ray can be fooled by
  low floor behind a ramp's crease).
- Grounding: walkable contact grounds the character and records
  the body under its feet (`m3Character_GetGroundBody`). A
  descending character glues to floor within `snapDistance`; no
  floor in reach means airborne, honestly.
- Riders: each step ends by carrying grounded characters along
  their ground body's rigid motion, THROUGH the slide casts, so a
  platform can ferry a walker into a wall and the wall wins.
  Kinematic platforms, elevators, carousels, and dynamic bodies
  (fragments) all ferry riders.
- Pushing: a move blocked by a dynamic body applies impulse =
  `mass` * blocked displacement at the contact. Per tick that
  displacement is the walker's velocity in tick units, so the
  momentum flux comes out mass * speed with no dt in the contract.
  Walkable floors are standing, not pushing; bodies heavier than
  `pushMaxMassRatio` * `mass` are walls.
- Destruction interplay: carving the floor from under a character
  flips it airborne the SAME step (the voxel edit runs a grounding
  refresh over overlapping characters).
- Hostile moves (non-finite, or any component beyond the caster's
  1e18 float budget) are documented no-ops that never journal.

## Units and conventions

SI units: meters, kilograms, seconds, radians. Gravity defaults to
(0, -10, 0). Right-handed coordinates. Quaternions are xyzw, unit
length, and a body def's rotation must be within 1 percent of unit
length (a corrupted quaternion is refused, not renormalized).
Positions are double precision; everything else is single. Mesh
triangles wind counter-clockwise seen from outside; backfaces do not
collide. Planes are half-spaces: `dot(normal, p) <= offset` is
solid. Capsules are a segment plus a radius; a zero-length segment
is refused (use a sphere).

## Capacities

Every pool is fixed at world creation (`bodyCapacity`,
`shapeCapacity`, `jointCapacity`, `meshCapacity`) and never grows.
Exhaustion refuses with a null id and the world keeps stepping. Caps
per shape: hulls up to 24 vertices from up to 64 input points,
meshes up to 1024 vertices and 2048 triangles, heightfields up to
32 by 32 per chunk (tile larger terrain as chunks).
