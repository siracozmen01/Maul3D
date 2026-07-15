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

Twelve types, every one built from the same proven row blocks
(point, axial, rotation-vector) so a new joint is a recipe, not
new math:

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
- `m3_wheelJoint`: suspension slide plus free axle spin, its own
  chapter below.
- `m3_filterJoint`: no rows at all, BY LAW. Its whole effect is
  the connected-pair collision filter: ragdoll limbs stop
  grinding without buying a single constraint.
- `m3_parallelJoint`: keeps `localAxisA` on body A parallel to
  `localAxisB` on body B (two angular rows); every translation
  and the twist about the shared axis stay free.
- `m3_motorJoint`: the servo weld, its own chapter below.
- `m3_gearJoint` and `m3_pulleyJoint`: transmissions, their own
  chapter below.

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

## Scale

The engine's scale posture is measured, not promised: the bench
binary's cityblock scene (nine welded voxel tower stacks, five
thousand mixed bodies, a mid-run fracture wave) and the scale
gauntlet behind it are the ledger; the scale suite proves the same
contracts at CI size with a real four-thread host pool.

- Worker counts are performance knobs, never state inputs: serial
  and threaded runs of the same ops land on identical bits, at
  every population.
- Rollback at scale is priced, not feared: at five thousand bodies
  a full snapshot runs tens of milliseconds and tens of megabytes,
  restore the same, both bit-exact (the gauntlet prints the
  numbers on your machine).
- The step scratch pre-flights its size from pinned per-item
  budgets (bytes per body, shape, pair, joint), so growth happens
  BEFORE a step on every platform at the same tick; a scene can
  never stall differently on different compilers.
- Sleep is the sandbox's economy. Give ground-contact debris a
  small rollingResistance (0.01 to 0.1): a frictionless-rolling
  sphere never stops on its own, and one eternal roller keeps its
  whole contact island awake.
- Capacities are hard books, not hints: worlds create up to
  exactly their capacity and refuse the next, at 256 and at 8192
  alike. Size the def for the scene, plus fragments.
- Large levels tile as chunks (voxel chunks, mesh chunks,
  heightfield chunks) under one static body each; welding makes
  grid-aligned voxel seams contact-seamless.

## Soft bodies

A soft body (`m3CreateSoftBody`) is an XPBD particle lattice: a
box of particles (rope n x 1 x 1, cloth n x m x 1, jelly n x m x
k) bound by structural and face-diagonal distance constraints
with compliance (zero is rigid rods), solved one Gauss-Seidel
sweep per substep in fixed index order: deterministic by
construction, under the same four gates as everything else.

- Particles collide with the whole rigid shape set (planes,
  spheres, capsules, boxes and hulls, voxel chunks, meshes,
  heightfields) and take friction from the touched shape. Contact
  with dynamic bodies is two-way: jelly has weight.
- `m3SoftBody_PinParticle` freezes a particle where it is;
  `m3SoftBody_AnchorParticle` captures it in a BODY's frame: it
  rides the body, the lattice's pull lands on the body at the
  anchor, and a dead body releases its anchors silently. Cloth
  hangs from beams and follows them when they move.
- Soft-versus-soft and self-collision are NOT in this version
  (documented out, the bullet-versus-bullet precedent); particle
  counts cap at 512 and anchors at 32 per body.
- Destruction interplay holds: carve the floor from under a
  resting rope and it falls the same step.
- All soft state (positions, previous positions, inverse masses,
  edges, anchors) snapshots, hashes, replays, and rolls back
  bit-exact like every other pool.
- A deterministic curiosity worth knowing: a perfectly vertical
  rope BALANCES on its end forever, because bit-exact floats
  carry no noise to seed the buckle. Give real scenes a nudge.

## Collision filters

Every shape carries `categoryBits` (default 1), `maskBits`
(default all), and `groupIndex` (default 0). Two shapes meet only
when each one's category intersects the other's mask, both ways; a
shared positive group forces collision, a shared negative group
forbids it. One chokepoint gates ordinary pairs, sensors, and the
continuous phase alike. Queries behave like shapes: the `Ex`
variants of every ray, cast, and overlap take an `m3QueryFilter`.
Filters are state (snapshot v29) and hash only off-default.

## Forces and impulses

`m3Body_ApplyForce`, `ApplyTorque`, and `ApplyForceAtPoint`
accumulate for exactly one step: integrated beside gravity every
substep, consumed after the last. `ApplyLinearImpulse`,
`ApplyAngularImpulse`, and `ApplyImpulseAtPoint` act immediately.
At-point arms are measured from the center of mass. Pending
accumulators are snapshot state: a snapshot taken between an
application and its step re-lands the same trajectory. Targets
must be dynamic with mass; anything nonzero wakes the body.

## Runtime body control

`m3Body_SetTransform` teleports and wakes both the departure and
arrival neighborhoods. `m3Body_SetTargetTransform` is a kinematic
servo: the body lands exactly on the target next step and the
order clears; the exit velocity stays yours by contract.
`m3Body_SetType` flips dynamic/kinematic/static with a full mass
rebuild. `m3Body_SetEnabled(false)` removes the body from
contacts, CCD, rays, casts, overlaps, and soft-lattice collision.
`m3Body_SetMotionLocks` freezes any subset of the six axes (bits
0..2 linear xyz, 3..5 angular xyz), re-zeroed every substep so
contacts cannot bank motion on a frozen axis.
`m3Body_SetSleepControls` gives a per-body threshold and a
canSleep override; `m3Body_SetAwake` forces either edge.

## Materials and world tuning

Shape materials are runtime-settable and journaled:
`m3Shape_SetFriction`, `SetRestitution`, `SetRollingResistance`,
and `SetDensity` (the latter with an optional mass rebuild).
Contacts read materials at prepare, so changes bind next step; a
sleeping stack keeps its old mix until something wakes it. World
knobs: `m3World_SetGravity` (sleepers stay asleep until
disturbed), `SetContactTuning` (hertz, damping ratio, max
depenetration speed; static contacts run twice the hertz),
`SetRestitutionThreshold`, `SetMaximumLinearSpeed` (a hard
per-substep clamp), `EnableSleeping` (off wakes everyone), and
`EnableContinuous`. Knobs are STATE, not config: they journal,
they snapshot, and they hash only off-default. The config hash
stays version + solver revision + precision + FP policy, because
a config-hash knob would refuse the journal instead of replaying
it.

Painted mesh materials (17-2): m3Shape_SetMeshMaterials gives a
mesh shape up to eight surface materials and one group byte per
triangle. The struck triangle's entry replaces the MESH side of
the contact mix (friction, restitution, rolling resistance), and
its surface velocity feeds the conveyor row per triangle: a
moving walkway, an ice patch, and tarmac can share one level
mesh. One welded manifold wears one material (the first,
id-canonical point picks it, documented v1 contract). Unpainted
meshes keep using their shape material everywhere, bit-exactly
as before. Journaled, snapshotted, hostile paint refused loudly.

The broadphase rebuild (17-4): m3World_RebuildBroadphase replaces
the incrementally grown tree with a balanced top-down build over
the live shapes, in one deterministic, journaled pass. Bulk level
construction inserts one shape at a time and can leave the tree
lopsided; call this once after building and queries walk a
balanced tree. It changes NO answer: pairs are canonical and the
tree is never hashed, so a rebuilt world steps bit-identically to
its unrebuilt twin (the suite proves it by hash equality).

## Water volumes

m3CreateWaterVolume fills a world-anchored axis-aligned box with
still or flowing water (up to 8 per world; the surface is the box
top; there is NO fluid simulation by law). Rigid bodies get
buoyancy proportional to their submerged bounds applied at the
submerged centroid (off-center bites right a listing crate), drag
that pulls their velocity toward the flow, and spin damping; the
model reads shape bounds, deliberately approximate and
deterministically so. Soft particles trade gravity for buoyancy
under the rho-1000 particle convention (density 1000 suspends a
particle, denser fluids lift it, thinner ones let it sink) and
drag toward the flow like wind. The tide law: creating or
destroying a volume wakes every body its box touches, so sleepers
float when the flood arrives and fall when it leaves. Volumes
journal (ops 74/75), snapshot (v48), and hash only while one is
alive; a dry world keeps its exact pre-water bits.

Snapshot economics (17-1): hull and mesh content travel
count-derived, so an empty slot costs bytes, not kilobytes, and a
box hull costs its used prefix instead of a 5808-byte slab.
Snapshot sizes GROW with world content: measure with
m3World_SnapshotSize per keyframe, never once per session.

## Hit events, move events, and the pre-solve veto

Hit events are opt-in per shape (`m3Shape_EnableHitEvents`) and
fire at most once per contact per step, for the fastest
approaching manifold point, when the approach speed clears
`m3World_SetHitEventThreshold`. Body move events arrive one per
mover per step in ascending body order with the post-step
transform and a fell-asleep mark; sleeping bodies cost nothing.
The joint break stream reports joints that destroyed themselves
this step; the id inside is already stale, use it as a key. The
pre-solve veto (`m3World_SetPreSolveCallback` plus
`m3Shape_EnablePreSolve`) runs serially in canonical pair order
and may disable any flagged contact for the step: the one-way
platform tool. THE CONTRACT, LOUD: the callback must be a pure
function of its arguments; the journal records inputs, never host
whims, so an impure callback breaks YOUR replay, not the
engine's. All event streams are transient observers: the next
step or a restore clears them.

## Joint runtime control, drives, and breakage

`m3Joint_SetLimits` and `m3Joint_SetMotor` rebind mid-run
(toggling zeroes the row's stored impulse).
`m3Joint_SetCollideConnected` lets the two jointed bodies collide.
`m3Joint_SetSpring(enable, hertz, dampingRatio)` arms the position
drive on revolute (target angle), prismatic (target translation),
and spherical (target rotation) joints;
`m3Joint_SetTargetAngle/SetTargetTranslation/SetTargetRotation`
move the goal. The rotation target lives in the JOINT FRAMES
(frame z is the create-time local axis). Reaction readback:
`m3Joint_GetConstraintForce/GetConstraintTorque` return last-step
magnitudes assembled per type; they read 0 until the first step
after a restore. Breakage: `m3Joint_SetBreakThresholds` arms
force/torque caps (zero = off); an over-threshold reaction
destroys the joint at the end of the step and emits the break
event. Breakage is an in-step deterministic transition on
purpose: a host poll would race the journal under rollback.

The four-state drive law: a velocity motor alone brakes, a
position spring alone holds, and TOGETHER they share ONE budget,
the motor's `maxMotorEffort`. A starved budget makes the drive
sag honestly instead of borrowing force from nowhere; a zero
budget with the motor off leaves the spring unbudgeted. This is
the difference between a servo that stalls under load and one
that lies.

## Soft interactions and fields

Lattices collide with EACH OTHER (canonical particle pairs behind
lattice-bound gates; self-collision stays out, the structure rods
hold a lattice apart), and they hold hands:
m3SoftBody_AnchorToSoft pins particle to particle across lattices
with silent release when either side dies. Wind
(m3World_SetWind) drives particles with proportional drag toward
a gusted velocity whose phase ACCUMULATES in the snapshot: a
rollback resumes the exact same wave. Conveyors
(m3Shape_SetSurfaceVelocity) feed the contact solver's tangential
target: crates ride belts at belt speed, and a belt on a dynamic
deck recoils by Newton. All of it journals, snapshots, and twins.

Particle cloth is POROUS to rigid bodies of comparable radius
under load: a lattice sheet is a grid of contact spheres, and a
sphere the size of the grid gaps can press between them when the
span stretches. Densify the particles, thicken their radius, or
back the sheet with a thin box when you need a true barrier (a v1
bound of the box-lattice substrate, recorded here so nobody
relearns it).

## The drivetrain

The raycast vehicle's flat driveForce grows an engine when you
attach an m3DrivetrainDef (m3Vehicle_SetDrivetrain, journaled):
a torque curve of 2..8 pinned control points with deterministic
linear interpolation (only +,-,*,/ touch the numbers), 1..6
forward gear ratios plus reverse, a final drive, pinned shift
RPMs, and a clutch measured in steps of torque cut. The first
control point is the idle line (a standing car keeps launch
torque); past the last point the curve extends flat. Engine
speed derives from chassis forward speed through the driven
wheels' mean radius and the active ratio; crank torque splits
equally across driven wheels (an open differential) and the
friction circle still has the last word at each tire.

With a drivetrain attached, throttle drives forward gears only:
reverse is gear -1 (m3Vehicle_SelectGear, journaled; 0 is
neutral) and takes throttle magnitude with direction from the
gear. Auto shift climbs above shiftUpRpm and drops below
shiftDownRpm, forward gears only, never while the clutch is
open (the thrash brake). Current gear, clutch countdown, and
the tachometer (m3Vehicle_GetGear, m3Vehicle_GetEngineRpm) are
state: snapshotted, hashed when attached, rolled back mid-shift
to the bit. Vehicles without a drivetrain keep the flat model
untouched.

Differentials: the def's `diffMode` picks open (0, the classic
equal split), limited (1), or locked (2), and `diffCouple` sets
the coupling force per m/s of wheel-speed disparity. The
coupling reads each driven wheel's contact speed with one step
of lag (the drivetrain's own wheel-reading convention) and
squeezes the split toward the driven mean; limited clamps the
transfer to the wheel's own drive force. The couple is
HOST-TUNED TO MASS SCALE: a 300 kg buggy locks solid near 20000
and oscillates at absurd values, so tune against your own car,
not a constant from this manual. All of it lives inside the
friction circle, so the effect is emergent grip management: a
locked diff resists yaw, a soft limited one LIVENS rotation by
feeding the unloaded wheel less.

## The wheel joint

m3_wheelJoint is the OPTIONAL rigid-wheel path: the wheel is a
real body on real contacts (trucks over rubble, wheels in
fragments), while the raycast vehicle stays the default.
localAxisA is the suspension axis in the chassis frame,
localAxisB the axle in the wheel frame; the axle is captured at
create and snapped exactly perpendicular to the strut (more
than a small skew refuses loudly). One joint composes both
halves: the wheel slides along the strut (travel limits from
enableLimit, suspension spring from m3Joint_SetSpring plus
m3Joint_SetTargetTranslation) and spins freely about its axle
(drive with m3Joint_SetMotor; read the spin with
m3Joint_GetAngle and the travel with m3Joint_GetTranslation).

Breakage is the 8-6 contract unchanged: the force cap snaps a
wheel torn sideways, the torque cap a drive axle overdriven,
in-step, evented, and deterministic through rollback. The def
grew nothing and there is no new state: a wheel joint is
ordinary joint state and rides every snapshot and hash law that
already existed.

Steering: `m3Joint_SetSteer(enable, targetAngle, hertz, zeta,
maxTorque)` yaws the wheel about its strut toward the target
(radians, at most 1.0 either way), a soft drive with its own
budget (0 = uncapped); `m3Joint_GetSteerAngle` reads the live
angle back. Steer the front pair of a wheel-joint cart and you
have rack-and-pinion steering on real contacts, no vehicle
required. Journaled, snapshotted, bit-exact through rollback.

## The servo weld

`m3_motorJoint` drives body B toward a target pose relative to
body A with NO hard rows: a soft 3-DOF rotation drive plus a
soft 3-DOF translation drive, each clamped to its own budget.
Tune stiffness with `m3Joint_SetSpring` (shared by both drives),
aim with `m3Joint_SetMotorPose(offset, rotation)` (the offset in
A's create-time frame, the rotation relative to the create
pose), and budget with `m3Joint_SetLimits` where lower = max
force and upper = max torque, 0 = uncapped (for this type they
are independent allowances, not a range). A fresh servo aims at
its own create pose; without a spring it holds NOTHING and the
bodies drift free. Use it for animated platforms, grabbers, and
kinematic-feeling props that still lose honestly to a bigger
force: a starved servo sags, it does not teleport.

## Gears and pulleys

`m3_gearJoint` couples spin so `spinA + ratio * spinB` keeps its
create value, where each spin is that body's rotation about its
own `localAxis`. Positive ratio is the external mesh (counter-
rotation); negative couples same-direction like a belt. The
mounting contract, documented and unchecked: both bodies should
be hinged on a common rigid frame (revolute or wheel joints to
the same chassis); the gear holds no other degree of freedom and
puts no reaction on that frame. Angle drift is corrected softly,
so a mesh cannot creep under load.

`m3_pulleyJoint` runs a rope from `localAnchorA` over the WORLD
point `groundAnchorA`, across to `groundAnchorB`, down to
`localAnchorB`: `length1 + ratio * length2` keeps its create
value, `ratio > 0`. The rope is RIGID both ways by contract (it
can push); pair it with a distance-joint rope when slack
matters. Ratio 2 is a block and tackle: B moves half as fast and
carries twice the force. Both types create through the same def
(`ratio`, `groundAnchorA/B` at the tail), journal, snapshot, and
hash like every other joint.

## Stance

m3Character_SetStance resizes the capsule in place, FEET
ANCHORED: the center moves so the capsule bottom stays level,
which also means a mid-air crouch lands shorter, never higher.
Shrinking always applies (removing volume cannot create
contact). Growing runs the stand-up veto: the grown capsule is
cast at its new pose through the same core every move uses, and
any contact within one skin (a pressing ceiling, a wall against
a wider radius) refuses the whole change and returns false with
stance untouched. The veto is bit-deterministic: twins and
replays grant and refuse in the same pattern.

Only APPLIED changes journal; a vetoed stand writes nothing.
Stance rides existing state (the capsule shape, the body
transform, the character config), so snapshots, hashes, and
rollback cover it with no new machinery. Read it back with
m3Character_GetStance.

## The angular speed cap

The linear speed cap (8-4) has an angular twin: every substep,
any dynamic body spinning past the world cap is scaled back onto
it. The default (800 rad/s) is a catastrophe guard in the same
philosophy as the 400 m/s linear default: far above anything a
legal scene does, it exists to stop solver-explosion artifacts,
not gameplay. Hosts that want the reference's tight clamp set a
low cap with m3World_SetMaximumAngularSpeed (journaled, hashed
only off-default) and flag their legal fast spinners, wheels
above all, with m3Body_SetAllowFastRotation (journaled). The
flag lives beside the motion locks, survives lock writes, and
reads back with m3Body_GetAllowFastRotation.

## Explosions

m3World_Explode is one journaled call that does a whole
demolition. Fill an m3ExplosionDef (from m3DefaultExplosionDef,
cookie included): position, radius, falloff, impulsePerArea,
optionally voxelCarve and softPush, optionally a query filter.
Every dynamic sphere, capsule, and hull in range takes an
impulse scaled by the AREA it shows to the blast (the reference
projected-area model), directed from the center through the
closest point on the shape, which means off-center hits spin
their targets through the real lever arm. Past the radius the
impulse fades linearly to zero across the falloff band. The
blast wakes everything it reaches; a negative impulsePerArea
implodes.

voxelCarve > 0 bites a sphere of that radius out of every voxel
chunk in range through one surface rebuild and one fracture
sweep per chunk; freed islands arrive as the usual fragment
events for the host to spawn (the testbed kicks its newborn
fragments radially for the look; the engine keeps events and
bodies separate on purpose). softPush scales the same falloff
push onto soft-body particles: the kick is applied exactly once
by the next step, is real snapshot state in the window between
the call and that step, and folds into the hash only while it
exists.

Hostile defs (NaN anywhere, negative radius, dead cookie) apply
nothing and journal nothing; hostile tape bytes fail the replay
loudly. Explosions are events, not state: two worlds that
explode identically stay bit-identical through twins, replay,
and rollback.

## Introspection

m3World_GetCounters reports the live census: bodies, shapes,
joints, contacts, characters, vehicles, soft bodies, voxel
chunks, interned hull and mesh slabs, awake dynamic bodies, the
last step's island and solver-color counts, the broadphase tree
height, the step scratch high water against its reserve, the
exact snapshot size, and the process-wide persistent
allocation/free call counts. m3World_GetProfile reports
wall-clock milliseconds per phase of the last completed step
(vehicles, broadphase, narrowphase, events, prepare, solve,
continuous, sleep, characters, soft bodies, and the whole).

Both are PURE observers: reading them never touches the
simulation, the snapshot, or the hash, and a suite holds that
promise with a twin that polls every tick against a twin that
never looks. Counter values are deterministic; profile times are
honest wall-clock and never will be.

m3World_DrawExtras adds the analysis layers on top of the base
draw: island tint points (colors cycled by island root, sleeping
bodies keep the tint of the island they slept in), center-of-mass
axis frames, and the broadphase tree's internal node boxes. A
separate additive struct, the frozen m3DebugDraw and m3SolidDraw
stay untouched, and the same purity gate covers the new walk.

Bodies take debug names (m3Body_SetName, 31 bytes plus the
terminator, journaled, carried by snapshots, NEVER hashed) and
answer "who touches me" through m3Body_GetContactData and
m3Shape_GetContactData: manifold entries in canonical pair order
with world points, separations, and last-step normal impulses.

m3SetAssertHandler installs a global host hook ahead of the
debug abort; returning nonzero declares the failure handled
(test harnesses, crash reporters). There is deliberately NO log
hook: the engine has no log stream to route, and a dead API is
worse than a missing one.

## The honest cylinder, reshaping, and the overlap family

m3CreateCylinderShape mints a cylinder as a 2N-vertex prism
through the interned hull path (N = segments, clamped to 3..32
so 2N fits the 64-vertex hull law). Everything downstream (mass,
contacts, casts, CCD, carving, the blast's projected area)
treats the prism exactly; the documented trade is that the side
is an N-gon, not a curve, and 24 segments roll visually smooth
at meter scales. The analytic round cylinder stays on the ledger
for the day a consumer needs true roundness.

m3Shape_SetSphere and m3Shape_SetCapsule swap a shape's geometry
in place, conversions between the two included (journaled, op
69). Hulls, meshes, voxel chunks, and planes refuse: interned
slabs are immutable by law. A swap recomputes mass from the new
volume, wakes everything around BOTH the old and the new bounds
(a shrink frees what leaned on the old silhouette), and the
broadphase picks the new box up the same step machinery that
tracks every mover.

The overlap family joins the sphere: m3World_OverlapCapsule, an
oriented m3World_OverlapBox, and m3World_OverlapHullPoints (a
base plus up to 64 base-relative points and an optional radius,
the cast convention). All answer EXACTLY per family: convex
candidates through the GJK core, planes by minimum point
distance, voxel chunks and meshes by BVH gather and per-box or
per-triangle distance. Ascending shape ids, query filters, and
the observer purity law throughout.

## The replay studio

An M3J1 file is [header | initial snapshot | journal], encoded
and decoded by pure-memory calls in maul3d/replay.h; file IO
belongs to hosts and tools (tools/m3replay is the reference
consumer). The header carries step and op counts and the
recorder's final hash, so verification is self-contained: restore
the snapshot, replay the journal, compare hashes. A journal
prefix cut at record boundaries is itself a valid journal; the
scrubber in m3replay keyframes every 30 steps and re-steps to
seek anywhere, proving each landing against a straight-run
prefix hash. When two sessions disagree, m3replay diff finds the
first divergent step by prefix-hash binary search and
m3World_DiffReport names the worst bodies at it. Corrupt
containers refuse loudly: the codec validates magic, version,
exact lengths, record framing, and that the header's counts match
its own stream.

## Lockstep and rollback networking

The engine's bit contract makes the classic rollback recipe safe
to build exactly as the books describe it, and tools/m3lockstep
is that recipe as a runnable document. The shape:

1. Inputs are the only truth on the wire. Send each player's
   inputs per frame; never send state.
2. Keep a snapshot of the last frame whose inputs are ALL
   confirmed (m3World_Snapshot: one buffer is enough).
3. Predict forward past the frontier with guessed remote inputs
   (empty is the honest cheap guess).
4. When a late input arrives, restore the frontier snapshot,
   re-apply confirmed inputs, re-predict to the present
   (m3World_Restore + your step loop; bit-exactness makes the
   repair invisible).
5. Hash at agreed checkpoints (m3World_Hash). Equal hashes ARE
   the sync proof; a mismatch is a desync alarm on the exact
   frame it was born, and the divergence finder (m3replay diff)
   tells you which body carried it.

The sample runs an online peer against an offline truth peer
through a jittered 1..4 frame wire at 60 Hz, absorbs hundreds of
rollbacks, and demands checkpoint equality in CI on every commit.
Input delay, rollback windows, and per-peer prediction policies
are host decisions; the engine's promise is only, and exactly,
that the same inputs produce the same bits.

## The integration checklist

What an engine evaluator wires first, and where Maul3D answers:

1. Create a world, step it, read transforms: `m3CreateWorld`,
   `m3World_Step`, body move events (no polling).
2. Spawn and destroy at runtime: create ops are journaled with id
   verification; destroys cascade shapes, joints, and anchors.
3. Layers and masks: collision filters, filtered queries.
4. Apply gameplay forces: the force/impulse family.
5. Teleports, character platforms, elevators: SetTransform, the
   kinematic servo, motion locks.
6. Tune materials live: the material setters and world knobs.
7. Contact callbacks: begin/end events, opt-in hit events, the
   pre-solve veto with its purity contract.
8. Motorized doors, servos, breakable structures: joint runtime
   control, drives, and breakage with the break stream.
9. Queries: rays, casts, overlaps, all filtered, plus the
   character controller, vehicles, voxels, and soft bodies from
   earlier arcs.
10. Determinism: every one of the above is journaled, snapshot
    round-trips bit-exactly, and CI enforces cross-platform
    equality on every commit. That last line is the moat.

## Compound shapes and the geometry ceilings

Every shape may carry a local transform relative to its body:
localPosition and localRotation in m3ShapeDef (identity by
default, near-unit rotations demanded loudly). One composed read
serves the whole engine, so an offset shape collides, casts,
rays, draws, and weighs exactly as it sits. Mass properties
compose per shape before the parallel-axis walk: a dumbbell built
from two offset boxes spins by the composite books. Caster sweeps
take explicit poses and CCD composes at t0 (documented v1 bound).
m3DestroyShape removes one shape at runtime and rebuilds the
owner's mass; the last shape leaves a unit-mass shapeless body.

Ceilings: hulls hold up to 64 vertices (124 faces, 372 half
edges, input clouds up to 256 points); mesh chunks hold up to
65,535 triangles and 65,535 vertices with 16-bit indices, stored
count-derived so empty capacity costs nothing; heightfields
triangulate through the same mesh path. One triangle or vertex
past a ceiling refuses at create, loudly.

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
