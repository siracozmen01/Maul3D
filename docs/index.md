# Maul3D

A deterministic, rollback-native 3D physics engine in pure C17.
Zero dependencies, MIT licensed, built for sandbox destruction and
lockstep multiplayer: the whole world snapshots, restores, and
replays to the bit, on every platform CI can reach (x86-64, ARM,
RISC-V, 32-bit, SIMD and scalar, Windows, wasm).

- [The manual](manual.html): the engine-host contract, chapter by
  chapter: rollback, the journal, destruction, vehicles, the
  character controller, soft bodies, water, heightfields, the
  replay studio, and lockstep networking.
- [Samples](samples.html): small complete programs, from a falling
  stack to a snapshot round-trip to a skid-steer tank.
- [The changelog](changelog.html): every release with its
  determinism ledger.
- [The repository](https://github.com/siracozmen01/Maul3D):
  source, releases, and the test suites that keep the promises.

## The promises

1. Same inputs, same bits, every platform, every thread count.
2. Snapshot, restore, journal replay: bit-exact, always.
3. Additive API under 1.x: code written today compiles tomorrow.
4. Loud failure: bad input refuses; it never corrupts quietly.
