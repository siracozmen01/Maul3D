# Contributing to Maul3D

Contributions are welcome. A few ground rules keep the project sane:

- **The maintainer has final say on every merge.** Expect design questions on
  anything that changes simulation behavior.
- **All CI gates must be green.** The determinism gates are non-negotiable:
  the whole point of this engine is bit-identical simulation across
  platforms, compilers, and build flavors. A PR that trades determinism for
  speed or convenience will not merge, however fast it is.
- **The golden hash and the benchmark pins move only with an argued
  hash-input change.** If your PR moves them, the PR body must say exactly
  which arithmetic changed and why the new trajectories are more correct
  than the old ones. "The tests were updated" is not an argument.
- **Sign your commits with DCO** (`git commit -s`). Contributions are MIT,
  inbound = outbound. No CLA.
- **Maul's own laws come first.** Determinism across platforms, bit-exact
  rollback, canonical ordering, loud failure: these are the constitution, and
  no outside precedent overrides them. Where Box2D, Box3D or Jolt have already
  solved a problem well, we study them and take the lesson; where their
  approach conflicts with Maul's laws, Maul wins - our snapshot, journal and
  replay layers exist precisely because the references never promised what
  this engine promises. If your PR takes a different road on a solved problem,
  say why in the body.
- **Everything under 1.x is additive.** Existing signatures, semantics and id
  layouts do not change. New def fields ride sizeof-derived cookies; new
  simulation state hashes off-default so worlds that never touch a feature
  keep their hashes.
- Formatting is enforced by the checked-in `.clang-format` - run it, don't
  debate it. Commit messages: imperative subject, body explains why, no
  trailers.
- Bug reports with a reproducing scene are gold. Determinism bug reports
  should include the platform pair and the world hash outputs; replay bugs
  should attach the `.m3j` container when possible.
